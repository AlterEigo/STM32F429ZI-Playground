#![allow(dead_code)]
#![no_main]
#![no_std]

mod init;

use cortex_m::peripheral::syst::SystClkSource;
use stm32f429_rt::{
    CorePeripherals,
    Peripherals, GPIOC, GPIOF, GPIOD, RCC,
};

use core::panic::PanicInfo;
use core::ptr;

static RODATA: &[u8] = b"Hello world";
static mut BSS: u8 = 7;
static mut DATA: u16 = 9;

#[no_mangle]
pub unsafe extern "C" fn reset() -> ! {
    extern "C" {
        static mut _sbss: u8;
        static mut _ebss: u8;
        static mut _sdata: u8;
        static mut _edata: u8;
        static _sidata: u8;
    }

    let bsslen = &_ebss as *const u8 as usize - &_sbss as *const u8 as usize;
    ptr::write_bytes(&mut _sbss as *mut u8, 0, bsslen);

    let datalen = &_edata as *const u8 as usize - &_sdata as *const u8 as usize;
    ptr::copy_nonoverlapping(&_sidata as *const u8, &mut _sdata as *mut u8, datalen);

    entrypoint()
}

#[no_mangle]
pub unsafe extern "C" fn sys_tick() {
    // Toggling value of PG13
    
    (*stm32f429_rt::GPIOG::ptr())
        .odr.modify(|r, w| w.odr13().bit(!r.odr13().bit()));
    (*stm32f429_rt::GPIOG::ptr())
        .odr.modify(|r, w| w.odr14().bit(!r.odr14().bit()));
}

fn configure_clock(syst: &mut stm32f429_rt::SYST, freq: u32) {

    const OVMASK: u32 = 0x1100_0000;

    if (freq & OVMASK) != 0x0 {
        panic!("SYST reload value overflow (24-bit limitation)");
    }

    syst.set_clock_source(SystClkSource::Core);
    
    // Setting our own reload value
    // Push reload value into [0,23] bits of SYST_RVR register
    syst.set_reload(freq);

    // Clearing any garbage value
    // Reset value of [0,23] bits of SYST_CVR register
    // (any value resets the value to 0)
    syst.clear_current();

    syst.enable_interrupt();

    syst.enable_counter();
}

fn configure_gpioc(gpioc: &mut GPIOC) {
    gpioc.moder.write(|w| unsafe {
        w.moder2().bits(0b01) // Mode: Output
    });
    gpioc.otyper.write(|w| {
        w.ot2().clear_bit() // Output type: push-pull
    });
    gpioc.ospeedr.write(|w| unsafe {
        w.ospeedr2().bits(0b11) // Speed: maximum
    });
}

fn configure_gpiod(gpiod: &mut GPIOD) {
    // Output modes: gp output
    gpiod.moder.write(|w| unsafe {
        w.moder12().bits(0b01);
        w.moder13().bits(0b01)
    });
    // Output type: push-pull
    gpiod.otyper.write(|w| {
        w.ot12().clear_bit();
        w.ot13().clear_bit()
    });
    // Speed: maximum
    gpiod.ospeedr.write(|w| unsafe {
        w.ospeedr12().bits(0b11);
        w.ospeedr12().bits(0b11)
    });
}

fn configure_gpiof(gpiof: &mut GPIOF) {
    // Setting alternate function 5 for pins 7,8 and 9 of port GPIOF
    gpiof.afrl.write(|w| unsafe {
        w.afrl7().bits(0b0101) // AF5 = SPI1/2/3/4/5
    });

    gpiof.afrh.write(|w| unsafe {
        w.afrh8().bits(0b0101);
        w.afrh9().bits(0b0101)
    });

    gpiof.otyper.write(|w| {
        w.ot7().clear_bit(); // push-pull
        w.ot8().set_bit();   // open-drain
        w.ot9().clear_bit()  // push-pull
    });

    // Setting max speed for all used pins
    gpiof.ospeedr.write(|w| unsafe {
        w.ospeedr7().bits(0b11);
        w.ospeedr8().bits(0b11);
        w.ospeedr9().bits(0b11)
    });

    // Enabling alternate function mode for 7th, 8th and 9th pins 
    gpiof.moder.write(|w| unsafe {
        w.moder7().bits(0x10);
        w.moder8().bits(0x10);
        w.moder9().bits(0x10)
    });
}

fn configure_rcc(rcc: &mut RCC) {
    rcc.apb1enr.write(|w| {
        w.pwren().set_bit()
    });

    rcc.apb1lpenr.write(|w| {
        w.pwrlpen().set_bit()
    });

    // rcc.apb2enr.write(|w| w.);
    rcc.apb2enr.write(|w| w.ltdcen().clear_bit());
    rcc.apb2lpenr.write(|w| w.ltdclpen().clear_bit());

    rcc.apb2enr.write(|w| w.spi5en().set_bit());
    rcc.apb2lpenr.write(|w| w.spi5lpen().set_bit());
}

fn program_led(mut peripherals: Peripherals, mut cperipherals: CorePeripherals) {
    // Activating the GPIOG
    peripherals.RCC.ahb1enr.modify(|_, w| w.gpiogen().set_bit());

    // Setting output mode for the PG13 pin
    peripherals.GPIOG.moder.modify(|_, w| unsafe { w.moder13().bits(0x01) });
    peripherals.GPIOG.moder.modify(|_, w| unsafe { w.moder14().bits(0x01) });

    peripherals.GPIOG.odr.modify(|r, w| w.odr14().bit(!r.odr14().bit()));

    let count: u32 = (16_000_000 / 1_000) - 1;
    configure_clock(&mut cperipherals.SYST, 8_000_000 - 1);

    loop {}
}

fn entrypoint() -> ! {
    let mut peripherals = unsafe { Peripherals::steal() };
    let mut cperipherals = unsafe { CorePeripherals::steal() };

    // TODO
    // apb1enr pwren high
    // ...

    configure_rcc(&mut peripherals.RCC);

    // Activating GPIO C, D and F for LTDC
    peripherals.RCC.ahb1enr.modify(|_, w| {
        w.gpiocen().set_bit();
        w.gpioden().set_bit();
        w.gpiofen().set_bit()
    });

    configure_gpioc(&mut peripherals.GPIOC);
    configure_gpiod(&mut peripherals.GPIOD);
    configure_gpiof(&mut peripherals.GPIOF);

    loop {}
}

#[panic_handler]
fn panic(_panic: &PanicInfo<'_>) -> ! {
    loop {}
}

#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
pub static RESET_VECTOR: unsafe extern "C" fn() -> ! = reset;

#[link_section = ".vector_table.systick_vector"]
#[no_mangle]
pub static SYSTICK_VECTOR: unsafe extern "C" fn() = sys_tick;
