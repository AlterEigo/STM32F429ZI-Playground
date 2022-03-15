#![no_main]
#![no_std]

extern crate stm32f429_rt;
extern crate cortex_m;
mod init;

use cortex_m::peripheral::syst::SystClkSource;
use stm32f429_rt::{
    CorePeripherals,
    Peripherals
};

use core::panic::PanicInfo;
use core::ptr;

static RODATA: &[u8] = b"Hello world";
static mut BSS: u8 = 7;
static mut DATA: u16 = 9;

#[no_mangle]
pub unsafe extern "C" fn Reset() -> ! {
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

    // Init
    // HAL_SYSTICK_Config(16000000 / (1000u32 / 1u32))

    entrypoint()
}

#[no_mangle]
pub unsafe extern "C" fn SysTick() {
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

fn entrypoint() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut cperipherals = stm32f429_rt::CorePeripherals::take().unwrap();

    // Activating the GPIOG
    peripherals.RCC.ahb1enr.modify(|_, w| w.gpiogen().set_bit());

    // Setting output mode for the PG13 pin
    peripherals.GPIOG.moder.modify(|_, w| unsafe { w.moder13().bits(0x01) });
    peripherals.GPIOG.moder.modify(|_, w| unsafe { w.moder14().bits(0x01) });

    peripherals.GPIOG.odr.modify(|r, w| w.odr14().bit(!r.odr14().bit()));

    let count: u32 = 16_000_000;
    configure_clock(&mut cperipherals.SYST, count);

    loop {}
}

#[panic_handler]
fn panic(_panic: &PanicInfo<'_>) -> ! {
    loop {}
}

#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
pub static RESET_VECTOR: unsafe extern "C" fn() -> ! = Reset;

#[link_section = ".vector_table.systick_vector"]
#[no_mangle]
pub static SYSTICK_VECTOR: unsafe extern "C" fn() = SysTick;
