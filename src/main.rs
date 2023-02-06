#![allow(dead_code)]
#![no_main]
#![no_std]

mod init;

use stm32f429_rt::{
    CorePeripherals,
    Peripherals as NativePeripherals, GPIOC, GPIOF, GPIOD, tim2, tim5,
};

use core::cell::Cell;
use core::ops::{Deref, DerefMut, Shr};
use core::panic::PanicInfo;
use core::ptr;

const SCREEN_X_MAX: u16 = 240;
const SCREEN_Y_MAX: u16 = 320;

struct Peripherals {
    native: NativePeripherals,
    screen_max_x: Cell<u16>,
    screen_max_y: Cell<u16>,
}

impl Peripherals {
    unsafe fn steal() -> Self {
        Self {
            native: NativePeripherals::steal(),
            screen_max_x: Cell::new(SCREEN_X_MAX),
            screen_max_y: Cell::new(SCREEN_Y_MAX)
        }
    }
    
    fn tft_write<M>(&self, msg: M)
        where M: Into<TftMessage>
    {
        let msg = msg.into();
        self.GPIOD.odr.modify(|_, w| {
            match msg {
                TftMessage::Command(_) => w.odr13().set_bit(),
                _ => w.odr13().clear_bit(),
            }
        });

        // Start transmission
        self.GPIOC.odr.modify(|_, w| w.odr2().clear_bit());

        match msg {
            TftMessage::Command(cmd) => {
                self.SPI5.write_byte(cmd as u8);
            },
            TftMessage::Byte(byte) => {
                self.SPI5.write_byte(byte);
            },
            TftMessage::HWord(hword) => {
                self.SPI5.write_byte( (hword >> 8) as u8 );
                self.SPI5.write_byte( (hword & 0x0011) as u8 );
            },
            TftMessage::Word(word) => {
                let hhb = (word & 0x1100_0000).shr(24) as u8;
                let hlb = (word & 0x0011_0000).shr(16) as u8;
                let lhb = (word & 0x0000_1100).shr(8) as u8;
                let llb = (word & 0x0000_0011) as u8;

                self.SPI5.write_byte(hhb);
                self.SPI5.write_byte(hlb);
                self.SPI5.write_byte(lhb);
                self.SPI5.write_byte(llb);
            },
        }

        // End transmission
        self.GPIOC.odr.modify(|_, w| w.odr2().set_bit());
    }

    fn tft_reset(&self) {
        self.GPIOD.odr.modify(|_, w| w.odr12().clear_bit());
        
        self.TIM5.delay_ms(20);

        self.GPIOD.odr.modify(|_, w| w.odr12().set_bit());
    }

    fn tft_on_off(&self, value: bool) {
        if value {
            self.tft_write(TftCommand::DisplayOn);
        } else {
            self.tft_write(TftCommand::DisplayOff);
        }
    }

    fn tft_set_rotation(&self, rotation: TftRotation) {
        self.tft_write(TftCommand::Mac);

        match rotation {
            TftRotation::Portrait1 => {
                self.tft_write(0x58 as u8);
            },
            TftRotation::Portrait2 => {
                self.tft_write(0x88 as u8);
            },
            TftRotation::Landscape1 => {
                self.tft_write(0x28 as u8);
            },
            TftRotation::Landscape2 => {
                self.tft_write(0xE8 as u8);
            },
        }

        match rotation {
            TftRotation::Portrait1 | TftRotation::Portrait2 => {
                self.screen_max_x.replace(SCREEN_X_MAX);
                self.screen_max_y.replace(SCREEN_Y_MAX);
            },
            TftRotation::Landscape1 | TftRotation::Landscape2 => {
                self.screen_max_x.replace(SCREEN_Y_MAX);
                self.screen_max_y.replace(SCREEN_X_MAX);
            },
        }
    }

    fn tft_set_display_window(
        &self, x_pos1: u16, y_pos1: u16, x_pos2: u16, y_pos2: u16
    ) {
        self.tft_write(TftCommand::ColumnAddr);
        self.tft_write(x_pos1);
        self.tft_write(x_pos2);
        self.tft_write(TftCommand::PageAddr);
        self.tft_write(y_pos1);
        self.tft_write(y_pos2);
        self.tft_write(TftCommand::Gram);
    }

}

impl Deref for Peripherals {
    type Target = NativePeripherals;

    fn deref(&self) -> &Self::Target {
        &self.native
    }
}

impl DerefMut for Peripherals {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.native
    }
}

#[derive(Clone, Copy)]
enum TftRotation {
    Portrait1,
    Portrait2,
    Landscape1,
    Landscape2
}

#[derive(Clone, Copy)]
enum TftMessage {
    Command(TftCommand),
    Byte(u8),
    HWord(u16),
    Word(u32)
}

impl From<TftCommand> for TftMessage {
    fn from(value: TftCommand) -> Self {
        Self::Command(value)
    }
}

impl From<u8> for TftMessage {
    fn from(value: u8) -> Self {
        Self::Byte(value)
    }
}

impl From<u16> for TftMessage {
    fn from(value: u16) -> Self {
        Self::HWord(value)
    }
}

impl From<u32> for TftMessage {
    fn from(value: u32) -> Self {
        Self::Word(value)
    }
}

#[derive(Clone, Copy)]
enum TftCommand {
    Reset = 0x01,
    ReadDisplayIdentificationInformation = 0x04,
    ReadDisplayStatus = 0x09,
    ReadDisplayPowerMode = 0x0A,
    ReadDisplayMadctl = 0x0B,
    ReadDisplayPixelFormat = 0x0C,
    ReadDisplayImageFormat = 0x0D,
    ReadDisplaySignalMode = 0x0E,
    ReadDisplaySelfDiagnosticResult = 0x0F,
    EnterSleepMode = 0x10,
    SleepOut = 0x11,
    PartialModeOn = 0x12,
    NormalDisplayModeOn = 0x13,
    DisplayInversionOff = 0x20,
    DisplayInversionOn = 0x21,
    Gamma = 0x26,
    DisplayOff = 0x28,
    DisplayOn = 0x29,
    ColumnAddr = 0x2A,
    PageAddr = 0x2B,
    Gram = 0x2C,
    ColorSet = 0x2D,
    MemoryRead = 0x2E,
    PartialArea = 0x30,
    VerticalScrollingDefinition = 0x33,
    TearingEffectLineOff = 0x34,
    TearingEffectLineOn = 0x35,
    Mac = 0x36,
    VerticalScrollingStartAddress = 0x37,
    IdleModeOff = 0x38,
    IdleModeOn = 0x39,
    PixelFormat = 0x3A,
    Wmc = 0x3C,
    Rmc = 0x3E,
    SetTearScanline = 0x44,
    Wdb = 0x51,
    ReadDisplayBrightness = 0x52,
    Wcd = 0x53,
    ReadCtrlDisplay = 0x54,
    Wcabc = 0x55,
    Rcabc = 0x56,
    Wcabcmb = 0x5E,
    Rcabcmb = 0x5F,
    RgbInterface = 0xB0,
    Frc = 0xB1,
    FrameCtrlNm = 0xB2,
    FrameCtrlIm = 0xB3,
    FrameCtrlPm = 0xB4,
    Bpc = 0xB5,
    Dfc = 0xB6,
    EntryModeSet = 0xB7,
    BacklightControl1 = 0xB8,
    BacklightControl2 = 0xB9,
    BacklightControl3 = 0xBA,
    BacklightControl4 = 0xBB,
    BacklightControl5 = 0xBC,
    BacklightControl6 = 0xBD,
    BacklightControl7 = 0xBE,
    BacklightControl8 = 0xBF,
    Power1 = 0xC0,
    Power2 = 0xC1,
    Vcom1 = 0xC5,
    Vcom2 = 0xC7,
    PowerA = 0xCB,
    PowerB = 0xCF,
    ReadId1 = 0xDA,
    ReadId2 = 0xDB,
    ReadId3 = 0xDC,
    Pgamma = 0xE0,
    Ngamma = 0xE1,
    DtcA = 0xE8,
    DtcB = 0xEA,
    PowerSeq = 0xED,
    Gamma3En = 0xF2,
    Interface = 0xF6,
    Prc = 0xF7,
}

/// High-level SPI module methods
trait SpiHl {
    fn write_byte(&self, value: u8);
}

impl SpiHl for stm32f429_rt::SPI5 {
    fn write_byte(&self, value: u8) {
        // Assert the TXE flag is set before writing
        // (We wait until it is set)
        while ! self.sr.read().txe().bit_is_set() { /* do nothing */ }

        // Write byte into Tx buffer
        self.dr.write(|w| w.dr().variant(value as u16));

        // Wait until transmitted data is sampled
        while ! self.sr.read().rxne().bit_is_set() {}

        // Discarding received data and causing RXNE to clear
        let _: u16 = self.dr.read().dr().bits();

        // Assert the TXE flag is set after writing
        while ! self.sr.read().txe().bit_is_set() { /* do nothing */ }

        // Assert the BSY flag is set after writing
        // while ! self.sr.read().bsy().bit_is_clear() { [> do nothing <] }
    }
}

trait MsDelay: Deref<Target = Self::TargetTimRegister> {
    type TargetTimRegister;

    fn delay_ms(&self, dt: u32);
}

impl MsDelay for stm32f429_rt::TIM5
{
    type TargetTimRegister = tim5::RegisterBlock;

    fn delay_ms(&self, dt: u32) {
        // Asserting the timer is active at this precise moment
        assert!(self.cr1.read().cen().bit_is_set());

        // Asserting timer is not in pulse mode
        assert!(self.cr1.read().opm().bit_is_clear());

        // Setting no pending update
        self.sr.write(|w| w.uif().clear_bit());

        // Setting 0 value in the count register
        self.cnt.write(|w| w
            .cnt_l().variant(0)
            .cnt_h().variant(0)
        );

        // Assuming running in 16MHz mode with 1ms step
        assert_eq!(self.arr.read().bits(), (16_000_000 / 1000) - 1);
        // let mut dt: u32 = (16_000_000 / 1000 * dt) - 1;

        let mut dt = Some(dt);
        loop {
            if dt.is_none() {
                break;
            }
            if self.sr.read().uif().bit_is_clear() {
                continue;
            }
            self.sr.write(|w| w.uif().clear_bit());
            dt = dt.unwrap().checked_sub(1);
        }
    }
}

impl MsDelay for stm32f429_rt::TIM2
{
    type TargetTimRegister = tim2::RegisterBlock;

    fn delay_ms(&self, dt: u32) {
        todo!()
    }
}

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
pub unsafe extern "C" fn sys_tick() {}

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

fn configure_rcc(p: &mut Peripherals) {
    let rcc = &p.RCC;
    let tim5 = &p.TIM5;

    // Activating APB1 and APB1LP
    rcc.apb1enr.write(|w| {
        w.pwren().set_bit()
    });

    rcc.apb1lpenr.write(|w| {
        w.pwrlpen().set_bit()
    });

    // Enabling TIM5 clock
    {
        rcc.apb1enr.modify(|_, w| w.tim5en().set_bit());

        tim5.arr.write(|w| w
            // We want an update each millisecond, if the
            // clock frequency is 16MHz
            .arr_l().variant(16_000 - 1)
        );

        tim5.cr1.write(|w| w
            .dir().clear_bit()
            // .arpe().set_bit()
            // .cms().variant(0)
            // .opm().set_bit()
            // .urs().set_bit()
            .udis().clear_bit()
            .cen().set_bit()
        );
    }

    // rcc.apb2enr.write(|w| w.);
    rcc.apb2enr.modify(|_, w| w.ltdcen().clear_bit());
    rcc.apb2lpenr.modify(|_, w| w.ltdclpen().clear_bit());

    rcc.apb2enr.modify(|_, w| w.spi5en().set_bit());
    rcc.apb2lpenr.modify(|_, w| w.spi5lpen().set_bit());
}

fn entrypoint() -> ! {
    let mut peripherals = unsafe { Peripherals::steal() };
    let mut cperipherals = unsafe { CorePeripherals::steal() };

    // TODO
    // apb1enr pwren high
    // ...

    configure_rcc(&mut peripherals);

    // Activating GPIO C, D and F for LTDC
    peripherals.RCC.ahb1enr.modify(|_, w| {
        w.gpiocen().set_bit();
        w.gpioden().set_bit();
        w.gpiofen().set_bit()
    });

    configure_gpioc(&mut peripherals.GPIOC);
    configure_gpiod(&mut peripherals.GPIOD);
    configure_gpiof(&mut peripherals.GPIOF);

    peripherals.TIM5.delay_ms(10);

    // Configuring SPI
    {
        // Enabling clock
        peripherals.RCC.apb2enr.modify(|_, w| w
            .spi5en().set_bit()
        );
        // Enabling clock in sleep mode
        peripherals.RCC.apb2lpenr.modify(|_, w| w
            .spi5lpen().set_bit()
        );

        peripherals.SPI5.cr1.modify(|_, w| w
            // Enabling SPI
            .spe().set_bit()
            // Fpclk/4
            .br().variant(0b001)
            // As master
            .mstr().set_bit()
            // 8-bit mode
            .dff().clear_bit()
            // Idle low
            .cpol().clear_bit()
            // First clock transfer = first data capture
            .cpha().clear_bit()
            // Most significant bit first
            .lsbfirst().clear_bit()
            // -
            .ssm().set_bit()
            // -
            .ssi().set_bit()
        );

        // peripherals.SPI5.cr2.write(|w| w
            // SS disable
            // .ssoe().clear_bit()
            // TI (8080) mode
            // .frf().set_bit()
        // );

        peripherals.GPIOD.odr.modify(|_, w| w
            .odr5().set_bit()
            .odr4().set_bit()
        );
    }

    {
        peripherals.tft_reset();

        // WRDISBV
        peripherals.tft_write(TftCommand::Reset);
        peripherals.tft_write(0x00 as u8);

        peripherals.tft_write(TftCommand::PowerA);
        peripherals.tft_write(0x39 as u8);
        peripherals.tft_write(0x2C as u8);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0x34 as u8);
        peripherals.tft_write(0x02 as u8);

        peripherals.tft_write(TftCommand::PowerB);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0xC1 as u8);
        peripherals.tft_write(0x30 as u8);

        peripherals.tft_write(TftCommand::DtcA);
        peripherals.tft_write(0x85 as u8);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0x78 as u8);

        peripherals.tft_write(TftCommand::DtcB);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0x00 as u8);

        peripherals.tft_write(TftCommand::PowerSeq);
        peripherals.tft_write(0x64 as u8);
        peripherals.tft_write(0x03 as u8);
        peripherals.tft_write(0x12 as u8);
        peripherals.tft_write(0x81 as u8);

        peripherals.tft_write(TftCommand::Prc);
        peripherals.tft_write(0x20 as u8);

        peripherals.tft_write(TftCommand::Power1);
        peripherals.tft_write(0x23 as u8);

        peripherals.tft_write(TftCommand::Power2);
        peripherals.tft_write(0x10 as u8);

        peripherals.tft_write(TftCommand::Vcom1);
        peripherals.tft_write(0x3E as u8);
        peripherals.tft_write(0x28 as u8);

        peripherals.tft_write(TftCommand::Vcom2);
        peripherals.tft_write(0x86 as u8);

        peripherals.tft_write(TftCommand::Mac);
        peripherals.tft_write(0x48 as u8);

        peripherals.tft_write(TftCommand::PixelFormat);
        peripherals.tft_write(0x55 as u8);

        peripherals.tft_write(TftCommand::Frc);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0x18 as u8);

        peripherals.tft_write(TftCommand::Dfc);
        peripherals.tft_write(0x08 as u8);
        peripherals.tft_write(0x82 as u8);
        peripherals.tft_write(0x27 as u8);

        peripherals.tft_write(TftCommand::Gamma3En);
        peripherals.tft_write(0x00 as u8);

        peripherals.tft_write(TftCommand::ColumnAddr);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0xEF as u8);

        peripherals.tft_write(TftCommand::PageAddr);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0x01 as u8);
        peripherals.tft_write(0x3F as u8);

        peripherals.tft_write(TftCommand::Gamma);
        peripherals.tft_write(0x01 as u8);

        peripherals.tft_write(TftCommand::Pgamma);
        peripherals.tft_write(0x0F as u8);
        peripherals.tft_write(0x31 as u8);
        peripherals.tft_write(0x2B as u8);
        peripherals.tft_write(0x0C as u8);
        peripherals.tft_write(0x0E as u8);
        peripherals.tft_write(0x08 as u8);
        peripherals.tft_write(0x4E as u8);
        peripherals.tft_write(0xF1 as u8);
        peripherals.tft_write(0x37 as u8);
        peripherals.tft_write(0x07 as u8);
        peripherals.tft_write(0x10 as u8);
        peripherals.tft_write(0x03 as u8);
        peripherals.tft_write(0x0E as u8);
        peripherals.tft_write(0x09 as u8);
        peripherals.tft_write(0x00 as u8);

        peripherals.tft_write(TftCommand::Ngamma);
        peripherals.tft_write(0x00 as u8);
        peripherals.tft_write(0x0E as u8);
        peripherals.tft_write(0x14 as u8);
        peripherals.tft_write(0x03 as u8);
        peripherals.tft_write(0x11 as u8);
        peripherals.tft_write(0x07 as u8);
        peripherals.tft_write(0x31 as u8);
        peripherals.tft_write(0xC1 as u8);
        peripherals.tft_write(0x48 as u8);
        peripherals.tft_write(0x08 as u8);
        peripherals.tft_write(0x0F as u8);
        peripherals.tft_write(0x0C as u8);
        peripherals.tft_write(0x31 as u8);
        peripherals.tft_write(0x36 as u8);
        peripherals.tft_write(0x0F as u8);

        peripherals.tft_write(TftCommand::SleepOut);
        peripherals.TIM5.delay_ms(10);
        
        peripherals.tft_on_off(true);
        peripherals.tft_write(TftCommand::Gram);

        peripherals.tft_set_rotation(TftRotation::Landscape1);
    }

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
