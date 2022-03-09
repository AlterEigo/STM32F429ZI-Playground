#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_halt;
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let _y;
    let x = 42;
    _y = x;

    loop {

    }
}
