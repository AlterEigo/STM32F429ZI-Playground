#![no_main]
#![no_std]

//extern crate cortex_m;
extern crate panic_itm;

use core::panic::PanicInfo;

#[no_mangle]
pub extern "C" fn _start() -> ! {
    entrypoint()
}

#[no_mangle]
pub extern "C" fn Reset() -> ! {
    let _x = 42;

    loop {}
}

#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
pub static RESET_VECTOR: unsafe extern "C" fn() -> ! = Reset;

fn entrypoint() -> ! {
    let _y;
    let x = 42;
    _y = x / 2;

    loop {

    }
}
