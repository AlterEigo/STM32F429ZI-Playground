#![no_main]
#![no_std]

//extern crate cortex_m;
extern crate panic_itm;

use core::panic::PanicInfo;

#[no_mangle]
pub extern "C" fn _start() -> ! {
    entrypoint()
}

fn entrypoint() -> ! {
    let _y;
    let x = 42;
    _y = x;

    loop {

    }
}
