#![no_main]
#![no_std]

//extern crate cortex_m;
extern crate panic_itm;

static RODATA: &[u8] = b"Hello world";
static mut BSS: u8 = 0;
static mut DATA: u16 = 1;

#[no_mangle]
pub extern "C" fn Reset() -> ! {
    let x = 42;
    let y = x / 2;
    let _z = y;

    loop {}
}

#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
pub static RESET_VECTOR: unsafe extern "C" fn() -> ! = Reset;
