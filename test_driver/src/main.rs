// use core intrinsics 
#![feature(core_intrinsics)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::{intrinsics::abort, panic::PanicInfo};

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    loop {}
}
