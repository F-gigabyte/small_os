// use core intrinsics 
#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{arch::asm, fmt::{self, Write}, intrinsics::abort, panic::PanicInfo};

use small_os_lib::send_empty;


/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

struct UARTPrint {}

impl Write for UARTPrint {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let data = s.as_bytes();
        let mut pos = 0;
        for _ in 0..data.len() / 32 {
            send_empty(0, 0, &data[pos..pos + 32]).map_err(|_| fmt::Error)?;
            pos += 32;
        }
        if pos < data.len() {
            send_empty(0, 0, &data[pos..]).map_err(|_| fmt::Error)?;
        }
        Ok(())
    }
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => ($crate::_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! println {
    () => ($crate::print!("\n"));
    ($($arg:tt)*) => ($crate::print!("{}\n", format_args!($($arg)*)));
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    let mut print = UARTPrint {};
    let res = print.write_fmt(args);
    res.unwrap();
}

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    loop {
        println!("This is test proc!\r");
    }
}
