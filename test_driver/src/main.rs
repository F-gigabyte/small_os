// use core intrinsics 
#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{fmt::{self, Write}, intrinsics::abort, panic::PanicInfo};

use small_os_lib::{send, send_empty};


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
        let mut temp_buffer = [0; 2];
        send(1, 2, &mut temp_buffer, 0, 2).unwrap();
        let temp = u16::from_le_bytes(temp_buffer);
        if temp & 1 == 0 {
            // based of https://microcontrollerslab.com/raspberry-pi-pico-adc-tutorial/ accessed
            // 16/03/2026 where it was mentioned the entire level between 0 and 2 ^ 12 - 1 
            // corresponds to 0V to 3.3V
            let temp = temp >> 1;
            let temp = ((temp as u32) * 1650) / 2048;
            let temp = (temp as i16) - 500;
            let frac = temp % 10;
            let frac = if frac < 0 {
                (-frac) as u8
            } else {
                frac as u8
            };
            let int = temp / 10;
            println!("This is test proc! ({}.{}C)\r", int, frac);
        }
    }
}
