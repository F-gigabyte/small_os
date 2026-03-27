#![no_std]
#![no_main]

use core::fmt::{self, Write};

use small_os_lib::{send, send_empty};

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
    () => ($crate::print!("\r\n"));
    ($($arg:tt)*) => ($crate::print!("{}\r\n", format_args!($($arg)*)));
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
pub extern "C" fn main(num_args: usize) {
    assert!(num_args == 0);
    /*send_empty(2, 4, &[16]).unwrap();
    send_empty(2, 4, &[17]).unwrap();
    send_empty(2, 4, &[18]).unwrap();
    send_empty(2, 4, &[19]).unwrap();
    send_empty(2, 1, &[16]).unwrap();
    send_empty(2, 1, &[17]).unwrap();
    send_empty(2, 1, &[18]).unwrap();
    send_empty(2, 1, &[19]).unwrap();
    */
    loop {
        let mut temp_buffer = [2, 0];
        send(1, 0, &mut temp_buffer, 1, 2).unwrap();
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
        let mut version_info = [0; 4];
        send(2, 4, &mut version_info, 0, 4).unwrap();
        println!("Camera version: {}.{}\r\nYear: {}\r\nMonth: {}\r\nDate: {}", version_info[0] >> 4, version_info[0] & 0xf, version_info[1] as usize + 2000, version_info[2], version_info[3]);
        send(2, 5, &mut version_info, 0, 2).unwrap();
        println!("Manufacture ID: 0x{:x}", u16::from_le_bytes(version_info[..2].try_into().unwrap()));
    }
}
