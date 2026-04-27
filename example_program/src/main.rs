/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Example Program.
 *
 * The SmallOS Example Program is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Example Program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS Example Program. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::fmt::{self, Write};

use small_os_lib::{args, do_yield, send, send_empty};

/// Prints to the UART driver
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


/// Print macro  
/// Based off <https://os.phil-opp.com/vga-text-mode/> accessed 22/01/2026
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => ($crate::_print(format_args!($($arg)*)));
}

/// Print line macro  
/// Based off <https://os.phil-opp.com/vga-text-mode/> accessed 22/01/2026
#[macro_export]
macro_rules! println {
    () => ($crate::print!("\r\n"));
    ($($arg:tt)*) => ($crate::print!("{}\r\n", format_args!($($arg)*)));
}


/// Print function  
/// `args` is a list of formatting arguments which dictate what's printed  
/// Based off <https://os.phil-opp.com/vga-text-mode/> accessed 22/01/2026
#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    let mut print = UARTPrint {};
    let res = print.write_fmt(args);
    res.unwrap();
}

/// Analog to Digital Converter endpoint
const ADC_QUEUE: u32 = 1;
/// Camera endpoint
const CAMERA_QUEUE: u32 = 2;
/// Kermit endpoint
const KERMIT_QUEUE: u32 = 3;

/// Tag to set image format
const CAMERA_IMAGE_FORMAT_TAG: u16 = 4;
/// Tag to start camera capture
const CAMERA_CAPTURE_IMAGES_TAG: u16 = 5;
/// Tag to get camera fifo size
const CAMERA_FIFO_SIZE_TAG: u16 = 6;
/// Tag to determine if the camera has finished taking pictures
const CAMERA_CAPTURES_DONE_TAG: u16 = 8;
/// Tag to read the camera's fifo
const CAMERA_READ_FIFO_TAG: u16 = 9;

/// Tag to start a kermit transaction
const KERMIT_START_TRANSACTION_TAG: u16 = 0;
/// Tag to start writing a file from kermit
const KERMIT_START_FILE_TAG: u16 = 1;
/// Tag to send file data to kermit
const KERMIT_SEND_FILE_DATA_TAG: u16 = 2;
/// Tag to finish writing a file from kermit
const KERMIT_FINISH_FILE_TAG: u16 = 3;
/// Tag to end a kermit transaction
const KERMIT_END_TRANSACTION_TAG: u16 = 4;

/// Temperature Sensor Example Program
#[cfg(not(feature = "camera"))]
fn temp_sensor() -> ! {
    loop {
        let mut temp_buffer = [2, 0];
        send(ADC_QUEUE, 0, &mut temp_buffer, 1, 2).unwrap();
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

/// Camera Sensor Example Program
#[cfg(feature = "camera")]
fn camera_sensor() -> ! {
    send_empty(CAMERA_QUEUE, CAMERA_IMAGE_FORMAT_TAG, &[1]).unwrap();
    loop {
        send_empty(CAMERA_QUEUE, CAMERA_CAPTURE_IMAGES_TAG, &[1]).unwrap();
        let mut cap_done_buffer = [0; 1];
        send(CAMERA_QUEUE, CAMERA_CAPTURES_DONE_TAG, &mut cap_done_buffer, 0, 1).unwrap(); 
        let mut cap_done = bool::try_from(cap_done_buffer[0]).unwrap();
        while !cap_done {
            do_yield().unwrap();
            send(CAMERA_QUEUE, CAMERA_CAPTURES_DONE_TAG, &mut cap_done_buffer, 0, 1).unwrap(); 
            cap_done = bool::try_from(cap_done_buffer[0]).unwrap();
        }
        send_empty(KERMIT_QUEUE, KERMIT_START_TRANSACTION_TAG, &[]).unwrap();
        send_empty(KERMIT_QUEUE, KERMIT_START_FILE_TAG, b"img2.jpeg").unwrap();
        let mut data = [0; 1028];
        let len = data.len();
        send(CAMERA_QUEUE, CAMERA_READ_FIFO_TAG, &mut data, 0, len).unwrap();
        let mut data_len = u32::from_le_bytes(data[..4].try_into().unwrap()) as usize;
        while data_len > 0 {
            let rep = (data_len + 37) / 38;
            for i in 0..rep {
                let offset = i * 38;
                let len = 38.min(data_len - offset);
                send_empty(KERMIT_QUEUE, KERMIT_SEND_FILE_DATA_TAG, &data[4 + offset..4 + offset + len]).unwrap();
            }
            send(CAMERA_QUEUE, CAMERA_READ_FIFO_TAG, &mut data, 0, len).unwrap();
            data_len = u32::from_le_bytes(data[..4].try_into().unwrap()) as usize;
        }
        send_empty(KERMIT_QUEUE, KERMIT_FINISH_FILE_TAG, &[]).unwrap();
        send_empty(KERMIT_QUEUE, KERMIT_END_TRANSACTION_TAG, &[]).unwrap();
    }
}

/// Program entry point
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 0);
    #[cfg(test)]
    test_main();
    #[cfg(not(feature = "camera"))]
    temp_sensor();
    #[cfg(feature = "camera")]
    camera_sensor();
}

/// Test framework which runs all the tests  
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::kprintln;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for example program", tests.len());
        for test in tests {
            test();
        }
    }
}
