// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::fmt::{self, Write};

use small_os_lib::{args, kprintln, send, send_empty};

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

const ADC_QUEUE: u32 = 1;
const CAMERA_QUEUE: u32 = 2;
const KERMIT_QUEUE: u32 = 3;

const CAMERA_IMAGE_FORMAT_TAG: u16 = 4;
const CAMERA_CAPTURE_IMAGES_TAG: u16 = 5;
const CAMERA_FIFO_SIZE_TAG: u16 = 6;
const CAMERA_CAPTURES_DONE_TAG: u16 = 8;
const CAMERA_READ_FIFO_TAG: u16 = 9;

const KERMIT_START_TRANSACTION_TAG: u16 = 0;
const KERMIT_START_FILE_TAG: u16 = 1;
const KERMIT_SEND_FILE_DATA_TAG: u16 = 2;
const KERMIT_FINISH_FILE_TAG: u16 = 3;
const KERMIT_END_TRANSACTION_TAG: u16 = 4;

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 0);
    /*send_empty(2, 4, &[16]).unwrap();
    send_empty(2, 4, &[17]).unwrap();
    send_empty(2, 4, &[18]).unwrap();
    send_empty(2, 4, &[19]).unwrap();
    send_empty(2, 1, &[16]).unwrap();
    send_empty(2, 1, &[17]).unwrap();
    send_empty(2, 1, &[18]).unwrap();
    send_empty(2, 1, &[19]).unwrap();
    */
    send_empty(CAMERA_QUEUE, CAMERA_IMAGE_FORMAT_TAG, &[9]).unwrap();
    send_empty(CAMERA_QUEUE, CAMERA_CAPTURE_IMAGES_TAG, &[1]).unwrap();
    let mut prev_cap_done = false;
    let mut prev_len_zero = false;
    #[cfg(test)]
    test_main();
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
            //println!("This is test proc! ({}.{}C)\r", int, frac);
        }
        let mut cap_done_buffer = [0; 1];
        send(CAMERA_QUEUE, CAMERA_CAPTURES_DONE_TAG, &mut cap_done_buffer, 0, 1).unwrap(); 
        let cap_done = bool::try_from(cap_done_buffer[0]).unwrap();
        if cap_done {
            if !prev_cap_done {
                send_empty(KERMIT_QUEUE, KERMIT_START_TRANSACTION_TAG, &[]).unwrap();
                send_empty(KERMIT_QUEUE, KERMIT_START_FILE_TAG, b"img2.jpeg").unwrap();
                prev_cap_done = true;
            }
            let mut data = [0; 1028];
            let len = data.len();
            send(CAMERA_QUEUE, CAMERA_READ_FIFO_TAG, &mut data, 0, len).unwrap();
            let data_len = u32::from_le_bytes(data[..4].try_into().unwrap()) as usize;
            if data_len > 0 {
                let rep = (data_len + 37) / 38;
                for i in 0..rep {
                    let offset = i * 38;
                    let len = 38.min(data_len - offset);
                    send_empty(KERMIT_QUEUE, KERMIT_SEND_FILE_DATA_TAG, &data[4 + offset..4 + offset + len]).unwrap();
                }
            } else if !prev_len_zero {
                send_empty(KERMIT_QUEUE, KERMIT_FINISH_FILE_TAG, &[]).unwrap();
                send_empty(KERMIT_QUEUE, KERMIT_END_TRANSACTION_TAG, &[]).unwrap();
                prev_len_zero = true;
            }
        }
    }
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
