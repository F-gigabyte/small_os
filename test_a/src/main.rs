/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Test A driver.
 *
 * The SmallOS Test A driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Test A driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS Test A driver. 
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

use small_os_lib::{QueueError, ReplyError, args, kprint, kprintln, notify_read_header, notify_receive, notify_reply, notify_send, read_header, read_header_async_non_blocking, read_header_non_blocking, send, send_async, send_empty};
use core::assert_matches;

/// Test B endpoint
pub const QUEUE_B: u32 = 0;

/// Test main function
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 0);
    kprintln!("Test A Tests");
    kprint!("Testing read header on empty queue ");
    assert_matches!(read_header_non_blocking(QUEUE_B), Err(QueueError::QueueEmpty));
    assert_matches!(read_header_async_non_blocking(QUEUE_B), Err(QueueError::QueueEmpty));
    kprintln!("[ok]");
    kprint!("Testing send empty to B ");
    assert_matches!(send_empty(QUEUE_B, 1, &[b'h', b'e', b'l', b'l', b'o']), Err(ReplyError::RequestError(4)));
    kprintln!("[ok]");
    kprint!("Testing send async ");
    send_async(QUEUE_B, 95, &[1, 2, 3, 4, 5]).unwrap();
    kprintln!("[ok]");
    kprint!("Testing notify send ");
    let header = read_header(QUEUE_B).unwrap();
    assert_eq!(header.driver, 0);
    assert_eq!(header.tag, 83);
    assert_eq!(header.pin_mask, 0);
    assert_eq!(header.send_len, 7);
    assert_eq!(header.reply_len, 7);
    notify_send(QUEUE_B, 0).unwrap();
    kprintln!("[ok]");
    kprint!("Testing notify read header ");
    let header = notify_read_header(0).unwrap();
    assert_eq!(header.driver, 0);
    assert_eq!(header.tag, 83);
    assert_eq!(header.pin_mask, 0);
    assert_eq!(header.send_len, 7);
    assert_eq!(header.reply_len, 7);
    kprintln!("[ok]");
    kprint!("Testing notify receive ");
    let mut data = [0; 7];
    assert_matches!(notify_receive(0, &mut []), Err(QueueError::BufferTooSmall));
    assert_eq!(notify_receive(0, &mut data).unwrap(), 7);
    assert_eq!(data[0], b'H');
    assert_eq!(data[1], b'e');
    assert_eq!(data[2], b'l');
    assert_eq!(data[3], b'l');
    assert_eq!(data[4], b'o');
    assert_eq!(data[5], b' ');
    assert_eq!(data[6], b'A');
    kprintln!("[ok]");
    kprint!("Testing notify reply ");
    data[6] = b'B';
    notify_reply(0, 0, &data).unwrap();
    kprintln!("[ok]");
    loop {}
}

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::kprintln;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for test_a", tests.len());
        for test in tests {
            test();
        }
    }
}
