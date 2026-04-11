// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use small_os_lib::{QueueError, ReplyError, args, kprint, kprintln, read_header, read_header_async, read_header_non_blocking, receive, receive_async, reply_empty, send};
use core::assert_matches;

pub const QUEUE_A: u32 = 0;

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 0);
    kprintln!("Test B Tests");
    let header = read_header(QUEUE_A).unwrap();
    assert_eq!(header.driver, 0);
    assert_eq!(header.tag, 1);
    assert_eq!(header.pin_mask, 0);
    assert_eq!(header.send_len, 5);
    assert_eq!(header.reply_len, 0);
    let header = read_header_non_blocking(QUEUE_A).unwrap();
    assert_eq!(header.driver, 0);
    assert_eq!(header.tag, 1);
    assert_eq!(header.pin_mask, 0);
    assert_eq!(header.send_len, 5);
    assert_eq!(header.reply_len, 0);
    let mut data = [0; 5];
    assert_eq!(receive(QUEUE_A, &mut data).unwrap(), 5);
    assert_eq!(data[0], b'h');
    assert_eq!(data[1], b'e');
    assert_eq!(data[2], b'l');
    assert_eq!(data[3], b'l');
    assert_eq!(data[4], b'o');
    kprintln!("[ok]");
    kprint!("Testing reply empty to A ");
    reply_empty(QUEUE_A, 4).unwrap();
    kprint!("Testing read header async ");
    let header = read_header_async(QUEUE_A).unwrap();
    assert_eq!(header.driver, 0);
    assert_eq!(header.tag, 95);
    assert_eq!(header.pin_mask, 0);
    assert_eq!(header.message_len, 5);
    kprintln!("[ok]");
    kprint!("Testing receive async ");
    assert_eq!(receive_async(QUEUE_A, &mut data).unwrap(), 5);
    assert_eq!(data[0], 1);
    assert_eq!(data[1], 2);
    assert_eq!(data[2], 3);
    assert_eq!(data[3], 4);
    assert_eq!(data[4], 5);
    kprintln!("[ok]");
    let mut data = [b'H', b'e', b'l', b'l', b'o', b' ', b'A'];
    kprint!("Testing send with reply ");
    assert_matches!(send(QUEUE_A, 83, &mut data, 8, 8), Err(ReplyError::QueueError(QueueError::BufferTooSmall)));
    send(QUEUE_A, 83, &mut data, 7, 7).unwrap();
    assert_eq!(data[0], b'H');
    assert_eq!(data[1], b'e');
    assert_eq!(data[2], b'l');
    assert_eq!(data[3], b'l');
    assert_eq!(data[4], b'o');
    assert_eq!(data[5], b' ');
    assert_eq!(data[6], b'B');
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
