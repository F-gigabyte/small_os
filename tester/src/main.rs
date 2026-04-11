// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::{arch::asm, assert_matches};

use small_os_lib::{IRQError, QueueError, QueueIRQError, ReplyError, args, clear_irq, do_kprint, do_yield, kprintln, notify_read_header, notify_receive, notify_reply, notify_send, read_header, read_header_async, read_header_async_non_blocking, read_header_non_blocking, receive, receive_async, reply, send, send_async, wait_irq, wait_queues, wait_queues_async, wait_queues_irq, wait_queues_irq_async};

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 0);
    let mut res: u32;
    // yield
    do_yield().unwrap();
    // wait IRQ
    assert_matches!(wait_irq(), Err(IRQError::NoIRQ));
    // clear IRQ
    assert_matches!(clear_irq(), Err(IRQError::NoIRQ));
    // send
    assert_matches!(send(0, 0, &mut [], 0, 0), Err(ReplyError::QueueError(QueueError::InvalidQueue)));
    // send async
    assert_matches!(send_async(0, 0, &[]), Err(QueueError::InvalidQueue));
    // notify send
    assert_matches!(notify_send(0, 0), Err(QueueError::InvalidQueue));
    // wait queues
    assert_matches!(wait_queues(1), Err(QueueError::InvalidQueue));
    // wait queues
    assert_matches!(wait_queues(0), Err(QueueError::NoQueueMask));
    // wait queues async
    assert_matches!(wait_queues_async(1), Err(QueueError::InvalidQueue));
    // wait queues async
    assert_matches!(wait_queues_async(0), Err(QueueError::NoQueueMask));
    // wait queues irq
    assert_matches!(wait_queues_irq(0), Err(QueueIRQError::QueueError(QueueError::NoQueueMask)));
    // wait queues irq
    assert_matches!(wait_queues_irq(1), Err(QueueIRQError::QueueError(QueueError::InvalidQueue)));
    // wait queues irq async
    assert_matches!(wait_queues_irq_async(0), Err(QueueIRQError::QueueError(QueueError::NoQueueMask)));
    // wait queues irq async
    assert_matches!(wait_queues_irq_async(1), Err(QueueIRQError::QueueError(QueueError::InvalidQueue)));
    // header
    assert_matches!(read_header(0), Err(QueueError::InvalidQueue));
    // header async
    assert_matches!(read_header_async(0), Err(QueueError::InvalidQueue));
    // header non blocking
    assert_matches!(read_header_non_blocking(0), Err(QueueError::InvalidQueue));
    // header async non blocking
    assert_matches!(read_header_async_non_blocking(0), Err(QueueError::InvalidQueue));
    // notify header
    assert_matches!(notify_read_header(0), Err(QueueError::InvalidQueue));
    // receive
    assert_matches!(receive(0, &mut []), Err(QueueError::InvalidQueue));
    // receive async
    assert_matches!(receive_async(0, &mut []), Err(QueueError::InvalidQueue));
    // notify receive
    assert_matches!(notify_receive(0, &mut []), Err(QueueError::InvalidQueue));
    // reply
    assert_matches!(reply(0, 0, &[]), Err(QueueError::InvalidQueue));
    // notify reply
    assert_matches!(notify_reply(0, 0, &[]), Err(QueueError::InvalidQueue));
    let test_message = "This is the tester";
    let len = test_message.len();
    // kprint
    assert_eq!(do_kprint(test_message).unwrap(), len);
    let nums: [u8; _] = [0xff, 0xff, 0xff, 0xff];
    // kprint
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 21,
            res = out(reg) res,
            inout("r0") nums.len() => _,
            in("r1") nums.as_ptr()
        );
    };
    // not UTF8
    assert_eq!(res, 3);
    // kprint
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 21,
            res = out(reg) res,
            inout("r0") 1 => _,
            in("r1") 0
        );
    };
    // invalid access
    assert_eq!(res, 2);
    loop {}
}

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::kprintln;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for tester", tests.len());
        for test in tests {
            test();
        }
    }
}
