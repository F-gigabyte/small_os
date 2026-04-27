/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Library.
 *
 * The SmallOS Library is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS Library. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![feature(macro_metavar_expr)]
#![test_runner(crate::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::{arch::asm, fmt::{self, Write}, intrinsics::abort, panic::PanicInfo, ptr, slice};

unsafe extern "C" {
    /// Where the kernel stack arguments are stored
    static _stack_args: u32;
}

/// Returns the program arguments passed in on the stack from the kernel
pub fn args() -> &'static [u32] {
    let stack = unsafe {
        *(&raw const _stack_args)
    };
    let stack: *const u32 = ptr::with_exposed_provenance(stack as usize);
    let stack = unsafe {
        stack.sub(1)
    };
    let len = unsafe {
        stack.read()
    };
    let start = unsafe {
        stack.sub(len as usize)
    };
    unsafe {
        slice::from_raw_parts(start, len as usize)
    }
}

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
pub fn test_runner(tests: &[&dyn Fn()]) {
    kprintln!("Running {} tests", tests.len());
    for test in tests {
        test();
    }
}

/// Panic Handler  
/// This function is called when a panic happens
#[panic_handler]
pub fn panic(_info: &PanicInfo) -> ! {
    kprintln!("{}", _info);
    abort()
}

/// MMIO set alias offset from register base
/// Writing a bit sets the corresponding bit in the base registers  
/// Register aliases as specified in SDK <https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21>
/// lines 18 to 21 (accessed 19/01/2026)
/// under the license
/// Copyright (c) 2024 Raspberry Pi Ltd.
///
/// SPDX-License-Identifier: BSD-3-Clause
pub const REG_ALIAS_SET_BITS: usize = 0x2 << 12;
/// MMIO clear alias offset from register base  
/// Writing a bit clears the corresponding bit in the base registers  
/// Register aliases as specified in SDK <https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21>
/// lines 18 to 21 (accessed 19/01/2026)
/// under the license
/// Copyright (c) 2024 Raspberry Pi Ltd.
///
/// SPDX-License-Identifier: BSD-3-Clause
pub const REG_ALIAS_CLR_BITS: usize = 0x3 << 12;

/// Source of which this process was woken by
#[derive(Debug, Clone, Copy)]
pub enum WakeSrc {
    /// Woken up by a queue
    Queue(u32),
    /// Woken up by an IRQ
    IRQ(u32),
    /// Woken up by an unknown source
    Unknown(u32)
}

/// Converts a pair of `u32` to their corresponding `WakeSrc`
impl From<(u32, u32)> for WakeSrc {
    fn from(value: (u32, u32)) -> Self {
        match value.0 {
            0 => Self::Queue(value.1),
            1 => Self::IRQ(value.1),
            _ => Self::Unknown(value.1)
        }
    }
}

/// Errors returned when completing queue operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueError {
    /// Unknown system call
    UnknownSysCall,
    /// Queue is empty
    QueueEmpty,
    /// Provided buffer is too small
    BufferTooSmall,
    /// Queue is full
    QueueFull,
    /// Invalid queue was specified
    InvalidQueue,
    /// Invalid notifier was specified
    InvalidNotifier,
    /// Provided buffer is too large
    BufferTooLarge,
    /// Process died while in queue
    Died,
    /// Invalid memory access performed in queue
    InvalidMemoryAccess,
    /// Sender performed an invalid memory access
    SenderInvalidMemoryAccess,
    /// A queue mask with no queues was provided
    NoQueueMask,
    /// An unknown error happened
    Unknown(u32)
}

/// A queue or an IRQ error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueIRQError {
    /// An IRQ error
    IRQError(IRQError),
    /// A queue error
    QueueError(QueueError)
}

/// Casts from a `u32` to a `QueueError`
impl TryFrom<u32> for QueueError {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Err(()),
            1 => Ok(Self::UnknownSysCall),
            2 => Ok(Self::InvalidQueue),
            3 => Ok(Self::InvalidNotifier),
            4 => Ok(Self::QueueEmpty),
            5 => Ok(Self::QueueFull),
            6 => Ok(Self::BufferTooSmall),
            7 => Ok(Self::BufferTooLarge),
            8 => Ok(Self::Died),
            9 => Ok(Self::InvalidMemoryAccess),
            10 => Ok(Self::SenderInvalidMemoryAccess),
            11 => Ok(Self::NoQueueMask),
            _ => Ok(Self::Unknown(value))
        }
    }
}

/// Casts from a `u32` to a `QueueIRQError`
impl TryFrom<u32> for QueueIRQError {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        if value == 0 {
            Err(())
        } else if value > 2 {
            QueueError::try_from(value - 1).map(|err| Self::QueueError(err))
        } else {
            IRQError::try_from(value).map(|err| Self::IRQError(err))
        }
    }
}

/// A request reply error
#[derive(Debug, PartialEq, Eq)]
pub enum ReplyError {
    /// Error with queue operations
    QueueError(QueueError),
    /// Error with the request
    RequestError(u32)
}

/// Converts from a `QueueError` to a `ReplyError`
impl From<QueueError> for ReplyError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// A yield error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YieldError {
    /// Unknown system call
    UnknownSysCall,
    /// An unknown error happened
    Unknown(u32)
}

/// Casts from a `u32` to a `YieldError`
impl TryFrom<u32> for YieldError {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Err(()),
            1 => Ok(Self::UnknownSysCall),
            _ => Ok(Self::Unknown(value))
        }
    }
}

impl QueueError {
    /// Determines if the `QueueError` was critical
    pub fn critical(&self) -> bool {
        !matches!(self, QueueError::Died | QueueError::SenderInvalidMemoryAccess)
    }
}

/// Determines if the result is a critical queue error and transforms the error to `None` if not  
/// `res` is the result to check
pub fn check_critical<T>(res: Result<T, QueueError>) -> Option<Result<T, QueueError>> {
    match res {
        Ok(res) => Some(Ok(res)),
        Err(err) => {
            if err.critical() {
                Some(Err(err))
            } else {
                None
            }
        }
    }
}

/// An IRQ error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IRQError {
    /// An unknown system call
    UnknownSysCall,
    /// This process has no IRQs
    NoIRQ,
    /// An unknown error happened
    Unknown(u32)
}

/// Casts from a `u32` to an `IRQError`
impl TryFrom<u32> for IRQError {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Err(()),
            1 => Ok(Self::UnknownSysCall),
            2 => Ok(Self::NoIRQ),
            _ => Ok(Self::Unknown(value))
        }
    }
}

/// An asynchronous message header
#[derive(Debug, Clone)]
pub struct AsyncHeader {
    /// PID of sending process
    pub pid: u32,
    /// Sending process' GPIO pin mask
    pub pin_mask: u32,
    /// Sending process' driver
    pub driver: u16,
    /// Message tag
    pub tag: u16,
    /// Message length
    pub message_len: u32
}

/// A synchronous message header
#[derive(Debug, Clone)]
pub struct SyncHeader {
    /// PID of sending process
    pub pid: u32,
    /// Sending process' GPIO pin mask
    pub pin_mask: u32,
    /// Sending process' driver
    pub driver: u16,
    /// Message tag
    pub tag: u16,
    /// Message send length
    pub send_len: u16,
    /// Message reply length
    pub reply_len: u16
}

/// Gives up the rest of this process' time quantum
pub fn do_yield() -> Result<(), YieldError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 0, 
            res = out(reg) res,
        );
    }
    decode_res::<YieldError>(res)
}

/// Kills the current process  
/// `code` is the exit code
pub fn exit(code: u32) {
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            num = in(reg) 1,
            in("r0") code
        );
    }
}

/// Wait on one of the process' IRQs to fire  
/// Returns the IRQ index on success or an `IRQError` on failure
pub fn wait_irq() -> Result<u32, IRQError> {
    let mut res: u32;
    let mut irq: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 2,
            res = out(reg) res,
            out("r0") irq 
        );
    }
    decode_res::<IRQError>(res).map(|_| irq)
}

/// Clears the process' IRQ state masks
pub fn clear_irq() -> Result<(), IRQError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 3,
            res = out(reg) res,
        );
    }
    decode_res::<IRQError>(res)
}

/// Converts a result integer into its corresponding error  
/// `res` is the result to convert
fn decode_res<E: TryFrom<u32, Error = ()>>(res: u32) -> Result<(), E> {
    match E::try_from(res) {
        Ok(err) => Err(err),
        Err(()) => Ok(())
    }
}

/// Sends a message to another process  
/// `target` is the endpoint to use  
/// `tag` is the message tag to send  
/// `data` is the message data to send and where the received data will be placed  
/// `send_len` is the length of the message to send  
/// `reply_len` is the maximum length of the reply  
/// On success returns the number of bytes written into `data` or else returns the corresponding
/// error
pub fn send(target: u32, tag: u16, data: &mut [u8], send_len: usize, reply_len: usize) -> Result<usize, ReplyError> {
    let mut res: u32;
    let mut reply: u32;
    let mut len: usize;
    let tag = tag as u32;
    unsafe {
        if send_len > data.len() || reply_len > data.len() {
            return Err(ReplyError::QueueError(QueueError::BufferTooSmall));
        } else if send_len > u16::MAX as usize || reply_len > u16::MAX as usize {
            return Err(ReplyError::QueueError(QueueError::BufferTooLarge));
        }
        let r2 = (send_len & 0xffff) | ((reply_len & 0xffff) << 16);
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 4,
            res = out(reg) res,
            inout("r0") target => reply,
            inout("r1") tag => len,
            in("r2") r2,
            in("r3") data.as_ptr()
        );
    }
    decode_res::<QueueError>(res)?;
    check_reply_zero((reply, len)).map_err(|err| ReplyError::RequestError(err))
}

/// Sends a message to another process with no reply  
/// `target` is the endpoint to use  
/// `tag` is the message tag to send  
/// `data` is the message data to send  
/// On success returns 0 or else returns the corresponding error
pub fn send_empty(target: u32, tag: u16, data: &[u8]) -> Result<usize, ReplyError> {
    let mut res: u32;
    let mut reply: u32;
    let mut len: usize;
    let tag = tag as u32;
    unsafe {
        if data.len() > u16::MAX as usize {
            return Err(ReplyError::QueueError(QueueError::BufferTooLarge));
        }
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 4,
            res = out(reg) res,
            inout("r0") target => reply,
            inout("r1") tag => len,
            in("r2") data.len(),
            in("r3") data.as_ptr()
        );
    }
    decode_res::<QueueError>(res)?;
    check_reply_zero((reply, len)).map_err(|err| ReplyError::RequestError(err))
}

/// Sends an asynchronous message to another process   
/// `target` is the asynchronous endpoint to use  
/// `tag` is the message tag to send  
/// `data` is the message data to send  
pub fn send_async(target: u32, tag: u16, data: &[u8]) -> Result<(), QueueError> {
    let mut res: u32;
    let tag = tag as u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 5,
            res = out(reg) res,
            in("r0") target,
            in("r1") tag,
            in("r2") data.len(),
            in("r3") data.as_ptr()
        );
    }
    decode_res::<QueueError>(res)
}

/// Sends a process in one of this process' synchronous queues into one of its notifier queues  
/// `queue` is the synchronous queue to send the process from  
/// `notifier` is the notifier queue to send the process to
pub fn notify_send(queue: u32, notifier: u32) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 6,
            res = out(reg) res,
            in("r0") queue,
            in("r1") notifier,
        );
    }
    decode_res::<QueueError>(res)
}

/// Waits on queues in a queue mask to receive data  
/// `queue_mask` is a mask of all the queues to wait on  
/// Returns the index of one of the queues containing data on success or a `QueueError` on error
pub fn wait_queues(queue_mask: u32) -> Result<u32, QueueError> {
    let mut res: u32;
    let mut queue: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 7,
            res = out(reg) res,
            inout("r0") queue_mask => queue,
        );
    }
    decode_res::<QueueError>(res).map(|_| queue)
}

/// Waits on asynchronous queues in a queue mask to receive data  
/// `queue_mask` is a mask of all the asynchronous queues to wait on  
/// Returns the index of one of the queues containing data on success or a `QueueError` on error
pub fn wait_queues_async(queue_mask: u32) -> Result<u32, QueueError> {
    let mut res: u32;
    let mut queue: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 8,
            res = out(reg) res,
            inout("r0") queue_mask => queue,
        );
    }
    decode_res::<QueueError>(res).map(|_| queue)
}

/// Waits on queues in a queue mask to receive data or for an IRQ event  
/// `queue_mask` is a mask of all the queues to wait on  
/// On success returns the source of what woke this process or returns a `QueueIRQError` on failure
pub fn wait_queues_irq(queue_mask: u32) -> Result<WakeSrc, QueueIRQError> {
    let mut res: u32;
    let mut queue_irq: u32;
    let mut src: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 9,
            res = out(reg) res,
            inout("r0") queue_mask => queue_irq,
            out("r1") src
        );
    }
    decode_res::<QueueIRQError>(res).map(|_| WakeSrc::from((queue_irq, src)))
}

/// Waits on asynchronous queues in a queue mask to receive data or for an IRQ event  
/// `queue_mask` is a mask of all the asynchronous queues to wait on  
/// On success returns the source of what woke this process or returns a `QueueIRQError` on failure
pub fn wait_queues_irq_async(queue_mask: u32) -> Result<WakeSrc, QueueIRQError> {
    let mut res: u32;
    let mut queue_irq: u32;
    let mut src: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 10,
            res = out(reg) res,
            inout("r0") queue_mask => queue_irq,
            out("r1") src
        );
    }
    decode_res::<QueueIRQError>(res).map(|_| WakeSrc::from((queue_irq, src)))
}

/// Checks if the queue's reply was 0 and returns the number of bytes sent  
/// `reply` is a pair of the queue's reply and the number of bytes written
fn check_reply_zero(reply: (u32, usize)) -> Result<usize, u32> {
    if reply.0 == 0 {
        Ok(reply.1)
    } else {
        Err(reply.0)
    }
}

/// Reads the header of the next process in the queue
/// `queue` is the queue to read the header from  
/// On success returns the header or a `QueueError` on failure
pub fn read_header(queue: u32) -> Result<SyncHeader, QueueError> {
    let mut res: u32;
    let mut pid: u32;
    let mut pin_mask: u32;
    let mut driver_tag: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 11,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") pin_mask,
            out("r2") driver_tag,
            out("r3") len,
        );
    }
    let send_len = (len & 0xffff) as u16;
    let reply_len = ((len >> 16) & 0xffff) as u16;
    decode_res::<QueueError>(res).map(|_| 
        SyncHeader { 
            pid, 
            pin_mask,
            driver: (driver_tag >> 16) as u16, 
            tag: (driver_tag & 0xffff) as u16, 
            send_len, 
            reply_len 
        }
    )
}

/// Reads the header of the next process in the queue and repeats until a header is read or a
/// critical error happens
/// `queue` is the queue to read the header from  
/// `invalid_response` is the response to send on errors
/// On success returns the header or a `QueueError` on failure
pub fn next_valid_header(queue: u32, invalid_response: u32) -> Result<SyncHeader, QueueError> {
    loop {
        let header = read_header(queue);
        match header {
            Ok(header) => return Ok(header),
            Err(err) => {
                // died means sender died, sender invalid memory access means the sender made an
                // invalid memory access and these are issues with the sender not the replier
                match err {
                    QueueError::Died => {},
                    QueueError::SenderInvalidMemoryAccess => {
                        match reply_empty(queue, invalid_response) {
                            Ok(_) => Ok(()),
                            Err(err) => {
                                if err.critical() {
                                    Err(err)
                                } else {
                                    Ok(())
                                }
                            }
                        }?;
                    },
                    err => {
                        return Err(err);
                    }
                }
            }
        }
    }
}

/// Reads the message data of the next process in the queue into the provided buffer and repeats
/// until a message's data is read or a critical error happens  
/// `queue` is the queue to read the data from  
/// `buffer` is the buffer to write the data to  
/// `invalid_response` is the response to send on errors  
/// `too_large_response` is the response to send if the message data is too large
/// On success returns the number of bytes read or a `QueueError` on failure
pub fn next_message(queue: u32, buffer: &mut[u8], invalid_response: u32, too_large_response: u32) -> Result<usize, QueueError> {
    loop {
        let res = receive(queue, buffer);
        match res {
            Ok(res) => return Ok(res),
            Err(err) => {
                if matches!(err, QueueError::Died | QueueError::SenderInvalidMemoryAccess) {
                    reply_empty(queue, invalid_response)?;
                } else if err == QueueError::BufferTooSmall {
                    reply_empty(queue, too_large_response)?;
                } else {
                    return Err(err);
                }
            }
        }
    }
}

/// Reads the header of the message in the asynchronous queue
/// `queue` is the asynchronous queue to read the header from  
/// On success returns the header or a `QueueError` on failure
pub fn read_header_async(queue: u32) -> Result<AsyncHeader, QueueError> {
    let mut res: u32;
    let mut pid: u32;
    let mut pin_mask: u32;
    let mut driver_tag: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 12,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") pin_mask,
            out("r2") driver_tag,
            out("r3") len,
        );
    }
    decode_res::<QueueError>(res).map(|_| 
        AsyncHeader { 
            pid, 
            pin_mask,
            driver: (driver_tag >> 16) as u16, 
            tag: (driver_tag & 0xffff) as u16, 
            message_len: len 
        }
    )
}

/// Reads the header of the next process in the queue without blocking this process
/// `queue` is the queue to read the header from  
/// On success returns the header or a `QueueError` on failure
pub fn read_header_non_blocking(queue: u32) -> Result<SyncHeader, QueueError> {
    let mut res: u32;
    let mut pid: u32;
    let mut pin_mask: u32;
    let mut driver_tag: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 13,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") pin_mask,
            out("r2") driver_tag,
            out("r3") len,
        );
    }
    let send_len = (len & 0xffff) as u16;
    let reply_len = ((len >> 16) & 0xffff) as u16;
    decode_res::<QueueError>(res).map(|_| 
        SyncHeader { 
            pid, 
            pin_mask,
            driver: (driver_tag >> 16) as u16, 
            tag: (driver_tag & 0xffff) as u16, 
            send_len, 
            reply_len 
        }
    )
}

/// Reads the header of the message in the asynchronous queue without blocking this process
/// `queue` is the asynchronous queue to read the header from  
/// On success returns the header or a `QueueError` on failure
pub fn read_header_async_non_blocking(queue: u32) -> Result<AsyncHeader, QueueError> {
    let mut res: u32;
    let mut pid: u32;
    let mut pin_mask: u32;
    let mut driver_tag: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 14,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") pin_mask,
            out("r2") driver_tag,
            out("r3") len,
        );
    }
    decode_res::<QueueError>(res).map(|_| 
        AsyncHeader { 
            pid,
            pin_mask,
            driver: (driver_tag >> 16) as u16, 
            tag: (driver_tag & 0xffff) as u16, 
            message_len: len 
        }
    )
}

/// Reads the header of the next process in the notifier queue
/// `queue` is the notifier queue to read the header from  
/// On success returns the header or a `QueueError` on failure
pub fn notify_read_header(queue: u32) -> Result<SyncHeader, QueueError> {
    let mut res: u32;
    let mut pid: u32;
    let mut pin_mask: u32;
    let mut driver_tag: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 15,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") pin_mask,
            out("r2") driver_tag,
            out("r3") len,
        );
    }
    let send_len = (len & 0xffff) as u16;
    let reply_len = ((len >> 16) & 0xffff) as u16;
    decode_res::<QueueError>(res).map(|_| 
        SyncHeader { 
            pid, 
            pin_mask,
            driver: (driver_tag >> 16) as u16, 
            tag: (driver_tag & 0xffff) as u16, 
            send_len, 
            reply_len 
        }
    )
}

/// Receives message data from the process at the front of the queue into the buffer  
/// `queue` is the queue being read from  
/// `buffer` is the buffer being written into  
/// Returns the number of bytes read on success or a `QueueError` on failure
pub fn receive(queue: u32, buffer: &mut[u8]) -> Result<usize, QueueError> {
    let mut res: u32;
    let mut read: usize;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 16,
            res = out(reg) res,
            inout("r0") queue => read,
            in("r1") buffer.len(),
            in("r2") buffer.as_ptr()
        );
    }
    decode_res::<QueueError>(res).map(|_| read)
}

/// Receives message data from the message at the front of the asynchronous queue into the buffer  
/// `queue` is the asynchronous queue being read from  
/// `buffer` is the buffer being written into  
/// Returns the number of bytes read on success or a `QueueError` on failure
pub fn receive_async(queue: u32, buffer: &mut[u8]) -> Result<usize, QueueError> {
    let mut res: u32;
    let mut read: usize;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 17,
            res = out(reg) res,
            inout("r0") queue => read,
            in("r1") buffer.len(),
            in("r2") buffer.as_ptr()
        );
    }
    decode_res::<QueueError>(res).map(|_| read)
}

/// Receives message data from the process at the front of the notifier queue into the buffer  
/// `queue` is the notifier queue being read from  
/// `buffer` is the buffer being written into  
/// Returns the number of bytes read on success or a `QueueError` on failure
pub fn notify_receive(notifier: u32, buffer: &mut[u8]) -> Result<usize, QueueError> {
    let mut res: u32;
    let mut read: usize;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 18,
            res = out(reg) res,
            inout("r0") notifier => read,
            in("r1") buffer.len(),
            in("r2") buffer.as_ptr()
        );
    }
    decode_res::<QueueError>(res).map(|_| read)
}

/// Sends a reply to the process at the front of the queue  
/// `queue` is the queue to reply to  
/// `reply` is the tag of the reply message  
/// `buffer` is the content of the reply message
pub fn reply(queue: u32, reply: u32, buffer: &[u8]) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 19,
            res = out(reg) res,
            in("r0") queue,
            in("r1") reply,
            in("r2") buffer.len(),
            in("r3") buffer.as_ptr()
        );
    }
    decode_res::<QueueError>(res)
}

/// Sends a reply to the process at the front of the queue with no content  
/// `queue` is the queue to reply to  
/// `reply` is the tag of the reply message  
pub fn reply_empty(queue: u32, reply: u32) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 19,
            res = out(reg) res,
            in("r0") queue,
            in("r1") reply,
            in("r2") 0,
        );
    }
    decode_res::<QueueError>(res)
}

/// Sends a reply to the process at the front of the notifier queue  
/// `queue` is the notifier queue to reply to  
/// `reply` is the tag of the reply message  
/// `buffer` is the content of the reply message
pub fn notify_reply(notifier: u32, reply: u32, buffer: &[u8]) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 20,
            res = out(reg) res,
            in("r0") notifier,
            in("r1") reply,
            in("r2") buffer.len(),
            in("r3") buffer.as_ptr()
        );
    }
    decode_res::<QueueError>(res)
}

/// Sends a reply to the process at the front of the notifier queue with no content  
/// `queue` is the notifier queue to reply to  
/// `reply` is the tag of the reply message  
pub fn notify_reply_empty(notifier: u32, reply: u32) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 20,
            res = out(reg) res,
            in("r0") notifier,
            in("r1") reply,
            in("r2") 0,
        );
    }
    decode_res::<QueueError>(res)
}

/// Kernel print system call errors
#[derive(Debug, Clone, Copy)]
pub enum KPrintError {
    /// Unknown system call
    UnknownSysCall,
    /// Invalid memory access
    InvalidAccess,
    /// Data is not a UTF8 string
    NotUTF8,
    /// An unknown error happened
    Unknown(u32)
}

/// Casts from a `u32` to a `KPrintError`
impl TryFrom<u32> for KPrintError {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Err(()),
            1 => Ok(Self::UnknownSysCall),
            2 => Ok(Self::InvalidAccess),
            3 => Ok(Self::NotUTF8),
            _ => Ok(Self::Unknown(value))
        }
    }
}

/// Gets the kernel to print a message  
/// `text` is the message to print  
/// Returns the number of bytes printed on sucess or a `KPrintError` on failure
pub fn do_kprint(text: &str) -> Result<usize, KPrintError> {
    let data = text.as_bytes();
    let mut res: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 21,
            res = out(reg) res,
            inout("r0") data.len() => len,
            in("r1") data.as_ptr()
        );
    }
    decode_res::<KPrintError>(res).map(|_| len as usize)
}

/// Kernel print object for handling kernel printing
struct KPrint {}

// Based off https://os.phil-opp.com/vga-text-mode/ accessed 22/01/2026
impl Write for KPrint {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        do_kprint(s).map(|_| ()).map_err(|_| fmt::Error)
    }
}

/// Kernel print macro  
/// Based off <https://os.phil-opp.com/vga-text-mode/> accessed 22/01/2026
#[macro_export]
macro_rules! kprint {
    ($($arg:tt)*) => ($crate::_kprint(format_args!($($arg)*)));
}

/// Kernel print line macro  
/// Based off <https://os.phil-opp.com/vga-text-mode/> accessed 22/01/2026
#[macro_export]
macro_rules! kprintln {
    () => ($crate::kprint!("\r\n"));
    ($($arg:tt)*) => ($crate::kprint!("{}\r\n", format_args!($($arg)*)));
}

/// Kernel print function  
/// `args` is a list of formatting arguments which dictate what's printed  
/// Based off <https://os.phil-opp.com/vga-text-mode/> accessed 22/01/2026
#[doc(hidden)]
pub fn _kprint(args: fmt::Arguments) {
    let mut print = KPrint {};
    let res = print.write_fmt(args);
    res.unwrap();
}

/// Queue header errors
pub enum HeaderError {
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Checks the header has the correct length  
/// `header` is the header to check  
/// `send_len` is the send length the header should have  
/// `reply_len` is the reply length the header should have
pub fn check_header_len(header: &SyncHeader, send_len: u16, reply_len: u16) -> Result<(), HeaderError> {
    if header.send_len != send_len {
        return Err(HeaderError::InvalidSendBuffer);
    }
    if header.reply_len != reply_len {
        return Err(HeaderError::InvalidReplyBuffer);
    }
    Ok(())
}
