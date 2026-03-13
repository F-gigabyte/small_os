// use core intrinsics 
#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{arch::asm};

// offset for clearing registers as specified in SDK https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21
// lines 18 to 21 (accessed 19/01/2026)
pub const REG_ALIAS_SET_BITS: usize = 0x2 << 12;
pub const REG_ALIAS_CLR_BITS: usize = 0x3 << 12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueError {
    QueueEmpty,
    BufferTooSmall,
    QueueFull,
    InvalidQueue,
    BufferTooLarge,
    Died,
    InvalidMemoryAccess,
    SenderInvalidMemoryAccess,
    Unknown(u32)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YieldError {
    Unknown(u32)
}

impl QueueError {
    pub fn critical(&self) -> bool {
        !matches!(self, QueueError::Died)
    }
}

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IRQError {
    NoIRQ,
    Unknown(u32)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KPutError {
    NotUTF8,
    Unknown(u32),
}

#[derive(Debug, Clone)]
pub struct AsyncHeader {
    pub pid: u32,
    pub tag: u32,
    pub message_len: u32
}

#[derive(Debug, Clone)]
pub struct SyncHeader {
    pub pid: u32,
    pub tag: u32,
    pub send_len: u16,
    pub reply_len: u16
}

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
    match res {
        0 => Ok(()),
        _ => Err(YieldError::Unknown(res))
    }
}

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

pub fn wait_irq() -> Result<(), IRQError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 2,
            res = out(reg) res,
        );
    }
    match res {
        0 => Ok(()),
        2 => Err(IRQError::NoIRQ),
        _ => Err(IRQError::Unknown(res))
    }
}

fn decode_queue_res(res: u32) -> Result<(), QueueError> {
    match res {
        0 => Ok(()),
        2 => Err(QueueError::InvalidQueue),
        3 => Err(QueueError::QueueEmpty),
        4 => Err(QueueError::QueueFull),
        5 => Err(QueueError::BufferTooSmall),
        6 => Err(QueueError::BufferTooLarge),
        7 => Err(QueueError::Died),
        8 => Err(QueueError::InvalidMemoryAccess),
        9 => Err(QueueError::SenderInvalidMemoryAccess),
        _ => Err(QueueError::Unknown(res))
    }
}

pub fn send(target: u32, tag: u32, data: &mut [u8], send_len: usize, reply_len: usize) -> Result<(u32, usize), QueueError> {
    let mut res: u32;
    let mut reply: u32;
    let mut len: usize;
    unsafe {
        if send_len > data.len() || reply_len > data.len() {
            return Err(QueueError::BufferTooSmall);
        } else if send_len > u16::MAX as usize || reply_len > u16::MAX as usize {
            return Err(QueueError::BufferTooLarge);
        }
        let r2 = (send_len & 0xffff) | ((reply_len & 0xffff) << 16);
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 3,
            res = out(reg) res,
            inout("r0") target => reply,
            inout("r1") tag => len,
            in("r2") r2,
            in("r3") data.as_ptr()
        );
    }
    decode_queue_res(res).map(|_| (reply, len))
}

pub fn send_empty(target: u32, tag: u32, data: &[u8]) -> Result<(u32, usize), QueueError> {
    let mut res: u32;
    let mut reply: u32;
    let mut len: usize;
    unsafe {
        if data.len() > u16::MAX as usize {
            return Err(QueueError::BufferTooLarge);
        }
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 3,
            res = out(reg) res,
            inout("r0") target => reply,
            inout("r1") tag => len,
            in("r2") data.len(),
            in("r3") data.as_ptr()
        );
    }
    decode_queue_res(res).map(|_| (reply, len))
}

pub fn send_async(target: u32, tag: u32, data: &[u8]) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 4,
            res = out(reg) res,
            in("r0") target,
            in("r1") tag,
            in("r2") data.len(),
            in("r3") data.as_ptr()
        );
    }
    decode_queue_res(res)
}

pub fn read_header(queue: u32) -> Result<SyncHeader, QueueError> {
    let mut res: u32;
    let mut pid: u32;
    let mut tag: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 5,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") tag,
            out("r2") len,
        );
    }
    let send_len = (len & 0xffff) as u16;
    let reply_len = ((len >> 16) & 0xffff) as u16;
    decode_queue_res(res).map(|_| SyncHeader { pid, tag, send_len, reply_len })
}

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

pub fn read_header_async(queue: u32) -> Result<AsyncHeader, QueueError> {
    let mut res: u32;
    let mut pid: u32;
    let mut tag: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 6,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") tag,
            out("r2") len,
        );
    }
    decode_queue_res(res).map(|_| AsyncHeader { pid, tag, message_len: len })
}

pub fn read_header_non_blocking(queue: u32) -> Result<SyncHeader, QueueError> {
    let mut res: u32;
    let mut pid: u32;
    let mut tag: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 7,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") tag,
            out("r2") len,
        );
    }
    let send_len = (len & 0xffff) as u16;
    let reply_len = ((len >> 16) & 0xffff) as u16;
    decode_queue_res(res).map(|_| SyncHeader { pid, tag, send_len, reply_len })
}

pub fn read_header_async_non_blocking(queue: u32) -> Result<AsyncHeader, QueueError> {
    let mut res: u32;
    let mut pid: u32;
    let mut tag: u32;
    let mut len: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 8,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") tag,
            out("r2") len,
        );
    }
    decode_queue_res(res).map(|_| AsyncHeader { pid, tag, message_len: len })
}

pub fn receive(queue: u32, buffer: &mut[u8]) -> Result<usize, QueueError> {
    let mut res: u32;
    let mut read: usize;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 9,
            res = out(reg) res,
            inout("r0") queue => read,
            in("r1") buffer.len(),
            in("r2") buffer.as_ptr()
        );
    }
    decode_queue_res(res).map(|_| read)
}

pub fn receive_async(queue: u32, buffer: &mut[u8]) -> Result<usize, QueueError> {
    let mut res: u32;
    let mut read: usize;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 10,
            res = out(reg) res,
            inout("r0") queue => read,
            in("r1") buffer.len(),
            in("r2") buffer.as_ptr()
        );
    }
    decode_queue_res(res).map(|_| read)
}

pub fn reply(queue: u32, reply: u32, buffer: &[u8]) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 11,
            res = out(reg) res,
            in("r0") queue,
            in("r1") reply,
            in("r2") buffer.len(),
            in("r3") buffer.as_ptr()
        );
    }
    decode_queue_res(res)
}

pub fn reply_empty(queue: u32, reply: u32) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 11,
            res = out(reg) res,
            in("r0") queue,
            in("r1") reply,
            in("r2") 0,
        );
    }
    decode_queue_res(res)
}
