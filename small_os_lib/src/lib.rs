// use core intrinsics 
#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{arch::asm, intrinsics::abort, panic::PanicInfo};

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
pub fn panic(_info: &PanicInfo) -> ! {
    abort()
}

// offset for clearing registers as specified in SDK https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21
// lines 18 to 21 (accessed 19/01/2026)
pub const REG_ALIAS_SET_BITS: usize = 0x2 << 12;
pub const REG_ALIAS_CLR_BITS: usize = 0x3 << 12;

#[derive(Debug, Clone, Copy)]
pub enum WakeSrc {
    Queue(u32),
    IRQ(u32),
    Unknown(u32)
}

impl From<(u32, u32)> for WakeSrc {
    fn from(value: (u32, u32)) -> Self {
        match value.0 {
            0 => Self::Queue(value.1),
            1 => Self::IRQ(value.1),
            _ => Self::Unknown(value.1)
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueError {
    UnknownSysCall,
    QueueEmpty,
    BufferTooSmall,
    QueueFull,
    InvalidQueue,
    InvalidNotifier,
    BufferTooLarge,
    Died,
    InvalidMemoryAccess,
    SenderInvalidMemoryAccess,
    NoQueueMask,
    Unknown(u32)
}

impl TryFrom<u32> for QueueError {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Err(()),
            1 => Ok(QueueError::UnknownSysCall),
            2 => Ok(QueueError::InvalidQueue),
            3 => Ok(QueueError::InvalidNotifier),
            4 => Ok(QueueError::QueueEmpty),
            5 => Ok(QueueError::QueueFull),
            6 => Ok(QueueError::BufferTooSmall),
            7 => Ok(QueueError::BufferTooLarge),
            8 => Ok(QueueError::Died),
            9 => Ok(QueueError::InvalidMemoryAccess),
            10 => Ok(QueueError::SenderInvalidMemoryAccess),
            11 => Ok(QueueError::NoQueueMask),
            _ => Ok(QueueError::Unknown(value))
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum ReplyError {
    QueueError(QueueError),
    RequestError(u32)
}

impl From<QueueError> for ReplyError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YieldError {
    Unknown(u32)
}

impl QueueError {
    pub fn critical(&self) -> bool {
        !matches!(self, QueueError::Died | QueueError::SenderInvalidMemoryAccess)
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
    UnknownSysCall,
    NoIRQ,
    Unknown(u32)
}

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

#[derive(Debug, Clone)]
pub struct AsyncHeader {
    pub pid: u32,
    pub pin_mask: u32,
    pub driver: u16,
    pub tag: u16,
    pub message_len: u32
}

#[derive(Debug, Clone)]
pub struct SyncHeader {
    pub pid: u32,
    pub pin_mask: u32,
    pub driver: u16,
    pub tag: u16,
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
    decode_irq_res(res).map(|_| irq)
}

pub fn clear_irq() -> Result<(), IRQError> {
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
    decode_irq_res(res)
}

fn decode_queue_res(res: u32) -> Result<(), QueueError> {
    match QueueError::try_from(res) {
        Ok(err) => Err(err),
        Err(()) => Ok(())
    }
}

fn decode_irq_res(res: u32) -> Result<(), IRQError> {
    match IRQError::try_from(res) {
        Ok(err) => Err(err),
        Err(()) => Ok(())
    }
}

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
            num = in(reg) 3,
            res = out(reg) res,
            inout("r0") target => reply,
            inout("r1") tag => len,
            in("r2") r2,
            in("r3") data.as_ptr()
        );
    }
    decode_queue_res(res)?;
    check_reply_zero((reply, len)).map_err(|err| ReplyError::RequestError(err))
}

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
            num = in(reg) 3,
            res = out(reg) res,
            inout("r0") target => reply,
            inout("r1") tag => len,
            in("r2") data.len(),
            in("r3") data.as_ptr()
        );
    }
    decode_queue_res(res)?;
    check_reply_zero((reply, len)).map_err(|err| ReplyError::RequestError(err))
}

pub fn send_async(target: u32, tag: u16, data: &[u8]) -> Result<(), QueueError> {
    let mut res: u32;
    let tag = tag as u32;
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

pub fn notify_send(queue: u32, notifier: u32) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 5,
            res = out(reg) res,
            in("r0") queue,
            in("r1") notifier,
        );
    }
    decode_queue_res(res)
}

pub fn wait_queues(queue_mask: u32) -> Result<u32, QueueError> {
    let mut res: u32;
    let mut queue: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 6,
            res = out(reg) res,
            inout("r0") queue_mask => queue,
        );
    }
    decode_queue_res(res).map(|_| queue)
}

pub fn wait_queues_async(queue_mask: u32) -> Result<u32, QueueError> {
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
    decode_queue_res(res).map(|_| queue)
}

pub fn wait_queues_irq(queue_mask: u32) -> Result<WakeSrc, QueueError> {
    let mut res: u32;
    let mut queue_irq: u32;
    let mut src: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 8,
            res = out(reg) res,
            inout("r0") queue_mask => queue_irq,
            out("r1") src
        );
    }
    decode_queue_res(res).map(|_| WakeSrc::from((queue_irq, src)))
}

pub fn wait_queues_irq_async(queue_mask: u32) -> Result<WakeSrc, QueueError> {
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
    decode_queue_res(res).map(|_| WakeSrc::from((queue_irq, src)))
}

fn check_reply_zero(reply: (u32, usize)) -> Result<usize, u32> {
    if reply.0 == 0 {
        Ok(reply.1)
    } else {
        Err(reply.0)
    }
}

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
            num = in(reg) 10,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") pin_mask,
            out("r2") driver_tag,
            out("r3") len,
        );
    }
    let send_len = (len & 0xffff) as u16;
    let reply_len = ((len >> 16) & 0xffff) as u16;
    decode_queue_res(res).map(|_| 
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
    decode_queue_res(res).map(|_| 
        AsyncHeader { 
            pid, 
            pin_mask,
            driver: (driver_tag >> 16) as u16, 
            tag: (driver_tag & 0xffff) as u16, 
            message_len: len 
        }
    )
}

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
            num = in(reg) 12,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") pin_mask,
            out("r2") driver_tag,
            out("r3") len,
        );
    }
    let send_len = (len & 0xffff) as u16;
    let reply_len = ((len >> 16) & 0xffff) as u16;
    decode_queue_res(res).map(|_| 
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
            num = in(reg) 13,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") pin_mask,
            out("r2") driver_tag,
            out("r3") len,
        );
    }
    decode_queue_res(res).map(|_| 
        AsyncHeader { 
            pid,
            pin_mask,
            driver: (driver_tag >> 16) as u16, 
            tag: (driver_tag & 0xffff) as u16, 
            message_len: len 
        }
    )
}

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
            num = in(reg) 14,
            res = out(reg) res,
            inout("r0") queue => pid,
            out("r1") pin_mask,
            out("r2") driver_tag,
            out("r3") len,
        );
    }
    let send_len = (len & 0xffff) as u16;
    let reply_len = ((len >> 16) & 0xffff) as u16;
    decode_queue_res(res).map(|_| 
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

pub fn receive(queue: u32, buffer: &mut[u8]) -> Result<usize, QueueError> {
    let mut res: u32;
    let mut read: usize;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 15,
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
            num = in(reg) 16,
            res = out(reg) res,
            inout("r0") queue => read,
            in("r1") buffer.len(),
            in("r2") buffer.as_ptr()
        );
    }
    decode_queue_res(res).map(|_| read)
}

pub fn notify_receive(notifier: u32, buffer: &mut[u8]) -> Result<usize, QueueError> {
    let mut res: u32;
    let mut read: usize;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 17,
            res = out(reg) res,
            inout("r0") notifier => read,
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
            num = in(reg) 18,
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
            num = in(reg) 18,
            res = out(reg) res,
            in("r0") queue,
            in("r1") reply,
            in("r2") 0,
        );
    }
    decode_queue_res(res)
}

pub fn notify_reply(notifier: u32, reply: u32, buffer: &[u8]) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 19,
            res = out(reg) res,
            in("r0") notifier,
            in("r1") reply,
            in("r2") buffer.len(),
            in("r3") buffer.as_ptr()
        );
    }
    decode_queue_res(res)
}

pub fn notify_reply_empty(notifier: u32, reply: u32) -> Result<(), QueueError> {
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) 19,
            res = out(reg) res,
            in("r0") notifier,
            in("r1") reply,
            in("r2") 0,
        );
    }
    decode_queue_res(res)
}

pub enum HeaderError {
    InvalidSendBuffer,
    InvalidReplyBuffer
}

pub fn check_header_len(header: &SyncHeader, send_len: u16, reply_len: u16) -> Result<(), HeaderError> {
    if header.send_len != send_len {
        return Err(HeaderError::InvalidSendBuffer);
    }
    if header.reply_len != reply_len {
        return Err(HeaderError::InvalidReplyBuffer);
    }
    Ok(())
}
