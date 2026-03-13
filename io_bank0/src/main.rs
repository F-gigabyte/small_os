#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{intrinsics::abort, panic::PanicInfo, ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};
use small_os_lib::{QueueError, check_critical, read_header, receive, reply_empty, send};

mod gpio_ctrl_register {
    pub const FUNCSEL_SHIFT: usize = 0;
    pub const OUTOVER_SHIFT: usize = 8;
    pub const OEOVER_SHIFT: usize = 12;
    pub const INOVER_SHIFT: usize = 16;
    pub const IRQOVER_SHIFT: usize = 28;

    pub const FUNCSEL_MASK: u32 = 0x1f << FUNCSEL_SHIFT;
    pub const OUTOVER_MASK: u32 = 0x2 << OUTOVER_SHIFT;
    pub const OEOVER_MASK: u32 = 0x2 << OEOVER_SHIFT;
    pub const INOVER_MASK: u32 = 0x2 << INOVER_SHIFT;
    pub const IRQOVER_MASK: u32 = 0x2 << IRQOVER_SHIFT;

    pub const FUNCSEL_MIN: u32 = 1;
    pub const FUNCSEL_MAX: u32 = 9;
    pub const FUNCSEL_NULL: u32 = 31;

    pub const OUTOVER_NORMAL: u32 = 0x0 << OUTOVER_SHIFT;
    pub const OUTOVER_INVERT: u32 = 0x1 << OUTOVER_SHIFT;
    pub const OUTOVER_LOW: u32 = 0x2 << OUTOVER_SHIFT;
    pub const OUTOVER_HIGH: u32 = 0x3 << OUTOVER_SHIFT;
    
    pub const OEOVER_NORMAL: u32 = 0x0 << OEOVER_SHIFT;
    pub const OEOVER_INVERT: u32 = 0x1 << OEOVER_SHIFT;
    pub const OEOVER_DISABLE: u32 = 0x2 << OEOVER_SHIFT;
    pub const OEOVER_ENABLE: u32 = 0x3 << OEOVER_SHIFT;
    
    pub const INOVER_NORMAL: u32 = 0x0 << INOVER_SHIFT;
    pub const INOVER_INVERT: u32 = 0x1 << INOVER_SHIFT;
    pub const INOVER_LOW: u32 = 0x2 << INOVER_SHIFT;
    pub const INOVER_HIGH: u32 = 0x3 << INOVER_SHIFT;
    
    pub const IRQOVER_NORMAL: u32 = 0x0 << IRQOVER_SHIFT;
    pub const IRQOVER_INVERT: u32 = 0x1 << IRQOVER_SHIFT;
    pub const IRQOVER_LOW: u32 = 0x2 << IRQOVER_SHIFT;
    pub const IRQOVER_HIGH: u32 = 0x3 << IRQOVER_SHIFT;
}

pub const MAX_GPIO: u8 = 29;
pub const MAX_FUNC: u8 = 9;
pub const FUNC_NULL: u8 = 0x1f;

pub struct IOBank0 {
    registers: *mut u32
}

impl IOBank0 {
    const unsafe fn new(base: usize) -> Self {
        Self {
            registers: ptr::with_exposed_provenance_mut(base)
        }
    }

    pub fn set_gpio_func(&mut self, gpio: u8, func: u8) {
        assert!(func <= FUNC_NULL);
        assert!(gpio <= MAX_GPIO);
        let gpio_ptr = unsafe {
            self.registers.add((gpio as usize) * 2 + 1)
        };
        let val = (((func as u32) << gpio_ctrl_register::FUNCSEL_SHIFT) & gpio_ctrl_register::FUNCSEL_MASK) |
            gpio_ctrl_register::OUTOVER_NORMAL |
            gpio_ctrl_register::OEOVER_NORMAL |
            gpio_ctrl_register::INOVER_NORMAL |
            gpio_ctrl_register::IRQOVER_NORMAL;
        unsafe {
            gpio_ptr.write_volatile(val);
        }
    }
}

pub enum IOBank0ReplyError {
    SendError,
    InvalidGPIO,
    InvalidFunc,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<IOBank0ReplyError> for u32 {
    fn from(value: IOBank0ReplyError) -> Self {
        match value {
            IOBank0ReplyError::SendError => 1,
            IOBank0ReplyError::InvalidGPIO => 2,
            IOBank0ReplyError::InvalidFunc => 3,
            IOBank0ReplyError::InvalidSendBuffer => 4,
            IOBank0ReplyError::InvalidReplyBuffer => 5
        }
    }
}

pub enum IOBank0Error {
    ReplyError(IOBank0ReplyError),
    QueueError(QueueError)
}

impl From<IOBank0ReplyError> for IOBank0Error {
    fn from(value: IOBank0ReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for IOBank0Error {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

pub struct Request {
    gpio: u8,
    func: u8
}

impl Request {
    pub fn parse() -> Result<Self, IOBank0Error> {
        let header = read_header(0)?;
        if header.tag > MAX_GPIO as u32 {
            return Err(IOBank0Error::ReplyError(IOBank0ReplyError::InvalidGPIO));
        }
        let gpio = header.tag as u8;
        if header.send_len != 1 {
            return Err(IOBank0Error::ReplyError(IOBank0ReplyError::InvalidSendBuffer));
        }
        if header.reply_len != 0 {
            return Err(IOBank0Error::ReplyError(IOBank0ReplyError::InvalidReplyBuffer));
        }
        let mut buffer = [0; 1];
        _ = receive(0, &mut buffer)?;
        let mut func = buffer[0];
        if func == 0xff {
            func = FUNC_NULL;
        } else if func > MAX_FUNC {
            return Err(IOBank0Error::ReplyError(IOBank0ReplyError::InvalidFunc));
        }
        Ok(Self { 
            gpio, 
            func 
        })
    }
}

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

const RESET_QUEUE: u32 = 0;
const RESET_IO_BANK0: u32 = 5;

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(io_bank0_base: usize) {
    // reset io bank 0
    send(RESET_QUEUE, RESET_IO_BANK0, &mut [], 0, 0).unwrap();
    let mut io_bank0 = unsafe {
        IOBank0::new(io_bank0_base)
    };
    loop {
        match Request::parse() {
            Ok(request) => {
                io_bank0.set_gpio_func(request.gpio, request.func);
                check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
            },
            Err(err) => {
                match err {
                    IOBank0Error::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    IOBank0Error::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(IOBank0ReplyError::SendError))).unwrap_or(Ok(())).unwrap();
                            },
                            _ => {
                                panic!("{:?}", err);
                            }
                        }
                    }
                }
            }
        }
    }
}
