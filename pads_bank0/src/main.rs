// use core intrinsics
#![feature(core_intrinsics)]
#![no_std]
#![no_main]

use core::{intrinsics::abort, panic::PanicInfo, ptr, slice};

use small_os_lib::{HeaderError, QueueError, check_critical, check_header_len, read_header, receive, reply_empty, send_empty};

mod voltage_register {
    pub const VOLTAGE_SHIFT: usize = 0;

    pub const VOLTAGE_MASK: u32 = 1 << VOLTAGE_SHIFT;

    pub const VOLTAGE_3V3: u32 = 0 << VOLTAGE_MASK;
    pub const VOLTAGE_1V8: u32 = 1 << VOLTAGE_MASK;
}

mod gpio_register {
    pub const SLEWFAST_SHIFT: usize = 0;
    pub const SCHMITT_SHIFT: usize = 1;
    pub const PULL_DOWN_ENABLE_SHIFT: usize = 2;
    pub const PULL_UP_ENABLE_SHIFT: usize = 3;
    pub const DRIVE_SHIFT: usize = 4;
    pub const INPUT_ENABLE_SHIFT: usize = 6;
    pub const OUTPUT_DISABLE_SHIFT: usize = 7;

    pub const SLEWFAST_MASK: u32 = 1 << SLEWFAST_SHIFT;
    pub const SCHMITT_MASK: u32 = 1 << SCHMITT_SHIFT;
    pub const PULL_DOWN_ENABLE_MASK: u32 = 1 << PULL_DOWN_ENABLE_SHIFT;
    pub const PULL_UP_ENABLE_MASK: u32 = 1 << PULL_UP_ENABLE_SHIFT;
    pub const DRIVE_MASK: u32 = 0x3 << DRIVE_SHIFT;
    pub const INPUT_ENABLE_MASK: u32 = 1 << INPUT_ENABLE_SHIFT;
    pub const OUTPUT_DISABLE_MASK: u32 = 1 << OUTPUT_DISABLE_SHIFT;

    pub const DRIVE_2MA: u32 = 0x0 << DRIVE_SHIFT;
    pub const DRIVE_4MA: u32 = 0x1 << DRIVE_SHIFT;
    pub const DRIVE_8MA: u32 = 0x2 << DRIVE_SHIFT;
    pub const DRIVE_12MA: u32 = 0x3 << DRIVE_SHIFT;
}

pub enum PadsBank0ReplyError {
    SendError,
    InvalidRequest,
    InvalidGPIO,
    InvalidGPIOType,
    InvalidVoltage,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<HeaderError> for PadsBank0ReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

impl From<PadsBank0ReplyError> for u32 {
    fn from(value: PadsBank0ReplyError) -> Self {
        match value {
            PadsBank0ReplyError::SendError => 1,
            PadsBank0ReplyError::InvalidRequest => 2,
            PadsBank0ReplyError::InvalidGPIO => 3,
            PadsBank0ReplyError::InvalidGPIOType => 4,
            PadsBank0ReplyError::InvalidVoltage => 5,
            PadsBank0ReplyError::InvalidSendBuffer => 6,
            PadsBank0ReplyError::InvalidReplyBuffer => 7
        }
    }
}

pub enum PadsBank0Error {
    ReplyError(PadsBank0ReplyError),
    QueueError(QueueError)
}

impl From<PadsBank0ReplyError> for PadsBank0Error {
    fn from(value: PadsBank0ReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for PadsBank0Error {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

impl From<HeaderError> for PadsBank0Error {
    fn from(value: HeaderError) -> Self {
        Self::from(PadsBank0ReplyError::from(value))
    }
}

pub struct PadsBank0 {
    registers: *mut u32,
    pads: [u32; 2]
}

#[derive(Debug, Clone, Copy)]
pub enum GPIOType {
    Input,
    Output,
    Analog
}

pub enum Request {
    Finished,
    ResetGPIO(u8)
}

impl Request {
    pub fn parse() -> Result<Self, PadsBank0Error> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // Finished request
                check_header_len(&header, 0, 0)?;
                Ok(Self::Finished)
            },
            1 => {
                // GPIO request
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                if gpio > MAX_GPIO {
                    return Err(PadsBank0Error::ReplyError(PadsBank0ReplyError::InvalidGPIO));
                }
                Ok(Self::ResetGPIO(gpio))
            },
            _ => Err(PadsBank0Error::ReplyError(PadsBank0ReplyError::InvalidRequest)),
        }
    }
}

pub const MAX_GPIO: u8 = 31;

pub enum GPIOState {
    Disable = 0,
    Normal = 1,
    Analog = 2
}

impl TryFrom<u8> for GPIOState {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Disable),
            1 => Ok(Self::Normal),
            2 => Ok(Self::Analog),
            _ => Err(value)
        }
    }
}

impl PadsBank0 {
    unsafe fn new(base: usize, pads0: u32, pads1: u32) -> Self {
        let mut res = Self {
            registers: ptr::with_exposed_provenance_mut(base),
            pads: [pads0, pads1]
        };
        for i in 0..30 {
            res.reset_gpio(i as u8);
        }
        res
    }

    pub fn reset_gpio(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let index = (gpio / 16) as usize;
        let shift = (gpio % 16) * 2;
        self.set_gpio(gpio, GPIOState::try_from(((self.pads[index] >> shift) & 0x3) as u8).unwrap());
    }

    pub fn set_gpio(&mut self, gpio: u8, gpio_state: GPIOState) {
        assert!(gpio <= MAX_GPIO);
        match gpio_state {
            GPIOState::Disable => self.disable_gpio(gpio),
            GPIOState::Normal => self.set_gpio_normal(gpio),
            GPIOState::Analog => self.set_gpio_analog(gpio),
        }
    }

    pub fn set_gpio_normal(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let pad_ptr = unsafe {
            self.registers.add((gpio as usize) + 1)
        };
        let val = gpio_register::DRIVE_4MA | 
            gpio_register::PULL_DOWN_ENABLE_MASK | 
            gpio_register::INPUT_ENABLE_MASK |
            gpio_register::SCHMITT_MASK;
        unsafe {
            pad_ptr.write_volatile(val);
        }
    }

    pub fn set_gpio_analog(&mut self, gpio: u8) {
        assert!(gpio < MAX_GPIO);
        let pad_ptr = unsafe {
            self.registers.add((gpio as usize) + 1)
        };
        let val = gpio_register::DRIVE_4MA | 
            gpio_register::PULL_DOWN_ENABLE_MASK | 
            gpio_register::OUTPUT_DISABLE_MASK |
            gpio_register::SCHMITT_MASK;
        unsafe {
            pad_ptr.write_volatile(val);
        }
    }

    pub fn disable_gpio(&mut self, gpio: u8) {
        assert!(gpio < MAX_GPIO);
        let pad_ptr = unsafe {
            self.registers.add((gpio as usize) + 1)
        };
        let val = 0; 
        unsafe {
            pad_ptr.write_volatile(val);
        }
    }
}

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

const RESET_QUEUE: u32 = 0;

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(num_args: usize, pads_base: usize, pads0: u32, pads1: u32) {
    assert!(num_args == 3);
    // check reset is done
    send_empty(RESET_QUEUE, 0, &[]).unwrap();
    let mut pads = unsafe {
        PadsBank0::new(pads_base, pads0, pads1)
    };
    loop {
        match Request::parse() {
            Ok(request) => {
                match request {
                    Request::Finished => {},
                    Request::ResetGPIO(gpio) => pads.reset_gpio(gpio),
                }
                check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
            },
            Err(err) => {
                match err {
                    PadsBank0Error::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    PadsBank0Error::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(PadsBank0ReplyError::SendError))).unwrap_or(Ok(())).unwrap();
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
