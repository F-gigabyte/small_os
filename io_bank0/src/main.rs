#![no_std]
#![no_main]

use core::ptr::{self};

use small_os_lib::{HeaderError, QueueError, check_critical, check_header_len, read_header, receive, reply_empty, send, send_empty};

mod gpio_ctrl_register {
    pub const FUNCSEL_SHIFT: usize = 0;
    pub const OUTOVER_SHIFT: usize = 8;
    pub const OEOVER_SHIFT: usize = 12;
    pub const INOVER_SHIFT: usize = 16;
    pub const IRQOVER_SHIFT: usize = 28;

    pub const FUNCSEL_MASK: u32 = 0x1f << FUNCSEL_SHIFT;
    pub const OUTOVER_MASK: u32 = 0x3 << OUTOVER_SHIFT;
    pub const OEOVER_MASK: u32 = 0x3 << OEOVER_SHIFT;
    pub const INOVER_MASK: u32 = 0x3 << INOVER_SHIFT;
    pub const IRQOVER_MASK: u32 = 0x3 << IRQOVER_SHIFT;

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

#[derive(Debug, Clone, Copy)]
pub enum GPIODrive {
    Normal = 0x0,
    Low = 0x2,
    High = 0x3
}

pub struct IOBank0 {
    registers: *mut u32,
    func_sel: [u32; 4]
}

impl IOBank0 {
    unsafe fn new(base: usize, func_sel0: u32, func_sel1: u32, func_sel2: u32, func_sel3: u32) -> Self {
        let mut res = Self {
            registers: ptr::with_exposed_provenance_mut(base),
            func_sel: [func_sel0, func_sel1, func_sel2, func_sel3]
        };
        for i in 0..30 {
            res.reset_gpio(i as u8);
        }
        res
    }

    pub fn reset_gpio(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let index = (gpio / 8) as usize;
        let shift = (gpio % 8) * 4;
        let mut func = (self.func_sel[index] >> shift & 0xf) as u8;
        if func > 9 {
            func = FUNC_NULL;
        }
        self.set_gpio_func(gpio, func);
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

    pub fn drive_gpio(&mut self, gpio: u8, drive: GPIODrive) {
        assert!(gpio <= MAX_GPIO);
        let gpio_ptr = unsafe {
            self.registers.add((gpio as usize) * 2 + 1)
        };
        let mut val = unsafe {
            gpio_ptr.read_volatile()
        };
        val &= !gpio_ctrl_register::OUTOVER_MASK;
        let drive = drive as u32;
        val |= drive << gpio_ctrl_register::OUTOVER_SHIFT;
        unsafe {
            gpio_ptr.write_volatile(val);
        }
    }
}

pub enum IOBank0ReplyError {
    SendError,
    InvalidRequest,
    InvalidGPIO,
    InvalidFunc,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<HeaderError> for IOBank0ReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer,
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer
        }
    }
}

impl From<IOBank0ReplyError> for u32 {
    fn from(value: IOBank0ReplyError) -> Self {
        match value {
            IOBank0ReplyError::SendError => 1,
            IOBank0ReplyError::InvalidRequest => 2,
            IOBank0ReplyError::InvalidGPIO => 3,
            IOBank0ReplyError::InvalidFunc => 4,
            IOBank0ReplyError::InvalidSendBuffer => 5,
            IOBank0ReplyError::InvalidReplyBuffer => 6
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

impl From<HeaderError> for IOBank0Error {
    fn from(value: HeaderError) -> Self {
        Self::from(IOBank0ReplyError::from(value))
    }
}

pub enum Request {
    Finished,
    ResetGPIO(u8),
    DriveGPIO(u8, GPIODrive),
}

fn gpio_from_mask(mut mask: u32, mut index: u8) -> Option<u8> {
    let mut gpio = 0;
    while mask > 0 {
        if mask & 1 != 0 {
            if index == 0 {
                return Some(gpio)
            }
            index -= 1;
        }
        gpio += 1;
        mask >>= 1;
    }
    None
}

impl Request {
    pub fn parse() -> Result<Self, IOBank0Error> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // Finished request
                check_header_len(&header, 0, 0)?;
                Ok(Self::Finished)
            },
            1 => {
                // Reset GPIO request
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = gpio_from_mask(header.pin_mask, buffer[0]);
                if let Some(gpio) = gpio {
                    Ok(Self::ResetGPIO(gpio))
                } else {
                    Err(IOBank0Error::ReplyError(IOBank0ReplyError::InvalidGPIO))
                }
            },
            2 => {
                // Drive GPIO high
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = gpio_from_mask(header.pin_mask, buffer[0]);
                if let Some(gpio) = gpio {
                    Ok(Self::DriveGPIO(gpio, GPIODrive::High))
                } else {
                    Err(IOBank0Error::ReplyError(IOBank0ReplyError::InvalidGPIO))
                }
            },
            3 => {
                // Drive GPIO low
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = gpio_from_mask(header.pin_mask, buffer[0]);
                if let Some(gpio) = gpio {
                    Ok(Self::DriveGPIO(gpio, GPIODrive::Low))
                } else {
                    Err(IOBank0Error::ReplyError(IOBank0ReplyError::InvalidGPIO))
                }
            },
            4 => {
                // Drive GPIO normal
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = gpio_from_mask(header.pin_mask, buffer[0]);
                if let Some(gpio) = gpio {
                    Ok(Self::DriveGPIO(gpio, GPIODrive::Normal))
                } else {
                    Err(IOBank0Error::ReplyError(IOBank0ReplyError::InvalidGPIO))
                }
            },
            _ => Err(IOBank0Error::ReplyError(IOBank0ReplyError::InvalidRequest))
        }
    }
}

const RESET_QUEUE: u32 = 0;

const PADS_QUEUE: u32 = 1;

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(num_args: usize, io_bank0_base: usize, func_sel0: u32, func_sel1: u32, func_sel2: u32, func_sel3: u32) {
    assert!(num_args == 5);
    // check reset is finished
    send_empty(RESET_QUEUE, 0, &[]).unwrap();
    // check pads is finished
    send_empty(PADS_QUEUE, 0, &[]).unwrap();
    // don't reset IO Bank 0 as reset by kernel
    let mut io_bank0 = unsafe {
        IOBank0::new(io_bank0_base, func_sel0, func_sel1, func_sel2, func_sel3)
    };
    loop {
        match Request::parse() {
            Ok(request) => {
                match request {
                    Request::Finished => {},
                    Request::ResetGPIO(gpio) => io_bank0.reset_gpio(gpio),
                    Request::DriveGPIO(gpio, drive) => io_bank0.drive_gpio(gpio, drive)
                }
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
