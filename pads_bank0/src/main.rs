// use core intrinsics
#![feature(core_intrinsics)]
#![no_std]
#![no_main]

use core::{intrinsics::abort, panic::PanicInfo, ptr};

use small_os_lib::{HeaderError, QueueError, check_critical, check_header_len, read_header, receive, reply_empty};

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
    registers: *mut u32
}

#[derive(Debug, Clone, Copy)]
pub enum Voltage {
    Voltage1V8,
    Voltage3V3
}

#[derive(Debug, Clone, Copy)]
pub enum GPIOType {
    Input,
    Output,
    Analog
}

pub struct GPIORequest {
    gpio: u8,
    gpio_type: GPIOType
}

pub enum Request {
    GPIO(GPIORequest),
    Voltage(Voltage)
}

impl Request {
    pub fn parse() -> Result<Self, PadsBank0Error> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // GPIO request
                check_header_len(&header, 2, 0)?;
                let mut buffer = [0; 2];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                let gpio_type = buffer[1];
                if gpio > MAX_GPIO {
                    return Err(PadsBank0Error::ReplyError(PadsBank0ReplyError::InvalidGPIO));
                }
                let gpio_type = match gpio_type {
                    0 => GPIOType::Input,
                    1 => GPIOType::Output,
                    2 => GPIOType::Analog,
                    _ => {
                        return Err(PadsBank0Error::ReplyError(PadsBank0ReplyError::InvalidGPIOType));
                    }
                };
                Ok(Self::GPIO(
                    GPIORequest { 
                        gpio, 
                        gpio_type 
                    }
                ))
            },
            1 => {
                // Voltage request
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let voltage = buffer[0];
                let voltage = match voltage {
                    0 => Voltage::Voltage3V3,
                    1 => Voltage::Voltage1V8,
                    _ => {
                        return Err(PadsBank0Error::ReplyError(PadsBank0ReplyError::InvalidVoltage));
                    }
                };
                Ok(Self::Voltage(voltage))
            },
            _ => {
                return Err(PadsBank0Error::ReplyError(PadsBank0ReplyError::InvalidRequest));
            }
        }
    }
}

pub const MAX_GPIO: u8 = 31;

impl PadsBank0 {
    const unsafe fn new(base: usize) -> Self {
        Self {
            registers: ptr::with_exposed_provenance_mut(base)
        }
    }

    pub fn set_gpio_output(&mut self, gpio: u8) {
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

    pub fn set_gpio_input(&mut self, gpio: u8) {
        assert!(gpio < MAX_GPIO);
        let pad_ptr = unsafe {
            self.registers.add((gpio as usize) + 1)
        };
        let val = gpio_register::DRIVE_4MA | 
            gpio_register::PULL_DOWN_ENABLE_MASK | 
            gpio_register::INPUT_ENABLE_MASK |
            gpio_register::OUTPUT_DISABLE_MASK |
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

    pub fn set_voltage(&mut self, voltage: Voltage) {
        let voltage = match voltage {
            Voltage::Voltage1V8 => voltage_register::VOLTAGE_1V8,
            Voltage::Voltage3V3 => voltage_register::VOLTAGE_3V3
        };
        let volt_ptr = unsafe {
            self.registers.add(1)
        };
        unsafe {
            volt_ptr.write_volatile(voltage);
        }
    }
}

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(pads_base: usize) {
    let mut pads = unsafe {
        PadsBank0::new(pads_base)
    };
    loop {
        match Request::parse() {
            Ok(request) => {
                match request {
                    Request::GPIO(gpio) => {
                        match gpio.gpio_type {
                            GPIOType::Input => {
                                pads.set_gpio_input(gpio.gpio);
                            },
                            GPIOType::Output => {
                                pads.set_gpio_output(gpio.gpio);
                            },
                            GPIOType::Analog => {
                                pads.set_gpio_analog(gpio.gpio);
                            }
                        }
                    },
                    Request::Voltage(voltage) => {
                        pads.set_voltage(voltage);
                    }
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
