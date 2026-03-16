#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{intrinsics::abort, panic::PanicInfo, ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, WriteOnly}};
use small_os_lib::{HeaderError, QueueError, check_critical, check_header_len, read_header, receive, reply, reply_empty};

#[repr(C)]
struct SIORegisters {
    cpuid: ReadPure<u32>,
    gpio_in: ReadPure<u32>,
    gpio_high_in: ReadPure<u32>,
    gpio_out: WriteOnly<u32>,
    gpio_out_set: WriteOnly<u32>,
    gpio_out_clear: WriteOnly<u32>,
    gpio_out_xor: WriteOnly<u32>,
    gpio_out_enable: WriteOnly<u32>,
    gpio_out_enable_set: WriteOnly<u32>,
    gpio_out_enable_clear: WriteOnly<u32>,
    gpio_out_enable_xor: WriteOnly<u32>,
    gpio_high_out: WriteOnly<u32>,
    gpio_high_out_set: WriteOnly<u32>,
    gpio_high_out_clear: WriteOnly<u32>,
    gpio_high_out_xor: WriteOnly<u32>,
    gpio_high_out_enable: WriteOnly<u32>,
    gpio_high_out_enable_set: WriteOnly<u32>,
    gpio_high_out_enable_clear: WriteOnly<u32>,
    gpio_high_out_enable_xor: WriteOnly<u32>,
}

pub struct SIO {
    registers: UniqueMmioPointer<'static, SIORegisters>,
}

pub const MAX_GPIO: u8 = 35;
pub const MAX_LOW_GPIO: u8 = 29;
pub const MAX_HIGH_GPIO: u8 = 5;

impl SIO {
    pub unsafe fn new(sio_base: usize) -> Self {
        unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(sio_base)).unwrap()) 
            }
        }
    }

    pub fn set_output_low(&mut self, gpio: u8) {
        assert!(gpio < MAX_LOW_GPIO);
        field!(self.registers, gpio_out_enable_set).write(1 << gpio);
    }
    
    pub fn set_output_high(&mut self, gpio: u8) {
        assert!(gpio < MAX_HIGH_GPIO);
        field!(self.registers, gpio_high_out_enable_set).write(1 << gpio);
    }
    
    pub fn set_input_low(&mut self, gpio: u8) {
        assert!(gpio < MAX_LOW_GPIO);
        field!(self.registers, gpio_out_enable_clear).write(1 << gpio);
    }
    
    pub fn set_input_high(&mut self, gpio: u8) {
        assert!(gpio < MAX_HIGH_GPIO);
        field!(self.registers, gpio_high_out_enable_clear).write(1 << gpio);
    }

    pub fn write_gpio_low(&mut self, gpio: u8) {
        assert!(gpio < MAX_LOW_GPIO);
        field!(self.registers, gpio_out_set).write(1 << gpio);
    }
    
    pub fn clear_gpio_low(&mut self, gpio: u8) {
        assert!(gpio < MAX_LOW_GPIO);
        field!(self.registers, gpio_out_clear).write(1 << gpio);
    }
    
    pub fn write_gpio_high(&mut self, gpio: u8) {
        assert!(gpio < MAX_HIGH_GPIO);
        field!(self.registers, gpio_high_out_set).write(1 << gpio);
    }
    
    pub fn clear_gpio_high(&mut self, gpio: u8) {
        assert!(gpio < MAX_HIGH_GPIO);
        field!(self.registers, gpio_high_out_clear).write(1 << gpio);
    }

    pub fn read_gpio_low(&mut self, gpio: u8) -> bool {
        assert!(gpio < MAX_LOW_GPIO);
        field!(self.registers, gpio_in).read() & (1 << gpio) != 0
    }
    
    pub fn read_gpio_high(&mut self, gpio: u8) -> bool {
        assert!(gpio < MAX_HIGH_GPIO);
        field!(self.registers, gpio_high_in).read() & (1 << gpio) != 0
    }

    pub fn read_cpuid(&mut self) -> u32 {
        field!(self.registers, cpuid).read()
    }
}

pub enum SIOReplyError {
    SendError,
    InvalidRequest,
    InvalidGPIO,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<SIOReplyError> for u32 {
    fn from(value: SIOReplyError) -> Self {
        match value {
            SIOReplyError::SendError => 1,
            SIOReplyError::InvalidRequest => 2,
            SIOReplyError::InvalidGPIO => 3,
            SIOReplyError::InvalidSendBuffer => 4,
            SIOReplyError::InvalidReplyBuffer => 5
        }
    }
}

impl From<HeaderError> for SIOReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer,
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer
        }
    }
}

pub enum SIOError {
    ReplyError(SIOReplyError),
    QueueError(QueueError)
}

impl From<SIOReplyError> for SIOError {
    fn from(value: SIOReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for SIOError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

impl From<HeaderError> for SIOError {
    fn from(value: HeaderError) -> Self {
        Self::from(SIOReplyError::from(value))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GPIORequestType {
    SetGPIOInput,
    SetGPIOOutput,
    WriteGPIO,
    ClearGPIO,
    ReadGPIO
}

pub struct GPIORequest {
    gpio: u8,
    request_type: GPIORequestType
}

pub enum Request {
    GPIO(GPIORequest),
    CPUID
}

impl Request {
    pub fn parse() -> Result<Self, SIOError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // set GPIO input
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                if gpio > MAX_GPIO {
                    return Err(SIOError::ReplyError(SIOReplyError::InvalidGPIO));
                }
                Ok(Request::GPIO(GPIORequest { 
                    gpio, 
                    request_type: GPIORequestType::SetGPIOInput 
                }))
            },
            1 => {
                // set GPIO output
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                if gpio > MAX_GPIO {
                    return Err(SIOError::ReplyError(SIOReplyError::InvalidGPIO));
                }
                Ok(Request::GPIO(GPIORequest { 
                    gpio, 
                    request_type: GPIORequestType::SetGPIOOutput 
                }))
            },
            2 => {
                // write GPIO
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                if gpio > MAX_GPIO {
                    return Err(SIOError::ReplyError(SIOReplyError::InvalidGPIO));
                }
                Ok(Request::GPIO(GPIORequest { 
                    gpio, 
                    request_type: GPIORequestType::WriteGPIO 
                }))
            },
            3 => {
                // clear GPIO
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                if gpio > MAX_GPIO {
                    return Err(SIOError::ReplyError(SIOReplyError::InvalidGPIO));
                }
                Ok(Request::GPIO(GPIORequest { 
                    gpio, 
                    request_type: GPIORequestType::ClearGPIO 
                }))
            },
            4 => {
                // read GPIO
                check_header_len(&header, 1, 1)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                if gpio > MAX_GPIO {
                    return Err(SIOError::ReplyError(SIOReplyError::InvalidGPIO));
                }
                Ok(Request::GPIO(GPIORequest { 
                    gpio, 
                    request_type: GPIORequestType::ReadGPIO 
                }))
            },
            5 => {
                // CPUID
                check_header_len(&header, 0, 4)?;
                Ok(Self::CPUID)
            },
            _ => {
                return Err(SIOError::ReplyError(SIOReplyError::InvalidRequest));
            }
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
pub extern "C" fn main(sio_base: usize) {
    let mut sio = unsafe {
        SIO::new(sio_base)
    };
    loop {
        match Request::parse() {
            Ok(request) => {
                match request {
                    Request::GPIO(gpio) => {
                        match gpio.request_type {
                            GPIORequestType::SetGPIOInput => {
                                if gpio.gpio <= MAX_LOW_GPIO {
                                    sio.set_input_low(gpio.gpio);
                                } else {
                                    sio.set_input_high(gpio.gpio - MAX_LOW_GPIO - 1);
                                }
                                check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                            },
                            GPIORequestType::SetGPIOOutput => {
                                if gpio.gpio <= MAX_LOW_GPIO {
                                    sio.set_output_low(gpio.gpio);
                                } else {
                                    sio.set_output_high(gpio.gpio - MAX_LOW_GPIO - 1);
                                }
                                check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                            },
                            GPIORequestType::ReadGPIO => {
                                let res = if gpio.gpio <= MAX_LOW_GPIO {
                                    sio.read_gpio_low(gpio.gpio)
                                } else {
                                    sio.read_gpio_high(gpio.gpio - MAX_LOW_GPIO - 1)
                                };
                                check_critical(reply(0, 0, &[res as u8])).unwrap_or(Ok(())).unwrap();
                            },
                            GPIORequestType::WriteGPIO => {
                                if gpio.gpio <= MAX_LOW_GPIO {
                                    sio.write_gpio_low(gpio.gpio);
                                } else {
                                    sio.write_gpio_high(gpio.gpio - MAX_LOW_GPIO - 1);
                                }
                                check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                            },
                            GPIORequestType::ClearGPIO => {
                                if gpio.gpio <= MAX_LOW_GPIO {
                                    sio.clear_gpio_low(gpio.gpio);
                                } else {
                                    sio.clear_gpio_high(gpio.gpio - MAX_LOW_GPIO - 1);
                                }
                                check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                            }
                        }
                    },
                    Request::CPUID => {
                        let res = sio.read_cpuid();
                        check_critical(reply(0, 0, &res.to_le_bytes())).unwrap_or(Ok(())).unwrap();
                    }
                }
            },
            Err(err) => {
                match err {
                    SIOError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    SIOError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(SIOReplyError::SendError))).unwrap_or(Ok(())).unwrap();
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
