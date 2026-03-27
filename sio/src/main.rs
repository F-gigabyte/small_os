#![no_std]
#![no_main]

use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, WriteOnly}};
use small_os_lib::{HeaderError, QueueError, check_critical, check_header_len, read_header, receive, reply, reply_empty, send_empty};

#[repr(C)]
struct SIORegisters {
    cpuid: ReadPure<u32>, // 0x0
    gpio_in: ReadPure<u32>, // 0x4
    gpio_high_in: ReadPure<u32>, // 0x8
    _reserved0: u32, // 0xc
    gpio_out: WriteOnly<u32>, // 0x10
    gpio_out_set: WriteOnly<u32>, // 0x14
    gpio_out_clear: WriteOnly<u32>, // 0x18
    gpio_out_xor: WriteOnly<u32>, // 0x1c
    gpio_out_enable: WriteOnly<u32>, // 0x20
    gpio_out_enable_set: WriteOnly<u32>, // 0x24
    gpio_out_enable_clear: WriteOnly<u32>, // 0x28
    gpio_out_enable_xor: WriteOnly<u32>, // 0x2c
    gpio_high_out: WriteOnly<u32>, // 0x30
    gpio_high_out_set: WriteOnly<u32>, // 0x34
    gpio_high_out_clear: WriteOnly<u32>, // 0x38
    gpio_high_out_xor: WriteOnly<u32>, // 0x3c
    gpio_high_out_enable: WriteOnly<u32>, // 0x40
    gpio_high_out_enable_set: WriteOnly<u32>, // 0x44
    gpio_high_out_enable_clear: WriteOnly<u32>, // 0x48
    gpio_high_out_enable_xor: WriteOnly<u32>, // 0x4c
}

pub struct SIO {
    registers: UniqueMmioPointer<'static, SIORegisters>,
    bitmap: u32
}

pub const MAX_GPIO: u8 = 29;

impl SIO {
    pub unsafe fn new(sio_base: usize, bitmap: u32) -> Self {
        unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(sio_base)).unwrap()),
                bitmap 
            }
        }
    }

    pub fn set_output(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_out_enable_set).write(val);
    }
    
    pub fn disable_output(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_out_enable_clear).write(val);
    }
    
    pub fn write_gpio(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_out_set).write(val);
    }
    
    pub fn clear_gpio(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_out_clear).write(val);
    }
    
    pub fn read_gpio(&mut self, gpio: u8) -> bool {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_in).read() & val != 0
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
    ReadGPIO,
    WriteGPIO,
    ClearGPIO,
    DisableGPIOOutput,
    SetGPIOOutput
}

pub struct Request {
    gpio: u8,
    request_type: GPIORequestType
}

fn check_valid_gpio(gpio: u8, bitmap: u32) -> Result<(), SIOError> {
    if gpio > MAX_GPIO || ((1 << gpio) & bitmap == 0) {
        Err(SIOError::ReplyError(SIOReplyError::InvalidGPIO))
    } else {
        Ok(())
    }
}

impl Request {
    pub fn parse(bitmap: u32) -> Result<Self, SIOError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // read GPIO
                check_header_len(&header, 1, 1)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                check_valid_gpio(gpio, bitmap)?;
                Ok(Request { 
                    gpio, 
                    request_type: GPIORequestType::ReadGPIO 
                })
            },
            1 => {
                // write GPIO
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                check_valid_gpio(gpio, bitmap)?;
                Ok(Request { 
                    gpio, 
                    request_type: GPIORequestType::WriteGPIO 
                })
            },
            2 => {
                // clear GPIO
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                check_valid_gpio(gpio, bitmap)?;
                Ok(Request { 
                    gpio, 
                    request_type: GPIORequestType::ClearGPIO 
                })
            },
            3 => {
                // disable GPIO output
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                check_valid_gpio(gpio, bitmap)?;
                Ok(Request { 
                    gpio, 
                    request_type: GPIORequestType::DisableGPIOOutput 
                })
            },
            4 => {
                // set GPIO output
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let gpio = buffer[0];
                check_valid_gpio(gpio, bitmap)?;
                Ok(Request { 
                    gpio, 
                    request_type: GPIORequestType::SetGPIOOutput 
                })
            },
            _ => {
                return Err(SIOError::ReplyError(SIOReplyError::InvalidRequest));
            }
        }
    }
}

const IO_BANK0_QUEUE: u32 = 0;

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(num_args: usize, sio_base: usize, bitmap: u32) {
    assert!(num_args == 2);
    // wait for func select to be set
    send_empty(IO_BANK0_QUEUE, 0, &[]).unwrap();
    let mut sio = unsafe {
        SIO::new(sio_base, bitmap)
    };
    loop {
        match Request::parse(sio.bitmap) {
            Ok(request) => {
                match request.request_type {
                    GPIORequestType::ReadGPIO => {
                        let res = sio.read_gpio(request.gpio);
                        check_critical(reply(0, 0, &[res as u8])).unwrap_or(Ok(())).unwrap();
                    },
                    GPIORequestType::WriteGPIO => {
                        sio.write_gpio(request.gpio);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    GPIORequestType::ClearGPIO => {
                        sio.clear_gpio(request.gpio);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    GPIORequestType::DisableGPIOOutput => {
                        sio.disable_output(request.gpio);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    GPIORequestType::SetGPIOOutput => {
                        sio.set_output(request.gpio);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
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
