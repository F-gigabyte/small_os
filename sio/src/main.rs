/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS SIO driver.
 *
 * The SmallOS SIO driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS SIO driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS SIO driver. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, WriteOnly}};
use small_os_lib::{HeaderError, QueueError, args, check_critical, check_header_len, read_header, receive, reply, reply_empty, send_empty};

/// Single Cycle IO memory mapped registers
#[repr(C)]
struct SIORegisters {
    /// CPUID register (0x00)
    cpuid: ReadPure<u32>, // 0x0
    /// GPIO input register (0x04)
    gpio_in: ReadPure<u32>, // 0x4
    /// GPIO high input register (0x08)
    gpio_high_in: ReadPure<u32>, // 0x8
    _reserved0: u32, // 0xc
    /// GPIO output register (0x10)
    gpio_out: WriteOnly<u32>, // 0x10
    /// GPIO output set register (0x14)
    gpio_out_set: WriteOnly<u32>, // 0x14
    /// GPIO output clear register (0x18)
    gpio_out_clear: WriteOnly<u32>, // 0x18
    /// GPIO output xor register (0x1c)
    gpio_out_xor: WriteOnly<u32>, // 0x1c
    /// GPIO output enable register (0x20)
    gpio_out_enable: WriteOnly<u32>, // 0x20
    /// GPIO output enable set register (0x24)
    gpio_out_enable_set: WriteOnly<u32>, // 0x24
    /// GPIO output enable clear register (0x28)
    gpio_out_enable_clear: WriteOnly<u32>, // 0x28
    /// GPIO output enable xor register (0x2c)
    gpio_out_enable_xor: WriteOnly<u32>, // 0x2c
    /// GPIO high output register (0x30)
    gpio_high_out: WriteOnly<u32>, // 0x30
    /// GPIO high output set register (0x34)
    gpio_high_out_set: WriteOnly<u32>, // 0x34
    /// GPIO high output clear register (0x38)
    gpio_high_out_clear: WriteOnly<u32>, // 0x38
    /// GPIO high output xor register (0x3c)
    gpio_high_out_xor: WriteOnly<u32>, // 0x3c
    /// GPIO high output enable register (0x40)
    gpio_high_out_enable: WriteOnly<u32>, // 0x40
    /// GPIO high output enable set register (0x44)
    gpio_high_out_enable_set: WriteOnly<u32>, // 0x44
    /// GPIO high output enable clear register (0x48)
    gpio_high_out_enable_clear: WriteOnly<u32>, // 0x48
    /// GPIO high output enable xor register (0x4c)
    gpio_high_out_enable_xor: WriteOnly<u32>, // 0x4c
}

/// Single Cycle IO object to manage the Single Cycle IO device
pub struct SIO {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, SIORegisters>,
    /// GPIO bitmap
    bitmap: u32
}

/// Maximum GPIO that can be indexed
pub const MAX_GPIO: u8 = 29;

impl SIO {
    /// Creates a new `SIO` object  
    /// `sio_base` is the base address of the Single Cycle IO memory mapped registers
    /// # Safety
    /// `sio_base` must be a valid address which points to an Single Cycle IO memory mapped registers and not
    /// being used by anything else
    pub unsafe fn new(sio_base: usize, bitmap: u32) -> Self {
        unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(sio_base)).unwrap()),
                bitmap 
            }
        }
    }

    /// Configures a GPIO as an output  
    /// `gpio` is the GPIO pin to configure  
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn set_output(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_out_enable_set).write(val);
    }
    
    /// Disables a GPIO's output   
    /// `gpio` is the GPIO pin to configure  
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn disable_output(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_out_enable_clear).write(val);
    }
    
    ///  Writes to a GPIO pin (turning it on)  
    /// `gpio` is the GPIO pin to write to  
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn write_gpio(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_out_set).write(val);
    }
    
    ///  Clears a GPIO pin (turning it off)  
    /// `gpio` is the GPIO pin to clear  
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn clear_gpio(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_out_clear).write(val);
    }
    
    /// Reads from a GPIO pin  
    /// `gpio` is the GPIO pin to read from   
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn read_gpio(&mut self, gpio: u8) -> bool {
        assert!(gpio <= MAX_GPIO);
        let val = 1 << gpio;
        field!(self.registers, gpio_in).read() & val != 0
    }
}

/// Single Cycle IO reply errors
pub enum SIOReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// An invalid GPIO was provided
    InvalidGPIO,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Converts from a `SIOReplyError` to a `u32`
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

/// Converts from a `HeaderError` to a `SIOReplyError`
impl From<HeaderError> for SIOReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer,
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer
        }
    }
}

/// Single Cycle IO Errors
pub enum SIOError {
    /// Error with the request
    ReplyError(SIOReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts from a `SIOReplyError` to a `SIOError`
impl From<SIOReplyError> for SIOError {
    fn from(value: SIOReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts from a `QueueError` to a `SIOError`
impl From<QueueError> for SIOError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// Converts from a `HeaderError` to a `SIOError`
impl From<HeaderError> for SIOError {
    fn from(value: HeaderError) -> Self {
        Self::from(SIOReplyError::from(value))
    }
}

/// A GPIO request type
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GPIORequestType {
    /// Read a GPIO pin
    ReadGPIO,
    /// Write to a GPIO pin (turning it on)
    WriteGPIO,
    /// Clear a GPIO pin (turning it off)
    ClearGPIO,
    /// Disable a GPIO's output
    DisableGPIOOutput,
    /// Configure a GPIO pin as an output pin
    SetGPIOOutput
}

/// A GPIO request
pub struct Request {
    /// The GPIO pin
    gpio: u8,
    /// The GPIO request type
    request_type: GPIORequestType
}

/// Checks if the provided GPIO is valid  
/// `gpio` is the pin to check  
/// `bitmap` is the bitmap of all GPIO pins allocated to the Single Cycle IO driver
fn check_valid_gpio(gpio: u8, bitmap: u32) -> Result<(), SIOError> {
    if gpio > MAX_GPIO || ((1 << gpio) & bitmap == 0) {
        Err(SIOError::ReplyError(SIOReplyError::InvalidGPIO))
    } else {
        Ok(())
    }
}

impl Request {
    /// Parses the next request  
    /// Returns the request on success or a `SIOError` on failure
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

/// IO Bank 0 driver endpoint
const IO_BANK0_QUEUE: u32 = 0;

/// Driver entry point
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 2);
    let sio_base = args[0] as usize;
    let bitmap = args[1];
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

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::kprintln;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for SIO", tests.len());
        for test in tests {
            test();
        }
    }
}
