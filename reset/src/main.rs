/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Subsystem Reset Driver.
 *
 * The SmallOS Subsystem Reset Driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Subsystem Reset Driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS Subsystem Reset Driver. 
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

use core::{ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPureWrite, WriteOnly}};
use small_os_lib::{HeaderError, QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, args, check_critical, check_header_len, do_yield, read_header, reply_empty};

/// Subsystem Reset memory mapped registers
#[repr(C)]
struct ResetRegisters {
    /// Subsystem reset reset register (0x00)
    reset: ReadPureWrite<u32>,
    _reserved0: u32,
    /// Subsystem reset reset done register (0x08)
    reset_done: ReadOnly<u32>
}

/// Subsystem Reset object for managing device resets
pub struct Reset {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, ResetRegisters>,
    /// Memory mapped registers where writing a bit clears the corresponding bit in `registers`
    clear_reg: UniqueMmioPointer<'static, ResetRegisters>,
    /// Memory mapped registers where writing a bit sets the corresponding bit in `registers`
    set_reg: UniqueMmioPointer<'static, ResetRegisters>,
    /// Reset bitmap
    bitmap: u32
}

/// Mask of all non-reserved reset register bits
const RESET_VALID: u32 = 0x1ffffff;

impl Reset {
    /// Creates a new `Reset` object and resets all devices specified by `bitmap`  
    /// `base` is the base address of the Reset memory mapped registers
    /// `bitmap` is the reset bitmap of devices to reset  
    /// # Safety
    /// `base` must be a valid address which points to the Reset memory mapped registers and not
    /// being used by anything else
    unsafe fn new(base: usize, bitmap: u32) -> Self {
        let mut res = unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap()),
                bitmap
            }
        };
        res.reset_all();
        res
    }

    /// Resets all devices specified by the device bitmap
    pub fn reset_all(&mut self) {
        // mask is all the devices that need reseting
        let mask = (!field!(self.registers, reset).read()) & self.bitmap;
        if mask != 0 {
            // reset all masked devices not already reset
            field!(self.set_reg, reset).write(mask);
        }
        // deassert reset on all masked devices
        field!(self.clear_reg, reset).write(self.bitmap);
        // wait until devices ready to be used
        while field!(self.registers, reset_done).read() & self.bitmap != self.bitmap {
            do_yield().unwrap();
        }
    }

    /// Resets the specified device  
    /// `device` is the device to reset and is the index of its bit in the reset register
    /// Panics if `device` is greater than 24
    pub fn reset_device(&mut self, device: u8) {
        assert!(device <= 24);
        let mask = 1 << device;
        if field!(self.registers, reset).read() & mask == 0 {
            field!(self.set_reg, reset).write(mask);
        }
        field!(self.clear_reg, reset).write(mask);
        while field!(self.registers, reset_done).read() & mask == 0 {
            do_yield().unwrap();
        }
    }
}

/// Subsystem Reset reply errors
pub enum ResetReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// An invalid device was specified
    InvalidDevice,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Converts from a `HeaderError` to an `ResetReplyError`
impl From<HeaderError> for ResetReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

/// Converts from a `ResetReplyError` to a `u32`
impl From<ResetReplyError> for u32 {
    fn from(value: ResetReplyError) -> Self {
        match value {
            ResetReplyError::SendError => 1,
            ResetReplyError::InvalidRequest => 2,
            ResetReplyError::InvalidDevice => 3,
            ResetReplyError::InvalidSendBuffer => 4,
            ResetReplyError::InvalidReplyBuffer => 5
        }
    }
}

/// Reset Errors
pub enum ResetError {
    /// Error with the request
    ReplyError(ResetReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts from a `ResetReplyError` to a `ResetError`
impl From<ResetReplyError> for ResetError {
    fn from(value: ResetReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts from a `QueueError` to a `ResetError`
impl From<QueueError> for ResetError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// Converts from a `HeaderError` to a `ResetError`
impl From<HeaderError> for ResetError {
    fn from(value: HeaderError) -> Self {
        Self::from(ResetReplyError::from(value))
    }
}

/// Subsystem Reset request
pub enum Request {
    /// Check if Subsystem Reset has initialised
    Finished,
    /// Reset a device
    ResetDevice(u8)
}

impl Request {
    /// Parses the next request  
    /// `bitmap` is the device bitmap  
    /// Returns the request on success or a `ResetError` on failure
    pub fn parse(bitmap: u32) -> Result<Self, ResetError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // Finished request
                check_header_len(&header, 0, 0)?;
                Ok(Self::Finished)
            },
            1 => {
                // Reset Device request
                check_header_len(&header, 0, 0)?; 
                if header.device >= 1 && header.device <= 22 {
                    let extra_shift = if header.device > 21 {
                        // skip UART1, TBMAN and JTAG
                        3
                    } else if header.device > 19 {
                        // skip TBMAN and JTAG
                        2
                    } else if header.device > 7 {
                        // skip JTAG
                        1
                    } else {
                        0
                    };
                    let device = (header.device + extra_shift - 1) as u8;
                    if (1 << device) & bitmap != 0 {
                        Ok(Self::ResetDevice(device))
                    } else {
                        Err(ResetError::ReplyError(ResetReplyError::InvalidDevice))
                    }
                } else {
                    Err(ResetError::ReplyError(ResetReplyError::InvalidDevice))
                }
            },
            _ => Err(ResetError::ReplyError(ResetReplyError::InvalidRequest))
        }
    }
}

/// Driver entry point
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 2);
    #[cfg(test)]
    test_main();
    let reset_base = args[0] as usize;
    let bitmap = args[1];
    let mut reset = unsafe {
        Reset::new(reset_base, bitmap)
    };
    loop {
        match Request::parse(reset.bitmap) {
            Ok(request) => {
                match request {
                    Request::Finished => {},
                    Request::ResetDevice(device) => reset.reset_device(device),
                }
                check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
            },
            Err(err) => {
                match err {
                    ResetError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    ResetError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(ResetReplyError::SendError))).unwrap_or(Ok(())).unwrap();
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

#[cfg(test)]
mod test {
    use small_os_lib::{args, kprint, kprintln};

    use super::*;

    /// Test framework which runs all the tests
    /// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for reset", tests.len());
        for test in tests {
            test();
        }
    }

    #[test_case]
    fn test_setup() {
        let args = args();
        let reset_base = args[0] as usize;
        let bitmap = args[1];
        let mut reset = unsafe {
            Reset::new(reset_base, bitmap)
        };
        kprintln!("Testing reset setup");
        let reset_mask = field!(reset.registers, reset).read();
        kprint!("Testing reset register ");
        assert_eq!(reset_mask & reset.bitmap, 0);
        kprintln!("[ok]");
    }
}
