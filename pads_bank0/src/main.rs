/*   
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS IO Bank 0 Pads driver.
 *
 * The SmallOS IO Bank 0 Pads driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS IO Bank 0 Pads driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS IO Bank 0 Pads driver. 
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

use core::ptr;

use small_os_lib::{HeaderError, QueueError, args, check_critical, check_header_len, read_header, receive, reply_empty, send_empty};

/// GPIO register masks and shifts for a IO Bank 0 Pads GPIO memory mapped register
mod gpio_register {
    /// Slew rate control shift (0 for slow and 1 for fast)
    pub const SLEWFAST_SHIFT: usize = 0;
    /// Shift for enabling the schmitt trigger
    pub const SCHMITT_SHIFT: usize = 1;
    /// Shift for enabling pull down (pin drawn low)
    pub const PULL_DOWN_ENABLE_SHIFT: usize = 2;
    /// Shift for enabling pull up (pin drawn high)
    pub const PULL_UP_ENABLE_SHIFT: usize = 3;
    /// Drive strength shift
    pub const DRIVE_SHIFT: usize = 4;
    /// Shift for input enable
    pub const INPUT_ENABLE_SHIFT: usize = 6;
    /// Shift for output disable
    pub const OUTPUT_DISABLE_SHIFT: usize = 7;

    /// Slew rate control mask (0 for slow and 1 for fast)
    pub const SLEWFAST_MASK: u32 = 1 << SLEWFAST_SHIFT;
    /// Mask for enabling the schmitt trigger
    pub const SCHMITT_MASK: u32 = 1 << SCHMITT_SHIFT;
    /// Mask for enabling pull down (pin drawn low)
    pub const PULL_DOWN_ENABLE_MASK: u32 = 1 << PULL_DOWN_ENABLE_SHIFT;
    /// Mask for enabling pull up (pin drawn high)
    pub const PULL_UP_ENABLE_MASK: u32 = 1 << PULL_UP_ENABLE_SHIFT;
    /// Drive strength mask
    pub const DRIVE_MASK: u32 = 0x3 << DRIVE_SHIFT;
    /// Mask for input enable
    pub const INPUT_ENABLE_MASK: u32 = 1 << INPUT_ENABLE_SHIFT;
    /// Mask for output disable
    pub const OUTPUT_DISABLE_MASK: u32 = 1 << OUTPUT_DISABLE_SHIFT;

    /// Drive strength of 2mA
    pub const DRIVE_2MA: u32 = 0x0 << DRIVE_SHIFT;
    /// Drive strength of 4mA
    pub const DRIVE_4MA: u32 = 0x1 << DRIVE_SHIFT;
    /// Drive strength of 8mA
    pub const DRIVE_8MA: u32 = 0x2 << DRIVE_SHIFT;
    /// Drive strength of 12mA
    pub const DRIVE_12MA: u32 = 0x3 << DRIVE_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SLEWFAST_MASK |
        SCHMITT_MASK |
        PULL_DOWN_ENABLE_MASK |
        PULL_UP_ENABLE_MASK |
        DRIVE_MASK |
        INPUT_ENABLE_MASK |
        OUTPUT_DISABLE_MASK;
}

/// IO Bank 0 Pads reply errors
pub enum PadsBank0ReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// An invalid GPIO was selected
    InvalidGPIO,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Converts from a `HeaderError` to a `PadsBank0ReplyError`
impl From<HeaderError> for PadsBank0ReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

/// Converts from a `PadsBank0ReplyError` to a `u32`
impl From<PadsBank0ReplyError> for u32 {
    fn from(value: PadsBank0ReplyError) -> Self {
        match value {
            PadsBank0ReplyError::SendError => 1,
            PadsBank0ReplyError::InvalidRequest => 2,
            PadsBank0ReplyError::InvalidGPIO => 3,
            PadsBank0ReplyError::InvalidSendBuffer => 4,
            PadsBank0ReplyError::InvalidReplyBuffer => 5
        }
    }
}

/// IO Bank 0 Pads Errors
pub enum PadsBank0Error {
    /// Error with the request
    ReplyError(PadsBank0ReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts from a `PadsBank0ReplyError` to a `PadsBank0Error`
impl From<PadsBank0ReplyError> for PadsBank0Error {
    fn from(value: PadsBank0ReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts from a `QueueError` to a `PadsBank0Error`
impl From<QueueError> for PadsBank0Error {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// Converts from a `HeaderError` to a `PadsBank0Error`
impl From<HeaderError> for PadsBank0Error {
    fn from(value: HeaderError) -> Self {
        Self::from(PadsBank0ReplyError::from(value))
    }
}

/// IO Bank 0 Pads object for managing the IO Bank 0 pads
pub struct PadsBank0 {
    /// Memory mapped registers
    registers: *mut u32,
    /// GPIO pad arguments
    pads: &'static [u32]
}

/// GPIO pad type
#[derive(Debug, Clone, Copy)]
pub enum GPIOType {
    /// GPIO is an input
    Input,
    /// GPIO is an output
    Output,
    /// GPIO is an analog input
    Analog
}

/// IO Bank 0 Pads request
pub enum Request {
    /// Check if IO Bank 0 Pads has initialised
    Finished,
    /// Reset GPIO pad
    ResetGPIO(u8)
}

impl Request {
    /// Parses the next request  
    /// Returns the request on success or an `PadsBank0Error` on failure
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

/// Maximum GPIO number
pub const MAX_GPIO: u8 = 31;

/// GPIO Pad state
pub enum GPIOState {
    /// Disable GPIO pad
    Disable = 0,
    /// GPIO pad setup for normal operation
    Normal = 1,
    /// GPIO pad setup as analog
    Analog = 2,
    /// GPIO pad set to pull up
    PullUp = 3,
}

/// Converts from a `u8` to a `GPIOState`
impl TryFrom<u8> for GPIOState {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Disable),
            1 => Ok(Self::Normal),
            2 => Ok(Self::Analog),
            3 => Ok(Self::PullUp),
            _ => Err(value)
        }
    }
}

impl PadsBank0 {
    /// Creates a new `PadsBank0` object and initialises the GPIO pads
    /// `base` is the base address of the IO Bank 0 pads memory mapped registers  
    /// `pads` is a set of GPIO pad arguments
    /// # Safety
    /// `base` must be a valid address which points to the IO Bank 0 pads memory mapped registers and not
    /// being used by anything else
    unsafe fn new(base: usize, pads: &'static [u32]) -> Self {
        let mut res = Self {
            registers: ptr::with_exposed_provenance_mut(base),
            pads
        };
        for i in 0..30 {
            res.reset_gpio(i as u8);
        }
        res
    }

    /// Resets a GPIO pad based on its pad argument  
    /// `gpio` is the GPIO pad to reset  
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn reset_gpio(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let index = (gpio / 16) as usize;
        let shift = (gpio % 16) * 2;
        self.set_gpio(gpio, GPIOState::try_from(((self.pads[index] >> shift) & 0x3) as u8).unwrap());
    }

    /// Sets a GPIO pad to `gpio_state` state  
    /// `gpio` is the GPIO pad to set  
    /// `gpio_state` is the pad set to configure it to
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn set_gpio(&mut self, gpio: u8, gpio_state: GPIOState) {
        assert!(gpio <= MAX_GPIO);
        match gpio_state {
            GPIOState::Disable => self.disable_gpio(gpio),
            GPIOState::Normal => self.set_gpio_normal(gpio),
            GPIOState::Analog => self.set_gpio_analog(gpio),
            GPIOState::PullUp => self.set_gpio_pull_up(gpio),
        }
    }

    /// Sets a GPIO pad to its normal state  
    /// `gpio` is the GPIO pad to set  
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn set_gpio_normal(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let pad_ptr = unsafe {
            self.registers.add((gpio as usize) + 1)
        };
        let val = unsafe {
            pad_ptr.read_volatile()
        };
        let val = 
            (val & !gpio_register::VALID_MASK) |
            gpio_register::DRIVE_4MA | 
            gpio_register::PULL_DOWN_ENABLE_MASK | 
            gpio_register::INPUT_ENABLE_MASK |
            gpio_register::SCHMITT_MASK;
        unsafe {
            pad_ptr.write_volatile(val);
        }
    }

    /// Sets a GPIO pad to its analog state  
    /// `gpio` is the GPIO pad to set  
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn set_gpio_analog(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let pad_ptr = unsafe {
            self.registers.add((gpio as usize) + 1)
        };
        let val = unsafe {
            pad_ptr.read_volatile()
        };
        let val = (val & !gpio_register::VALID_MASK) |
            gpio_register::DRIVE_4MA | 
            gpio_register::PULL_DOWN_ENABLE_MASK | 
            gpio_register::OUTPUT_DISABLE_MASK |
            gpio_register::SCHMITT_MASK;
        unsafe {
            pad_ptr.write_volatile(val);
        }
    }

    /// Sets a GPIO pad to its pull up state  
    /// `gpio` is the GPIO pad to set  
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn set_gpio_pull_up(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let pad_ptr = unsafe {
            self.registers.add((gpio as usize) + 1)
        };
        let val = unsafe {
            pad_ptr.read_volatile()
        };
        let val = (val & !gpio_register::VALID_MASK) |
            gpio_register::DRIVE_4MA | 
            gpio_register::PULL_UP_ENABLE_MASK | 
            gpio_register::INPUT_ENABLE_MASK |
            gpio_register::SCHMITT_MASK;
        unsafe {
            pad_ptr.write_volatile(val);
        }
    }

    /// Sets a GPIO pad to its disabled state  
    /// `gpio` is the GPIO pad to set  
    /// Panics if `gpio` is greater than `MAX_GPIO`
    pub fn disable_gpio(&mut self, gpio: u8) {
        assert!(gpio <= MAX_GPIO);
        let pad_ptr = unsafe {
            self.registers.add((gpio as usize) + 1)
        };
        let val = unsafe {
            pad_ptr.read_volatile()
        };
        let val = val & !gpio_register::VALID_MASK; 
        unsafe {
            pad_ptr.write_volatile(val);
        }
    }
}

/// Reset driver endpoint
const RESET_QUEUE: u32 = 0;

/// Driver entry point
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 3);
    let pads_base = args[0] as usize;
    let pads = &args[1..];
    // check reset is done
    send_empty(RESET_QUEUE, 0, &[]).unwrap();
    #[cfg(test)]
    test_main();
    let mut pads = unsafe {
        PadsBank0::new(pads_base, pads)
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

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::{kprint, kprintln};

    use super::*;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for pads bank 0", tests.len());
        for test in tests {
            test();
        }
    }

    #[test_case]
    fn test_valid() {
        kprintln!("Testing pads bank 0 register mask values");
        kprint!("Testing gpio ctrl register ");
        assert_eq!(gpio_register::VALID_MASK, 0xff);
        kprintln!("[ok]")
    }

    #[test_case]
    fn test_setup() {
        let args = args();
        let base = args[0] as usize;
        let pads = &args[1..];
        let pads_bank0 = unsafe {
            PadsBank0::new(base, pads)
        };
        kprintln!("Testing pads bank 0 setup");
        for i in 0..30 {
            kprint!("Testing gpio register {} ", i);
            let index = (i / 16) as usize;
            let shift = (i % 16) * 2;
            let state = (pads_bank0.pads[index] >> shift) & 0x3;
            let pad = unsafe {
                pads_bank0.registers.add((i as usize) + 1).read()
            };
            match state {
                0 => {
                    // disable
                    assert_eq!(pad & gpio_register::VALID_MASK, 0);
                },
                1 => {
                    // normal
                    assert_eq!(pad & gpio_register::VALID_MASK, 0x56);
                },
                2 => {
                    // analog
                    assert_eq!(pad & gpio_register::VALID_MASK, 0x96);
                },
                3 => {
                    // pull up
                    assert_eq!(pad & gpio_register::VALID_MASK, 0x5a);
                },
                _ => unreachable!()
            }
            kprintln!("[ok]");
        }
    }
}
