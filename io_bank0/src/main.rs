/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS IO Bank 0 Driver.
 *
 * The SmallOS IO Bank 0 Driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS IO Bank 0 Driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS IO Bank 0 Driver. 
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

use core::ptr::{self};

use small_os_lib::{HeaderError, QueueError, args, check_critical, check_header_len, read_header, receive, reply_empty, send, send_empty};

/// Control register masks and shifts for an IO Bank 0 Control register
mod gpio_ctrl_register {
    /// Function selector shift
    pub const FUNCSEL_SHIFT: usize = 0;
    /// Output drive shift
    pub const OUTOVER_SHIFT: usize = 8;
    /// Output drive shift
    pub const OEOVER_SHIFT: usize = 12;
    /// Input drive shift
    pub const INOVER_SHIFT: usize = 16;
    /// Interrupt shift
    pub const IRQOVER_SHIFT: usize = 28;

    /// Function selector mask
    pub const FUNCSEL_MASK: u32 = 0x1f << FUNCSEL_SHIFT;
    /// Output drive mask
    pub const OUTOVER_MASK: u32 = 0x3 << OUTOVER_SHIFT;
    /// Output enable mask
    pub const OEOVER_MASK: u32 = 0x3 << OEOVER_SHIFT;
    /// Input drive mask
    pub const INOVER_MASK: u32 = 0x3 << INOVER_SHIFT;
    /// Interrupt mask
    pub const IRQOVER_MASK: u32 = 0x3 << IRQOVER_SHIFT;

    /// Minimum function selector
    pub const FUNCSEL_MIN: u32 = 1;
    /// Maximum function selector
    pub const FUNCSEL_MAX: u32 = 9;
    /// No function selected
    pub const FUNCSEL_NULL: u32 = 31;

    /// Output from peripheral
    pub const OUTOVER_NORMAL: u32 = 0x0 << OUTOVER_SHIFT;
    /// Output inverse of that from the peripheral
    pub const OUTOVER_INVERT: u32 = 0x1 << OUTOVER_SHIFT;
    /// Output driven low
    pub const OUTOVER_LOW: u32 = 0x2 << OUTOVER_SHIFT;
    /// Output driven high
    pub const OUTOVER_HIGH: u32 = 0x3 << OUTOVER_SHIFT;
    
    /// Output enabled / disabled based on peripheral
    pub const OEOVER_NORMAL: u32 = 0x0 << OEOVER_SHIFT;
    /// Output inverse of that from the peripheral
    pub const OEOVER_INVERT: u32 = 0x1 << OEOVER_SHIFT;
    /// Output driven low
    pub const OEOVER_DISABLE: u32 = 0x2 << OEOVER_SHIFT;
    /// Output driven high
    pub const OEOVER_ENABLE: u32 = 0x3 << OEOVER_SHIFT;
    
    /// Output enabled / disabled based on peripheral
    pub const INOVER_NORMAL: u32 = 0x0 << INOVER_SHIFT;
    /// Output inverse of that from the peripheral
    pub const INOVER_INVERT: u32 = 0x1 << INOVER_SHIFT;
    /// Output driven low
    pub const INOVER_LOW: u32 = 0x2 << INOVER_SHIFT;
    /// Output driven high
    pub const INOVER_HIGH: u32 = 0x3 << INOVER_SHIFT;
    
    /// IRQ same as from peripheral
    pub const IRQOVER_NORMAL: u32 = 0x0 << IRQOVER_SHIFT;
    /// IRQ inverted from that given by the peripheral
    pub const IRQOVER_INVERT: u32 = 0x1 << IRQOVER_SHIFT;
    /// IRQ driven low
    pub const IRQOVER_LOW: u32 = 0x2 << IRQOVER_SHIFT;
    /// IRQ driven high
    pub const IRQOVER_HIGH: u32 = 0x3 << IRQOVER_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = FUNCSEL_MASK |
        OUTOVER_MASK |
        OEOVER_MASK |
        INOVER_MASK |
        IRQOVER_MASK;
}

/// Maximum GPIO number
pub const MAX_GPIO: u8 = 29;
/// Maximum function selector
pub const MAX_FUNC: u8 = 9;
/// Function selector for NULL function
pub const FUNC_NULL: u8 = 0x1f;

/// GPIO Output drive
#[derive(Debug, Clone, Copy)]
pub enum GPIODrive {
    /// Don't drive GPIO
    Normal = 0x0,
    /// Drive GPIO low
    Low = 0x2,
    /// Drive GPIO high
    High = 0x3
}

/// IO Bank 0 object for managing the IO Bank 0 GPIOs
pub struct IOBank0 {
    /// IO Bank 0 memory mapped registers
    registers: *mut u32,
    /// Function select of all GPIO pins
    func_sel: &'static [u32]
}

impl IOBank0 {
    /// Creates a new `IOBank0` object and initialises all GPIO pins   
    /// `base` is the base address the IO Bank 0 memory mapped registers
    /// # Safety
    /// `base` must be a valid addresss which points to the IO Bank 0 memory mapped registers and not
    /// being used by anything else
    unsafe fn new(base: usize, func_sel: &'static [u32]) -> Self {
        let mut res = Self {
            registers: ptr::with_exposed_provenance_mut(base),
            func_sel
        };
        for i in 0..30 {
            res.reset_gpio(i as u8);
        }
        res
    }

    /// Resets GPIO pin `gpio` to what's specified in `func_sel`  
    /// `gpio` is the GPIO pin to reset  
    /// Panics if `gpio` is greater than `MAX_GPIO`
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

    /// Sets GPIO pin `gpio` to function select `func`  
    /// `gpio` is the GPIO pin to set  
    /// `func` is the GPIO function select to set it to  
    /// Panics if `gpio` is greater than `MAX_GPIO` or `func` is greater than `FUNC_NULL`
    pub fn set_gpio_func(&mut self, gpio: u8, func: u8) {
        assert!(func <= FUNC_NULL);
        assert!(gpio <= MAX_GPIO);
        let gpio_ptr = unsafe {
            self.registers.add((gpio as usize) * 2 + 1)
        };
        let val = unsafe {
            gpio_ptr.read_volatile()
        };
        let val = (val & !gpio_ctrl_register::VALID_MASK) |
            ((func as u32) << gpio_ctrl_register::FUNCSEL_SHIFT) |
            gpio_ctrl_register::OUTOVER_NORMAL |
            gpio_ctrl_register::OEOVER_NORMAL |
            gpio_ctrl_register::INOVER_NORMAL |
            gpio_ctrl_register::IRQOVER_NORMAL;
        unsafe {
            gpio_ptr.write_volatile(val);
        }
    }

    /// Sets GPIO pin `gpio` to have drive `drive`   
    /// `gpio` is the GPIO pin to set  
    /// `drive` is how to drive the GPIO pin  
    /// Panics if `gpio` is greater than `MAX_GPIO`
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

/// IO Bank 0 reply errors
pub enum IOBank0ReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// An invalid GPIO was selected
    InvalidGPIO,
    /// An invalid function select was selected
    InvalidFunc,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Converts from a `HeaderError` to an `IOBank0ReplyError`
impl From<HeaderError> for IOBank0ReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer,
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer
        }
    }
}

/// Converts from an `IOBank0ReplyError` to a `u32`
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

/// IO Bank 0 Errors
pub enum IOBank0Error {
    /// Error with the request
    ReplyError(IOBank0ReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts from an `IOBank0ReplyError` to an `IOBank0Error`
impl From<IOBank0ReplyError> for IOBank0Error {
    fn from(value: IOBank0ReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts from a `QueueError` to an `IOBank0Error`
impl From<QueueError> for IOBank0Error {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// Converts from a `HeaderError` to an `IOBank0Error`
impl From<HeaderError> for IOBank0Error {
    fn from(value: HeaderError) -> Self {
        Self::from(IOBank0ReplyError::from(value))
    }
}

/// IO Bank 0 request
pub enum Request {
    /// Check if IO Bank 0 has initialised
    Finished,
    /// Reset GPIO pin
    ResetGPIO(u8),
    /// Drive a GPIO pin
    DriveGPIO(u8, GPIODrive),
}

/// Converts from an index into `mask` into its corresponding GPIO pin  
/// `mask` is the GPIO mask  
/// `index` is the bit index into `mask`  
/// Returns None if the GPIO pin is not in `mask`
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
    /// Parses the next request  
    /// Returns the request on success or an `IOBank0Error` on failure
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

/// Reset driver endpoint
const RESET_QUEUE: u32 = 0;

/// IO Bank 0 pads driver endpoint
const PADS_QUEUE: u32 = 1;

/// Driver entry point
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 5);
    let io_bank0_base = args[0] as usize;
    let func_sel = &args[1..];
    // check reset is finished
    send_empty(RESET_QUEUE, 0, &[]).unwrap();
    // check pads is finished
    send_empty(PADS_QUEUE, 0, &[]).unwrap();
    #[cfg(test)]
    test_main();
    // don't reset IO Bank 0 as reset by kernel
    let mut io_bank0 = unsafe {
        IOBank0::new(io_bank0_base, func_sel)
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

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::{kprint, kprintln};

    use super::*;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for IO Bank 0", tests.len());
        for test in tests {
            test();
        }
    }

    #[test_case]
    fn test_valid() {
        kprintln!("Testing IO bank 0 register mask values");
        kprint!("Testing gpio ctrl register ");
        assert_eq!(gpio_ctrl_register::VALID_MASK, 0x3003331f);
        kprintln!("[ok]")
    }

    #[test_case]
    fn test_setup() {
        let args = args();
        let io_bank0_base = args[0] as usize;
        let func_sel = &args[1..];
        let io_bank0 = unsafe {
            IOBank0::new(io_bank0_base, func_sel)
        };
        kprintln!("Testing IO bank 0 setup"); 
        for i in 0..30 {
            let index = (i / 8) as usize;
            let shift = (i % 8) * 4;
            let mut func = (io_bank0.func_sel[index] >> shift & 0xf) as u8;
            if func > 9 {
                func = FUNC_NULL;
            }
            let expected = func as u32;
            let offset = i * 2 + 1;
            unsafe {
                kprint!("Testing register gpio {} ctrl ", i);
                assert_eq!(io_bank0.registers.add(offset).read_volatile() & gpio_ctrl_register::VALID_MASK, expected);
                kprintln!("[ok]");
            }
        }
    }
}
