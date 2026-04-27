/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Analog to Digital Converter driver.
 *
 * The SmallOS Analog to Digital Converter driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Analog to Digital Converter driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS Analog to Digital Converter driver. 
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

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPure, ReadPureWrite, ReadWrite}};
use small_os_lib::{HeaderError, QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, args, check_critical, check_header_len, do_yield, read_header, receive, reply, reply_empty, send_empty, wait_irq};

/// Analog to Digital Converter memory mapped registers
#[repr(C)]
struct ADCRegisters {
    /// Control and status register (0x00)
    cs: ReadPureWrite<u32>, // 0x0
    /// Result register (0x04)
    result: ReadOnly<u32>, // 0x4
    /// Fifo control and status register (0x08)
    fifo_ctrl_status: ReadPureWrite<u32>, // 0x8
    /// Fifo register (0x0c)
    fifo: ReadWrite<u32>, // 0xc
    /// Divider register (0x10)
    div: ReadPureWrite<u32>, // 0x10
    /// Raw interrupt register (0x14)
    raw_inter: ReadPure<u32>, // 0x14
    /// Interrupt enable register (0x18)
    inter_enable: ReadPureWrite<u32>, // 0x18
    /// Force interrupt register (0x1c)
    inter_force: ReadPureWrite<u32>, // 0x1c
    /// Interrupt status register (0x20)
    inter_status: ReadPure<u32> // 0x20
}

/// Analog to Digital Converter control and status register masks and shifts
mod cs_register {
    /// Shift for enabling the Analog to Digital Converter
    pub const ENABLE_SHIFT: usize = 0;
    /// Shift for enabling the Analog to Digital Converter temperature sensor
    pub const TEMP_ENABLE_SHIFT: usize = 1;
    /// Shift for starting a single conversion
    pub const START_ONCE_SHIFT: usize = 2;
    /// Shift for starting many conversions
    pub const START_MANY_SHIFT: usize = 3;
    /// Shift for determining if the Analog to Digital Converter is ready to start a new conversion
    pub const READY_SHIFT: usize = 8;
    /// Shift for if the last result contained an error
    pub const ERROR_SHIFT: usize = 9;
    /// Shift for if a past result contained an error
    pub const ERROR_STICKY_SHIFT: usize = 10;
    /// Shift for selecting the analog input
    pub const ANALOG_MUX_SHIFT: usize = 12;
    /// Round robin sampling shift
    pub const ROUND_ROBIN_SHIFT: usize = 16;

    /// Mask for enabling the Analog to Digital Converter
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Mask for enabling the Analog to Digital Converter temperature sensor
    pub const TEMP_ENABLE_MASK: u32 = 1 << TEMP_ENABLE_SHIFT;
    /// Mask for starting a single conversion
    pub const START_ONCE_MASK: u32 = 1 << START_ONCE_SHIFT;
    /// Mask for starting many conversions
    pub const START_MANY_MASK: u32 = 1 << START_MANY_SHIFT;
    /// Mask for determining if the Analog to Digital Converter is ready to start a new conversion
    pub const READY_MASK: u32 = 1 << READY_SHIFT;
    /// Mask for if the last result contained an error
    pub const ERROR_MASK: u32 = 1 << ERROR_SHIFT;
    /// Mask for if a past result contained an error
    pub const ERROR_STICKY_MASK: u32 = 1 << ERROR_STICKY_SHIFT;
    /// Mask for selecting the analog input
    pub const ANALOG_MUX_MASK: u32 = 0x7 << ANALOG_MUX_SHIFT;
    /// Round robin sampling mask
    pub const ROUND_ROBIN_MASK: u32 = 0x1f << ROUND_ROBIN_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ENABLE_MASK |
        TEMP_ENABLE_MASK |
        START_ONCE_MASK |
        START_MANY_MASK |
        READY_MASK |
        ERROR_MASK |
        ERROR_STICKY_MASK |
        ANALOG_MUX_MASK |
        ROUND_ROBIN_MASK;
}

/// Result register masks and shifts
mod result_register {
    /// Result shift
    pub const RESULT_SHIFT: usize = 0;

    /// Result mask
    pub const RESULT_MASK: u32 = 0xfff << RESULT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = RESULT_MASK;
}

/// Fifo control and status register masks and shifts
mod fifo_ctrl_status_register {
    /// Shift to enable the fifo
    pub const ENABLE_SHIFT: usize = 0;
    /// Shift to enable shifting the results right by 1 byte to make them 1 byte in size
    pub const SHIFT_SHIFT: usize = 1;
    /// Shift for enabling the error bit for conversion errors
    pub const ERROR_SHIFT: usize = 2;
    /// Shift for enabling DMA requests when the fifo has data
    pub const DMA_SHIFT: usize = 3;
    /// Shift for determining if the fifo is empty
    pub const EMPTY_SHIFT: usize = 8;
    /// Shift for determining if the fifo is full
    pub const FULL_SHIFT: usize = 9;
    /// Shift for determining if the fifo has underflowed
    pub const UNDERFLOW_SHIFT: usize = 10;
    /// Shift for determining if the fifo has overflowed
    pub const OVERFLOW_SHIFT: usize = 11;
    /// Shift for determining the number of results in the fifo
    pub const LEVEL_SHIFT: usize = 16;
    /// Shift for the threshold at which the fifo IRQ will be fired
    pub const THRESHOLD_SHIFT: usize = 24;

    /// Shift to enable the fifo
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Shift to enable shifting the results right by 1 byte to make them 1 byte in size
    pub const SHIFT_MASK: u32 = 1 << SHIFT_SHIFT;
    /// Shift for enabling the error bit for conversion errors
    pub const ERROR_MASK: u32 = 1 << ERROR_SHIFT;
    /// Shift for enabling DMA requests when the fifo has data
    pub const DMA_MASK: u32 = 1 << DMA_SHIFT;
    /// Shift for determining if the fifo is empty
    pub const EMPTY_MASK: u32 = 1 << EMPTY_SHIFT;
    /// Shift for determining if the fifo is full
    pub const FULL_MASK: u32 = 1 << FULL_SHIFT;
    /// Shift for determining if the fifo has underflowed
    pub const UNDERFLOW_MASK: u32 = 1 << UNDERFLOW_SHIFT;
    /// Shift for determining if the fifo has overflowed
    pub const OVERFLOW_MASK: u32 = 1 << OVERFLOW_SHIFT;
    /// Shift for determining the number of results in the fifo
    pub const LEVEL_MASK: u32 = 0xf << LEVEL_SHIFT;
    /// Shift for the threshold at which the fifo IRQ will be fired
    pub const THRESHOLD_MASK: u32 = 0xf << THRESHOLD_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ENABLE_MASK |
        SHIFT_MASK |
        ERROR_MASK |
        DMA_MASK |
        EMPTY_MASK |
        FULL_MASK |
        UNDERFLOW_MASK |
        OVERFLOW_MASK |
        LEVEL_MASK |
        THRESHOLD_MASK;
}

/// fifo register masks and shifts
mod fifo_register {
    /// Value shift
    pub const VALUE_SHIFT: usize = 0;
    /// Error shift
    pub const ERROR_SHIFT: usize = 15;

    /// Value mask
    pub const VALUE_MASK: u32 = 0xfff << VALUE_SHIFT;
    /// Error mask
    pub const ERROR_MASK: u32 = 1 << ERROR_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = VALUE_MASK |
        ERROR_MASK;
}

/// divider register masks and shifts
mod div_register {
    /// Fractional part of the divider shift
    pub const FRAC_SHIFT: usize = 0;
    /// Integer part of the divider shift
    pub const INT_SHIFT: usize = 8;

    /// Fractional part of the divider mask
    pub const FRAC_MASK: u32 = 0xff << FRAC_SHIFT;
    /// Integer part of the divider mask
    pub const INT_MASK: u32 = 0xffff << INT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = FRAC_MASK |
        INT_MASK;
}

/// Interrupt registers masks and shifts
mod inter_register {
    /// Fifo interrupt shift
    pub const FIFO_SHIFT: usize = 0;
    
    /// Fifo interrupt mask
    pub const FIFO_MASK: u32 = 1 << FIFO_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = FIFO_MASK;
}

/// Analog to Digital Converter object for managing the Analog to Digital Converter
pub struct ADC {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, ADCRegisters>,
    /// Memory mapped registers where writing a bit clears the corresponding bit in `registers`
    clear_reg: UniqueMmioPointer<'static, ADCRegisters>,
    /// Sample bitmap
    bitmap: u32
}

/// Maximum number of sources
const MAX_SOURCE: u8 = 4;
/// Size of the fifo
const FIFO_SIZE: usize = 8;

impl ADC {
    /// Creates a new `ADC` object  
    /// `base` is the base address of the Analog to Digital Converter memory mapped registers  
    /// `bitmap` is a bitmap of the pins being used by this device
    /// # Safety
    /// `base` must be a valid address which points to the Analog to Digital Converter memory mapped registers and not
    /// being used by anything else
    pub unsafe fn new(adc_base: usize, bitmap: u32) -> Self {
        let mut res = unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(adc_base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(adc_base + REG_ALIAS_CLR_BITS)).unwrap()),
                bitmap
            }
        };
        field!(res.registers, cs).modify(|cs|
            (cs & !cs_register::VALID_MASK) |
            cs_register::ENABLE_MASK | 
            cs_register::TEMP_ENABLE_MASK
        );
        field!(res.registers, div).modify(|div| div & !div_register::VALID_MASK);
        field!(res.registers, inter_enable).modify(|inter_enable| 
            (inter_enable & !inter_register::VALID_MASK) |
            inter_register::FIFO_MASK
        );
        while field!(res.registers, cs).read() & cs_register::READY_MASK == 0 {
            do_yield().unwrap();
        }
        res
    }

    /// Converts an Analog to Digital Converter source into its corresponding GPIO pin  
    /// `src` is the source to convert  
    /// Returns `None` if there is no GPIO pin for that source
    fn src_to_gpio(src: u8) -> Option<u8> {
        assert!(src <= MAX_SOURCE);
        if src < 4 {
            Some(src + 26)
        } else {
            None
        }
    }

    /// Waits while the device is not ready to take more samples
    fn wait_busy(&mut self) {
        while field!(self.registers, cs).read() & cs_register::READY_MASK == 0 {
            do_yield().unwrap();
        }
    }

    /// Reads buffer.len() / 2 samples into buffer  
    /// `buffer` is the buffer to read the samples into  
    /// `src` is the source to read from  
    /// Panics if the buffer is not a multiple of 2 in size or `src` is more than or equal to
    /// `MAX_SOURCE`
    pub fn read_samples(&mut self, buffer: &mut [u8], src: u8) {
        assert!(buffer.len() & 1 == 0);
        assert!(src <= MAX_SOURCE);
        self.wait_busy();
        let mut len = buffer.len() / 2;
        let mut buffer_offset = 0;
        while len > FIFO_SIZE {
            field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
                (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
                fifo_ctrl_status_register::ENABLE_MASK | 
                fifo_ctrl_status_register::ERROR_MASK | 
                fifo_ctrl_status_register::UNDERFLOW_MASK | 
                fifo_ctrl_status_register::OVERFLOW_MASK |
                ((FIFO_SIZE as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
            );
            field!(self.registers, cs).modify(|cs|
                (cs & !cs_register::VALID_MASK) |
                cs_register::ENABLE_MASK |
                cs_register::TEMP_ENABLE_MASK |
                cs_register::START_MANY_MASK |
                ((src as u32) << cs_register::ANALOG_MUX_SHIFT)
            );
            wait_irq().unwrap();
            field!(self.clear_reg, cs).write(cs_register::START_MANY_MASK);
            assert!(buffer_offset + FIFO_SIZE * 2 < buffer.len());
            for _ in 0..FIFO_SIZE {
                let sample = field!(self.registers, fifo).read();
                let error = sample & fifo_register::ERROR_MASK != 0;
                let sample = (sample & fifo_register::VALUE_MASK) >> fifo_register::VALUE_SHIFT;
                let sample = (sample << 1) | (error as u32);
                let sample = (sample as u16).to_le_bytes();
                buffer[buffer_offset] = sample[0];
                buffer[buffer_offset + 1] = sample[1];
                buffer_offset += 2;
            }
            len -= FIFO_SIZE;
        }
        field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
            (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
            fifo_ctrl_status_register::ENABLE_MASK | 
            fifo_ctrl_status_register::ERROR_MASK | 
            fifo_ctrl_status_register::UNDERFLOW_MASK | 
            fifo_ctrl_status_register::OVERFLOW_MASK |
            ((len as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
        );
        field!(self.registers, cs).modify(|cs|
            (cs & !cs_register::VALID_MASK) |
            cs_register::ENABLE_MASK |
            cs_register::TEMP_ENABLE_MASK |
            cs_register::START_MANY_MASK |
            ((src as u32) << cs_register::ANALOG_MUX_SHIFT)
        );
        wait_irq().unwrap();
        field!(self.clear_reg, cs).write(cs_register::START_MANY_MASK);
        assert!(buffer_offset + len * 2 == buffer.len());
        for _ in 0..len {
            let sample = field!(self.registers, fifo).read();
            let error = sample & fifo_register::ERROR_MASK != 0;
            let sample = (sample & fifo_register::VALUE_MASK) >> fifo_register::VALUE_SHIFT;
            let sample = (sample << 1) | (error as u32);
            let sample = sample.to_le_bytes();
            buffer[buffer_offset] = sample[0];
            buffer[buffer_offset + 1] = sample[1];
            buffer_offset += 2;
        }
    }

    /// Reads buffer.len() / 2 samples into buffer in a round robin fashion
    /// `buffer` is the buffer to read the samples into  
    /// `samples` is a list of the sources to read from  
    /// `start` is the source to start with  
    /// Panics if the buffer is not a multiple of 2 in size, `samples` has a length of more than 4
    /// items or `start` is greater than or equal to the `samples` length
    /// `MAX_SOURCE`
    pub fn read_samples_round_robin(&mut self, buffer: &mut [u8], samples: &[u8], mut start: usize) {
        assert!(samples.len() < 5);
        assert!(buffer.len() & 1 == 0);
        assert!(start < samples.len());
        let mut bitmap = 0;
        for sample in samples {
            assert!(*sample <= MAX_SOURCE);
            bitmap |= 1 << sample;
        }
        if bitmap == 0 {
            // empty bitmap means no reading
            return;
        }
        self.wait_busy();
        let mut len = buffer.len() / 2;
        let mut buffer_offset = 0;
        while len > FIFO_SIZE {
            field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
                (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
                fifo_ctrl_status_register::ENABLE_MASK | 
                fifo_ctrl_status_register::ERROR_MASK | 
                fifo_ctrl_status_register::UNDERFLOW_MASK | 
                fifo_ctrl_status_register::OVERFLOW_MASK |
                ((FIFO_SIZE as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
            );
            field!(self.registers, cs).modify(|cs|
                (cs & !cs_register::VALID_MASK) |
                cs_register::ENABLE_MASK |
                cs_register::TEMP_ENABLE_MASK |
                cs_register::START_MANY_MASK |
                ((samples[start] as u32) << cs_register::ANALOG_MUX_SHIFT) |
                ((bitmap as u32) << cs_register::ROUND_ROBIN_SHIFT)
            );
            wait_irq().unwrap();
            field!(self.clear_reg, cs).write(cs_register::START_MANY_MASK);
            assert!(buffer_offset + FIFO_SIZE * 2 < buffer.len());
            for _ in 0..FIFO_SIZE {
                let sample = field!(self.registers, fifo).read();
                let error = sample & fifo_register::ERROR_MASK != 0;
                let sample = (sample & fifo_register::VALUE_MASK) >> fifo_register::VALUE_SHIFT;
                let sample = (sample << 1) | (error as u32);
                let sample = (sample as u16).to_le_bytes();
                buffer[buffer_offset] = sample[0];
                buffer[buffer_offset + 1] = sample[1];
                buffer_offset += 2;
            }
            start = (start + FIFO_SIZE) % (samples.len());
            len -= FIFO_SIZE;
        }
        field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
            (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
            fifo_ctrl_status_register::ENABLE_MASK | 
            fifo_ctrl_status_register::ERROR_MASK | 
            fifo_ctrl_status_register::UNDERFLOW_MASK | 
            fifo_ctrl_status_register::OVERFLOW_MASK |
            ((len as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
        );
        field!(self.registers, cs).modify(|cs|
            (cs & !cs_register::VALID_MASK) |
            cs_register::ENABLE_MASK |
            cs_register::TEMP_ENABLE_MASK |
            cs_register::START_MANY_MASK |
            ((samples[start] as u32) << cs_register::ANALOG_MUX_SHIFT) |
            ((bitmap as u32) << cs_register::ROUND_ROBIN_SHIFT)
        );
        wait_irq().unwrap();
        field!(self.clear_reg, cs).write(cs_register::START_MANY_MASK);
        assert!(buffer_offset + len * 2 == buffer.len());
        for _ in 0..len {
            let sample = field!(self.registers, fifo).read();
            let error = sample & fifo_register::ERROR_MASK != 0;
            let sample = (sample & fifo_register::VALUE_MASK) >> fifo_register::VALUE_SHIFT;
            let sample = (sample << 1) | (error as u32);
            let sample = sample.to_le_bytes();
            buffer[buffer_offset] = sample[0];
            buffer[buffer_offset + 1] = sample[1];
            buffer_offset += 2;
        }
    }

    /// Reads a single sample from a source  
    /// `src` is the source to read from  
    /// Returns the read sample  
    /// Panics if `src` is greater than or equal to `MAX_SOURCE`
    pub fn read_sample(&mut self, src: u8) -> u16 {
        assert!(src <= MAX_SOURCE);
        self.wait_busy();
        field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
            (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
            fifo_ctrl_status_register::ENABLE_MASK | 
            fifo_ctrl_status_register::ERROR_MASK | 
            fifo_ctrl_status_register::UNDERFLOW_MASK | 
            fifo_ctrl_status_register::OVERFLOW_MASK |
            ((1 as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
        );
        field!(self.registers, cs).modify(|cs|
            (cs & !cs_register::VALID_MASK) |
            cs_register::ENABLE_MASK |
            cs_register::TEMP_ENABLE_MASK |
            cs_register::START_ONCE_MASK |
            ((src as u32) << cs_register::ANALOG_MUX_SHIFT)
        );
        wait_irq().unwrap();
        let sample = field!(self.registers, fifo).read();
        let error = sample & fifo_register::ERROR_MASK != 0;
        let sample = (sample & fifo_register::VALUE_MASK) >> fifo_register::VALUE_SHIFT;
        let sample = (sample << 1) | (error as u32);
        sample as u16
    }
}

/// Analog to Digital Converter reply errors
pub enum ADCReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// An invalid source was selected
    InvalidSource,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer,
    /// An invalid start source was selected
    InvalidStart
}

/// Converts a `ADCReplyError` to a `u32`
impl From<ADCReplyError> for u32 {
    fn from(value: ADCReplyError) -> Self {
        match value {
            ADCReplyError::SendError => 1,
            ADCReplyError::InvalidRequest => 2,
            ADCReplyError::InvalidSource => 3,
            ADCReplyError::InvalidSendBuffer => 4,
            ADCReplyError::InvalidReplyBuffer => 5,
            ADCReplyError::InvalidStart => 6
        }
    }
}

/// Converts a `HeaderError` to an `ADCReplyError`
impl From<HeaderError> for ADCReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer,
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer
        }
    }
}

/// ADC Errors
pub enum ADCError {
    /// Error with the request
    ReplyError(ADCReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts an `ADCReplyError` to an `ADCError`
impl From<ADCReplyError> for ADCError {
    fn from(value: ADCReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts a `HeaderError` to an `ADCError`
impl From<HeaderError> for ADCError {
    fn from(value: HeaderError) -> Self {
        Self::from(ADCReplyError::from(value))
    }
}

/// Converts a `QueueError` to an `ADCError`
impl From<QueueError> for ADCError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// Maximum sample read buffer
const BUFFER_LEN: usize = 32;

/// Reading buffer
pub struct ReadBuffered {
    /// Buffer to use
    buffer: [u8; BUFFER_LEN],
    /// Length of buffer
    len: usize
}

/// Reading a single source multiple times
pub struct ReadMultiple {
    /// Source to read from
    src: u8,
    /// Buffer to store samples in
    buffer: ReadBuffered
}

/// Reading multiple sources in a round robin fashion
pub struct ReadRoundRobin {
    /// First sample to read
    start: usize,
    /// Bitmap of which sources to read from
    bitmap: u8,
    /// Buffer to store the samples in
    buffer: ReadBuffered
}

/// Analog to Digital Converter request
pub enum Request {
    /// Single read of a single source
    ReadSingle(u8),
    /// Reading multiple samples from a single source
    ReadMultiple(ReadMultiple),
    /// Reading multiple samples from multiple sources in a round robin fashion
    ReadRoundRobin(ReadRoundRobin),
}

impl Request {
    /// Parses the next request  
    /// `bitmap` is the GPIO bitmap for the Analog to Digital Connverter  
    /// Returns the request on success or an `ADCError` on failure
    pub fn parse(bitmap: u32) -> Result<Self, ADCError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // Read single
                check_header_len(&header, 1, 2)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let src = buffer[0];
                if src > MAX_SOURCE {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                if let Some(gpio) = ADC::src_to_gpio(src) && (1 << gpio) & bitmap == 0 {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                Ok(Request::ReadSingle(src))
            },
            1 => {
                // Read multiple
                if header.send_len != 1 {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSendBuffer));
                }
                if !header.reply_len.is_multiple_of(2) || header.reply_len as usize > BUFFER_LEN {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSendBuffer));
                }
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let src = buffer[0];
                let len = header.reply_len as usize;
                if src > MAX_SOURCE {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                if let Some(gpio) = ADC::src_to_gpio(src) && (1 << gpio) & bitmap == 0 {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                Ok(Request::ReadMultiple(ReadMultiple { 
                    src, 
                    buffer: ReadBuffered {
                        buffer: [0; BUFFER_LEN],
                        len
                    }
                }))
            },
            2 => {
                // Read round robin
                if header.send_len != 2 {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSendBuffer));
                }
                if !header.reply_len.is_multiple_of(2) || header.reply_len as usize > BUFFER_LEN {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSendBuffer));
                }
                let mut buffer = [0; 2];
                _ = receive(0, &mut buffer)?;
                let mask = buffer[0];
                let start = buffer[1] as usize;
                let len = header.reply_len as usize;
                if mask > 0x1f {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                let mut m = mask;
                let mut src = 0;
                while m > 0 {
                    if m & 1 != 0 {
                        if let Some(gpio) = ADC::src_to_gpio(src) && (1 << gpio) & bitmap == 0 {
                            return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                        }
                    }
                    m >>= 1;
                    src += 1;
                }
                if start >= mask.count_ones() as usize {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidStart));
                }
                Ok(Request::ReadRoundRobin(ReadRoundRobin { 
                    start, 
                    bitmap: mask, 
                    buffer: ReadBuffered { 
                        buffer: [0; BUFFER_LEN], 
                        len 
                    }
                }))
            },
            _ => {
                Err(ADCError::ReplyError(ADCReplyError::InvalidRequest))
            }
        }
    }
}

/// Endpoint for the IO Bank 0 driver
const IO_BANK0_QUEUE: u32 = 0;

/// Driver entry point
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 2);
    // check func sel has been set up
    send_empty(IO_BANK0_QUEUE, 0, &[]).unwrap();
    #[cfg(test)]
    {
        test_main();
    }
    let adc_base = args[0] as usize;
    let bitmap = args[1];
    let mut adc = unsafe {
        ADC::new(adc_base, bitmap)
    };
    loop {
        match Request::parse(adc.bitmap) {
            Ok(request) => {
                match request {
                    Request::ReadSingle(src) => {
                        let sample = adc.read_sample(src).to_le_bytes();
                        check_critical(reply(0, 0, &sample)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::ReadMultiple(mut read_multiple) => {
                        adc.read_samples(&mut read_multiple.buffer.buffer[..read_multiple.buffer.len], read_multiple.src);
                        check_critical(reply(0, 0, &read_multiple.buffer.buffer[..read_multiple.buffer.len])).unwrap_or(Ok(())).unwrap();
                    },
                    Request::ReadRoundRobin(mut round_robin) => {
                        let mut samples = [0; 5];
                        let mut sample_len = 0;
                        let mut mask = round_robin.bitmap;
                        let mut sample: u8 = 0;
                        while mask > 0 {
                            if mask & 1 != 0 {
                                samples[sample_len] = sample;
                                sample_len += 1;
                            }
                            sample += 1;
                            mask >>= 1;
                        }
                        adc.read_samples_round_robin(&mut round_robin.buffer.buffer[..round_robin.buffer.len], &samples[..sample_len], round_robin.start);
                        check_critical(reply(0, 0, &round_robin.buffer.buffer[..round_robin.buffer.len])).unwrap_or(Ok(())).unwrap();
                    }
                }
            },
            Err(err) => {
                match err {
                    ADCError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    ADCError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(ADCReplyError::SendError))).unwrap_or(Ok(())).unwrap();
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
        kprintln!("Running {} tests for ADC", tests.len());
        for test in tests {
            test();
        }
    }

    #[test_case]
    fn test_setup() {
        let args = args();
        let adc_base = args[0] as usize;
        let bitmap = args[1];
        let mut adc = unsafe {
            ADC::new(adc_base, bitmap)
        };
        kprintln!("Testing ADC setup");
        kprint!("Testing cs register ");
        let cs = field!(adc.registers, cs).read();
        assert_eq!(cs & cs_register::VALID_MASK, 0x103);
        kprintln!("[ok]");
        kprint!("Testing div register ");
        let div = field!(adc.registers, div).read();
        assert_eq!(div & div_register::VALID_MASK, 0);
        kprintln!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        kprintln!("Testing ADC register mask values");
        kprint!("Testing cs register ");
        assert_eq!(cs_register::VALID_MASK, 0x1f770f);
        kprintln!("[ok]");
        kprint!("Testing result register ");
        assert_eq!(result_register::VALID_MASK, 0xfff);
        kprintln!("[ok]");
        kprint!("Testing fifo ctrl status register ");
        assert_eq!(fifo_ctrl_status_register::VALID_MASK, 0xf0f0f0f);
        kprintln!("[ok]");
        kprint!("Testing fifo register ");
        assert_eq!(fifo_register::VALID_MASK, 0x8fff);
        kprintln!("[ok]");
        kprint!("Testing div register ");
        assert_eq!(div_register::VALID_MASK, 0xffffff);
        kprintln!("[ok]");
        kprint!("Testing interrupt register ");
        assert_eq!(inter_register::VALID_MASK, 0x1);
        kprintln!("[ok]");
    }
}
