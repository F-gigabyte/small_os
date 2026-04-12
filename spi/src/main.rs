/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS SPI driver.
 *
 * The SmallOS SPI driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS SPI driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS SPI driver. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

/* File based on the I2C implementation in the raspberry pi pico SDK at https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/i2c.c (accessed 25/03/2026)
 * under the license
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, ReadWrite, WriteOnly}};
use small_os_lib::{QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, args, check_critical, do_yield, read_header, receive, reply, reply_empty, send_empty, wait_irq};

/// Control register 0 masks and shifts
mod ctrl0_register {
    /// Data size select shift
    pub const DATA_SIZE_SHIFT: usize = 0;
    /// Frame format shift
    pub const FRAME_FORMAT_SHIFT: usize = 4;
    /// Output clock polarity shift
    pub const POLARITY_SHIFT: usize = 6;
    /// Output clock phase shift
    pub const PHASE_SHIFT: usize = 7;
    /// Serial clock rate shift
    pub const SERIAL_CLOCK_SHIFT: usize = 8;

    /// Data size select mask
    pub const DATA_SIZE_MASK: u32 = 0xf << DATA_SIZE_SHIFT;
    /// Frame format mask
    pub const FRAME_FORMAT_MASK: u32 = 0x3 << FRAME_FORMAT_SHIFT;
    /// Output clock polarity mask
    pub const POLARITY_MASK: u32 = 1 << POLARITY_SHIFT;
    /// Output clock phase mask
    pub const PHASE_MASK: u32 = 1 << PHASE_SHIFT;
    /// Serial clock rate mask
    pub const SERIAL_CLOCK_MASK: u32 = 0xff << SERIAL_CLOCK_SHIFT;

    /// 4 bit data size
    pub const DATA_SIZE_4BIT: u32 = 0x3 << DATA_SIZE_SHIFT;
    /// 5 bit data size
    pub const DATA_SIZE_5BIT: u32 = 0x4 << DATA_SIZE_SHIFT;
    /// 6 bit data size
    pub const DATA_SIZE_6BIT: u32 = 0x5 << DATA_SIZE_SHIFT;
    /// 7 bit data size
    pub const DATA_SIZE_7BIT: u32 = 0x6 << DATA_SIZE_SHIFT;
    /// 8 bit data size
    pub const DATA_SIZE_8BIT: u32 = 0x7 << DATA_SIZE_SHIFT;
    /// 9 bit data size
    pub const DATA_SIZE_9BIT: u32 = 0x8 << DATA_SIZE_SHIFT;
    /// 10 bit data size
    pub const DATA_SIZE_10BIT: u32 = 0x9 << DATA_SIZE_SHIFT;
    /// 11 bit data size
    pub const DATA_SIZE_11BIT: u32 = 0xa << DATA_SIZE_SHIFT;
    /// 12 bit data size
    pub const DATA_SIZE_12BIT: u32 = 0xb << DATA_SIZE_SHIFT;
    /// 13 bit data size
    pub const DATA_SIZE_13BIT: u32 = 0xc << DATA_SIZE_SHIFT;
    /// 14 bit data size
    pub const DATA_SIZE_14BIT: u32 = 0xd << DATA_SIZE_SHIFT;
    /// 15 bit data size
    pub const DATA_SIZE_15BIT: u32 = 0xe << DATA_SIZE_SHIFT;
    /// 16 bit data size
    pub const DATA_SIZE_16BIT: u32 = 0xf << DATA_SIZE_SHIFT;

    /// Motorola SPI format
    pub const FRAME_FORMAT_NORMAL: u32 = 0 << FRAME_FORMAT_SHIFT;
    /// Texas Instruments SPI format
    pub const FRAME_FORMAT_TI: u32 = 1 << FRAME_FORMAT_SHIFT;
    /// National semiconductor microwire frame format
    pub const FRAME_FORMAT_NATIONAL_MICROWIRE: u32 = 2 << FRAME_FORMAT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = DATA_SIZE_MASK |
        FRAME_FORMAT_MASK |
        POLARITY_MASK |
        PHASE_MASK |
        SERIAL_CLOCK_MASK;
}

/// Control register 1 masks and shifts
mod ctrl1_register {
    /// Shift to enable loop back mode
    pub const LOOP_BACK_SHIFT: usize = 0;
    /// Shift to enable SPI
    pub const ENABLE_SHIFT: usize = 1;
    /// Shift to enable SPI in slave mode
    pub const SLAVE_SHIFT: usize = 2;
    /// Shift to disable output in slave mode
    pub const SLAVE_OUTPUT_SHIFT: usize = 3;

    /// Mask to enable loop back mode
    pub const LOOP_BACK_MASK: u32 = 1 << LOOP_BACK_SHIFT;
    /// Mask to enable SPI
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Mask to enable SPI in slave mode
    pub const SLAVE_MASK: u32 = 1 << SLAVE_SHIFT;
    /// Mask to disable output in slave mode
    pub const SLAVE_OUTPUT_MASK: u32 = 1 << SLAVE_OUTPUT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = LOOP_BACK_MASK |
        ENABLE_MASK |
        SLAVE_MASK |
        SLAVE_OUTPUT_MASK;
}

/// Data register masks and shifts
mod data_register {
    /// Data shift
    pub const DATA_SHIFT: usize = 0;

    /// Data mask
    pub const DATA_MASK: u32 = 0xffff << DATA_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = DATA_MASK;
}

/// Status register masks and shifts
mod status_register {
    /// Shift for TX buffer not empty
    pub const TX_EMPTY_SHIFT: usize = 0;
    /// Shift for TX buffer not full
    pub const TX_NOT_FULL_SHIFT: usize = 1;
    /// Shift for RX buffer not empty
    pub const RX_NOT_EMPTY_SHIFT: usize = 2;
    /// Shift for RX buffer full
    pub const RX_FULL_SHIFT: usize = 3;
    /// Busy shift
    pub const BUSY_SHIFT: usize = 4;

    /// Mask for TX buffer not empty
    pub const TX_EMPTY_MASK: u32 = 1 << TX_EMPTY_SHIFT;
    /// Mask for TX buffer not full
    pub const TX_NOT_FULL_MASK: u32 = 1 << TX_NOT_FULL_SHIFT;
    /// Mask for RX buffer not empty
    pub const RX_NOT_EMPTY_MASK: u32 = 1 << RX_NOT_EMPTY_SHIFT;
    /// Mask for RX buffer full
    pub const RX_FULL_MASK: u32 = 1 << RX_FULL_SHIFT;
    /// Busy mask
    pub const BUSY_MASK: u32 = 1 << BUSY_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = TX_EMPTY_MASK |
        TX_NOT_FULL_MASK |
        RX_NOT_EMPTY_MASK |
        RX_FULL_MASK |
        BUSY_MASK;
}

/// Divider register masks and shifts
mod div_register {
    /// Divider shift
    pub const DIV_SHIFT: usize = 0;
    
    /// Divider mask
    pub const DIV_MASK: u32 = 0xff << DIV_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = DIV_MASK;
}

/// Interrupt registers masks and shifts
mod inter_register {
    /// RX fifo overflowed interrupt shift
    pub const RX_OVERRUN_SHIFT: usize = 0;
    /// RX timedout interrupt shift
    pub const RX_TIMEOUT_SHIFT: usize = 1;
    /// RX fifo half full or more shift
    pub const RX_FIFO_SHIFT: usize = 2;
    /// TX fifo half empty or less shift
    pub const TX_FIFO_SHIFT: usize = 3;

    /// RX fifo overflowed interrupt mask
    pub const RX_OVERRUN_MASK: u32 = 1 << RX_OVERRUN_SHIFT;
    /// RX timed out interrupt mask
    pub const RX_TIMEOUT_MASK: u32 = 1 << RX_TIMEOUT_SHIFT;
    /// RX fifo half full or more mask
    pub const RX_FIFO_MASK: u32 = 1 << RX_FIFO_SHIFT;
    /// TX fifo half empty or less mask
    pub const TX_FIFO_MASK: u32 = 1 << TX_FIFO_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = RX_OVERRUN_MASK | 
        RX_TIMEOUT_MASK | 
        RX_FIFO_MASK | 
        TX_FIFO_MASK;
}

/// Interrupt clear register masks and shifts
mod inter_clear_register {
    /// Shift to clear RX overrun interrupt
    pub const RX_OVERRUN_SHIFT: usize = 0;
    /// Shift to clear RX timeout interrupt
    pub const RX_TIMEOUT_SHIFT: usize = 1;
    
    /// Mask to clear RX overrun interrupt
    pub const RX_OVERRUN_MASK: u32 = 1 << RX_OVERRUN_SHIFT;
    /// Mask to clear RX timeout interrupt
    pub const RX_TIMEOUT_MASK: u32 = 1 << RX_TIMEOUT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = RX_OVERRUN_MASK | 
        RX_TIMEOUT_MASK;
}

/// DMA register masks and shifts
mod dma_register {
    /// Shift to enable receive DMA
    pub const RX_DMA_SHIFT: usize = 0;
    /// Shift to enable transmit DMA
    pub const TX_DMA_SHIFT: usize = 1;

    /// Mask to enable receive DMA
    pub const RX_DMA_MASK: u32 = 1 << RX_DMA_SHIFT;
    /// Mask to enable transmit DMA
    pub const TX_DMA_MASK: u32 = 1 << TX_DMA_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = RX_DMA_MASK |
        TX_DMA_MASK;
}

/// SPI memory mapped registers
#[repr(C)]
struct SPIRegisters {
    /// Control register 0 (0x00)
    ctrl0: ReadPureWrite<u32>, // 0x0
    /// Control register 1 (0x04)
    ctrl1: ReadPureWrite<u32>, // 0x4
    /// Data register (0x08)
    data: ReadWrite<u32>, // 0x8
    /// Status register (0x0c)
    status: ReadPure<u32>, // 0xc
    /// Divider register (0x10)
    div: ReadPureWrite<u32>, // 0x10
    /// Interrupt mask set clear register (0x14)
    inter_mask_set_clear: ReadPureWrite<u32>, // 0x14
    /// Raw interrupt status register (0x18)
    raw_inter: ReadPure<u32>, // 0x18
    /// Masked interrupt status register (0x1c)
    mask_inter: ReadPure<u32>, // 0x1c
    /// Interrupt clear register (0x20)
    inter_clear: WriteOnly<u32>, // 0x20
    /// DMA register (0x24)
    dma: ReadPureWrite<u32>, // 0x24
}

/// SPI object to manage an SPI device
pub struct SPI {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, SPIRegisters>,
    /// Memory mapped registers where writing a bit sets the corresponding bit in `registers`
    set_reg: UniqueMmioPointer<'static, SPIRegisters>,
    /// Memory mapped registers where writing a bit sets the corresponding bit in `registers`
    clear_reg: UniqueMmioPointer<'static, SPIRegisters>,
}

impl SPI {
    /// Creates a new `SPI` object  
    /// `spi_base` is the base address of the SPI memory mapped registers
    /// # Safety
    /// `spi_base` must be a valid address which points to an SPI memory mapped registers and not
    /// being used by anything else
    pub unsafe fn new(spi_base: usize) -> Self {
        let mut res = unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(spi_base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(spi_base + REG_ALIAS_SET_BITS)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(spi_base + REG_ALIAS_CLR_BITS)).unwrap())
            }
        };
        while field!(res.registers, status).read() & status_register::BUSY_MASK != 0 {
            do_yield().unwrap();
        }
        field!(res.registers, ctrl1).modify(|ctrl1| ctrl1 & !ctrl1_register::VALID_MASK);
        field!(res.registers, ctrl0).modify(|ctrl0| ctrl0 & !ctrl0_register::VALID_MASK);
        field!(res.registers, ctrl0).modify(|ctrl0|
            (ctrl0 & !ctrl0_register::VALID_MASK) |
            ctrl0_register::DATA_SIZE_8BIT |
            ctrl0_register::FRAME_FORMAT_NORMAL | 
            (2 << ctrl0_register::SERIAL_CLOCK_SHIFT)
        );
        field!(res.registers, div).modify(|div|
            (div & !div_register::VALID_MASK) |
            (2 << div_register::DIV_SHIFT)
        );
        field!(res.registers, inter_mask_set_clear).modify(|inter_mask| inter_mask & !inter_clear_register::VALID_MASK);
        field!(res.registers, inter_clear).write(inter_clear_register::VALID_MASK);
        field!(res.registers, dma).modify(|dma| dma & !dma_register::VALID_MASK);
        res.cs_high();
        res
    }

    /// Forces the SPI slave select signal to be high
    fn cs_high(&mut self) {
        send_empty(IO_BANK0_QUEUE, DRIVE_HIGH, &[CS_PIN]).unwrap();
    }

    /// Forces the SPI slave select signal to be low
    fn cs_low(&mut self) {
        send_empty(IO_BANK0_QUEUE, DRIVE_LOW, &[CS_PIN]).unwrap();
    }

    /// Waits while the SPI device is busy
    fn wait_busy(&mut self) {
        while field!(self.registers, status).read() & status_register::BUSY_MASK != 0 {
            do_yield().unwrap();
        }
    }

    /// Sends and receives data over SPI  
    /// `buffer` initially contains the data to send and ends up containing the received data  
    /// `tx_len` is the number of bytes to transmit  
    /// `rx_len` is the number of bytes to receive  
    /// `padding` is the number of padding bytes to transmit after the transmit data  
    /// `delay` is the number of delay bytes to wait before putting the received data into the
    /// buffer  
    /// Panics if `tx_len` or `rx_len` is greater than the buffer length
    pub fn send(&mut self, buffer: &mut [u8], tx_len: usize, rx_len: usize, padding: usize, delay: usize) -> Result<(), SPIReplyError> {
        assert!(tx_len <= buffer.len());
        assert!(rx_len <= buffer.len());
        self.wait_busy();
        field!(self.clear_reg, ctrl1).write(ctrl1_register::ENABLE_MASK);
        field!(self.registers, inter_clear).write(inter_register::RX_OVERRUN_MASK);
        let send_len = tx_len + padding;
        let receive_len = delay + rx_len;
        let len = send_len.max(receive_len);
        let mut tx_index = 0;
        let mut rx_index = 0;
        while field!(self.registers, status).read() & status_register::TX_NOT_FULL_MASK != 0 && tx_index < len {
            if tx_index < tx_len {
                field!(self.registers, data).write(buffer[tx_index] as u32);
            } else {
                field!(self.registers, data).write(0);
            }
            tx_index += 1;
        }
        let mut tx_inter = 0;
        if tx_index < len {
            tx_inter = inter_register::TX_FIFO_MASK;
        }
        field!(self.registers, inter_mask_set_clear).modify(|inter_mask|
            (inter_mask & !inter_register::VALID_MASK) |
            inter_register::RX_FIFO_MASK | 
            inter_register::RX_OVERRUN_MASK | 
            tx_inter
        );
        self.cs_low();
        field!(self.set_reg, ctrl1).write(ctrl1_register::ENABLE_MASK);
        while rx_index < len.saturating_sub(4) {
            wait_irq().unwrap();
            let inter = field!(self.registers, mask_inter).read();
            if inter & inter_register::RX_OVERRUN_MASK != 0 {
                self.cs_high();
                return Err(SPIReplyError::ReceieveOverrun);
            }
            let mut tx_pop = 0;
            if inter & inter_register::RX_FIFO_MASK != 0 {
                while field!(self.registers, status).read() & status_register::RX_NOT_EMPTY_MASK != 0 {
                    if rx_index >= delay && rx_index - delay < rx_len {
                        buffer[rx_index - delay] = (field!(self.registers, data).read() & 0xff) as u8;
                    } else {
                        _ = field!(self.registers, data).read();
                    }
                    rx_index += 1;
                    tx_pop += 1;
                }
            }
            if field!(self.registers, mask_inter).read() & inter_register::TX_FIFO_MASK != 0 {
                if tx_index == len {
                    field!(self.registers, inter_mask_set_clear).modify(|inter_mask|
                        (inter_mask & !inter_register::VALID_MASK) |
                        inter_register::RX_FIFO_MASK | 
                        inter_register::RX_OVERRUN_MASK
                    );
                } else {
                    loop {
                        let status = field!(self.registers, status).read();
                        if tx_index < len && status & status_register::TX_NOT_FULL_MASK != 0 && status & status_register::RX_FULL_MASK == 0 && tx_pop > 0 {
                            if tx_index < tx_len {
                                field!(self.registers, data).write(buffer[tx_index] as u32);
                            } else {
                                field!(self.registers, data).write(0);
                            }
                            tx_index += 1;
                            tx_pop -= 1;
                        } else {
                            break;
                        }
                    }
                }
            }
        }
        while rx_index < len {
            let status = field!(self.registers, status).read();
            let mut prog = false;
            let mut tx_pop = 0;
            if field!(self.registers, status).read() & status_register::RX_NOT_EMPTY_MASK != 0{
                if rx_index >= delay && rx_index - delay < rx_len {
                    buffer[rx_index - delay] = (field!(self.registers, data).read() & 0xff) as u8;
                } else {
                    _ = field!(self.registers, data).read();
                }
                prog = true;
                tx_pop += 1;
                rx_index += 1;
            }
            if tx_index < len {
                if status & status_register::TX_NOT_FULL_MASK != 0 && tx_pop > 0 {
                    if tx_index < tx_len {
                        field!(self.registers, data).write(buffer[tx_index] as u32);
                    } else {
                        field!(self.registers, data).write(0);
                    }
                    tx_index += 1;
                }
            } else if !prog {
                do_yield().unwrap();
            }
        }
        self.cs_high();
        // clear out extra data
        loop {
            let status = field!(self.registers, status).read();
            if status & status_register::RX_NOT_EMPTY_MASK != 0 {
                _ = field!(self.registers, data).read();
            } else {
                break;
            }
        }
        Ok(())
    }
}

/// SPI clock frequency
const CLOCK_FREQ: u32 = 12000000;
/// SPI frequency
const SPI_FREQ: u32 = 6000000;

/// IO Bank 0 driver endpoint
const IO_BANK0_QUEUE: u32 = 0;

/// Slave select pin relative to driver pin offsets
const CS_PIN: u8 = 1;

/// Command to drive the slave select pin high
const DRIVE_HIGH: u16 = 2;
/// Command to drive the slave select pin low
const DRIVE_LOW: u16 = 3;

/// SPI reply errors
pub enum SPIReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// The receive buffer overflowed
    ReceieveOverrun,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Converts from a `SPIReplyError` to a `u32`
impl From<SPIReplyError> for u32 {
    fn from(value: SPIReplyError) -> Self {
        match value {
            SPIReplyError::SendError => 1,
            SPIReplyError::InvalidRequest => 2,
            SPIReplyError::ReceieveOverrun => 3,
            SPIReplyError::InvalidSendBuffer => 4,
            SPIReplyError::InvalidReplyBuffer => 5
        }
    }
}

/// SPI Errors
pub enum SPIError {
    /// Error with the request
    ReplyError(SPIReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts from a `SPIReplyError` to a `SPIError`
impl From<SPIReplyError> for SPIError {
    fn from(value: SPIReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts from a `QueueError` to a `SPIError`
impl From<QueueError> for SPIError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// An SPI Request
pub struct Request {
    /// Request delay
    delay: usize,
    /// Request padding
    padding: usize,
    /// Request transmit buffer length
    tx_len: usize,
    /// Request receive buffer length
    rx_len: usize,
    /// Request buffer
    buffer: [u8; 1024],
}

impl Request {
    /// Parses the next request  
    /// Returns the request on success or a `SPIError` on failure
    pub fn parse() -> Result<Self, SPIError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // synchronous
                let mut req_buffer = [0; 1024];
                if header.send_len as usize > req_buffer.len() {
                    return Err(SPIError::ReplyError(SPIReplyError::InvalidSendBuffer));
                }
                if header.reply_len as usize > req_buffer.len() {
                    return Err(SPIError::ReplyError(SPIReplyError::InvalidReplyBuffer));
                }
                _ = receive(0, &mut req_buffer)?;
                Ok(Request { 
                    delay: header.send_len as usize, 
                    padding: 0, 
                    tx_len: header.send_len as usize, 
                    rx_len: header.reply_len as usize, 
                    buffer: req_buffer
                })
            },
            1 => {
                // asynchronous
                let mut req_buffer = [0; 1028];
                if header.send_len < 8 || header.send_len as usize > req_buffer.len() {
                    return Err(SPIError::ReplyError(SPIReplyError::InvalidSendBuffer));
                }
                if header.reply_len as usize > req_buffer.len() - 8 {
                    return Err(SPIError::ReplyError(SPIReplyError::InvalidReplyBuffer));
                }
                _ = receive(0, &mut req_buffer)?;
                let mut buffer = [0; 1024];
                buffer[..header.send_len as usize - 8].copy_from_slice(&req_buffer[8..header.send_len as usize]);
                Ok(Request {
                    delay: u32::from_le_bytes(req_buffer[0..4].try_into().unwrap()) as usize,
                    padding: u32::from_le_bytes(req_buffer[4..8].try_into().unwrap()) as usize,
                    tx_len: (header.send_len - 8) as usize,
                    rx_len: header.reply_len as usize,
                    buffer
                })
            },
            _ => {
                Err(SPIError::ReplyError(SPIReplyError::InvalidRequest))
            }
        }
    }
}

/// Driver entry point
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 2);
    send_empty(IO_BANK0_QUEUE, 0, &[]).unwrap();
    #[cfg(test)]
    test_main();
    let spi_base = args[0] as usize;
    let mut spi = unsafe {
        SPI::new(spi_base)
    };
    loop {
        match Request::parse() {
            Ok(mut request) => {
                match spi.send(&mut request.buffer, request.tx_len, request.rx_len, request.padding, request.delay) {
                    Ok(()) => {
                        check_critical(reply(0, 0, &request.buffer[..request.rx_len])).unwrap_or(Ok(())).unwrap();
                    },
                    Err(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    }
                }
            },
            Err(err) => {
                match err {
                    SPIError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    SPIError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(SPIReplyError::SendError))).unwrap_or(Ok(())).unwrap();
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
        kprintln!("Running {} tests for SPI", tests.len());
        for test in tests {
            test();
        }
    }

    #[test_case]
    fn test_valid() {
        kprintln!("Testing SPI register mask values");
        kprint!("Testing ctrl0 register ");
        assert_eq!(ctrl0_register::VALID_MASK, 0xffff);
        kprintln!("[ok]");
        kprint!("Testing ctrl1 register ");
        assert_eq!(ctrl1_register::VALID_MASK, 0xf);
        kprintln!("[ok]");
        kprint!("Testing data register ");
        assert_eq!(data_register::VALID_MASK, 0xffff);
        kprintln!("[ok]");
        kprint!("Testing status register ");
        assert_eq!(status_register::VALID_MASK, 0x1f);
        kprintln!("[ok]");
        kprint!("Testing div register ");
        assert_eq!(div_register::VALID_MASK, 0xff);
        kprintln!("[ok]");
        kprint!("Testing inter register ");
        assert_eq!(inter_register::VALID_MASK, 0xf);
        kprintln!("[ok]");
        kprint!("Testing inter clear register ");
        assert_eq!(inter_clear_register::VALID_MASK, 0x3);
        kprintln!("[ok]");
        kprint!("Testing dma register ");
        assert_eq!(dma_register::VALID_MASK, 0x3);
        kprintln!("[ok]");
    }

    #[test_case]
    fn test_setup() {
        let args = args();
        let spi_base = args[0] as usize;
        let mut spi = unsafe {
            SPI::new(spi_base)
        };
        kprintln!("Testing SPI setup");
        kprint!("Testing ctrl0 register ");
        let ctrl0 = field!(spi.registers, ctrl0).read();
        assert_eq!(ctrl0 & ctrl0_register::VALID_MASK, 0x207);
        kprintln!("[ok]");
        kprint!("Testing ctrl1 register ");
        let ctrl1 = field!(spi.registers, ctrl1).read();
        assert_eq!(ctrl1 & ctrl1_register::VALID_MASK, 0);
        kprintln!("[ok]");
        kprint!("Testing div register ");
        let div = field!(spi.registers, div).read();
        assert_eq!(div & div_register::VALID_MASK, 2);
        kprintln!("[ok]");
        kprint!("Testing inter mask set clear register ");
        let inter_mask = field!(spi.registers, inter_mask_set_clear).read();
        assert_eq!(inter_mask & inter_register::VALID_MASK, 0);
        kprintln!("[ok]");
        kprint!("Testing dma register ");
        let dma = field!(spi.registers, dma).read();
        assert_eq!(dma & dma_register::VALID_MASK, 0);
        kprintln!("[ok]");
    }
}
