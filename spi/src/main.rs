#![no_std]
#![no_main]

use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, ReadWrite, WriteOnly}};
use small_os_lib::{QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, check_critical, do_yield, read_header, receive, reply, reply_empty, send_empty, wait_irq};

mod ctrl0_register {
    pub const DATA_SIZE_SHIFT: usize = 0;
    pub const FRAME_FORMAT_SHIFT: usize = 4;
    pub const POLARITY_SHIFT: usize = 6;
    pub const PHASE_SHIFT: usize = 7;
    pub const SERIAL_CLOCK_SHIFT: usize = 8;

    pub const DATA_SIZE_MASK: u32 = 0xf << DATA_SIZE_SHIFT;
    pub const FRAME_FORMAT_MASK: u32 = 0x3 << FRAME_FORMAT_SHIFT;
    pub const POLARITY_MASK: u32 = 1 << POLARITY_SHIFT;
    pub const PHASE_MASK: u32 = 1 << PHASE_SHIFT;
    pub const SERIAL_CLOCK_MASK: u32 = 0xf << SERIAL_CLOCK_SHIFT;

    pub const DATA_SIZE_4BIT: u32 = 0x3 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_5BIT: u32 = 0x4 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_6BIT: u32 = 0x5 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_7BIT: u32 = 0x6 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_8BIT: u32 = 0x7 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_9BIT: u32 = 0x8 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_10BIT: u32 = 0x9 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_11BIT: u32 = 0xa << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_12BIT: u32 = 0xb << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_13BIT: u32 = 0xc << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_14BIT: u32 = 0xd << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_15BIT: u32 = 0xe << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_16BIT: u32 = 0xf << DATA_SIZE_SHIFT;

    pub const FRAME_FORMAT_NORMAL: u32 = 0 << FRAME_FORMAT_SHIFT;
    pub const FRAME_FORMAT_TI: u32 = 1 << FRAME_FORMAT_SHIFT;
    pub const FRAME_FORMAT_NATIONAL_MICROWIRE: u32 = 2 << FRAME_FORMAT_SHIFT;
}

mod ctrl1_register {
    pub const LOOP_BACK_SHIFT: usize = 0;
    pub const ENABLE_SHIFT: usize = 1;
    pub const SLAVE_SHIFT: usize = 2;
    pub const SLAVE_OUTPUT_SHIFT: usize = 3;

    pub const LOOP_BACK_MASK: u32 = 1 << LOOP_BACK_SHIFT;
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const SLAVE_MASK: u32 = 1 << SLAVE_SHIFT;
    pub const SLAVE_OUTPUT_MASK: u32 = 1 << SLAVE_OUTPUT_SHIFT;
}

mod data_register {
    pub const DATA_SHIFT: usize = 0;

    pub const DATA_MASK: u32 = 0xffff << DATA_SHIFT;
}

mod status_register {
    pub const TX_EMPTY_SHIFT: usize = 0;
    pub const TX_NOT_FULL_SHIFT: usize = 1;
    pub const RX_NOT_EMPTY_SHIFT: usize = 2;
    pub const RX_FULL_SHIFT: usize = 3;
    pub const BUSY_SHIFT: usize = 4;

    pub const TX_EMPTY_MASK: u32 = 1 << TX_EMPTY_SHIFT;
    pub const TX_NOT_FULL_MASK: u32 = 1 << TX_NOT_FULL_SHIFT;
    pub const RX_NOT_EMPTY_MASK: u32 = 1 << RX_NOT_EMPTY_SHIFT;
    pub const RX_FULL_MASK: u32 = 1 << RX_FULL_SHIFT;
    pub const BUSY_MASK: u32 = 1 << BUSY_SHIFT;
}

mod div_register {
    pub const DIV_SHIFT: usize = 0;
    
    pub const DIV_MASK: u32 = 0xff << DIV_SHIFT;
}

mod inter_register {
    pub const RX_OVERRUN_SHIFT: usize = 0;
    pub const RX_TIMEOUT_SHIFT: usize = 1;
    pub const RX_FIFO_SHIFT: usize = 2;
    pub const TX_FIFO_SHIFT: usize = 3;

    pub const RX_OVERRUN_MASK: u32 = 1 << RX_OVERRUN_SHIFT;
    pub const RX_TIMEOUT_MASK: u32 = 1 << RX_TIMEOUT_SHIFT;
    pub const RX_FIFO_MASK: u32 = 1 << RX_FIFO_SHIFT;
    pub const TX_FIFO_MASK: u32 = 1 << TX_FIFO_SHIFT;

    pub const ALL_MASK: u32 = RX_OVERRUN_MASK | RX_TIMEOUT_MASK | RX_FIFO_MASK | TX_FIFO_MASK;
}

mod inter_clear_register {
    pub const RX_OVERRUN_SHIFT: usize = 0;
    pub const RX_TIMEOUT_SHIFT: usize = 1;
    
    pub const RX_OVERRUN_MASK: u32 = 1 << RX_OVERRUN_SHIFT;
    pub const RX_TIMEOUT_MASK: u32 = 1 << RX_TIMEOUT_SHIFT;

    pub const ALL_MASK: u32 = RX_OVERRUN_MASK | RX_TIMEOUT_MASK;
}

#[repr(C)]
struct SPIRegisters {
    ctrl0: ReadPureWrite<u32>, // 0x0
    ctrl1: ReadPureWrite<u32>, // 0x4
    data: ReadWrite<u32>, // 0x8
    status: ReadPure<u32>, // 0xc
    div: ReadPureWrite<u32>, // 0x10
    inter_mask_set_clear: ReadPureWrite<u32>, // 0x14
    raw_inter: ReadPure<u32>, // 0x18
    mask_inter: ReadPure<u32>, // 0x1c
    inter_clear: WriteOnly<u32>, // 0x20
    dma: ReadPureWrite<u32>, // 0x24
}

pub struct SPI {
    registers: UniqueMmioPointer<'static, SPIRegisters>,
    set_reg: UniqueMmioPointer<'static, SPIRegisters>,
    clear_reg: UniqueMmioPointer<'static, SPIRegisters>,
}

impl SPI {
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
        field!(res.registers, ctrl1).write(0);
        field!(res.registers, ctrl0).write(0);
        field!(res.registers, ctrl0).write(
            ctrl0_register::DATA_SIZE_8BIT |
            ctrl0_register::FRAME_FORMAT_NORMAL | 
            (2 << ctrl0_register::SERIAL_CLOCK_SHIFT)
        );
        field!(res.registers, div).write(2);
        field!(res.registers, inter_mask_set_clear).write(inter_clear_register::ALL_MASK);
        field!(res.registers, inter_clear).write(inter_clear_register::ALL_MASK);
        field!(res.registers, dma).write(0);
        res.cs_high();
        res
    }

    fn cs_high(&mut self) {
        send_empty(IO_BANK0_QUEUE, DRIVE_HIGH, &[CS_PIN]).unwrap();
    }

    fn cs_low(&mut self) {
        send_empty(IO_BANK0_QUEUE, DRIVE_LOW, &[CS_PIN]).unwrap();
    }

    fn wait_busy(&mut self) {
        while field!(self.registers, status).read() & status_register::BUSY_MASK != 0 {
            do_yield().unwrap();
        }
    }

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
        field!(self.registers, inter_mask_set_clear).write(inter_register::RX_FIFO_MASK | inter_register::RX_OVERRUN_MASK | tx_inter);
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
                    field!(self.registers, inter_mask_set_clear).write(inter_register::RX_FIFO_MASK | inter_register::RX_OVERRUN_MASK);
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

const CLOCK_FREQ: u32 = 12000000;
const SPI_FREQ: u32 = 6000000;

const IO_BANK0_QUEUE: u32 = 0;

const CS_PIN: u8 = 1;

const DRIVE_HIGH: u16 = 2;
const DRIVE_LOW: u16 = 3;

pub enum SPIReplyError {
    SendError,
    InvalidRequest,
    ReceieveOverrun,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

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

pub enum SPIError {
    ReplyError(SPIReplyError),
    QueueError(QueueError)
}

impl From<SPIReplyError> for SPIError {
    fn from(value: SPIReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for SPIError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

pub struct Request {
    delay: usize,
    padding: usize,
    tx_len: usize,
    rx_len: usize,
    buffer: [u8; 1024],
}

impl Request {
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

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(num_args: usize, spi_base: usize) {
    assert!(num_args == 2);
    send_empty(IO_BANK0_QUEUE, 0, &[]).unwrap();
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
