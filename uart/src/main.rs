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
use small_os_lib::{HeaderError, QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, args, check_critical, check_header_len, clear_irq, do_yield, kprintln, read_header, receive, reply, reply_empty, send, send_empty, wait_irq};

#[repr(C)]
struct UARTRegisters {
    data: ReadWrite<u32>, // 0x0
    status: ReadPureWrite<u32>, // 0x4
    _reserved0: u32, // 0x8
    _reserved1: u32, // 0xc
    _reserved2: u32, // 0x10
    _reserved3: u32, // 0x14
    flag: ReadPure<u32>, // 0x18
    _reserved4: u32, // 0x1c
    _reserved5: u32, // 0x20
    int_baud: ReadPureWrite<u32>, // 0x24
    frac_baud: ReadPureWrite<u32>, // 0x28
    line_ctrl: ReadPureWrite<u32>, // 0x2c
    ctrl: ReadPureWrite<u32>, // 0x30
    inter_fifo_sel: ReadPureWrite<u32>, // 0x34
    mask_set_clr: ReadPureWrite<u32>, // 0x38
    raw_inter: ReadPureWrite<u32>, // 0x3c
    mask_inter: ReadPure<u32>, // 0x40
    inter_clr: WriteOnly<u32> // 0x44
}

/// Fields in UART data register
mod data_register {
    pub const DATA_SHIFT: usize = 0;
    pub const FRAMING_ERROR_SHIFT: usize = 8;
    pub const PARITY_ERROR_SHIFT: usize = 9;
    pub const BREAK_ERROR_SHIFT: usize = 10;
    pub const OVERRUN_ERROR_SHIFT: usize = 11;

    pub const DATA_MASK: u32 = 0xff << DATA_SHIFT;
    pub const FRAMING_ERROR_MASK: u32 = 1 << FRAMING_ERROR_SHIFT;
    pub const PARITY_ERROR_MASK: u32 = 1 << PARITY_ERROR_SHIFT;
    pub const BREAK_ERROR_MASK: u32 = 1 << BREAK_ERROR_SHIFT;
    pub const OVERRUN_ERROR_MASK: u32 = 1 << OVERRUN_ERROR_SHIFT;

    pub const VALID_MASK: u32 = DATA_MASK |
        FRAMING_ERROR_MASK |
        PARITY_ERROR_MASK |
        BREAK_ERROR_MASK |
        OVERRUN_ERROR_MASK;
}

/// Fields in UART status register
mod receive_status_register {
    pub const FRAMING_ERROR_SHIFT: usize = 0;
    pub const PARITY_ERROR_SHIFT: usize = 1;
    pub const BREAK_ERROR_SHIFT: usize = 2;
    pub const OVERRUN_ERROR_SHIFT: usize = 3;

    pub const FRAMING_ERROR_MASK: u32 = 1 << FRAMING_ERROR_SHIFT;
    pub const PARITY_ERROR_MASK: u32 = 1 << PARITY_ERROR_SHIFT;
    pub const BREAK_ERROR_MASK: u32 = 1 << BREAK_ERROR_SHIFT;
    pub const OVERRUN_ERROR_MASK: u32 = 1 << OVERRUN_ERROR_SHIFT;

    pub const VALID_MASK: u32 = FRAMING_ERROR_MASK |
        PARITY_ERROR_MASK |
        BREAK_ERROR_MASK |
        OVERRUN_ERROR_MASK;
}

mod flag_register {
    pub const CTS_SHIFT: usize = 0;
    pub const DSR_SHIFT: usize = 1;
    pub const DCD_SHIFT: usize = 2;
    pub const BUSY_SHIFT: usize = 3;
    pub const RXFE_SHIFT: usize = 4;
    pub const TXFF_SHIFT: usize = 5;
    pub const RXFF_SHIFT: usize = 6;
    pub const TXFE_SHIFT: usize = 7;
    pub const RI_SHIFT: usize = 8;

    pub const CTS_MASK: u32 = 1 << CTS_SHIFT;
    pub const DSR_MASK: u32 = 1 << DSR_SHIFT;
    pub const DCD_MASK: u32 = 1 << DCD_SHIFT;
    pub const BUSY_MASK: u32 = 1 << BUSY_SHIFT;
    pub const RXFE_MASK: u32 = 1 << RXFE_SHIFT;
    pub const TXFF_MASK: u32 = 1 << TXFF_SHIFT;
    pub const RXFF_MASK: u32 = 1 << RXFF_SHIFT;
    pub const TXFE_MASK: u32 = 1 << TXFE_SHIFT;
    pub const RI_MASK: u32 = 1 << RI_SHIFT;

    pub const VALID_MASK: u32 = CTS_MASK |
        DSR_MASK |
        DCD_MASK |
        BUSY_MASK |
        RXFE_MASK |
        TXFF_MASK |
        RXFF_MASK |
        TXFE_MASK |
        RI_MASK;
}

mod int_baud_rate_register {
    pub const BAUD_DIVINT_SHIFT: usize = 0;
    
    pub const BAUD_DIVINT_MASK: u32 = 0xffff << BAUD_DIVINT_SHIFT;

    pub const VALID_MASK: u32 = BAUD_DIVINT_MASK;
}

mod frac_baud_rate_register {
    pub const BAUD_DIVFRAC_SHIFT: usize = 0;
    
    pub const BAUD_DIVFRAC_MASK: u32 = 0x3f << BAUD_DIVFRAC_SHIFT;

    pub const VALID_MASK: u32 = BAUD_DIVFRAC_MASK;
}

mod line_ctrl_register {
    pub const BRK_SHIFT: usize = 0;
    pub const PEN_SHIFT: usize = 1;
    pub const EPS_SHIFT: usize = 2;
    pub const STP2_SHIFT: usize = 3;
    pub const FEN_SHIFT: usize = 4;
    pub const WLEN_SHIFT: usize = 5;
    pub const SPS_SHIFT: usize = 7;

    pub const BRK_MASK: u32 = 1 << BRK_SHIFT;
    pub const PEN_MASK: u32 = 1 << PEN_SHIFT;
    pub const EPS_MASK: u32 = 1 << EPS_SHIFT;
    pub const STP2_MASK: u32 = 1 << STP2_SHIFT;
    pub const FEN_MASK: u32 = 1 << FEN_SHIFT;
    pub const WLEN_MASK: u32 = 0x3 << WLEN_SHIFT;
    pub const SPS_MASK: u32 = 1 << SPS_SHIFT;

    pub const WLEN_5: u32 = 0b00 << WLEN_SHIFT;
    pub const WLEN_6: u32 = 0b01 << WLEN_SHIFT;
    pub const WLEN_7: u32 = 0b10 << WLEN_SHIFT;
    pub const WLEN_8: u32 = 0b11 << WLEN_SHIFT;

    pub const VALID_MASK: u32 = BRK_MASK |
        PEN_MASK |
        EPS_MASK |
        STP2_MASK |
        FEN_MASK |
        WLEN_MASK |
        SPS_MASK;
}

mod ctrl_register {
    pub const UARTEN_SHIFT: usize = 0;
    pub const SIREN_SHIFT: usize = 1;
    pub const SIRLP_SHIFT: usize = 2;
    pub const LBE_SHIFT: usize = 7;
    pub const TXE_SHIFT: usize = 8;
    pub const RXE_SHIFT: usize = 9;
    pub const DTR_SHIFT: usize = 10;
    pub const RTS_SHIFT: usize = 11;
    pub const OUT1_SHIFT: usize = 12;
    pub const OUT2_SHIFT: usize = 13;
    pub const RTSEN_SHIFT: usize = 14;
    pub const CTSEN_SHIFT: usize = 15;

    pub const UARTEN_MASK: u32 = 1 << UARTEN_SHIFT;
    pub const SIREN_MASK: u32 = 1 << SIREN_SHIFT;
    pub const SIRLP_MASK: u32 = 1 << SIRLP_SHIFT;
    pub const LBE_MASK: u32 = 1 << LBE_SHIFT;
    pub const TXE_MASK: u32 = 1 << TXE_SHIFT;
    pub const RXE_MASK: u32 = 1 << RXE_SHIFT;
    pub const DTR_MASK: u32 = 1 << DTR_SHIFT;
    pub const RTS_MASK: u32 = 1 << RTS_SHIFT;
    pub const OUT1_MASK: u32 = 1 << OUT1_SHIFT;
    pub const OUT2_MASK: u32 = 1 << OUT2_SHIFT;
    pub const RTSEN_MASK: u32 = 1 << RTSEN_SHIFT;
    pub const CTSEN_MASK: u32 = 1 << CTSEN_SHIFT;

    pub const VALID_MASK: u32 = UARTEN_MASK |
        SIREN_MASK |
        SIRLP_MASK |
        LBE_MASK |
        TXE_MASK |
        RXE_MASK |
        DTR_MASK |
        RTS_MASK |
        OUT1_MASK |
        OUT2_MASK |
        RTSEN_MASK |
        CTSEN_MASK;
}

mod interrupt_register {
    pub const RI_SHIFT: usize = 0;
    pub const CTS_SHIFT: usize = 1;
    pub const DCD_SHIFT: usize = 2;
    pub const DSR_SHIFT: usize = 3;
    pub const RX_SHIFT: usize = 4;
    pub const TX_SHIFT: usize = 5;
    pub const RT_SHIFT: usize = 6;
    pub const FE_SHIFT: usize = 7;
    pub const PE_SHIFT: usize = 8;
    pub const BE_SHIFT: usize = 9;
    pub const OE_SHIFT: usize = 10;

    pub const RI_MASK: u32 = 1 << RI_SHIFT;
    pub const CTS_MASK: u32 = 1 << CTS_SHIFT;
    pub const DCD_MASK: u32 = 1 << DCD_SHIFT;
    pub const DSR_MASK: u32 = 1 << DSR_SHIFT;
    pub const RX_MASK: u32 = 1 << RX_SHIFT;
    pub const TX_MASK: u32 = 1 << TX_SHIFT;
    pub const RT_MASK: u32 = 1 << RT_SHIFT;
    pub const FE_MASK: u32 = 1 << FE_SHIFT;
    pub const PE_MASK: u32 = 1 << PE_SHIFT;
    pub const BE_MASK: u32 = 1 << BE_SHIFT;
    pub const OE_MASK: u32 = 1 << OE_SHIFT;

    pub const VALID_MASK: u32 = 
        RI_MASK | 
        CTS_MASK |
        DCD_MASK |
        DSR_MASK |
        RX_MASK |
        TX_MASK |
        RT_MASK |
        FE_MASK |
        PE_MASK |
        BE_MASK |
        OE_MASK;
}

mod inter_fifo_sel_register {
    pub const TX_FIFO_SHIFT: usize = 0;
    pub const RX_FIFO_SHIFT: usize = 3;
    
    pub const TX_FIFO_MASK: u32 = 0x7 << TX_FIFO_SHIFT;
    pub const RX_FIFO_MASK: u32 = 0x7 << RX_FIFO_SHIFT;

    pub const TX_FIFO_1: u32 = 0x0 << TX_FIFO_SHIFT;
    pub const TX_FIFO_2: u32 = 0x1 << TX_FIFO_SHIFT;
    pub const TX_FIFO_4: u32 = 0x2 << TX_FIFO_SHIFT;
    pub const TX_FIFO_6: u32 = 0x3 << TX_FIFO_SHIFT;
    pub const TX_FIFO_7: u32 = 0x4 << TX_FIFO_SHIFT;
    
    pub const RX_FIFO_1: u32 = 0x0 << RX_FIFO_SHIFT;
    pub const RX_FIFO_2: u32 = 0x1 << RX_FIFO_SHIFT;
    pub const RX_FIFO_4: u32 = 0x2 << RX_FIFO_SHIFT;
    pub const RX_FIFO_6: u32 = 0x3 << RX_FIFO_SHIFT;
    pub const RX_FIFO_7: u32 = 0x4 << RX_FIFO_SHIFT;

    pub const VALID_MASK: u32 = TX_FIFO_MASK |
        RX_FIFO_MASK;
}

struct UART {
    registers: UniqueMmioPointer<'static, UARTRegisters>,
    set_reg: UniqueMmioPointer<'static, UARTRegisters>,
    clear_reg: UniqueMmioPointer<'static, UARTRegisters>,
}

impl UART {
    const SLOTS: usize = 32;

    pub unsafe fn new(uart_base: usize) -> Self {
        let mut res = unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(uart_base)).unwrap()), 
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(uart_base + REG_ALIAS_SET_BITS)).unwrap()), 
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(uart_base + REG_ALIAS_CLR_BITS)).unwrap()),
            }
        };
        while field!(res.registers, flag).read() & flag_register::BUSY_MASK != 0 {
            do_yield().unwrap();
        }
        field!(res.registers, ctrl).modify(|ctrl| ctrl & !ctrl_register::VALID_MASK);
        // clear all interrupts
        field!(res.registers, inter_clr).write(interrupt_register::VALID_MASK);

        // set baud rate to 115200
        // integer baud rate should be 6 and fractional should be 33 for a 12MHz clock
        //field!(res.registers, int_baud).write((6 << int_baud_rate_register::BAUD_DIVINT_SHIFT) & int_baud_rate_register::BAUD_DIVINT_MASK);
        //field!(res.registers, frac_baud).write((33 << frac_baud_rate_register::BAUD_DIVFRAC_SHIFT) & frac_baud_rate_register::BAUD_DIVFRAC_MASK);
        field!(res.registers, int_baud).modify(|int_baud|
            (int_baud & !int_baud_rate_register::VALID_MASK) |
            (26 << int_baud_rate_register::BAUD_DIVINT_SHIFT)
            );
        field!(res.registers, frac_baud).modify(|frac_baud|
            (frac_baud & !frac_baud_rate_register::VALID_MASK) |
            (3 << frac_baud_rate_register::BAUD_DIVFRAC_SHIFT)
        );
        field!(res.registers, line_ctrl).modify(|line_ctrl|
            (line_ctrl & !line_ctrl_register::VALID_MASK) |
            line_ctrl_register::FEN_MASK | 
            line_ctrl_register::WLEN_8
        );
        // mask all interrupts (no interrupts will be generated)
        field!(res.clear_reg, mask_set_clr).modify(|inter_mask|
            (inter_mask & !interrupt_register::VALID_MASK) |
            interrupt_register::VALID_MASK
        );
        // interrupt when tx fifo <= 1/8 full
        field!(res.registers, inter_fifo_sel).modify(|inter_fifo_sel|
            (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
            inter_fifo_sel_register::TX_FIFO_1
        );
        // enable UART, RX and TX section
        field!(res.registers, ctrl).modify(|ctrl|
            (ctrl & !ctrl_register::VALID_MASK) |
            ctrl_register::TXE_MASK | 
            ctrl_register::RXE_MASK | 
            ctrl_register::UARTEN_MASK
        );
        res
    }

    fn wait_for_space_tx(&mut self, len: usize) -> bool {
        if len >= (7 * Self::SLOTS) / 8 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::TX_FIFO_1
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::TX_MASK);
            true
        } else if len >= (3 * Self::SLOTS) / 4 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::TX_FIFO_2
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::TX_MASK);
            true
        } else if len >= Self::SLOTS / 2 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::TX_FIFO_4
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::TX_MASK);
            true
        } else if len >= Self::SLOTS / 4 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::TX_FIFO_6
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::TX_MASK);
            true
        } else if len >= Self::SLOTS / 8 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::TX_FIFO_7
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::TX_MASK);
            true
        } else {
            false
        }
    }
    
    fn wait_for_space_rx(&mut self, len: usize) -> bool {
        if len >= (7 * Self::SLOTS) / 8 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::RX_FIFO_7
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::RX_MASK);
            true
        } else if len >= (3 * Self::SLOTS) / 4 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::RX_FIFO_6
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::RX_MASK);
            true
        } else if len >= Self::SLOTS / 2 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::RX_FIFO_4
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::RX_MASK);
            true
        } else if len >= Self::SLOTS / 4 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::RX_FIFO_2
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::RX_MASK);
            true
        } else if len >= Self::SLOTS / 8 {
            field!(self.registers, inter_fifo_sel).modify(|inter_fifo_sel|
                (inter_fifo_sel & !inter_fifo_sel_register::VALID_MASK) |
                inter_fifo_sel_register::RX_FIFO_1
            );
            field!(self.set_reg, mask_set_clr).write(interrupt_register::RX_MASK);
            true
        } else {
            false
        }
    }

    fn tx_full(&mut self) -> bool {
        field!(self.registers, flag).read() & flag_register::TXFF_MASK != 0
    }
    
    fn rx_empty(&mut self) -> bool {
        field!(self.registers, flag).read() & flag_register::RXFE_MASK != 0
    }

    pub fn handle_receive(&mut self, buffer: &mut [u8]) {
        let mut pos = 0;
        while pos < buffer.len() && self.wait_for_space_rx(buffer.len() - pos) {
            wait_irq().unwrap();
            field!(self.clear_reg, mask_set_clr).write(interrupt_register::RX_MASK);
            field!(self.set_reg, inter_clr).write(interrupt_register::RX_MASK);
            while !self.rx_empty() && pos < buffer.len() {
                buffer[pos] = ((field!(self.registers, data).read() & data_register::DATA_MASK) >> data_register::DATA_SHIFT) as u8;
                pos += 1;
            }
        }
        while pos < buffer.len() {
            while self.rx_empty() {}
            buffer[pos] = ((field!(self.registers, data).read() & data_register::DATA_MASK) >> data_register::DATA_SHIFT) as u8;
            pos += 1;
        }
    }

    pub fn handle_send(&mut self, buffer: &[u8]) {
        let mut pos = 0;
        while pos < buffer.len() && !self.tx_full() {
            field!(self.registers, data).write(buffer[pos] as u32);
            pos += 1;
        }
        while pos < buffer.len() && self.wait_for_space_tx(buffer.len() - pos) {
            wait_irq().unwrap();
            field!(self.clear_reg, mask_set_clr).write(interrupt_register::TX_MASK);
            field!(self.set_reg, inter_clr).write(interrupt_register::TX_MASK);
            while !self.tx_full() && pos < buffer.len() {
                field!(self.registers, data).write(buffer[pos] as u32);
                pos += 1;
            }
        }
        while pos < buffer.len() {
            while self.tx_full() {}
            field!(self.registers, data).write(buffer[pos] as u32);
            pos += 1;
        }
    }

    pub fn clear_rx(&mut self, mut len: usize) {
        while len > 0 && self.wait_for_space_rx(len) {
            wait_irq().unwrap();
            field!(self.clear_reg, mask_set_clr).write(interrupt_register::RX_MASK);
            field!(self.set_reg, inter_clr).write(interrupt_register::RX_MASK);
            while !self.rx_empty() && len > 0 {
                _ = field!(self.registers, data).read();
                len -= 1;
            }
        }
        while len > 0 {
            while self.rx_empty() {}
            _ = field!(self.registers, data).read();
            len -= 1;
        }
    }

    pub fn clear_all_rx(&mut self) {
        while !self.rx_empty() {
            _ = field!(self.registers, data).read();
        }
    }
}

pub enum UARTReplyError {
    SendError,
    InvalidRequest,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<UARTReplyError> for u32 {
    fn from(value: UARTReplyError) -> Self {
        match value {
            UARTReplyError::SendError => 1,
            UARTReplyError::InvalidRequest => 2,
            UARTReplyError::InvalidSendBuffer => 3,
            UARTReplyError::InvalidReplyBuffer => 4
        }
    }
}

impl From<HeaderError> for UARTReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

pub enum UARTError {
    ReplyError(UARTReplyError),
    QueueError(QueueError)
}

impl From<UARTReplyError> for UARTError {
    fn from(value: UARTReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for UARTError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

impl From<HeaderError> for UARTError {
    fn from(value: HeaderError) -> Self {
        Self::from(UARTReplyError::from(value))
    }
}

pub struct DataRequest {
    buffer: [u8; 32],
    len: usize
}

pub enum Request {
    Send(DataRequest),
    Receive(DataRequest),
    ClearRX(u32),
    ClearAllRX,
}

impl Request {
    pub fn parse() -> Result<Self, UARTError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // Send
                let mut buffer = [0; 32];
                if header.send_len as usize > buffer.len() {
                    return Err(UARTError::ReplyError(UARTReplyError::InvalidSendBuffer));
                }
                if header.reply_len != 0 {
                    return Err(UARTError::ReplyError(UARTReplyError::InvalidReplyBuffer));
                }
                _ = receive(0, &mut buffer)?;
                Ok(Self::Send(DataRequest { 
                    buffer, 
                    len: header.send_len as usize
                }))
            },
            1 => {
                // Receive
                let buffer = [0; 32];
                if header.send_len != 0 {
                    return Err(UARTError::ReplyError(UARTReplyError::InvalidSendBuffer));
                }
                if header.reply_len as usize > buffer.len() {
                    return Err(UARTError::ReplyError(UARTReplyError::InvalidReplyBuffer));
                }
                Ok(Self::Receive(DataRequest { 
                    buffer, 
                    len: header.reply_len as usize
                }))
            },
            2 => {
                // Clear RX
                let mut buffer = [0; 4];
                check_header_len(&header, 4, 0)?;
                _ = receive(0, &mut buffer)?;
                let len = u32::from_le_bytes(buffer);
                Ok(Self::ClearRX(len))
            },
            3 => {
                // Clear All RX
                check_header_len(&header, 0, 0)?;
                Ok(Self::ClearAllRX)
            },
            _ => Err(UARTError::ReplyError(UARTReplyError::InvalidRequest))
        }
    }
}

const IO_BANK0_QUEUE: u32 = 0;

/// Program entry point
/// Disables mangling so it can be called from assembly
/// Also has argument for GPIO mask but the driver doesn't care about this
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 2);
    let uart_base = args[0] as usize;
    // check func sel has finished
    send_empty(IO_BANK0_QUEUE, 0, &[]).unwrap();
    let mut uart = unsafe {
        UART::new(uart_base)
    };
    #[cfg(test)]
    test_main();
    loop {
        match Request::parse() {
            Ok(request) => {
                match request {
                    Request::Send(request) => {
                        uart.handle_send(&request.buffer[..request.len]);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::Receive(mut request) => {
                        uart.handle_receive(&mut request.buffer[..request.len]);
                        check_critical(reply(0, 0, &request.buffer[..request.len])).unwrap_or(Ok(())).unwrap();
                    },
                    Request::ClearRX(len) => {
                        uart.clear_rx(len as usize);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::ClearAllRX => {
                        uart.clear_all_rx();
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    }
                }
            },
            Err(err) => {
                match err {
                    UARTError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    UARTError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(UARTReplyError::SendError))).unwrap_or(Ok(())).unwrap();
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
        kprintln!("Running {} tests for UART", tests.len());
        for test in tests {
            test();
        }
    }

    #[test_case]
    fn test_valid() {
        kprintln!("Testing UART register mask values");
        kprint!("Testing data register ");
        assert_eq!(data_register::VALID_MASK, 0xfff);
        kprintln!("[ok]");
        kprint!("Testing receive status register ");
        assert_eq!(receive_status_register::VALID_MASK, 0xf);
        kprintln!("[ok]");
        kprint!("Testing flag register ");
        assert_eq!(flag_register::VALID_MASK, 0x1ff);
        kprintln!("[ok]");
        kprint!("Testing int baud rate register ");
        assert_eq!(int_baud_rate_register::VALID_MASK, 0xffff);
        kprintln!("[ok]");
        kprint!("Testing frac baud rate register ");
        assert_eq!(frac_baud_rate_register::VALID_MASK, 0x3f);
        kprintln!("[ok]");
        kprint!("Testing line ctrl register ");
        assert_eq!(line_ctrl_register::VALID_MASK, 0xff);
        kprintln!("[ok]");
        kprint!("Testing ctrl register ");
        assert_eq!(ctrl_register::VALID_MASK, 0xff87);
        kprintln!("[ok]");
        kprint!("Testing interrupt register ");
        assert_eq!(interrupt_register::VALID_MASK, 0x7ff);
        kprintln!("[ok]");
    }

    #[test_case]
    fn test_setup() {
        let args = args();
        let uart_base = args[0] as usize;
        let mut uart = unsafe {
            UART::new(uart_base)
        };
        kprint!("Testing int baud register ");
        let int_baud = field!(uart.registers, int_baud).read();
        assert_eq!(int_baud & int_baud_rate_register::VALID_MASK, 26);
        kprintln!("[ok]");
        kprint!("Testing frac baud register ");
        let frac_baud = field!(uart.registers, frac_baud).read();
        assert_eq!(frac_baud & frac_baud_rate_register::VALID_MASK, 3);
        kprintln!("[ok]");
        kprint!("Testing line ctrl register ");
        let line_ctrl = field!(uart.registers, line_ctrl).read();
        assert_eq!(line_ctrl & line_ctrl_register::VALID_MASK, 0x70);
        kprintln!("[ok]");
        kprint!("Testing ctrl register ");
        let ctrl = field!(uart.registers, ctrl).read();
        assert_eq!(ctrl & ctrl_register::VALID_MASK, 0x301);
        kprintln!("[ok]");
        kprint!("Testing mask set clr register ");
        let mask = field!(uart.registers, mask_set_clr).read();
        assert_eq!(mask & interrupt_register::VALID_MASK, 0);
        kprintln!("[ok]");
    }
}
