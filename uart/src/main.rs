/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS UART driver.
 *
 * The SmallOS UART driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS UART driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS UART driver. 
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

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, ReadWrite, WriteOnly}};
use small_os_lib::{HeaderError, QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, args, check_critical, check_header_len, clear_irq, do_yield, kprintln, read_header, receive, reply, reply_empty, send, send_empty, wait_irq};

/// UART memory mapped registers
#[repr(C)]
struct UARTRegisters {
    /// UART data register (0x00)
    data: ReadWrite<u32>, // 0x0
    /// UART status register (0x04)
    status: ReadPureWrite<u32>, // 0x4
    _reserved0: u32, // 0x8
    _reserved1: u32, // 0xc
    _reserved2: u32, // 0x10
    _reserved3: u32, // 0x14
    /// UART flag register (0x18)
    flag: ReadPure<u32>, // 0x18
    _reserved4: u32, // 0x1c
    _reserved5: u32, // 0x20
    /// UART integer baud rate register (0x24)
    int_baud: ReadPureWrite<u32>, // 0x24
    /// UART fractional baud rate register (0x28)
    frac_baud: ReadPureWrite<u32>, // 0x28
    /// UART line control register (0x2c)
    line_ctrl: ReadPureWrite<u32>, // 0x2c
    /// UART control register (0x30)
    ctrl: ReadPureWrite<u32>, // 0x30
    /// UART interrupt fifo level select register (0x34)
    inter_fifo_sel: ReadPureWrite<u32>, // 0x34
    /// UART interrupt mask set clear register (0x38)
    mask_set_clr: ReadPureWrite<u32>, // 0x38
    /// UART raw interrupt register (0x3c)
    raw_inter: ReadPureWrite<u32>, // 0x3c
    /// UART masked interrupt register (0x40)
    mask_inter: ReadPure<u32>, // 0x40
    /// UART interrupt clear register (0x44)
    inter_clr: WriteOnly<u32> // 0x44
}

/// UART data register masks and shifts
mod data_register {
    /// Data shift
    pub const DATA_SHIFT: usize = 0;
    /// Shift for framing error
    pub const FRAMING_ERROR_SHIFT: usize = 8;
    /// Shift for parity error
    pub const PARITY_ERROR_SHIFT: usize = 9;
    /// Shift for break error
    pub const BREAK_ERROR_SHIFT: usize = 10;
    /// Shift for overrun error
    pub const OVERRUN_ERROR_SHIFT: usize = 11;

    /// Data mask
    pub const DATA_MASK: u32 = 0xff << DATA_SHIFT;
    /// Mask for framing error
    pub const FRAMING_ERROR_MASK: u32 = 1 << FRAMING_ERROR_SHIFT;
    /// Mask for parity error
    pub const PARITY_ERROR_MASK: u32 = 1 << PARITY_ERROR_SHIFT;
    /// Mask for break error
    pub const BREAK_ERROR_MASK: u32 = 1 << BREAK_ERROR_SHIFT;
    /// Mask for overrun error
    pub const OVERRUN_ERROR_MASK: u32 = 1 << OVERRUN_ERROR_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = DATA_MASK |
        FRAMING_ERROR_MASK |
        PARITY_ERROR_MASK |
        BREAK_ERROR_MASK |
        OVERRUN_ERROR_MASK;
}

/// UART status register masks and shifts
mod receive_status_register {
    /// Shift for framing error
    pub const FRAMING_ERROR_SHIFT: usize = 0;
    /// Shift for parity error
    pub const PARITY_ERROR_SHIFT: usize = 1;
    /// Shift for break error
    pub const BREAK_ERROR_SHIFT: usize = 2;
    /// Shift for overrun error
    pub const OVERRUN_ERROR_SHIFT: usize = 3;

    /// Mask for framing error
    pub const FRAMING_ERROR_MASK: u32 = 1 << FRAMING_ERROR_SHIFT;
    /// Mask for parity error
    pub const PARITY_ERROR_MASK: u32 = 1 << PARITY_ERROR_SHIFT;
    /// Mask for break error
    pub const BREAK_ERROR_MASK: u32 = 1 << BREAK_ERROR_SHIFT;
    /// Mask for overrun error
    pub const OVERRUN_ERROR_MASK: u32 = 1 << OVERRUN_ERROR_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = FRAMING_ERROR_MASK |
        PARITY_ERROR_MASK |
        BREAK_ERROR_MASK |
        OVERRUN_ERROR_MASK;
}

/// UART flag register masks and shifts
mod flag_register {
    /// Shift for clear to send
    pub const CTS_SHIFT: usize = 0;
    /// Shift for data set ready
    pub const DSR_SHIFT: usize = 1;
    /// Shift for data carrier detected
    pub const DCD_SHIFT: usize = 2;
    /// Shift for busy
    pub const BUSY_SHIFT: usize = 3;
    /// Shift for receive fifo empty
    pub const RXFE_SHIFT: usize = 4;
    /// Shift for transmit fifo full
    pub const TXFF_SHIFT: usize = 5;
    /// Shift for receive fifo full
    pub const RXFF_SHIFT: usize = 6;
    /// Shift for transmit fifo empty
    pub const TXFE_SHIFT: usize = 7;
    /// Shift for ring indicator
    pub const RI_SHIFT: usize = 8;

    /// Mask for clear to send
    pub const CTS_MASK: u32 = 1 << CTS_SHIFT;
    /// Mask for data set ready
    pub const DSR_MASK: u32 = 1 << DSR_SHIFT;
    /// Mask for data carrier detected
    pub const DCD_MASK: u32 = 1 << DCD_SHIFT;
    /// Mask for busy
    pub const BUSY_MASK: u32 = 1 << BUSY_SHIFT;
    /// Mask for receive fifo empty
    pub const RXFE_MASK: u32 = 1 << RXFE_SHIFT;
    /// Mask for transmit fifo full
    pub const TXFF_MASK: u32 = 1 << TXFF_SHIFT;
    /// Mask for receive fifo full
    pub const RXFF_MASK: u32 = 1 << RXFF_SHIFT;
    /// Mask for transmit fifo empty
    pub const TXFE_MASK: u32 = 1 << TXFE_SHIFT;
    /// Mask for ring indicator
    pub const RI_MASK: u32 = 1 << RI_SHIFT;

    /// Mask of all non-reserved bits
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

/// UART integer baud rate register masks and shifts
mod int_baud_rate_register {
    /// Integer baud rate shift
    pub const BAUD_DIVINT_SHIFT: usize = 0;
    
    /// Integer baud rate mask
    pub const BAUD_DIVINT_MASK: u32 = 0xffff << BAUD_DIVINT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = BAUD_DIVINT_MASK;
}

/// UART fractional baud rate register masks and shifts
mod frac_baud_rate_register {
    /// Fractional baud rate shift
    pub const BAUD_DIVFRAC_SHIFT: usize = 0;
    
    /// Fractional baud rate mask
    pub const BAUD_DIVFRAC_MASK: u32 = 0x3f << BAUD_DIVFRAC_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = BAUD_DIVFRAC_MASK;
}

/// UART line control register masks and shifts
mod line_ctrl_register {
    /// Shift for send break
    pub const BRK_SHIFT: usize = 0;
    /// Shift for parity enable
    pub const PEN_SHIFT: usize = 1;
    /// Shift for even parity selected
    pub const EPS_SHIFT: usize = 2;
    /// Shift for 2 stop bits
    pub const STP2_SHIFT: usize = 3;
    /// Shift for enabling fifos
    pub const FEN_SHIFT: usize = 4;
    /// Word length shift
    pub const WLEN_SHIFT: usize = 5;
    /// Shift for stick parity selected
    pub const SPS_SHIFT: usize = 7;

    /// Mask for send break
    pub const BRK_MASK: u32 = 1 << BRK_SHIFT;
    /// Mask for parity enable
    pub const PEN_MASK: u32 = 1 << PEN_SHIFT;
    /// Mask for even parity selected
    pub const EPS_MASK: u32 = 1 << EPS_SHIFT;
    /// Mask for 2 stop bits
    pub const STP2_MASK: u32 = 1 << STP2_SHIFT;
    /// Mask for enabling fifos
    pub const FEN_MASK: u32 = 1 << FEN_SHIFT;
    /// Word length mask
    pub const WLEN_MASK: u32 = 0x3 << WLEN_SHIFT;
    /// Mask for stick parity selected
    pub const SPS_MASK: u32 = 1 << SPS_SHIFT;

    /// Word length of 5 bits
    pub const WLEN_5: u32 = 0b00 << WLEN_SHIFT;
    /// Word length of 6 bits
    pub const WLEN_6: u32 = 0b01 << WLEN_SHIFT;
    /// Word length of 7 bits
    pub const WLEN_7: u32 = 0b10 << WLEN_SHIFT;
    /// Word length of 8 bits
    pub const WLEN_8: u32 = 0b11 << WLEN_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = BRK_MASK |
        PEN_MASK |
        EPS_MASK |
        STP2_MASK |
        FEN_MASK |
        WLEN_MASK |
        SPS_MASK;
}

/// UART control register masks and shifts
mod ctrl_register {
    /// Shift for enabling the UART
    pub const UARTEN_SHIFT: usize = 0;
    /// Shift for enabling SIR
    pub const SIREN_SHIFT: usize = 1;
    /// Shift for SIR low power IrDA mode
    pub const SIRLP_SHIFT: usize = 2;
    /// Shift for loopback enable
    pub const LBE_SHIFT: usize = 7;
    /// Shift for transmit enable
    pub const TXE_SHIFT: usize = 8;
    /// Shift for receive enable
    pub const RXE_SHIFT: usize = 9;
    /// Shift for data transmit ready
    pub const DTR_SHIFT: usize = 10;
    /// Shift for request to send
    pub const RTS_SHIFT: usize = 11;
    /// OUT1 shift
    pub const OUT1_SHIFT: usize = 12;
    /// OUT2 shift
    pub const OUT2_SHIFT: usize = 13;
    /// Shift for enabling RTS hardware flow control
    pub const RTSEN_SHIFT: usize = 14;
    /// Shift for enabling CTS hardware flow control
    pub const CTSEN_SHIFT: usize = 15;

    /// Mask for enabling the UART
    pub const UARTEN_MASK: u32 = 1 << UARTEN_SHIFT;
    /// Mask for enabling SIR
    pub const SIREN_MASK: u32 = 1 << SIREN_SHIFT;
    /// Mask for SIR low power IrDA mode
    pub const SIRLP_MASK: u32 = 1 << SIRLP_SHIFT;
    /// Mask for loopback enable
    pub const LBE_MASK: u32 = 1 << LBE_SHIFT;
    /// Mask for transmit enable
    pub const TXE_MASK: u32 = 1 << TXE_SHIFT;
    /// Mask for receive enable
    pub const RXE_MASK: u32 = 1 << RXE_SHIFT;
    /// Mask for data transmit ready
    pub const DTR_MASK: u32 = 1 << DTR_SHIFT;
    /// Mask for request to send
    pub const RTS_MASK: u32 = 1 << RTS_SHIFT;
    /// OUT1 mask
    pub const OUT1_MASK: u32 = 1 << OUT1_SHIFT;
    /// OUT2 mask
    pub const OUT2_MASK: u32 = 1 << OUT2_SHIFT;
    /// Mask for enabling RTS hardware flow control
    pub const RTSEN_MASK: u32 = 1 << RTSEN_SHIFT;
    /// Mask for enabling CTS hardware flow control
    pub const CTSEN_MASK: u32 = 1 << CTSEN_SHIFT;

    /// Mask of all non-reserved bits
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
    /// Ring indicator shift
    pub const RI_SHIFT: usize = 0;
    /// CTS shift
    pub const CTS_SHIFT: usize = 1;
    /// DCD shift
    pub const DCD_SHIFT: usize = 2;
    /// DSR shift
    pub const DSR_SHIFT: usize = 3;
    /// Receive shift
    pub const RX_SHIFT: usize = 4;
    /// Transmit shift
    pub const TX_SHIFT: usize = 5;
    /// Receive timeout shift
    pub const RT_SHIFT: usize = 6;
    /// Framing error shift
    pub const FE_SHIFT: usize = 7;
    /// Parity error shift
    pub const PE_SHIFT: usize = 8;
    /// Break error shift
    pub const BE_SHIFT: usize = 9;
    /// Overrun error shift
    pub const OE_SHIFT: usize = 10;

    /// Ring indicator mask
    pub const RI_MASK: u32 = 1 << RI_SHIFT;
    /// CTS mask
    pub const CTS_MASK: u32 = 1 << CTS_SHIFT;
    /// DCD mask
    pub const DCD_MASK: u32 = 1 << DCD_SHIFT;
    /// DSR mask
    pub const DSR_MASK: u32 = 1 << DSR_SHIFT;
    /// Receive mask
    pub const RX_MASK: u32 = 1 << RX_SHIFT;
    /// Transmit mask
    pub const TX_MASK: u32 = 1 << TX_SHIFT;
    /// Receive timeout mask
    pub const RT_MASK: u32 = 1 << RT_SHIFT;
    /// Framing error mask
    pub const FE_MASK: u32 = 1 << FE_SHIFT;
    /// Parity error mask
    pub const PE_MASK: u32 = 1 << PE_SHIFT;
    /// Break error mask
    pub const BE_MASK: u32 = 1 << BE_SHIFT;
    /// Overrun error mask
    pub const OE_MASK: u32 = 1 << OE_SHIFT;

    /// Mask of all non-reserved bits
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
    /// Transmit fifo level select shift
    pub const TX_FIFO_SHIFT: usize = 0;
    /// Receive fifo level select shift
    pub const RX_FIFO_SHIFT: usize = 3;
    
    /// Transmit fifo level select mask
    pub const TX_FIFO_MASK: u32 = 0x7 << TX_FIFO_SHIFT;
    /// Receive fifo level select mask
    pub const RX_FIFO_MASK: u32 = 0x7 << RX_FIFO_SHIFT;

    /// Transmit fifo <= 1/8 full
    pub const TX_FIFO_1: u32 = 0x0 << TX_FIFO_SHIFT;
    /// Transmit fifo <= 1/4 full
    pub const TX_FIFO_2: u32 = 0x1 << TX_FIFO_SHIFT;
    /// Transmit fifo <= 1/2 full
    pub const TX_FIFO_4: u32 = 0x2 << TX_FIFO_SHIFT;
    /// Transmit fifo <= 3/4 full
    pub const TX_FIFO_6: u32 = 0x3 << TX_FIFO_SHIFT;
    /// Transmit fifo <= 7/8 full
    pub const TX_FIFO_7: u32 = 0x4 << TX_FIFO_SHIFT;
    
    /// Receive fifo >= 1/8 full
    pub const RX_FIFO_1: u32 = 0x0 << RX_FIFO_SHIFT;
    /// Receive fifo >= 1/4 full
    pub const RX_FIFO_2: u32 = 0x1 << RX_FIFO_SHIFT;
    /// Receive fifo >= 1/2 full
    pub const RX_FIFO_4: u32 = 0x2 << RX_FIFO_SHIFT;
    /// Receive fifo >= 3/4 full
    pub const RX_FIFO_6: u32 = 0x3 << RX_FIFO_SHIFT;
    /// Receive fifo >= 7/8 full
    pub const RX_FIFO_7: u32 = 0x4 << RX_FIFO_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = TX_FIFO_MASK |
        RX_FIFO_MASK;
}

struct UART {
    registers: UniqueMmioPointer<'static, UARTRegisters>,
    set_reg: UniqueMmioPointer<'static, UARTRegisters>,
    clear_reg: UniqueMmioPointer<'static, UARTRegisters>,
}

impl UART {
    /// Size of UART fifo
    const SLOTS: usize = 32;

    /// Creates a new `UART` object and initialises it   
    /// `uart_base` is the base address of the UART memory mapped registers
    /// # Safety
    /// `uart_base` must be a valid address which points to the UART memory mapped registers and not
    /// being used by anything else  
    /// Based off <https://github.com/dwelch67/raspberrypi-pico/blob/main/uart01/notmain.c> accessed 21/01/2026
    /// under the license  
    ///-------------------------------------------------------------------------  
    ///  
    /// Copyright (c) 2021 David Welch dwelch@dwelch.com  
    ///  
    /// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:  
    ///  
    /// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.  
    ///  
    /// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.  
    ///  
    ///-------------------------------------------------------------------------
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

    /// Sets up waiting for `len` bytes of space in the TX fifo  
    /// Programs the TX interrupt fifo select to the right value and returns if the UART should wait
    /// for the interrupt to happen  
    /// `len` is how many bytes to wait for  
    /// Returns true if the UART should wait for an interrupt and false if it should busily wait
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
    
    /// Sets up waiting for `len` bytes of data to be in the RX fifo  
    /// Programs the RX interrupt fifo select to the right value and returns if the UART should wait
    /// for the interrupt to happen  
    /// `len` is how many bytes to wait for  
    /// Returns true if the UART should wait for an interrupt and false if it should busily wait
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

    /// Whether the TX fifo is full
    fn tx_full(&mut self) -> bool {
        field!(self.registers, flag).read() & flag_register::TXFF_MASK != 0
    }
    
    /// Whether the RX fifo is empty
    fn rx_empty(&mut self) -> bool {
        field!(self.registers, flag).read() & flag_register::RXFE_MASK != 0
    }

    /// Handles a receive request  
    /// `buffer` is the buffer to fill up
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

    /// Handles a transmit request  
    /// `buffer` is the buffer to transmit
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

    /// Discards `len` bytes from the RX fifo  
    /// `len` is the number of bytes to discard
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

    /// Discards all bytes in the RX fifo
    pub fn clear_all_rx(&mut self) {
        while !self.rx_empty() {
            _ = field!(self.registers, data).read();
        }
    }
}

/// UART reply errors
pub enum UARTReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Converts from a `UARTReplyError` to a `u32`
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

/// Converts from a `HeaderError` to a `UARTReplyError`
impl From<HeaderError> for UARTReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

/// UART Errors
pub enum UARTError {
    /// Error with the request
    ReplyError(UARTReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts from a `UARTReplyError` to a `UARTError`
impl From<UARTReplyError> for UARTError {
    fn from(value: UARTReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts from a `QueueError` to a `UARTError`
impl From<QueueError> for UARTError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// Converts from a `HeaderError` to a `UARTError`
impl From<HeaderError> for UARTError {
    fn from(value: HeaderError) -> Self {
        Self::from(UARTReplyError::from(value))
    }
}

/// UART send or receive request data
pub struct DataRequest {
    /// Request buffer
    buffer: [u8; 32],
    /// Request buffer length
    len: usize
}

/// UART request
pub enum Request {
    /// Send request
    Send(DataRequest),
    /// Receive request
    Receive(DataRequest),
    /// Clear n bytes from the RX fifo
    ClearRX(u32),
    /// Clear all bytes from the RX fifo
    ClearAllRX,
}

impl Request {
    /// Parses the next request  
    /// Returns the request on success or a `UARTError` on failure
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

/// IO Bank 0 driver endpoint
const IO_BANK0_QUEUE: u32 = 0;

/// Driver entry point
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
