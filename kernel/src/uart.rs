/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Kernel.
 *
 * The SmallOS Kernel is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Kernel is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS Kernel. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

use core::{
    fmt::Write,
    ptr::{self, NonNull},
    sync::atomic::{Ordering, compiler_fence},
};

use safe_mmio::{
    UniqueMmioPointer, field,
    fields::{ReadPure, ReadPureWrite, ReadWrite, WriteOnly},
};

use crate::{mmio::REG_ALIAS_SET_BITS, mutex::SpinIRQ, wait};

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
    _reserved6: u32, // 0x34
    /// UART interrupt mask set clear register (0x38)
    mask_set_clr: ReadPureWrite<u32>, // 0x38
    /// UART raw interrupt register (0x3c)
    raw_inter: ReadPureWrite<u32>, // 0x3c
    /// UART masked interrupt register (0x40)
    mask_inter: ReadPure<u32>, // 0x40
    /// UART interrupt clear register (0x44)
    inter_clr: WriteOnly<u32>, // 0x44
}

/// UART object for managing a UART device
pub struct UART {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, UARTRegisters>,
    /// Memory mapped registers where writing a bit sets the corresponding bit in `registers`
    set_reg: UniqueMmioPointer<'static, UARTRegisters>,
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
    pub const VALID_MASK: u32 =
        DATA_MASK | FRAMING_ERROR_MASK | PARITY_ERROR_MASK | BREAK_ERROR_MASK | OVERRUN_ERROR_MASK;
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
    pub const VALID_MASK: u32 =
        FRAMING_ERROR_MASK | PARITY_ERROR_MASK | BREAK_ERROR_MASK | OVERRUN_ERROR_MASK;
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
    pub const VALID_MASK: u32 = CTS_MASK
        | DSR_MASK
        | DCD_MASK
        | BUSY_MASK
        | RXFE_MASK
        | TXFF_MASK
        | RXFF_MASK
        | TXFE_MASK
        | RI_MASK;
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
    pub const VALID_MASK: u32 =
        BRK_MASK | PEN_MASK | EPS_MASK | STP2_MASK | FEN_MASK | WLEN_MASK | SPS_MASK;
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
    pub const VALID_MASK: u32 = UARTEN_MASK
        | SIREN_MASK
        | SIRLP_MASK
        | LBE_MASK
        | TXE_MASK
        | RXE_MASK
        | DTR_MASK
        | RTS_MASK
        | OUT1_MASK
        | OUT2_MASK
        | RTSEN_MASK
        | CTSEN_MASK;
}

/// UART interrupt registers masks and shifts
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
    pub const VALID_MASK: u32 = RI_MASK
        | CTS_MASK
        | DCD_MASK
        | DSR_MASK
        | RX_MASK
        | TX_MASK
        | RT_MASK
        | FE_MASK
        | PE_MASK
        | BE_MASK
        | OE_MASK;
}

impl UART {
    /// Creates a new `UART` object  
    /// `base` is the base address of the UART memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the UART memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(
                    NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap(),
                ),
                set_reg: UniqueMmioPointer::new(
                    NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS))
                        .unwrap(),
                )
            }
        }
    }

    /// Resets the UART ready for printing
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
    pub fn reset(&mut self) {
        // disable UART
        field!(self.registers, ctrl).modify(|ctrl| ctrl & !ctrl_register::VALID_MASK);
        // clear all interrupts
        field!(self.registers, inter_clr).write(interrupt_register::VALID_MASK);

        // set baud rate to 115200
        // integer baud rate should be 26 and fractional should be 3 for a 48MHz clock
        field!(self.registers, int_baud).modify(|baud| {
            (baud & !int_baud_rate_register::VALID_MASK)
                | (26 << int_baud_rate_register::BAUD_DIVINT_SHIFT)
        });
        field!(self.registers, frac_baud).modify(|baud| {
            (baud & !frac_baud_rate_register::VALID_MASK)
                | (3 << frac_baud_rate_register::BAUD_DIVFRAC_SHIFT)
        });
        field!(self.registers, line_ctrl).modify(|line_ctrl| {
            (line_ctrl & !line_ctrl_register::VALID_MASK)
                | line_ctrl_register::FEN_MASK
                | line_ctrl_register::WLEN_8
        });
        // mask all interrupts (no interrupts will be generated)
        field!(self.registers, mask_set_clr).modify(|mask| mask & !interrupt_register::VALID_MASK);
        // enable UART and TX section
        field!(self.registers, ctrl).modify(|ctrl| {
            (ctrl & !ctrl_register::VALID_MASK)
                | ctrl_register::TXE_MASK
                | ctrl_register::UARTEN_MASK
        });
        wait::wait_cycles(10000);
    }

    /// Waits for the transmit fifo to be non full
    #[inline(always)]
    fn wait(&mut self) {
        while (field!(self.registers, flag).read() & flag_register::TXFF_MASK) != 0 {}
    }

    /// Writes a byte into the transmit fifo
    #[inline(always)]
    fn put_byte(&mut self, val: u8) {
        self.wait();
        compiler_fence(Ordering::AcqRel);
        field!(self.registers, data)
            .write(((val as u32) << data_register::DATA_SHIFT) & data_register::DATA_MASK);
    }

    /// Writes a string into the transmit fifo
    #[inline(never)]
    pub fn put_str(&mut self, text: &str) {
        for byte in text.as_bytes() {
            assert!(byte.is_ascii());
            if *byte == b'\n' {
                self.put_byte(b'\n');
                self.put_byte(b'\r');
            } else if *byte != b'\r' {
                self.put_byte(*byte);
            }
        }
    }
}


// Based off https://os.phil-opp.com/vga-text-mode/ accessed 22/01/2026
impl Write for UART {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.put_str(s);
        Ok(())
    }
}

/// SAFETY
/// has no internal data so is safe to send between threads
unsafe impl Send for UART {}
unsafe impl Sync for UART {}

/// Base address for the UART memory mapped registers
static UART1_BASE: usize = 0x40038000;

/// UART object
pub static UART1: SpinIRQ<UART> = unsafe { SpinIRQ::new(UART::new(UART1_BASE)) };

#[cfg(test)]
mod test {
    use crate::{inter::CS, print, println};

    use super::*;

    #[test_case]
    fn test_setup_correct() {
        {
            println!("Testing UART setup");
        }
        let cs = unsafe { CS::new() };
        let mut uart = UART1.lock(&cs);
        write!(uart, "Testing int baud register ").unwrap();
        let int_baud = field!(uart.registers, int_baud).read();
        assert_eq!(int_baud & int_baud_rate_register::VALID_MASK, 26);
        write!(uart, "[ok]\n").unwrap();
        write!(uart, "Testing frac baud register ").unwrap();
        let frac_baud = field!(uart.registers, frac_baud).read();
        assert_eq!(frac_baud & frac_baud_rate_register::VALID_MASK, 3);
        write!(uart, "[ok]\n").unwrap();
        write!(uart, "Testing line ctrl register ").unwrap();
        let line_ctrl = field!(uart.registers, line_ctrl).read();
        assert_eq!(line_ctrl & line_ctrl_register::VALID_MASK, 0x70);
        write!(uart, "[ok]\n").unwrap();
        write!(uart, "Testing ctrl register ").unwrap();
        let ctrl = field!(uart.registers, ctrl).read();
        assert_eq!(ctrl & ctrl_register::VALID_MASK, 0x101);
        write!(uart, "[ok]\n").unwrap();
        write!(uart, "Testing mask set clr register ").unwrap();
        let mask = field!(uart.registers, mask_set_clr).read();
        assert_eq!(mask & interrupt_register::VALID_MASK, 0);
        write!(uart, "[ok]\n").unwrap();
    }

    #[test_case]
    fn test_valid() {
        println!("Testing UART register mask values");
        print!("Testing data register ");
        assert_eq!(data_register::VALID_MASK, 0xfff);
        println!("[ok]");
        print!("Testing receive status register ");
        assert_eq!(receive_status_register::VALID_MASK, 0xf);
        println!("[ok]");
        print!("Testing flag register ");
        assert_eq!(flag_register::VALID_MASK, 0x1ff);
        println!("[ok]");
        print!("Testing int baud rate register ");
        assert_eq!(int_baud_rate_register::VALID_MASK, 0xffff);
        println!("[ok]");
        print!("Testing frac baud rate register ");
        assert_eq!(frac_baud_rate_register::VALID_MASK, 0x3f);
        println!("[ok]");
        print!("Testing line ctrl register ");
        assert_eq!(line_ctrl_register::VALID_MASK, 0xff);
        println!("[ok]");
        print!("Testing ctrl register ");
        assert_eq!(ctrl_register::VALID_MASK, 0xff87);
        println!("[ok]");
        print!("Testing interrupt register ");
        assert_eq!(interrupt_register::VALID_MASK, 0x7ff);
        println!("[ok]");
    }
}
