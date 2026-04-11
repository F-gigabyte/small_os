use core::{fmt::Write, ptr::{self, NonNull}, sync::atomic::{Ordering, compiler_fence}};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, ReadWrite, WriteOnly}};

use crate::{mmio::REG_ALIAS_SET_BITS, mutex::SpinIRQ, wait};

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
    _reserved6: u32, // 0x34
    mask_set_clr: ReadPureWrite<u32>, // 0x38
    raw_inter: ReadPureWrite<u32>, // 0x3c
    mask_inter: ReadPure<u32>, // 0x40
    inter_clr: WriteOnly<u32> // 0x44
}

pub struct UART {
    registers: UniqueMmioPointer<'static, UARTRegisters>,
    set_reg: UniqueMmioPointer<'static, UARTRegisters>,
    iter: usize
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

impl UART {
    /// Creates a UART
    /// SAFETY
    /// `base` must be a valid UART base and has not been used anywhere else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap()),
                iter: 0
            }
        }
    }

    pub fn reset(&mut self) {
        // disable UART
        field!(self.registers, ctrl).modify(|ctrl| ctrl & !ctrl_register::VALID_MASK);
        // clear all interrupts
        field!(self.registers, inter_clr).write(interrupt_register::VALID_MASK);

        // set baud rate to 115200
        // integer baud rate should be 26 and fractional should be 3 for a 48MHz clock
        field!(self.registers, int_baud).modify(|baud| (baud & !int_baud_rate_register::VALID_MASK) | (26 << int_baud_rate_register::BAUD_DIVINT_SHIFT));
        field!(self.registers, frac_baud).modify(|baud| (baud & !frac_baud_rate_register::VALID_MASK) | (3 << frac_baud_rate_register::BAUD_DIVFRAC_SHIFT));
        field!(self.registers, line_ctrl).modify(|line_ctrl| (line_ctrl & !line_ctrl_register::VALID_MASK) | line_ctrl_register::FEN_MASK | line_ctrl_register::WLEN_8);
        // mask all interrupts (no interrupts will be generated)
        field!(self.registers, mask_set_clr).modify(|mask| mask & !interrupt_register::VALID_MASK);
        // enable UART and TX section
        field!(self.registers, ctrl).modify(|ctrl| (ctrl & !ctrl_register::VALID_MASK) | ctrl_register::TXE_MASK | ctrl_register::UARTEN_MASK);
        wait::wait_cycles(10000);
    }

    #[inline(always)]
    fn wait(&mut self) {
        // clear TX fifo every 64 cycles due to not exact baud rate
        while (field!(self.registers, flag).read() & flag_register::TXFF_MASK) != 0 {}
    }

    #[inline(always)]
    fn put_byte(&mut self, val: u8) {
        self.wait();
        compiler_fence(Ordering::AcqRel);
        field!(self.registers, data).write(((val as u32) << data_register::DATA_SHIFT) & data_register::DATA_MASK);
    }

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

// https://os.phil-opp.com/vga-text-mode/ accessed 22/01/2026
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

static UART1_BASE: usize = 0x40038000;

pub static UART1: SpinIRQ<UART> = unsafe {
    SpinIRQ::new(UART::new(UART1_BASE))
};

#[cfg(test)]
mod test {
    use crate::{inter::CS, print, println};

    use super::*;

    #[test_case]
    fn test_setup_correct() {
        {
            println!("Testing UART setup");
        }
        let cs = unsafe {
            CS::new()
        };
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
