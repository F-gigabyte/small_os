use core::{arch::asm, ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};

use crate::{mmio::REG_ALIAS_SET_BITS, mutex::SpinIRQ};

// https://github.com/dwelch67/raspberrypi-pico/blob/main/uart01/notmain.c accessed 21/01/2026

#[repr(C)]
struct XOSCRegisters {
    ctrl: ReadPureWrite<u32>, // 0x0
    status: ReadPureWrite<u32>, // 0x4
    dormant: ReadPureWrite<u32>, // 0x8
    startup: ReadPureWrite<u32>, // 0xc
    _reserved0: u32, // 0x10
    _reserved1: u32, // 0x14
    _reserved2: u32, // 0x18
    count: ReadPureWrite<u32> // 0x1c
}

mod ctrl_register {
    pub const FREQ_RANGE_SHIFT: usize = 0;
    pub const ENABLE_SHIFT: usize = 12;
    
    pub const FREQ_RANGE_MASK: u32 = 0xfff << FREQ_RANGE_SHIFT;
    pub const ENABLE_MASK: u32 = 0xfff << ENABLE_SHIFT;

    pub const FREQ_RANGE_1_15MHZ: u32 = 0xaa0 << FREQ_RANGE_SHIFT;

    pub const ENABLE_ENABLE: u32 = 0xfab << ENABLE_SHIFT;
    pub const ENABLE_DISABLE: u32 = 0xd1e << ENABLE_SHIFT;

    pub const VALID_MASK: u32 = FREQ_RANGE_MASK | 
        ENABLE_MASK;
}

mod status_register {
    pub const FREQ_RANGE_SHIFT: usize = 0;
    pub const ENABLED_SHIFT: usize = 12;
    pub const BADWRITE_SHIFT: usize = 24;
    pub const STABLE_SHIFT: usize = 31;
    
    pub const FREQ_RANGE_MASK: u32 = 0x3 << FREQ_RANGE_SHIFT;
    pub const ENABLED_MASK: u32 = 1 << ENABLED_SHIFT;
    pub const BADWRITE_MASK: u32 = 1 << BADWRITE_SHIFT;
    pub const STABLE_MASK: u32 = 1 << STABLE_SHIFT;

    pub const FREQ_RANGE_1_15MHZ: u32 = 0x0 << FREQ_RANGE_SHIFT;

    pub const VALID_MASK: u32 = FREQ_RANGE_MASK | 
        ENABLED_MASK |
        BADWRITE_MASK |
        STABLE_MASK;
}

mod dormant_register {
    pub const VALUE_SHIFT: usize = 0;

    pub const VALUE_MASK: u32 = 0xffffffff << VALUE_SHIFT;

    pub const DORMANT: u32 = 0x636f6d61 << VALUE_SHIFT;
    pub const WAKE: u32 = 0x77616b65 << VALUE_SHIFT;

    pub const VALID_MASK: u32 = VALUE_MASK;
}

mod startup_register {
    pub const DELAY_SHIFT: usize = 0;
    pub const X4_SHIFT: usize = 20;

    pub const DELAY_MASK: u32 = 0x3fff << DELAY_SHIFT;
    pub const X4_MASK: u32 = 1 << X4_SHIFT;

    pub const VALID_MASK: u32 = DELAY_MASK |
        X4_MASK;
}

mod count_register {
    pub const COUNTER_SHIFT: usize = 0;

    pub const COUNTER_MASK: u32 = 0xff << COUNTER_SHIFT;

    pub const VALID_MASK: u32 = COUNTER_MASK;
}

pub struct XOSC {
    registers: UniqueMmioPointer<'static, XOSCRegisters>,
    set_reg: UniqueMmioPointer<'static, XOSCRegisters>
}

impl XOSC {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap())
            }
        }
    }

    pub fn reset(&mut self) {
        field!(self.registers, startup).modify(|startup| (startup & !startup_register::VALID_MASK) | (47 << startup_register::DELAY_SHIFT));
        field!(self.registers, dormant).modify(|dormant| (dormant &!dormant_register::VALID_MASK) | dormant_register::WAKE);
        field!(self.registers, ctrl).modify(|ctrl| (ctrl & !ctrl_register::VALID_MASK) | ctrl_register::FREQ_RANGE_1_15MHZ | ctrl_register::ENABLE_ENABLE);
        while field!(self.registers, status).read() & status_register::STABLE_MASK == 0 {}
    }

    /// Waits for `cycles` / 12 us 
    pub fn wait_cycles(&mut self, cycles: u8) {
        field!(self.registers, count).modify(|count| (count & !count_register::VALID_MASK) | ((cycles as u32) << count_register::COUNTER_SHIFT) & count_register::COUNTER_MASK);
        while field!(self.registers, count).read() != 0 {}
    }
}

unsafe impl Send for XOSC {}
unsafe impl Sync for XOSC {}

static XOSC_BASE: usize = 0x40024000;

pub static XOSC: SpinIRQ<XOSC> = unsafe {
    SpinIRQ::new(XOSC::new(XOSC_BASE))
};

#[cfg(test)]
mod test {
    use crate::{inter::CS, print, println};

    use super::*;

    #[test_case]
    fn test_setup() {
        println!("Testing XOSC setup");
        let cs = unsafe {
            CS::new()
        };
        let mut xosc = XOSC.lock(&cs);
        print!("Testing startup register ");
        let startup = field!(xosc.registers, startup).read();
        assert_eq!(startup & startup_register::VALID_MASK, 0x2f);
        println!("[ok]");
        print!("Testing ctrl register ");
        let ctrl = field!(xosc.registers, ctrl).read();
        assert_eq!(ctrl & ctrl_register::VALID_MASK, 0xfabaa0);
        println!("[ok]");
        print!("Testing status register ");
        let status = field!(xosc.registers, status).read();
        // don't check badwrite due to hardware bugs (RP2040-E10) and freq range mask which appears
        // to read as 1 instead of 0 even though the crystal frequency only supports 12MHz (issue
        // noticed by dfrp at https://forums.raspberrypi.com/viewtopic.php?t=371238, accessed
        // 6/04/2026)
        assert_eq!((status & status_register::VALID_MASK) & !status_register::BADWRITE_MASK & !status_register::FREQ_RANGE_MASK, 0x80001000);
        println!("[ok]");
        print!("Testing dormant register ");
        let dormant = field!(xosc.registers, dormant).read();
        assert_eq!(dormant & dormant_register::VALID_MASK, 0x77616b65);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing XOSC register mask values");
        print!("Testing ctrl register ");
        assert_eq!(ctrl_register::VALID_MASK, 0xffffff);
        println!("[ok]");
        print!("Testing status register ");
        assert_eq!(status_register::VALID_MASK, 0x81001003);
        println!("[ok]");
        print!("Testing dormant register ");
        assert_eq!(dormant_register::VALID_MASK, 0xffffffff);
        println!("[ok]");
        print!("Testing startup register ");
        assert_eq!(startup_register::VALID_MASK, 0x103fff);
        println!("[ok]");
        print!("Testing count register ");
        assert_eq!(count_register::VALID_MASK, 0xff);
        println!("[ok]");
    }
}
