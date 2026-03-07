use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};

use crate::{mmio::REG_ALIAS_SET_BITS, mutex::SpinIRQ};

// https://github.com/dwelch67/raspberrypi-pico/blob/main/uart01/notmain.c accessed 21/01/2026

#[repr(C)]
struct XOSCRegisters {
    ctrl: ReadPureWrite<u32>,
    status: ReadPureWrite<u32>,
    dormant: ReadPureWrite<u32>,
    startup: ReadPureWrite<u32>,
}

mod ctrl_register {
    pub const FREQ_RANGE_SHIFT: usize = 0;
    pub const ENABLE_SHIFT: usize = 12;
    
    pub const FREQ_RANGE_MASK: u32 = 0xfff << FREQ_RANGE_SHIFT;
    pub const ENABLE_MASK: u32 = 0xfff << ENABLE_SHIFT;

    pub const FREQ_RANGE_1_15MHZ: u32 = 0xaa0 << FREQ_RANGE_SHIFT;

    pub const ENABLE_ENABLE: u32 = 0xfab << ENABLE_SHIFT;
    pub const ENABLE_DISABLE: u32 = 0xd1e << ENABLE_SHIFT;
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
}

mod dormant_register {
    pub const DORMANT: u32 = 0x636f6d61;
    pub const WAKE: u32 = 0x77616b65;
}

mod startup_register {
    pub const DELAY_SHIFT: usize = 0;
    pub const X4_SHIFT: usize = 20;

    pub const DELAY_MASK: u32 = 0x3fff << DELAY_SHIFT;
    pub const X4_MASK: u32 = 1 << X4_SHIFT;
}

mod count_register {
    pub const COUNTER_SHIFT: usize = 0;

    pub const COUNTER_MASK: u32 = 0xff << COUNTER_SHIFT;
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
        field!(self.registers, startup).write(47);
        field!(self.registers, dormant).write(dormant_register::WAKE);
        field!(self.registers, ctrl).write(ctrl_register::FREQ_RANGE_1_15MHZ);
        field!(self.set_reg, ctrl).write(ctrl_register::ENABLE_ENABLE);
        while field!(self.registers, status).read() & status_register::STABLE_MASK == 0 {}
    }
}

unsafe impl Send for XOSC {}
unsafe impl Sync for XOSC {}

static XOSC_BASE: usize = 0x40024000;

pub static XOSC: SpinIRQ<XOSC> = unsafe {
    SpinIRQ::new(XOSC::new(XOSC_BASE))
};
