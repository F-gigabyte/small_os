use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite}};

use crate::mutex::SpinIRQ;

#[repr(C)]
struct ROSCRegisters {
    ctrl: ReadPureWrite<u32>, // 0x0
    freqa: ReadPureWrite<u32>, // 0x4
    freqb: ReadPureWrite<u32>, // 0x8
    dormant: ReadPureWrite<u32>, // 0xc
    div: ReadPureWrite<u32>, // 0x10
    phase: ReadPureWrite<u32>, // 0x14
    status: ReadPureWrite<u32>, // 0x18
    random_bit: ReadPure<u32>, //0x1c
    count: ReadPureWrite<u32>, // 0x20
}

mod ctrl_register {
    pub const FREQ_RANGE_SHIFT: usize = 0;
    pub const ENABLE_SHIFT: usize = 12;

    pub const FREQ_RANGE_MASK: u32 = 0xfff << FREQ_RANGE_SHIFT;
    pub const ENABLE_MASK: u32 = 0xfff << ENABLE_SHIFT;

    pub const FREQ_LOW: u32 = 0xfa4 << FREQ_RANGE_SHIFT;
    pub const FREQ_MEDIUM: u32 = 0xfa5 << FREQ_RANGE_SHIFT;
    pub const FREQ_HIGH: u32 = 0xfa7 << FREQ_RANGE_SHIFT;
    pub const FREQ_TOOHIGH: u32 = 0xfa6 << FREQ_RANGE_SHIFT;

    pub const ENABLE_DISABLE: u32 = 0xd1e << ENABLE_SHIFT;
    pub const ENABLE_ENABLE: u32 = 0xfab << ENABLE_SHIFT;
}

mod freqa_register {
    pub const DS0_SHIFT: usize = 0;
    pub const DS1_SHIFT: usize = 4;
    pub const DS2_SHIFT: usize = 8;
    pub const DS3_SHIFT: usize = 12;
    pub const PASSWD_SHIFT: usize = 16;

    pub const DS0_MASK: u32 = 0x5 << DS0_SHIFT;
    pub const DS1_MASK: u32 = 0x5 << DS1_SHIFT;
    pub const DS2_MASK: u32 = 0x5 << DS2_SHIFT;
    pub const DS3_MASK: u32 = 0x5 << DS3_SHIFT;
    pub const PASSWD_MASK: u32 = 0xffff << PASSWD_SHIFT;
    
    pub const PASSWD_PASS: u32 = 0x9696 << PASSWD_SHIFT;
}

mod freqb_register {
    pub const DS4_SHIFT: usize = 0;
    pub const DS5_SHIFT: usize = 4;
    pub const DS6_SHIFT: usize = 8;
    pub const DS7_SHIFT: usize = 12;
    pub const PASSWD_SHIFT: usize = 16;

    pub const DS4_MASK: u32 = 0x5 << DS4_SHIFT;
    pub const DS5_MASK: u32 = 0x5 << DS5_SHIFT;
    pub const DS6_MASK: u32 = 0x5 << DS6_SHIFT;
    pub const DS7_MASK: u32 = 0x5 << DS7_SHIFT;
    pub const PASSWD_MASK: u32 = 0xffff << PASSWD_SHIFT;
    
    pub const PASSWD_PASS: u32 = 0x9696 << PASSWD_SHIFT;
}

mod dormant_register {
    pub const DORMANT: u32 = 0x636f6d61;
    pub const WAKE: u32 = 0x77616b65;
}

mod div_register {
    pub const DIV_SHIFT: usize = 0;

    pub const DIV_MASK: u32 = 0xfff << DIV_SHIFT;

    pub const DIV_MIN: u32 = 0xaa0 << DIV_SHIFT;
    pub const DIV_MAX: u32 = DIV_MIN + (0x31 << DIV_SHIFT);
}

mod phase_register {
    pub const SHIFT_SHIFT: usize = 0;
    pub const FLIP_SHIFT: usize = 2;
    pub const ENABLE_SHIFT: usize = 3;
    pub const PASSWD_SHIFT: usize = 4;

    pub const SHIFT_MASK: u32 = 0x3 << SHIFT_SHIFT;
    pub const FLIP_MASK: u32 = 1 << FLIP_SHIFT;
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const PASSWD_MASK: u32 = 0xff << PASSWD_SHIFT;

    pub const PASSWD_PASS: u32 = 0xaa << PASSWD_SHIFT;
}

mod status_register {
    pub const ENABLED_SHIFT: usize = 12;
    pub const DIV_RUNNING_SHIFT: usize = 16;
    pub const BADWRITE_SHIFT: usize = 24;
    pub const STABLE_SHIFT: usize = 31;

    pub const ENABLED_MASK: u32 = 1 << ENABLED_SHIFT;
    pub const DIV_RUNNING_MASK: u32 = 1 << DIV_RUNNING_SHIFT;
    pub const BADWRITE_MASK: u32 = 1 << BADWRITE_SHIFT;
    pub const STABLE_MASK: u32 = 1 << STABLE_SHIFT;
}

mod random_bit_register {
    pub const RANDOM_BIT_SHIFT: usize = 0;

    pub const RANDOM_BIT_MASK: u32 = 1 << RANDOM_BIT_SHIFT;
}

mod count_register {
    pub const COUNT_SHIFT: usize = 0;

    pub const COUNT_MASK: u32 = 0xff << COUNT_SHIFT;
}

pub struct ROSC {
    registers: UniqueMmioPointer<'static, ROSCRegisters>
}

impl ROSC {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    pub fn disable(&mut self) {
        field!(self.registers, ctrl).write(ctrl_register::ENABLE_DISABLE);
    }
}

unsafe impl Send for ROSC {}
unsafe impl Sync for ROSC {}

static ROSC_BASE: usize = 0x40060000;

pub static ROSC: SpinIRQ<ROSC> = unsafe {
    SpinIRQ::new(ROSC::new(ROSC_BASE))
};
