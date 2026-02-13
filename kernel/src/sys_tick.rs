use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPureWrite, ReadWrite}};

use crate::mutex::IRQMutex;

#[repr(C)]
struct SysTickRegisters {
    ctrl_status: ReadWrite<u32>,
    reload_value: ReadPureWrite<u32>,
    current_value: ReadPureWrite<u32>,
    callibration: ReadPureWrite<u32>
}

mod ctrl_status_register {
    pub const ENABLE_SHIFT: usize = 0;
    pub const TICKINT_SHIFT: usize = 1;
    pub const CLOCK_SRC_SHIFT: usize = 2;
    pub const COUNT_FLAG_SHIFT: usize = 16;

    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const TICKINT_MASK: u32 = 1 << TICKINT_SHIFT;
    pub const CLOCK_SRC_MASK: u32 = 1 << CLOCK_SRC_SHIFT;
    pub const COUNT_FLAG_MASK: u32 = 1 << COUNT_FLAG_SHIFT;

    pub const CLOCK_SRC_EXTERNAL: u32 = 0 << CLOCK_SRC_SHIFT;
    pub const CLOCK_SRC_PROCESSOR: u32 = 1 << CLOCK_SRC_SHIFT;
}

mod reload_value_register {
    pub const RELOAD_SHIFT: usize = 0;
    
    pub const RELOAD_MASK: u32 = 0xffffff << RELOAD_SHIFT;
}

mod current_value_register {
    pub const CURRENT_SHIFT: usize = 0;

    pub const CURRENT_MASK: u32 = 0xffffff << CURRENT_SHIFT;
}

mod callibration_register {
    pub const TEN_MS_SHIFT: usize = 0;
    pub const SKEW_SHIFT: usize = 30;
    pub const NOREF_SHIFT: usize = 31;

    pub const TEN_MS_MASK: u32 = 0xffffff << TEN_MS_SHIFT;
    pub const SKEW_MASK: u32 = 1 << SKEW_SHIFT;
    pub const NOREF_MASK: u32 = 1 << NOREF_SHIFT;
}

pub struct SysTick {
    registers: UniqueMmioPointer<'static, SysTickRegisters>
}

impl SysTick {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    /// We don't use the SysTick timer
    pub fn disable(&mut self) {
        field!(self.registers, ctrl_status).write(0);
    }
}

unsafe impl Send for SysTick {}
unsafe impl Sync for SysTick {}

static SYS_TICK_BASE: usize = 0xe000e010;

pub static SYS_TICK: IRQMutex<SysTick> = unsafe {
    IRQMutex::new(SysTick::new(SYS_TICK_BASE))
};
