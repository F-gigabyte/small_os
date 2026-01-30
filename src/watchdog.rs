use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};

use crate::mutex::SpinIRQ;

#[repr(C)]
struct WatchdogRegisters {
    _reserved0: u32, // 0x0
    _reserved1: u32, // 0x4
    _reserved2: u32, // 0x8
    _reserved3: u32, // 0xc
    _reserved4: u32, // 0x10
    _reserved5: u32, // 0x14
    _reserved6: u32, // 0x18
    _reserved7: u32, // 0x1c
    _reserved8: u32, // 0x20
    _reserved9: u32, // 0x24
    _reserved10: u32, // 0x28
    tick: ReadPureWrite<u32> // 0x2c
}

mod tick_register {
    pub const CYCLES_SHIFT: usize = 0;
    pub const ENABLE_SHIFT: usize = 9;
    pub const RUNNING_SHIFT: usize = 10;
    pub const COUNT_SHIFT: usize = 11;

    pub const CYCLES_MASK: u32 = 0x1ff << CYCLES_SHIFT;
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const RUNNING_MASK: u32 = 1 << RUNNING_SHIFT;
    pub const COUNT_MASK: u32 = 0x1ff << COUNT_SHIFT;
}

pub struct Watchdog {
    registers: UniqueMmioPointer<'static, WatchdogRegisters>
}

impl Watchdog {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    pub fn enable_ticks(&mut self) {
        field!(self.registers, tick).write(tick_register::ENABLE_MASK | (12 << tick_register::CYCLES_SHIFT));
    }

    pub fn get_counter(&mut self) -> u32 {
        (field!(self.registers, tick).read() & tick_register::COUNT_MASK) >> tick_register::COUNT_SHIFT
    }
}

unsafe impl Send for Watchdog {}
unsafe impl Sync for Watchdog {}

static WATCHDOG_BASE: usize = 0x40058000;

pub static WATCHDOG: SpinIRQ<Watchdog> = unsafe {
    SpinIRQ::new(Watchdog::new(WATCHDOG_BASE))
};
