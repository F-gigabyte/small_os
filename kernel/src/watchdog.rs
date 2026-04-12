/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS kernel.
 *
 * The SmallOS kernel is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS kernel is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS kernel. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};

use crate::mutex::SpinIRQ;

/// Watchdog memory mapped registers
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
    /// Watchdog tick register (0x2c)
    tick: ReadPureWrite<u32> // 0x2c
}

/// Watchdog tick register masks and shifts
mod tick_register {
    /// Cycles per tick shift
    pub const CYCLES_SHIFT: usize = 0;
    /// Shift for enabling tick generation
    pub const ENABLE_SHIFT: usize = 9;
    /// Tick generation running shift
    pub const RUNNING_SHIFT: usize = 10;
    /// Count down timer shift
    pub const COUNT_SHIFT: usize = 11;

    /// Cycles per tick mask
    pub const CYCLES_MASK: u32 = 0x1ff << CYCLES_SHIFT;
    /// Mask for enabling tick generation
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Tick generation running mask
    pub const RUNNING_MASK: u32 = 1 << RUNNING_SHIFT;
    /// Count down timer mask
    pub const COUNT_MASK: u32 = 0x1ff << COUNT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = CYCLES_MASK |
        ENABLE_MASK |
        RUNNING_MASK |
        COUNT_MASK;
}

/// Watchdog object for managing the watchdog timer
pub struct Watchdog {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, WatchdogRegisters>
}

impl Watchdog {
    /// Creates a new `Watchdog` object  
    /// `base` is the base address of the Watchdog memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the Watchdog memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    /// Enables watchdog tick generation
    pub fn enable_ticks(&mut self) {
        field!(self.registers, tick).modify(|tick| (tick & !tick_register::VALID_MASK) | tick_register::ENABLE_MASK | (12 << tick_register::CYCLES_SHIFT));
    }

    /// Gets the watchdog count down value
    pub fn get_counter(&mut self) -> u32 {
        (field!(self.registers, tick).read() & tick_register::COUNT_MASK) >> tick_register::COUNT_SHIFT
    }
}

unsafe impl Send for Watchdog {}
unsafe impl Sync for Watchdog {}

/// Base address for the Watchdog memory mapped registers
static WATCHDOG_BASE: usize = 0x40058000;

/// Watchdog object
pub static WATCHDOG: SpinIRQ<Watchdog> = unsafe {
    SpinIRQ::new(Watchdog::new(WATCHDOG_BASE))
};

#[cfg(test)]
mod test {
    use crate::{inter::CS, print, println};

    use super::*;

    #[test_case]
    fn test_setup() {
        println!("Testing watchdog setup");
        let cs = unsafe {
            CS::new()
        };
        let mut watchdog = WATCHDOG.lock(&cs);
        print!("Testing tick register ");
        let tick = field!(watchdog.registers, tick).read();
        // Running, enabled and with 12 cycles for a 1MHz clock
        assert_eq!(tick & tick_register::VALID_MASK & !tick_register::COUNT_MASK, 0x60c);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing watchdog register mask values");
        print!("Testing tick register ");
        assert_eq!(tick_register::VALID_MASK, 0xfffff);
        println!("[ok]");
    }
}
