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

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPureWrite, ReadWrite}};

use crate::{inter::CS, mutex::IRQMutex, proc::Proc};

/// System Tick memory mapped registers
#[repr(C)]
struct SysTickRegisters {
    /// System tick control and status register (0x00)
    ctrl_status: ReadWrite<u32>, // 0x0
    /// System tick reload value register (0x04)
    reload_value: ReadPureWrite<u32>, // 0x4
    /// System tick current value register (0x08)
    current_value: ReadPureWrite<u32>, // 0x8
    /// System tick callibration register (0x0c)
    callibration: ReadPureWrite<u32> // 0xc
}

/// System Tick control and status register masks and shifts
mod ctrl_status_register {
    /// Shift to enable the System Tick
    pub const ENABLE_SHIFT: usize = 0;
    /// Shift to enable System Tick interrupts
    pub const TICKINT_SHIFT: usize = 1;
    /// Shift for determining the System Tick clock source
    pub const CLOCK_SRC_SHIFT: usize = 2;
    /// Shift for determining if the System Tick counted to 0 since the last time this was read
    pub const COUNT_FLAG_SHIFT: usize = 16;

    /// Mask to enable the System Tick
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Mask to enable System Tick interrupts
    pub const TICKINT_MASK: u32 = 1 << TICKINT_SHIFT;
    /// Mask for determining the System Tick clock source
    pub const CLOCK_SRC_MASK: u32 = 1 << CLOCK_SRC_SHIFT;
    /// Mask for determining if the System Tick counted to 0 since the last time this was read
    pub const COUNT_FLAG_MASK: u32 = 1 << COUNT_FLAG_SHIFT;

    /// Clock source comes from outside the processor
    pub const CLOCK_SRC_EXTERNAL: u32 = 0 << CLOCK_SRC_SHIFT;
    /// Clock source comes from the processor
    pub const CLOCK_SRC_PROCESSOR: u32 = 1 << CLOCK_SRC_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ENABLE_MASK |
        TICKINT_MASK |
        CLOCK_SRC_MASK |
        COUNT_FLAG_MASK;
}

/// System Tick reload value register masks and shifts
mod reload_value_register {
    /// Reload value shift
    pub const RELOAD_SHIFT: usize = 0;
    
    /// Reload value mask
    pub const RELOAD_MASK: u32 = 0xffffff << RELOAD_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = RELOAD_MASK;
}

/// System Tick current value register masks and shifts
mod current_value_register {
    /// Current value shift
    pub const CURRENT_SHIFT: usize = 0;

    /// Current value mask
    pub const CURRENT_MASK: u32 = 0xffffff << CURRENT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = CURRENT_MASK;
}

/// System Tick callibration register masks and shifts
mod callibration_register {
    /// Shift for value that can be used for 10ms timing
    pub const TEN_MS_SHIFT: usize = 0;
    /// Shift for if value for 10ms timing is inexact
    pub const SKEW_SHIFT: usize = 30;
    /// Shift for whether there is no reference clock
    pub const NOREF_SHIFT: usize = 31;

    /// Mask for value that can be used for 10ms timing
    pub const TEN_MS_MASK: u32 = 0xffffff << TEN_MS_SHIFT;
    /// Mask for if value for 10ms timing is inexact
    pub const SKEW_MASK: u32 = 1 << SKEW_SHIFT;
    /// Mask for whether there is no reference clock
    pub const NOREF_MASK: u32 = 1 << NOREF_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = TEN_MS_MASK |
        SKEW_MASK |
        NOREF_MASK;
}

/// System Tick object for managing the System Tick
pub struct SysTick {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, SysTickRegisters>
}

impl SysTick {
    /// Creates a new `SysTick` object  
    /// `base` is the base address of the System Tick memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the System Tick memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    /// Initialises the System Tick ready to be used to interrupt processes  
    /// Is able to operate at a 1 micro second granularity
    #[inline(always)]
    pub fn init(&mut self) {
        // Use external source so will run at a fixed 1 MHz
        field!(self.registers, reload_value).modify(|reload_value| (reload_value & !reload_value_register::VALID_MASK) | 0);
        field!(self.registers, ctrl_status).modify(|ctrl_status| (ctrl_status & !ctrl_status_register::VALID_MASK) | ctrl_status_register::TICKINT_MASK | ctrl_status_register::CLOCK_SRC_EXTERNAL | ctrl_status_register::ENABLE_MASK);
        // reload time from reload value register (0)
        self.reload();
    }

    /// Reads and clears the count flag in the control and status register
    #[inline(always)]
    pub fn clear_count_flag(&mut self) {
        field!(self.registers, ctrl_status).read();
    }

    /// Sets the timeout for when the next System Tick interrupt will fire  
    /// `time` is the timeout in micro seconds
    #[inline(always)]
    pub fn set_timeout(&mut self, time: u32) {
        let time = (time << reload_value_register::RELOAD_SHIFT) & reload_value_register::RELOAD_MASK;
        field!(self.registers, reload_value).modify(|reload| (reload & !reload_value_register::VALID_MASK) | time);
        self.reload();
    }

    /// Starts the System Tick
    #[inline(always)]
    pub fn start(&mut self) {
        field!(self.registers, ctrl_status).modify(|ctrl| ctrl | ctrl_status_register::ENABLE_MASK);
    }

    /// Pauses the Sytem Tick
    #[inline(always)]
    pub fn pause(&mut self) {
        field!(self.registers, ctrl_status).modify(|ctrl| ctrl & !ctrl_status_register::ENABLE_MASK);
    }

    /// Reloads the System Tick from the reload value register
    #[inline(always)]
    pub fn reload(&mut self) {
        field!(self.registers, current_value).write(1);
    }

    /// Reads the count left in the current value register
    #[inline(always)]
    pub fn read_current(&mut self) -> u32 {
        (field!(self.registers, current_value).read() & current_value_register::CURRENT_MASK) >> current_value_register::CURRENT_SHIFT
    }
}

unsafe impl Send for SysTick {}
unsafe impl Sync for SysTick {}

/// Base address for the System Tick memory mapped registers
static SYS_TICK_BASE: usize = 0xe000e010;

/// System Tick object
pub static SYS_TICK: IRQMutex<SysTick> = unsafe {
    IRQMutex::new(SysTick::new(SYS_TICK_BASE))
};

/// Starts the System Tick
/// # Safety
/// Interrupts can't happen
#[unsafe(no_mangle)]
pub unsafe fn start_sys_tick(proc: *mut Proc) -> *mut Proc {
    let cs = unsafe {
        CS::new()
    };
    let mut sys_tick = SYS_TICK.lock(&cs);
    sys_tick.start();
    proc
}

/// Reloads the System Tick from the reload register and starts it
/// # Safety
/// Interrupts can't happen
#[unsafe(no_mangle)]
pub unsafe fn reload_start_sys_tick() {
    let cs = unsafe {
        CS::new()
    };
    let mut sys_tick = SYS_TICK.lock(&cs);
    sys_tick.reload();
    sys_tick.start();
}

/// Pauses the System Tick
/// # Safety
/// Interrupts can't happen
#[unsafe(no_mangle)]
pub unsafe fn pause_sys_tick() {
    let cs = unsafe {
        CS::new()
    };
    let mut sys_tick = SYS_TICK.lock(&cs);
    sys_tick.pause();
}

#[cfg(test)]
mod test {

    use crate::{print, println};

    use super::*;

    #[test_case]
    fn test_setup_correct() {
        println!("Testing sys tick setup");
        let cs = unsafe {
            CS::new()
        };
        let mut sys_tick = SYS_TICK.lock(&cs);
        print!("Testing ctrl status register ");
        let ctrl_status = field!(sys_tick.registers, ctrl_status).read();
        // Enabled, counting to 0 causes interrupt and clock source is external system clock
        // do not check count flag as we don't care if the timer counted down to 0 or not
        assert_eq!(ctrl_status & ctrl_status_register::VALID_MASK & (!ctrl_status_register::COUNT_FLAG_MASK), 0x3);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing sys tick register mask values");
        print!("Testing ctrl status register ");
        assert_eq!(ctrl_status_register::VALID_MASK, 0x10007);
        println!("[ok]");
        print!("Testing reload value register ");
        assert_eq!(reload_value_register::VALID_MASK, 0xffffff);
        println!("[ok]");
        print!("Testing current value register ");
        assert_eq!(current_value_register::VALID_MASK, 0xffffff);
        println!("[ok]");
        print!("Testing callibration register ");
        assert_eq!(callibration_register::VALID_MASK, 0xc0ffffff);
        println!("[ok]");
    }
}
