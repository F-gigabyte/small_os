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

use core::{ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};

use crate::{mmio::REG_ALIAS_SET_BITS, mutex::SpinIRQ};


/// Crystal Oscillator memory mapped registers
#[repr(C)]
struct XOSCRegisters {
    /// Control register (0x00)
    ctrl: ReadPureWrite<u32>, // 0x0
    /// Status register (0x04)
    status: ReadPureWrite<u32>, // 0x4
    /// Dormant register (0x08)
    dormant: ReadPureWrite<u32>, // 0x8
    /// Startup register (0x0c)
    startup: ReadPureWrite<u32>, // 0xc
    _reserved0: u32, // 0x10
    _reserved1: u32, // 0x14
    _reserved2: u32, // 0x18
    /// Count register (0x1c)
    count: ReadPureWrite<u32> // 0x1c
}

/// Crystal Oscillator control register masks and shifts
mod ctrl_register {
    /// Frequency range shift
    pub const FREQ_RANGE_SHIFT: usize = 0;
    /// Shift for enabling the Crystal Oscillator
    pub const ENABLE_SHIFT: usize = 12;
    
    /// Frequency range mask
    pub const FREQ_RANGE_MASK: u32 = 0xfff << FREQ_RANGE_SHIFT;
    /// Mask for enabling the Crystal Oscillator
    pub const ENABLE_MASK: u32 = 0xfff << ENABLE_SHIFT;

    /// Frequency range of 1 to 15 MHz
    pub const FREQ_RANGE_1_15MHZ: u32 = 0xaa0 << FREQ_RANGE_SHIFT;

    /// Enable the Crystal Oscillator
    pub const ENABLE_ENABLE: u32 = 0xfab << ENABLE_SHIFT;
    /// Disable the Crystal Oscillator
    pub const ENABLE_DISABLE: u32 = 0xd1e << ENABLE_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = FREQ_RANGE_MASK | 
        ENABLE_MASK;
}

/// Crystal Oscillator status register masks and shifts
mod status_register {
    /// Current frequency range shift (don't use)
    pub const FREQ_RANGE_SHIFT: usize = 0;
    /// Shift for determining if the oscillator is enabled
    pub const ENABLED_SHIFT: usize = 12;
    /// Shift for if an invalid value has been written (don't use)
    pub const BADWRITE_SHIFT: usize = 24;
    /// Shift for determining if the oscillator is stable
    pub const STABLE_SHIFT: usize = 31;
    
    /// Current frequency range mask (don't use)
    pub const FREQ_RANGE_MASK: u32 = 0x3 << FREQ_RANGE_SHIFT;
    /// Mask for determining if the oscillator is enabled
    pub const ENABLED_MASK: u32 = 1 << ENABLED_SHIFT;
    /// Mask for if an invalid value has been written (don't use)
    pub const BADWRITE_MASK: u32 = 1 << BADWRITE_SHIFT;
    /// Mask for determining if the oscillator is stable
    pub const STABLE_MASK: u32 = 1 << STABLE_SHIFT;

    /// Frequency range of 1 to 15 MHz
    pub const FREQ_RANGE_1_15MHZ: u32 = 0x0 << FREQ_RANGE_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = FREQ_RANGE_MASK | 
        ENABLED_MASK |
        BADWRITE_MASK |
        STABLE_MASK;
}

/// Crystal Oscillator dormant register masks and shifts
mod dormant_register {
    /// Dormant value shift
    pub const VALUE_SHIFT: usize = 0;

    /// Dormant value mask
    pub const VALUE_MASK: u32 = 0xffffffff << VALUE_SHIFT;

    /// Send Crystal Oscillator to sleep
    pub const DORMANT: u32 = 0x636f6d61 << VALUE_SHIFT;
    /// Wake Crystal Oscillator
    pub const WAKE: u32 = 0x77616b65 << VALUE_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = VALUE_MASK;
}

/// Crystal Oscillator startup register masks and shifts
mod startup_register {
    /// Shift for the delay time of the Crystal Oscillator startup
    pub const DELAY_SHIFT: usize = 0;
    /// Shift for multiplying the startup delay by 4
    pub const X4_SHIFT: usize = 20;

    /// Mask for the delay time of the Crystal Oscillator startup
    pub const DELAY_MASK: u32 = 0x3fff << DELAY_SHIFT;
    /// Mask for multiplying the startup delay by 4
    pub const X4_MASK: u32 = 1 << X4_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = DELAY_MASK |
        X4_MASK;
}

/// Crystal Oscillator count register masks and shifts
mod count_register {
    /// Counter shift
    pub const COUNTER_SHIFT: usize = 0;

    /// Counter mask
    pub const COUNTER_MASK: u32 = 0xff << COUNTER_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = COUNTER_MASK;
}

/// Crystal Oscillator object for managing the Crystal Oscillator
pub struct XOSC {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, XOSCRegisters>,
    /// Memory mapped registers where writing a bit sets the corresponding bit in `registers`
    set_reg: UniqueMmioPointer<'static, XOSCRegisters>
}

impl XOSC {
    /// Creates a new `XOSC` object  
    /// `base` is the base address of the Crystal Oscillator memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the Crystal Oscillator memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap())
            }
        }
    }

    /// Sets the Crystal Oscillator up so it's running at a 12 MHz rate  
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
        field!(self.registers, startup).modify(|startup| (startup & !startup_register::VALID_MASK) | (47 << startup_register::DELAY_SHIFT));
        field!(self.registers, dormant).modify(|dormant| (dormant &!dormant_register::VALID_MASK) | dormant_register::WAKE);
        field!(self.registers, ctrl).modify(|ctrl| (ctrl & !ctrl_register::VALID_MASK) | ctrl_register::FREQ_RANGE_1_15MHZ | ctrl_register::ENABLE_ENABLE);
        while field!(self.registers, status).read() & status_register::STABLE_MASK == 0 {}
    }

    /// Waits for `cycles` / 12 micro seconds  
    /// `cycles` is the number of cycles to wait  
    /// This shouldn't be used
    pub fn wait_cycles(&mut self, cycles: u8) {
        field!(self.registers, count).modify(|count| (count & !count_register::VALID_MASK) | ((cycles as u32) << count_register::COUNTER_SHIFT) & count_register::COUNTER_MASK);
        while field!(self.registers, count).read() != 0 {}
    }
}

unsafe impl Send for XOSC {}
unsafe impl Sync for XOSC {}

/// Base address for the Crystal Oscillator memory mapped registers
static XOSC_BASE: usize = 0x40024000;

/// Crystal Oscillator object
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
