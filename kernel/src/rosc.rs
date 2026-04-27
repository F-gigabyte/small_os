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

use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite}};

use crate::mutex::SpinIRQ;

/// Ring Oscillator memory mapped registers
#[repr(C)]
struct ROSCRegisters {
    /// Control register (0x00)
    ctrl: ReadPureWrite<u32>, // 0x0
    /// Frequency A register (0x04)
    freqa: ReadPureWrite<u32>, // 0x4
    /// Frequency B register (0x08)
    freqb: ReadPureWrite<u32>, // 0x8
    /// Dormant register (0x0c)
    dormant: ReadPureWrite<u32>, // 0xc
    /// Divider register (0x10)
    div: ReadPureWrite<u32>, // 0x10
    /// Phase register (0x14)
    phase: ReadPureWrite<u32>, // 0x14
    /// Status register (0x18)
    status: ReadPureWrite<u32>, // 0x18
    /// Random bit register (0x1c)
    random_bit: ReadPure<u32>, //0x1c
    /// Count register (0x20)
    count: ReadPureWrite<u32>, // 0x20
}

/// Ring Oscillator control register masks and shifts
mod ctrl_register {
    /// Frequency range shift
    pub const FREQ_RANGE_SHIFT: usize = 0;
    /// Shift for enabling the Ring Oscillator
    pub const ENABLE_SHIFT: usize = 12;

    /// Frequency range mask
    pub const FREQ_RANGE_MASK: u32 = 0xfff << FREQ_RANGE_SHIFT;
    /// Mask for enabling the Ring Oscillator
    pub const ENABLE_MASK: u32 = 0xfff << ENABLE_SHIFT;

    /// Use delay stages 0 to 7
    pub const FREQ_LOW: u32 = 0xfa4 << FREQ_RANGE_SHIFT;
    /// Use delay stages 2 to 7
    pub const FREQ_MEDIUM: u32 = 0xfa5 << FREQ_RANGE_SHIFT;
    /// Use delay stages 4 to 7
    pub const FREQ_HIGH: u32 = 0xfa7 << FREQ_RANGE_SHIFT;
    /// Use delay stages 6 to 7 (do not use)
    pub const FREQ_TOOHIGH: u32 = 0xfa6 << FREQ_RANGE_SHIFT;

    /// Disable the Ring Oscillator
    pub const ENABLE_DISABLE: u32 = 0xd1e << ENABLE_SHIFT;
    /// Enable the Ring Oscillator
    pub const ENABLE_ENABLE: u32 = 0xfab << ENABLE_SHIFT;
    
    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = FREQ_RANGE_MASK |
        ENABLE_MASK;
}

/// Ring Oscillator frequency A register masks and shifts
mod freqa_register {
    /// Stage 0 drive strength shift
    pub const DS0_SHIFT: usize = 0;
    /// Stage 1 drive strength shift
    pub const DS1_SHIFT: usize = 4;
    /// Stage 2 drive strength shift
    pub const DS2_SHIFT: usize = 8;
    /// Stage 3 drive strength shift
    pub const DS3_SHIFT: usize = 12;
    /// Password shift for applying settings
    pub const PASSWD_SHIFT: usize = 16;

    /// Stage 0 drive strength mask
    pub const DS0_MASK: u32 = 0x7 << DS0_SHIFT;
    /// Stage 1 drive strength mask
    pub const DS1_MASK: u32 = 0x7 << DS1_SHIFT;
    /// Stage 2 drive strength mask
    pub const DS2_MASK: u32 = 0x7 << DS2_SHIFT;
    /// Stage 3 drive strength mask
    pub const DS3_MASK: u32 = 0x7 << DS3_SHIFT;
    /// Password mask for applying settings
    pub const PASSWD_MASK: u32 = 0xffff << PASSWD_SHIFT;
    
    /// Password for applying settings
    pub const PASSWD_PASS: u32 = 0x9696 << PASSWD_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = DS0_MASK |
        DS1_MASK |
        DS2_MASK |
        DS3_MASK |
        PASSWD_MASK;
}

/// Ring Oscillator frequency B register masks and shifts
mod freqb_register {
    /// Stage 4 drive strength shift
    pub const DS4_SHIFT: usize = 0;
    /// Stage 5 drive strength shift
    pub const DS5_SHIFT: usize = 4;
    /// Stage 6 drive strength shift
    pub const DS6_SHIFT: usize = 8;
    /// Stage 7 drive strength shift
    pub const DS7_SHIFT: usize = 12;
    /// Password shift for applying settings
    pub const PASSWD_SHIFT: usize = 16;

    /// Stage 4 drive strength mask
    pub const DS4_MASK: u32 = 0x7 << DS4_SHIFT;
    /// Stage 5 drive strength mask
    pub const DS5_MASK: u32 = 0x7 << DS5_SHIFT;
    /// Stage 6 drive strength mask
    pub const DS6_MASK: u32 = 0x7 << DS6_SHIFT;
    /// Stage 7 drive strength mask
    pub const DS7_MASK: u32 = 0x7 << DS7_SHIFT;
    /// Password mask for applying settings
    pub const PASSWD_MASK: u32 = 0xffff << PASSWD_SHIFT;
    
    /// Password for applying settings
    pub const PASSWD_PASS: u32 = 0x9696 << PASSWD_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = DS4_MASK |
        DS5_MASK |
        DS6_MASK |
        DS7_MASK |
        PASSWD_MASK;
}

/// Ring Oscillator dormant register masks and shifts
mod dormant_register {
    /// Dormant value shift
    pub const VALUE_SHIFT: usize = 0;
    
    /// Dormant value mask
    pub const VALUE_MASK: u32 = 0xffffffff << VALUE_SHIFT;

    /// Dormant value to send Ring Oscillator to sleep
    pub const DORMANT: u32 = 0x636f6d61;
    /// Dormant value to wake Ring Oscillator
    pub const WAKE: u32 = 0x77616b65;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = VALUE_MASK;
}

/// Ring Oscillator divider register masks and shifts
mod div_register {
    /// Divisor shift
    pub const DIV_SHIFT: usize = 0;

    /// Divisor mask
    pub const DIV_MASK: u32 = 0xfff << DIV_SHIFT;

    /// Minimum divisor
    pub const DIV_MIN: u32 = 0xaa0 << DIV_SHIFT;
    /// Maximum divisor
    pub const DIV_MAX: u32 = DIV_MIN + (0x31 << DIV_SHIFT);

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = DIV_MASK;
}

/// Ring Oscillator phase register masks and shifts
mod phase_register {
    /// Shift for shifting the phase
    pub const SHIFT_SHIFT: usize = 0;
    /// Shift for inverting the phase
    pub const FLIP_SHIFT: usize = 2;
    /// Shift for enabling the phase shifted output
    pub const ENABLE_SHIFT: usize = 3;
    /// Password shift for updating the settings
    pub const PASSWD_SHIFT: usize = 4;

    /// Mask for shifting the phase
    pub const SHIFT_MASK: u32 = 0x3 << SHIFT_SHIFT;
    /// Mask for inverting the phase
    pub const FLIP_MASK: u32 = 1 << FLIP_SHIFT;
    /// Mask for enabling the phase shifted output
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Password mask for updating the settings
    pub const PASSWD_MASK: u32 = 0xff << PASSWD_SHIFT;

    /// Password for updating the settings
    pub const PASSWD_PASS: u32 = 0xaa << PASSWD_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SHIFT_MASK |
        FLIP_MASK |
        ENABLE_MASK |
        PASSWD_MASK;
}

/// Ring Oscillator status register masks and shifts
mod status_register {
    /// Shift for determining if the Ring Oscillator is enabled or not
    pub const ENABLED_SHIFT: usize = 12;
    /// Shift for determining if the post dividers are running
    pub const DIV_RUNNING_SHIFT: usize = 16;
    /// Shift for determining if an invalid value has been written (do not use)
    pub const BADWRITE_SHIFT: usize = 24;
    /// Shift for determining if the oscillator is running and stable
    pub const STABLE_SHIFT: usize = 31;

    /// Mask for determining if the Ring Oscillator is enabled or not
    pub const ENABLED_MASK: u32 = 1 << ENABLED_SHIFT;
    /// Mask for determining if the post dividers are running
    pub const DIV_RUNNING_MASK: u32 = 1 << DIV_RUNNING_SHIFT;
    /// Mask for determining if an invalid value has been written (do not use)
    pub const BADWRITE_MASK: u32 = 1 << BADWRITE_SHIFT;
    /// Mask for determining if the oscillator is running and stable
    pub const STABLE_MASK: u32 = 1 << STABLE_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ENABLED_MASK |
        DIV_RUNNING_MASK |
        BADWRITE_MASK |
        STABLE_MASK;
}

/// Ring Oscillator random bit register masks and shifts
mod random_bit_register {
    /// Random bit shift
    pub const RANDOM_BIT_SHIFT: usize = 0;

    /// Random bit mask
    pub const RANDOM_BIT_MASK: u32 = 1 << RANDOM_BIT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = RANDOM_BIT_MASK;
}

/// Random Oscillator count register masks and shifts
mod count_register {
    /// Count shift
    pub const COUNT_SHIFT: usize = 0;

    /// Count mask
    pub const COUNT_MASK: u32 = 0xff << COUNT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = COUNT_MASK;
}

/// Ring Oscillator object for managing the Ring Oscillator
pub struct ROSC {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, ROSCRegisters>
}

impl ROSC {
    /// Creates a new `ROSC` object  
    /// `base` is the base address of the Ring Oscillator memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the Ring Oscillator memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    /// Returns a series of random bits  
    /// `data` is where the bits should be stored  
    /// `bits` is the number of random bits to generate  
    /// Panics if there isn't enough room in `data`
    #[cfg(feature = "random")]
    pub fn get_random(&mut self, data: &mut [u8], bits: usize) {
        assert!(data.len() * 8 >= bits);
        for i in 0..bits / 8 {
            let mut byte = 0;
            for _ in 0..8 {
                byte |= ((field!(self.registers, random_bit).read() & random_bit_register::RANDOM_BIT_MASK) >> random_bit_register::RANDOM_BIT_SHIFT) as u8;
                byte <<= 1;
            }
            data[i] = byte;
        }
        let mut final_byte = 0;
        for j in 0..bits % 8 {
            final_byte |= ((field!(self.registers, random_bit).read() & random_bit_register::RANDOM_BIT_MASK) >> random_bit_register::RANDOM_BIT_SHIFT) as u8;
            final_byte <<= 1;
        }
        if bits % 8 != 0 {
            data[bits / 8] = final_byte;
        }
    }

    /// Enables the ROSC
    pub fn enable(&mut self) {
        field!(self.registers, phase).modify(|phase| 
            (phase & !phase_register::VALID_MASK) |
            phase_register::PASSWD_PASS |
            phase_register::ENABLE_MASK
        );
        field!(self.registers, div).modify(|div|
            (div & !div_register::VALID_MASK) |
            ((0xaa0 + 16) << div_register::DIV_SHIFT)
        );
        field!(self.registers, freqa).modify(|freqa|
            (freqa & !freqa_register::VALID_MASK) |
            freqa_register::PASSWD_PASS
        );
        field!(self.registers, freqb).modify(|freqb|
            (freqb & !freqa_register::VALID_MASK) |
            freqb_register::PASSWD_PASS
        );
        field!(self.registers, ctrl).modify(|ctrl|
            (ctrl & !ctrl_register::VALID_MASK) |
            ctrl_register::ENABLE_ENABLE |
            ctrl_register::FREQ_LOW
        );
        while field!(self.registers, status).read() & status_register::STABLE_MASK == 0 {}
    }

    /// Disables the ROSC
    pub fn disable(&mut self) {
        field!(self.registers, ctrl).modify(|ctrl| (ctrl & !ctrl_register::VALID_MASK) | ctrl_register::ENABLE_DISABLE);
    }
}

unsafe impl Send for ROSC {}
unsafe impl Sync for ROSC {}

/// Base address for the Ring Oscillator memory mapped registers
static ROSC_BASE: usize = 0x40060000;

/// Ring Oscillator object
pub static ROSC: SpinIRQ<ROSC> = unsafe {
    SpinIRQ::new(ROSC::new(ROSC_BASE))
};

#[cfg(test)]
mod test {
    use crate::{CS, print, println};

    use super::*;

    #[test_case]
    fn test_setup_correct() {
        #[cfg(not(feature = "random"))]
        {
            println!("Testing ROSC setup");
            let cs = unsafe {
                CS::new()
            };
            let mut rosc = ROSC.lock(&cs);
            print!("Testing ctrl register ");
            let ctrl = field!(rosc.registers, ctrl).read();
            // ROSC disabled
            assert_eq!(ctrl & ctrl_register::ENABLE_MASK, 0xd1e000);
            println!("[ok]");
        }
    }

    #[test_case]
    fn test_valid() {
        println!("Testing ROSC register mask values");
        print!("Testing ctrl register ");
        assert_eq!(ctrl_register::VALID_MASK, 0xffffff);
        println!("[ok]");
        print!("Testing freqa register ");
        assert_eq!(freqa_register::VALID_MASK, 0xffff7777);
        println!("[ok]");
        print!("Testing freqb register ");
        assert_eq!(freqb_register::VALID_MASK, 0xffff7777);
        println!("[ok]");
        print!("Testing dormant register ");
        assert_eq!(dormant_register::VALID_MASK, 0xffffffff);
        println!("[ok]");
        print!("Testing div register ");
        assert_eq!(div_register::VALID_MASK, 0xfff);
        println!("[ok]");
        print!("Testing phase register ");
        assert_eq!(phase_register::VALID_MASK, 0xfff);
        println!("[ok]");
        print!("Testing status register ");
        assert_eq!(status_register::VALID_MASK, 0x81011000);
        println!("[ok]");
        print!("Testing random bit register ");
        assert_eq!(random_bit_register::VALID_MASK, 0x1);
        println!("[ok]");
        print!("Testing count register ");
        assert_eq!(count_register::VALID_MASK, 0xff);
        println!("[ok]");
    }
}
