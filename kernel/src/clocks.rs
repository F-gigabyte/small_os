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

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, ReadWrite}};

use crate::{mmio::{REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS}, mutex::SpinIRQ, wait};

/// Clocks memory mapped registers
#[repr(C)]
struct ClockRegisters {
    /// GPOUT0 clock control register (0x00)
    gpout0_ctrl: ReadPureWrite<u32>, // 0x00
    /// GPOUT0 clock divider register (0x04)
    gpout0_div: ReadPureWrite<u32>, //  0x04
    /// GPOUT0 clock selected register (0x08)
    gpout0_selected: ReadPure<u32>, //  0x08
    /// GPOUT1 clock control register (0x0c)
    gpout1_ctrl: ReadPureWrite<u32>, // 0x0c
    /// GPOUT1 clock divider register (0x10)
    gpout1_div: ReadPureWrite<u32>, //  0x10
    /// GPOUT1 clock selected register (0x14)
    gpout1_selected: ReadPure<u32>, //  0x14
    /// GPOUT2 clock control register (0x18)
    gpout2_ctrl: ReadPureWrite<u32>, // 0x18
    /// GPOUT2 clock divider register (0x1c)
    gpout2_div: ReadPureWrite<u32>, //  0x1c
    /// GPOUT2 clock selected register (0x20)
    gpout2_selected: ReadPure<u32>, //  0x20
    /// GPOUT3 clock control register (0x24)
    gpout3_ctrl: ReadPureWrite<u32>, // 0x24
    /// GPOUT3 clock divider register (0x28)
    gpout3_div: ReadPureWrite<u32>, //  0x28
    /// GPOUT3 selected register (0x2c)
    gpout3_selected: ReadPure<u32>, //  0x2c
    /// Reference clock control register (0x30)
    ref_ctrl: ReadPureWrite<u32>, //    0x30
    /// Reference clock divider register (0x34)
    ref_div: ReadPureWrite<u32>, //     0x34
    /// Reference clock selected register (0x38)
    ref_selected: ReadPure<u32>, //     0x38
    /// System clock control register (0x3c)
    sys_ctrl: ReadPureWrite<u32>, //    0x3c
    /// System clock divider register (0x40)
    sys_div: ReadPureWrite<u32>, //     0x40
    /// System clock selected register (0x44)
    sys_selected: ReadPure<u32>, //     0x44
    /// Peripheral clock control register (0x48)
    peri_ctrl: ReadPureWrite<u32>, //   0x48
    _reserved: u32,
    /// Peripheral clock selected register (0x50)
    peri_selected: ReadPure<u32>, //    0x50
    /// USB clock control register (0x54)
    usb_ctrl: ReadPureWrite<u32>, //    0x54
    /// USB clock divider register (0x58)
    usb_div: ReadPureWrite<u32>, //     0x58
    /// USB clock selected register (0x5c)
    usb_selected: ReadPure<u32>, //     0x5c
    /// Analog to digital converter clock control register (0x60)
    adc_ctrl: ReadPureWrite<u32>, //    0x60
    /// Analog to digital converter clock divider register (0x64)
    adc_div: ReadPureWrite<u32>, //     0x64
    /// Analog to digital converter clock selected register (0x68)
    adc_selected: ReadPure<u32>, //     0x68
    /// RTC clock control register (0x6c)
    rtc_ctrl: ReadPureWrite<u32>, //    0x6c
    /// RTC clock divider register (0x70)
    rtc_div: ReadPureWrite<u32>, //     0x70
    /// RTC clock selected register (0x74)
    rtc_selected: ReadPure<u32>, //     0x74
    /// Resus clock control register (0x78)
    sys_resus_ctrl: ReadPureWrite<u32>, // 0x78
    /// Resus clock status register (0x7c)
    sys_resus_status: ReadWrite<u32>, // 0x7c
}

/// Control register masks and shifts for a GPOUT clock
mod gpout_ctrl_register {
    /// Auxhillary source shift (glitches on switching)
    pub const AUXSRC_SHIFT: usize = 5;
    /// Shift for killing the clock
    pub const KILL_SHIFT: usize = 10;
    /// Shift for enabling the clock
    pub const ENABLE_SHIFT: usize = 11;
    /// Shift for enabling duty cycle correction for odd divisors
    pub const DC50_SHIFT: usize = 12;
    /// Shift for adding a delay to clock startup
    pub const PHASE_SHIFT: usize = 16;
    /// Shift for shifting the phase of the clock by 1 cycle
    pub const NUDGE_SHIFT: usize = 20;

    /// Auxhillary source mask (glitches on switching)
    pub const AUXSRC_MASK: u32 = 0xf << AUXSRC_SHIFT;
    /// Mask for killing the clock
    pub const KILL_MASK: u32 = 1 << KILL_SHIFT;
    /// Mask for enabling the clock
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Mask for enabling duty cycle correction for odd divisors
    pub const DC50_MASK: u32 = 1 << DC50_SHIFT;
    /// Mask for adding a delay to clock startup
    pub const PHASE_MASK: u32 = 0x3 << PHASE_SHIFT;
    /// Mask for shifting the phase of the clock by 1 cycle
    pub const NUDGE_MASK: u32 = 1 << NUDGE_SHIFT;

    /// Auxhillary source PLL Sys (glitches on switching)
    pub const AUXSRC_PLL_SYS: u32 = 0x0 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN0 (glitches on switching)
    pub const AUXSRC_GPIN0: u32 = 0x1 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN1 (glitches on switching)
    pub const AUXSRC_GPIN1: u32 = 0x2 << AUXSRC_SHIFT;
    /// Auxhillary source PLL USB (glitches on switching)
    pub const AUXSRC_PLL_USB: u32 = 0x3 << AUXSRC_SHIFT;
    /// Auxhillary source Ring Oscillator (glitches on switching)
    pub const AUXSRC_ROSC: u32 = 0x4 << AUXSRC_SHIFT;
    /// Auxhillary source Crystal Oscillator (glitches on switching)
    pub const AUXSRC_XOSC: u32 = 0x5 << AUXSRC_SHIFT;
    /// Auxhillary source System Clock (glitches on switching)
    pub const AUXSRC_SYS: u32 = 0x6 << AUXSRC_SHIFT;
    /// Auxhillary source USB Clock (glitches on switching)
    pub const AUXSRC_USB: u32 = 0x7 << AUXSRC_SHIFT;
    /// Auxhillary source Analog to Digital Converter Clock (glitches on switching)
    pub const AUXSRC_ADC: u32 = 0x8 << AUXSRC_SHIFT;
    /// Auxhillary source Real Time Clock (glitches on switching)
    pub const AUXSRC_RTC: u32 = 0x9 << AUXSRC_SHIFT;
    /// Auxhillary source Reference Clock (glitches on switching)
    pub const AUXSRC_REF: u32 = 0xa << AUXSRC_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = AUXSRC_MASK |
        KILL_MASK |
        ENABLE_MASK |
        DC50_MASK |
        PHASE_MASK |
        NUDGE_MASK;
}

/// Clock divisor register mask and shifts for all clocks supporting a fractional divisor
mod div_register {
    /// Shift for the fractional component of the clock divisor
    pub const FRAC_SHIFT: usize = 0;
    /// Shift for the integer component of the clock divisor
    pub const INT_SHIFT: usize = 8;

    /// Mask for the integer component of the clock divisor
    pub const INT_MASK: u32 = 0xffffff << INT_SHIFT;
    /// Mask for the fractional component of the clock divisor
    pub const FRAC_MASK: u32 = 0xff << FRAC_SHIFT;

    /// Mask of all non-reservered bits
    pub const VALID_MASK: u32 = INT_MASK |
        FRAC_MASK;
}

/// Clock divisor register mask and shifts for all clocks not supporting a fractional divisor
mod int_div_register {
    /// Shift for the integer component of the clock divisor
    pub const INT_SHIFT: usize = 8;

    /// Mask for the integer component of the clock divisor
    pub const INT_MASK: u32 = 0x3 << INT_SHIFT;

    /// Mask of all non-reservered bits
    pub const VALID_MASK: u32 = INT_MASK;
}

/// Control register masks and shifts for the Reference Clock
mod ref_ctrl_register {
    /// Source shift
    pub const SRC_SHIFT: usize = 0;
    /// Auxhillary source shift (glitches on switching)
    pub const AUXSRC_SHIFT: usize = 5;

    /// Source mask
    pub const SRC_MASK: u32 = 0x3 << SRC_SHIFT;
    /// Auxhillary source mask (glitches on switching)
    pub const AUXSRC_MASK: u32 = 0x3 << AUXSRC_SHIFT;

    /// Source Ring Oscillator
    pub const SRC_ROSC: u32 = 0x0 << SRC_SHIFT;
    /// Source of whatevers referenced by the auxhillary source
    pub const SRC_REF_AUX: u32 = 0x1 << SRC_SHIFT;
    /// Source Crystal Oscillator
    pub const SRC_XOSC: u32 = 0x2 << SRC_SHIFT;

    /// Auxhillary source USB PLL (glitches on switching)
    pub const AUXSRC_PLL_USB: u32 = 0x0 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN0 (glitches on switching)
    pub const AUXSRC_GPIN0: u32 = 0x1 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN1 (glitches on switching)
    pub const AUXSRC_GPIN1: u32 = 0x2 << AUXSRC_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SRC_MASK |
        AUXSRC_MASK;
}

/// Control register masks and shifts for the System Clock
mod sys_ctrl_register {
    /// Source shift
    pub const SRC_SHIFT: usize = 0;
    /// Auxhillary source shift (glitches on switching)
    pub const AUXSRC_SHIFT: usize = 5;

    /// Source mask
    pub const SRC_MASK: u32 = 0x1 << SRC_SHIFT;
    /// Auxhillary source mask (glitches on switching)
    pub const AUXSRC_MASK: u32 = 0x7 << AUXSRC_SHIFT;

    /// Source Reference Clock
    pub const SRC_REF: u32 = 0 << SRC_SHIFT;
    /// Source of whatevers referenced by the auxhillary source
    pub const SRC_SYS_AUX: u32 = 1 << SRC_SHIFT;

    /// Auxhillary source Sys PLL (glitches on switching)
    pub const AUXSRC_PLL_SYS: u32 = 0x0 << AUXSRC_SHIFT;
    /// Auxhillary source USB PLL (glitches on switching)
    pub const AUXSRC_PLL_USB: u32 = 0x1 << AUXSRC_SHIFT;
    /// Auxhillary source Ring Oscillator
    pub const AUXSRC_ROSC: u32 = 0x2 << AUXSRC_SHIFT;
    /// Auxhillary source Crystal Oscillator (glitches on switching)
    pub const AUXSRC_XOSC: u32 = 0x3 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN0 (glitches on switching)
    pub const AUXSRC_GPIN0: u32 = 0x4 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN1 (glitches on switching)
    pub const AUXSRC_GPIN1: u32 = 0x5 << AUXSRC_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SRC_MASK |
        AUXSRC_MASK;
}

/// Control register masks and shifts for the Peripheral Clock
mod peri_ctrl_register {
    /// Auxhillary source shift (glitches on switching)
    pub const AUXSRC_SHIFT: usize = 5;
    /// Shift for killing the clock
    pub const KILL_SHIFT: usize = 10;
    /// Shift for enabling the clock
    pub const ENABLE_SHIFT: usize = 11;

    /// Auxhillary source mask (glitches on switching)
    pub const AUXSRC_MASK: u32 = 0x7 << AUXSRC_SHIFT;
    /// Mask for killing the clock
    pub const KILL_MASK: u32 = 1 << KILL_SHIFT;
    /// Mask for enabling the clock
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;

    /// Auxhillary source System Clock (glitches on switching)
    pub const AUXSRC_SYS: u32 = 0x0 << AUXSRC_SHIFT;
    /// Auxhillary source PLL Sys (glitches on switching)
    pub const AUXSRC_PLL_SYS: u32 = 0x1 << AUXSRC_SHIFT;
    /// Auxhillary source PLL USB (glitches on switching)
    pub const AUXSRC_PLL_USB: u32 = 0x2 << AUXSRC_SHIFT;
    /// Auxhillary source Ring Oscillator (glitches on switching)
    pub const AUXSRC_ROSC: u32 = 0x3 << AUXSRC_SHIFT;
    /// Auxhillary source Crystal Oscillator (glitches on switching)
    pub const AUXSRC_XOSC: u32 = 0x4 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN0 (glitches on switching)
    pub const AUXSRC_GPIN0: u32 = 0x5 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN1 (glitches on switching)
    pub const AUXSRC_GPIN1: u32 = 0x6 << AUXSRC_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = AUXSRC_MASK |
        KILL_MASK |
        ENABLE_MASK;
}

/// Control register masks and shifts for most clocks
mod ctrl_register {
    /// Auxhillary source shift (glitches on switching)
    pub const AUXSRC_SHIFT: usize = 5;
    /// Shift for killing the clock
    pub const KILL_SHIFT: usize = 10;
    /// Shift for enabling the clock
    pub const ENABLE_SHIFT: usize = 11;
    /// Shift for adding a delay to clock startup
    pub const PHASE_SHIFT: usize = 16;
    /// Shift for shifting the phase of the clock by 1 cycle
    pub const NUDGE_SHIFT: usize = 20;

    /// Auxhillary source mask (glitches on switching)
    pub const AUXSRC_MASK: u32 = 0x7 << AUXSRC_SHIFT;
    /// Mask for killing the clock
    pub const KILL_MASK: u32 = 1 << KILL_SHIFT;
    /// Mask for enabling the clock
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Mask for adding a delay to clock startup
    pub const PHASE_MASK: u32 = 0x3 << PHASE_SHIFT;
    /// Mask for shifting the phase of the clock by 1 cycle
    pub const NUDGE_MASK: u32 = 1 << NUDGE_SHIFT;

    /// Auxhillary source PLL USB (glitches on switching)
    pub const AUXSRC_PLL_USB: u32 = 0x0 << AUXSRC_SHIFT;
    /// Auxhillary source PLL Sys (glitches on switching)
    pub const AUXSRC_PLL_SYS: u32 = 0x1 << AUXSRC_SHIFT;
    /// Auxhillary source Ring Oscillator (glitches on switching)
    pub const AUXSRC_ROSC: u32 = 0x2 << AUXSRC_SHIFT;
    /// Auxhillary source Crystal Oscillator (glitches on switching)
    pub const AUXSRC_XOSC: u32 = 0x3 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN0 (glitches on switching)
    pub const AUXSRC_GPIN0: u32 = 0x4 << AUXSRC_SHIFT;
    /// Auxhillary source GPIN1 (glitches on switching)
    pub const AUXSRC_GPIN1: u32 = 0x5 << AUXSRC_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = AUXSRC_MASK |
        KILL_MASK |
        ENABLE_MASK |
        PHASE_MASK |
        NUDGE_MASK;
}

/// Control register masks and shifts for Resus
mod sys_resus_ctrl_register {
    /// Shift for the number of Reference Clock cycles that should pass before Resus is activated
    pub const TIMEOUT_SHIFT: usize = 0;
    /// Shift for enabling the Resus
    pub const ENABLE_SHIFT: usize = 8;
    /// Shift for forcing the Resus to activate
    pub const FORCE_SHIFT: usize = 12;
    /// Shift for clearing the Resus after the fault that caused it has been fixed
    pub const CLEAR_SHIFT: usize = 16;

    /// Mask for the number of Reference Clock cycles that should pass before Resus is activated
    pub const TIMEOUT_MASK: u32 = 0xff << TIMEOUT_SHIFT;
    /// Mask for enabling the Resus
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Mask for forcing the Resus to activate
    pub const FORCE_MASK: u32 = 1 << FORCE_SHIFT;
    /// Mask for clearing the Resus after the fault that caused it has been fixed
    pub const CLEAR_MASK: u32 = 1 << CLEAR_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = TIMEOUT_MASK |
        ENABLE_MASK |
        FORCE_MASK |
        CLEAR_MASK;
}

/// Status register masks and shifts for Resus
mod sys_resus_status_register {
    /// Shift for indicating the clock has been resuscitated. If set, the error should be corrected
    /// and `CLEAR` should be set in `sys_resus_ctrl_register`
    pub const RESUSSED_SHIFT: usize = 0;

    /// Mask for indicating the clock has been resuscitated. If set, the error should be corrected
    /// and `CLEAR` should be set in `sys_resus_ctrl_register`
    pub const RESUSSED_MASK: u32 = 1 << RESUSSED_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = RESUSSED_MASK;
}

/// Clocks object for managing all the clocks
pub struct Clocks {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, ClockRegisters>,
    /// Memory mapped registers where writing a bit sets the corresponding bit in `registers`
    set_reg: UniqueMmioPointer<'static, ClockRegisters>,
    /// Memory mapped registers where writing a bit clears the corresponding bit in `registers`
    clear_reg: UniqueMmioPointer<'static, ClockRegisters>
}

impl Clocks {
    /// Creates a new `Clocks` object  
    /// `base` is the base address of the Clocks memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the Clocks memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap())
            }
        }
    }

    /// Disables Resus
    pub fn disable_sys_resus(&mut self) {
        field!(self.registers, sys_resus_ctrl).modify(|sys_resus_ctrl| sys_resus_ctrl & !(ctrl_register::VALID_MASK));
    }

    /// Converts a clock's source clock mask to the corresponding mask in its selected register
    fn clock_bitmap(src: u32, shift: usize) -> u32 {
        1 << (src >> shift)
    }

    /// Sets up the Reference Clock to run from the Crystal Oscillator and the System Clock to run
    /// off the Reference Clock
    pub fn preinit_sys_ref(&mut self) {
        // ref clock
        field!(self.registers, ref_ctrl).modify(|ref_ctrl| (ref_ctrl & !ref_ctrl_register::SRC_MASK) | ref_ctrl_register::SRC_XOSC);
        while field!(self.registers, ref_selected).read() & Self::clock_bitmap(ref_ctrl_register::SRC_XOSC, ref_ctrl_register::SRC_SHIFT) == 0 {} 
        
        // sys clock
        field!(self.registers, sys_ctrl).modify(|sys_ctrl| (sys_ctrl & !sys_ctrl_register::SRC_MASK) | sys_ctrl_register::SRC_REF);
        while field!(self.registers, sys_selected).read() & Self::clock_bitmap(sys_ctrl_register::SRC_REF, sys_ctrl_register::SRC_SHIFT) == 0 {} 
    }

    /// Sets up the System Clock to run off the Sys PLL, the Peripheral Clock to run off the USB
    /// PLL and the Analog to Digital Converter Clock to also run off the USB PLL
    /// # Safety
    /// The Sys PLL and USB PLL must have been correctly initialised
    pub unsafe fn setup_clocks(&mut self) {
        // sys clock
        field!(self.registers, sys_ctrl).modify(|sys_ctrl| sys_ctrl & !sys_ctrl_register::SRC_MASK);
        while field!(self.registers, sys_selected).read() & sys_ctrl_register::SRC_MASK != 1 {}
        field!(self.registers, sys_ctrl).modify(|sys_ctrl| (sys_ctrl & !sys_ctrl_register::AUXSRC_MASK) | sys_ctrl_register::AUXSRC_PLL_SYS);
        field!(self.registers, sys_ctrl).modify(|sys_ctrl| (sys_ctrl & !sys_ctrl_register::SRC_MASK) | sys_ctrl_register::SRC_SYS_AUX);
        while field!(self.registers, sys_selected).read() & Self::clock_bitmap(sys_ctrl_register::SRC_SYS_AUX, sys_ctrl_register::SRC_SHIFT) == 0 {}
        // peripheral clock
        field!(self.clear_reg, peri_ctrl).write(peri_ctrl_register::ENABLE_MASK);
        wait::wait_cycles(100000);
        field!(self.registers, peri_ctrl).modify(|peri_ctrl| (peri_ctrl & !peri_ctrl_register::AUXSRC_MASK) | peri_ctrl_register::AUXSRC_PLL_USB);
        field!(self.set_reg, peri_ctrl).write(peri_ctrl_register::ENABLE_MASK);
        wait::wait_cycles(100000);
        // ADC clock
        field!(self.clear_reg, adc_ctrl).write(ctrl_register::ENABLE_MASK);
        wait::wait_cycles(10000);
        field!(self.registers, adc_ctrl).modify(|adc_ctrl| (adc_ctrl & !ctrl_register::AUXSRC_MASK) | ctrl_register::AUXSRC_PLL_USB);
        field!(self.set_reg, adc_ctrl).write(ctrl_register::ENABLE_MASK);
        wait::wait_cycles(10000);
    }
}

unsafe impl Send for Clocks {}
unsafe impl Sync for Clocks {}

/// Base address for the Clocks memory mapped registers
static CLOCKS_BASE: usize = 0x40008000;

/// Clocks object
pub static CLOCKS: SpinIRQ<Clocks> = unsafe {
    SpinIRQ::new(Clocks::new(CLOCKS_BASE))
};

#[cfg(test)]
mod test {
    use crate::{CS, print, println};
    use super::*;

    #[test_case]
    fn test_setup_correct() {
        println!("Testing clock setup");
        let cs = unsafe {
            CS::new()
        };
        let mut clocks = CLOCKS.lock(&cs);
        print!("Testing sys clock ");
        let sys_ctrl = field!(clocks.registers, sys_ctrl).read();
        let sys_div = field!(clocks.registers, sys_div).read();
        // PLL Sys selected
        assert_eq!(sys_ctrl & sys_ctrl_register::VALID_MASK, 0x1);
        // Haven't modified div from original value
        assert_eq!(sys_div & div_register::VALID_MASK, 0x100);
        println!("[ok]");
        print!("Testing peri clock ");
        let peri_ctrl = field!(clocks.registers, peri_ctrl).read();
        // Enable and PLL USB selected
        assert_eq!(peri_ctrl & peri_ctrl_register::VALID_MASK, 0x840);
        println!("[ok]");
        print!("Testing ADC clock ");
        let adc_ctrl = field!(clocks.registers, adc_ctrl).read();
        // Enable and PLL USB selected
        assert_eq!(adc_ctrl & ctrl_register::VALID_MASK, 0x800);
        println!("[ok]");
        print!("Testing ref clock ");
        let ref_ctrl = field!(clocks.registers, ref_ctrl).read();
        // XOSC selected
        assert_eq!(ref_ctrl & ref_ctrl_register::VALID_MASK, 0x2);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing clock register mask values");
        print!("Testing gpout ctrl register ");
        assert_eq!(gpout_ctrl_register::VALID_MASK, 0x131de0);
        println!("[ok]");
        print!("Testing div register ");
        assert_eq!(div_register::VALID_MASK, u32::MAX);
        println!("[ok]");
        print!("Testing int div register ");
        assert_eq!(int_div_register::VALID_MASK, 0x300);
        println!("[ok]");
        print!("Testing ref ctrl register ");
        assert_eq!(ref_ctrl_register::VALID_MASK, 0x63);
        println!("[ok]");
        print!("Testing sys ctrl register ");
        assert_eq!(sys_ctrl_register::VALID_MASK, 0xe1);
        println!("[ok]");
        print!("Testing peri ctrl register ");
        assert_eq!(peri_ctrl_register::VALID_MASK, 0xce0);
        println!("[ok]");
        print!("Testing ctrl register ");
        assert_eq!(ctrl_register::VALID_MASK, 0x130ce0);
        println!("[ok]");
        print!("Testing sys reseus ctrl register ");
        assert_eq!(sys_resus_ctrl_register::VALID_MASK, 0x111ff);
        println!("[ok]");
        print!("Testing sys reseus status register ");
        assert_eq!(sys_resus_status_register::VALID_MASK, 0x1);
        println!("[ok]");
    }
}
