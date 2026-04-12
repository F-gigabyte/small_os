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

use crate::{mmio::REG_ALIAS_CLR_BITS, mutex::SpinIRQ};

/// PLL control and status register masks and shifts
mod cs_register {
    /// PLL reference clock divisor shift
    pub const REF_DIV_SHIFT: usize = 0;
    /// Shift for passing the reference clock to the output instead of the divided VCO
    pub const BYPASS_SHIFT: usize = 8;
    /// Shift for determining if the PLL is locked
    pub const LOCK_SHIFT: usize = 31;

    /// PLL reference clock divisor mask
    pub const REF_DIV_MASK: u32 = 0x3f << REF_DIV_SHIFT;
    /// Mask for passing the reference clock to the output instead of the divided VCO
    pub const BYPASS_MASK: u32 = 1 << BYPASS_SHIFT;
    /// Mask for determining if the PLL is locked
    pub const LOCK_MASK: u32 = 1 << LOCK_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = REF_DIV_MASK |
        BYPASS_MASK |
        LOCK_MASK;
}

/// PLL power register masks and shifts
mod power_register {
    /// Shift for powering down the PLL
    pub const PD_SHIFT: usize = 0;
    /// Shift for PLL DSM power down
    pub const DSMPD_SHIFT: usize = 2;
    /// Shift for PLL post divider power down
    pub const POSTDIVPD_SHIFT: usize = 3;
    /// Shift for PLL VCO power down
    pub const VCOPD_SHIFT: usize = 5;

    /// Mask for powering down the PLL
    pub const PD_MASK: u32 = 1 << PD_SHIFT;
    /// Mask for PLL DSM power down
    pub const DSMPD_MASK: u32 = 1 << DSMPD_SHIFT;
    /// Mask for PLL post divider power down
    pub const POSTDIVPD_MASK: u32 = 1 << POSTDIVPD_SHIFT;
    /// Mask for PLL VCO power down
    pub const VCOPD_MASK: u32 = 1 << VCOPD_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = PD_MASK |
        DSMPD_MASK |
        POSTDIVPD_MASK |
        VCOPD_MASK;
}

/// PLL feedback divisor register masks and shifts
mod fbdiv_int_register {
    /// Shift for feeback divisor
    pub const VALUE_SHIFT: usize = 0;

    /// Mask for feeback divisor
    pub const VALUE_MASK: u32 = 0xfff << VALUE_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = VALUE_MASK;
}

/// PLL post dividers register masks and shifts
mod prim_register {
    /// 2nd post divider shift (in range 1 to 7)
    pub const POSTDIV2_SHIFT: usize = 12;
    /// 1st post divider shift (in range 1 to 7)
    pub const POSTDIV1_SHIFT: usize = 16;

    /// 2nd post divider mask (in range 1 to 7)
    pub const POSTDIV2_MASK: u32 = 0x7 << POSTDIV2_SHIFT;
    /// 1st post divider mask (in range 1 to 7)
    pub const POSTDIV1_MASK: u32 = 0x7 << POSTDIV1_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = POSTDIV2_MASK |
        POSTDIV1_MASK;
}

/// PLL memory mapped registers
#[repr(C)]
struct PLLRegisters {
    /// PLL control and status register (0x00)
    cs: ReadPureWrite<u32>, // 0x0
    /// PLL power register (0x04)
    power: ReadPureWrite<u32>, // 0x4
    /// PLL feedback divisor register (0x08)
    fbdiv_int: ReadPureWrite<u32>, // 0x8
    /// PLL post dividers register for primary output (0x0c)
    prim: ReadPureWrite<u32>, // 0xc
}

/// PLL object for managing a PLL
pub struct PLL {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, PLLRegisters>,
    /// Memory mapped registers where writing a bit clears the corresponding bit in `registers`
    clear_reg: UniqueMmioPointer<'static, PLLRegisters>
}

impl PLL {
    /// Creates a new `PLL` object  
    /// `base` is the base address of the Clocks memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to a PLL memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap())
            }
        }
    }

    /// Reset the PLL as the System PLL  
    /// This configures the PLL's output clock to run at 133 MHz which is the fastest clock rate that the
    /// pico can run at
    pub fn reset_sys(&mut self) {
        // reference divider is 1
        let ref_div = 1 << cs_register::REF_DIV_SHIFT;
        // feedback divisor is 133
        let fbdiv = 133 << fbdiv_int_register::VALUE_SHIFT;
        // 1st post divider is 6
        let pd1 = 6 << prim_register::POSTDIV1_SHIFT;
        // 2nd post divider is 2
        let pd2 = 2 << prim_register::POSTDIV2_SHIFT;
        let prim = pd1 | pd2;
        if field!(self.registers, cs).read() & cs_register::LOCK_MASK != 0 && 
            field!(self.registers, cs).read() & cs_register::REF_DIV_MASK == ref_div && 
            field!(self.registers, fbdiv_int).read() & fbdiv_int_register::VALUE_MASK == fbdiv && 
            field!(self.registers, prim).read() & (prim_register::POSTDIV1_MASK | prim_register::POSTDIV2_MASK) == prim {
                return;
        }
        field!(self.registers, cs).modify(|cs| (cs & !cs_register::VALID_MASK) | ref_div);
        field!(self.registers, fbdiv_int).modify(|fbdiv_int| (fbdiv_int & !fbdiv_int_register::VALID_MASK) | fbdiv);
        field!(self.clear_reg, power).modify(|power| (power & !power_register::VALID_MASK) | power_register::PD_MASK | power_register::VCOPD_MASK);
        // wait for PLL to lock
        while field!(self.registers, cs).read() & cs_register::LOCK_MASK == 0 {}
        field!(self.registers, prim).modify(|prim_reg| (prim_reg & !prim_register::VALID_MASK) | prim);
        field!(self.clear_reg, power).modify(|power| (power & !power_register::VALID_MASK) | power_register::POSTDIVPD_MASK);
    }
    
    /// Reset the PLL as the USB PLL  
    /// This configures the PLL's output clock to run at 48 MHz which can be used by the Analog to
    /// Digital Converter and the Peripheral Clock  
    /// This would also be used as a clock for a USB driver as well if implemented but a USB driver
    /// is a big undertaking
    pub fn reset_usb(&mut self) {
        // reference divider is 1
        let ref_div = 1 << cs_register::REF_DIV_SHIFT;
        // feedback divisor is 120
        let fbdiv = 120 << fbdiv_int_register::VALUE_SHIFT;
        // 1st post divisor is 6
        let pd1 = 6 << prim_register::POSTDIV1_SHIFT;
        // 2nd post divisor is 5
        let pd2 = 5 << prim_register::POSTDIV2_SHIFT;
        let prim = pd1 | pd2;
        if field!(self.registers, cs).read() & cs_register::LOCK_MASK != 0 && 
            field!(self.registers, cs).read() & cs_register::REF_DIV_MASK == ref_div && 
            field!(self.registers, fbdiv_int).read() & fbdiv_int_register::VALUE_MASK == fbdiv && 
            field!(self.registers, prim).read() & (prim_register::POSTDIV1_MASK | prim_register::POSTDIV2_MASK) == prim {
                return;
        }
        field!(self.registers, cs).modify(|cs| (cs & !cs_register::VALID_MASK) | ref_div);
        field!(self.registers, fbdiv_int).modify(|fbdiv_int| (fbdiv_int & !fbdiv_int_register::VALID_MASK) | fbdiv);
        field!(self.clear_reg, power).modify(|power| (power & !power_register::VALID_MASK) | power_register::PD_MASK | power_register::VCOPD_MASK);
        // wait for PLL to lock
        while field!(self.registers, cs).read() & cs_register::LOCK_MASK == 0 {}
        field!(self.registers, prim).modify(|prim_reg| (prim_reg & !prim_register::VALID_MASK) | prim);
        field!(self.clear_reg, power).modify(|power| (power & !power_register::VALID_MASK) | power_register::POSTDIVPD_MASK);
    }
}

unsafe impl Send for PLL {}
unsafe impl Sync for PLL {}

/// Base address for the System PLL memory mapped registers
static PLL_SYS_BASE: usize = 0x40028000;
/// Base address for the USB PLL memory mapped registers
static PLL_USB_BASE: usize = 0x4002c000;

/// System PLL object
pub static PLL_SYS: SpinIRQ<PLL> = unsafe {
    SpinIRQ::new(PLL::new(PLL_SYS_BASE))
};

/// USB PLL object
pub static PLL_USB: SpinIRQ<PLL> = unsafe {
    SpinIRQ::new(PLL::new(PLL_USB_BASE))
};

#[cfg(test)]
mod test {
    use crate::{inter::CS, print, println};
    use super::*;

    #[test_case]
    fn test_setup_correct_pll_sys() {
        println!("Testing PLL Sys setup");
        let cs = unsafe {
            CS::new()
        };
        let mut pll_sys = PLL_SYS.lock(&cs);
        print!("Testing pll sys cs ");
        let cs = field!(pll_sys.registers, cs).read();
        // Ref div is 1 and PLL should be locked
        assert_eq!(cs & cs_register::VALID_MASK, 0x80000001);
        println!("[ok]");
        print!("Testing pll sys power ");
        let power = field!(pll_sys.registers, power).read();
        // Only PLL DSM powerdown should be enabled
        assert_eq!(power & power_register::VALID_MASK, 0x4);
        println!("[ok]");
        print!("Testing pll sys fb div ");
        let fb_div = field!(pll_sys.registers, fbdiv_int).read();
        // Feedback divider should be 133
        assert_eq!(fb_div, 133);
        println!("[ok]");
        print!("Testing pll sys prim ");
        let prim = field!(pll_sys.registers, prim).read();
        // PD1 6, PD2 2
        assert_eq!(prim & prim_register::VALID_MASK, 0x62000);
        println!("[ok]");
    }
    
    #[test_case]
    fn test_setup_correct_pll_usb() {
        println!("Testing PLL USB setup");
        let cs = unsafe {
            CS::new()
        };
        let mut pll_sys = PLL_USB.lock(&cs);
        print!("Testing pll usb cs ");
        let cs = field!(pll_sys.registers, cs).read();
        // Ref div is 1 and PLL should be locked
        assert_eq!(cs & cs_register::VALID_MASK, 0x80000001);
        println!("[ok]");
        print!("Testing pll usb power ");
        let power = field!(pll_sys.registers, power).read();
        // Only PLL DSM powerdown should be enabled
        assert_eq!(power & power_register::VALID_MASK, 0x4);
        println!("[ok]");
        print!("Testing pll usb fb div ");
        let fb_div = field!(pll_sys.registers, fbdiv_int).read();
        // Feedback divider should be 120
        assert_eq!(fb_div & fbdiv_int_register::VALID_MASK, 120);
        println!("[ok]");
        print!("Testing pll usb prim ");
        let prim = field!(pll_sys.registers, prim).read();
        // PD1 6, PD2 5
        assert_eq!(prim & prim_register::VALID_MASK, 0x65000);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing PLL register mask values");
        print!("Testing cs register ");
        assert_eq!(cs_register::VALID_MASK, 0x8000013f);
        println!("[ok]");
        print!("Testing power register ");
        assert_eq!(power_register::VALID_MASK, 0x2d);
        println!("[ok]");
        print!("Testing fbdiv int register ");
        assert_eq!(fbdiv_int_register::VALID_MASK, 0xfff);
        println!("[ok]");
        print!("Testing prim register ");
        assert_eq!(prim_register::VALID_MASK, 0x77000);
        println!("[ok]");
    }
}
