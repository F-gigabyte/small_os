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

/// Control register masks and shifts for an IO Bank 0 Control register
mod gpio_ctrl_register {
    /// Function selector shift
    pub const FUNCSEL_SHIFT: usize = 0;
    /// Output drive shift
    pub const OUTOVER_SHIFT: usize = 8;
    /// Output enable shift
    pub const OEOVER_SHIFT: usize = 12;
    /// Input drive shift
    pub const INOVER_SHIFT: usize = 16;
    /// Interrupt shift
    pub const IRQOVER_SHIFT: usize = 28;

    /// Function selector mask
    pub const FUNCSEL_MASK: u32 = 0x1f << FUNCSEL_SHIFT;
    /// Output drive mask
    pub const OUTOVER_MASK: u32 = 0x3 << OUTOVER_SHIFT;
    /// Output enable mask
    pub const OEOVER_MASK: u32 = 0x3 << OEOVER_SHIFT;
    /// Input drive mask
    pub const INOVER_MASK: u32 = 0x3 << INOVER_SHIFT;
    /// Interrupt mask
    pub const IRQOVER_MASK: u32 = 0x3 << IRQOVER_SHIFT;

    /// Minimum function selector
    pub const FUNCSEL_MIN: u32 = 1 << FUNCSEL_SHIFT;
    /// Maximum function selector
    pub const FUNCSEL_MAX: u32 = 9 << FUNCSEL_SHIFT;
    /// No function selected
    pub const FUNCSEL_NULL: u32 = 31 << FUNCSEL_SHIFT;

    /// Output from peripheral
    pub const OUTOVER_NORMAL: u32 = 0x0 << OUTOVER_SHIFT;
    /// Output inverse of that from the peripheral
    pub const OUTOVER_INVERT: u32 = 0x1 << OUTOVER_SHIFT;
    /// Output driven low
    pub const OUTOVER_LOW: u32 = 0x2 << OUTOVER_SHIFT;
    /// Output driven high
    pub const OUTOVER_HIGH: u32 = 0x3 << OUTOVER_SHIFT;
    
    /// Output enabled / disabled based on peripheral
    pub const OEOVER_NORMAL: u32 = 0x0 << OEOVER_SHIFT;
    /// Output enabled / disabled the opposite of that from the peripheral
    pub const OEOVER_INVERT: u32 = 0x1 << OEOVER_SHIFT;
    /// Output disabled
    pub const OEOVER_DISABLE: u32 = 0x2 << OEOVER_SHIFT;
    /// Output enabled
    pub const OEOVER_ENABLE: u32 = 0x3 << OEOVER_SHIFT;
    
    /// Input given directly to peripheral
    pub const INOVER_NORMAL: u32 = 0x0 << INOVER_SHIFT;
    /// Input inverted before being passed to peripheral
    pub const INOVER_INVERT: u32 = 0x1 << INOVER_SHIFT;
    /// Input driven low
    pub const INOVER_LOW: u32 = 0x2 << INOVER_SHIFT;
    /// Input driven high
    pub const INOVER_HIGH: u32 = 0x3 << INOVER_SHIFT;
    
    /// IRQ same as from peripheral
    pub const IRQOVER_NORMAL: u32 = 0x0 << IRQOVER_SHIFT;
    /// IRQ inverted from that given by the peripheral
    pub const IRQOVER_INVERT: u32 = 0x1 << IRQOVER_SHIFT;
    /// IRQ driven low
    pub const IRQOVER_LOW: u32 = 0x2 << IRQOVER_SHIFT;
    /// IRQ driven high
    pub const IRQOVER_HIGH: u32 = 0x3 << IRQOVER_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = FUNCSEL_MASK |
        OUTOVER_MASK |
        OEOVER_MASK |
        INOVER_MASK |
        IRQOVER_MASK;
}

/// IO Bank 0 memory mapped registers
#[repr(C)]
struct IOBank0Registers {
    // GP0
    _reserved0: u32,
    /// GPIO 0 control register
    gp0_ctrl: ReadPureWrite<u32>,
    // GP1
    _reserved2: u32,
    /// GPIO 1 control register
    gp1_ctrl: ReadPureWrite<u32>,
    // GP2
    _reserved4: u32,
    /// GPIO 2 control register
    gp2_ctrl: ReadPureWrite<u32>,
    // GP3
    _reserved6: u32,
    /// GPIO 3 control register
    gp3_ctrl: ReadPureWrite<u32>,
    // GP4
    _reserved8: u32,
    /// GPIO 4 control register
    gp4_ctrl: ReadPureWrite<u32>,
    // GP5
    _reserved9: u32,
    /// GPIO 5 control register
    gp5_ctrl: ReadPureWrite<u32>,
    // GP6
    _reserved10: u32,
    /// GPIO 6 control register
    gp6_ctrl: ReadPureWrite<u32>,
    // GP7
    _reserved11: u32,
    /// GPIO 7 control register
    gp7_ctrl: ReadPureWrite<u32>,
}

/// IO Bank 0 object for managing the IO Bank 0 GPIOs
pub struct IOBank0 {
    /// IO Bank 0 memory mapped registers
    registers: UniqueMmioPointer<'static, IOBank0Registers>
}

impl IOBank0 {
    /// Creates a new `IOBank0` object  
    /// `base` is the base address the IO Bank 0 memory mapped registers
    /// # Safety
    /// `base` must be a valid addresss which points to the IO Bank 0 memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    /// Initialises GPIO 4 to 7 to be controlled by UART1 (kernel's UART)
    pub fn set_gpio_uart1(&mut self) {
        let val = ((2 << gpio_ctrl_register::FUNCSEL_SHIFT) & gpio_ctrl_register::FUNCSEL_MASK) |
            gpio_ctrl_register::OUTOVER_NORMAL |
            gpio_ctrl_register::OEOVER_NORMAL |
            gpio_ctrl_register::INOVER_NORMAL |
            gpio_ctrl_register::IRQOVER_NORMAL;
        field!(self.registers, gp4_ctrl).modify(|gp4_ctrl| (gp4_ctrl & !gpio_ctrl_register::VALID_MASK) | val);
        field!(self.registers, gp5_ctrl).modify(|gp5_ctrl| (gp5_ctrl & !gpio_ctrl_register::VALID_MASK) | val);
        field!(self.registers, gp6_ctrl).modify(|gp6_ctrl| (gp6_ctrl & !gpio_ctrl_register::VALID_MASK) | val);
        field!(self.registers, gp7_ctrl).modify(|gp7_ctrl| (gp7_ctrl & !gpio_ctrl_register::VALID_MASK) | val);
    }
}

unsafe impl Send for IOBank0 {}
unsafe impl Sync for IOBank0 {}

/// Base address for the IO Bank 0 memory mapped registers
static IOBANK0_BASE: usize = 0x40014000;

/// IO Bank 0 object
pub static IOBANK0: SpinIRQ<IOBank0> = unsafe {
    SpinIRQ::new(IOBank0::new(IOBANK0_BASE))
};

#[cfg(test)]
mod test {
    use super::*;
    use crate::{println, print, CS};

    #[test_case]
    fn test_setup_correct() {
        println!("Testing IO bank 0 setup");
        let cs = unsafe {
            CS::new()
        };
        let mut io_bank0 = IOBANK0.lock(&cs);
        print!("Testing GPIO 4 ");
        let gp4_ctrl = field!(io_bank0.registers, gp4_ctrl).read();
        // UART function selected
        assert_eq!(gp4_ctrl & gpio_ctrl_register::VALID_MASK, 2);
        println!("[ok]");
        print!("Testing GPIO 5 ");
        let gp5_ctrl = field!(io_bank0.registers, gp5_ctrl).read();
        // UART function selected
        assert_eq!(gp5_ctrl & gpio_ctrl_register::VALID_MASK, 2);
        println!("[ok]");
        print!("Testing GPIO 6 ");
        let gp6_ctrl = field!(io_bank0.registers, gp6_ctrl).read();
        // UART function selected
        assert_eq!(gp6_ctrl & gpio_ctrl_register::VALID_MASK, 2);
        println!("[ok]");
        print!("Testing GPIO 7 ");
        let gp7_ctrl = field!(io_bank0.registers, gp7_ctrl).read();
        // UART function selected
        assert_eq!(gp7_ctrl & gpio_ctrl_register::VALID_MASK, 2);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing IO Bank 0 register mask values");
        print!("Testing gpio ctrl register ");
        assert_eq!(gpio_ctrl_register::VALID_MASK, 0x3003331f);
        println!("[ok]");
    }
}
