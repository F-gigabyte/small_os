use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};

use crate::mutex::SpinIRQ;

/// GPIO register masks and shifts for a IO Bank 0 GPIO memory mapped register
mod gpio_register {
    /// Slew rate control shift (0 for slow and 1 for fast)
    pub const SLEWFAST_SHIFT: usize = 0;
    /// Shift for enabling the schmitt trigger
    pub const SCHMITT_SHIFT: usize = 1;
    /// Shift for enabling pull down (pin drawn low)
    pub const PULL_DOWN_ENABLE_SHIFT: usize = 2;
    /// Shift for enabling pull up (pin drawn high)
    pub const PULL_UP_ENABLE_SHIFT: usize = 3;
    /// Drive strength shift
    pub const DRIVE_SHIFT: usize = 4;
    /// Shift for input enable
    pub const INPUT_ENABLE_SHIFT: usize = 6;
    /// Shift for output disable
    pub const OUTPUT_DISABLE_SHIFT: usize = 7;

    /// Slew rate control mask (0 for slow and 1 for fast)
    pub const SLEWFAST_MASK: u32 = 1 << SLEWFAST_SHIFT;
    /// Mask for enabling the schmitt trigger
    pub const SCHMITT_MASK: u32 = 1 << SCHMITT_SHIFT;
    /// Mask for enabling pull down (pin drawn low)
    pub const PULL_DOWN_ENABLE_MASK: u32 = 1 << PULL_DOWN_ENABLE_SHIFT;
    /// Mask for enabling pull up (pin drawn high)
    pub const PULL_UP_ENABLE_MASK: u32 = 1 << PULL_UP_ENABLE_SHIFT;
    /// Drive strength mask
    pub const DRIVE_MASK: u32 = 0x3 << DRIVE_SHIFT;
    /// Mask for input enable
    pub const INPUT_ENABLE_MASK: u32 = 1 << INPUT_ENABLE_SHIFT;
    /// Mask for output disable
    pub const OUTPUT_DISABLE_MASK: u32 = 1 << OUTPUT_DISABLE_SHIFT;

    /// Drive strength of 2mA
    pub const DRIVE_2MA: u32 = 0x0 << DRIVE_SHIFT;
    /// Drive strength of 4mA
    pub const DRIVE_4MA: u32 = 0x1 << DRIVE_SHIFT;
    /// Drive strength of 8mA
    pub const DRIVE_8MA: u32 = 0x2 << DRIVE_SHIFT;
    /// Drive strength of 12mA
    pub const DRIVE_12MA: u32 = 0x3 << DRIVE_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SLEWFAST_MASK |
        SCHMITT_MASK |
        PULL_DOWN_ENABLE_MASK |
        PULL_UP_ENABLE_MASK |
        DRIVE_MASK |
        INPUT_ENABLE_MASK |
        OUTPUT_DISABLE_MASK;
}

/// IO Bank 0 memory mapped registers
#[repr(C)]
struct PadsBank0Registers {
    // voltage select
    _reserved: u32, // 0x0
    /// GPIO 0 register (0x04)
    gp0: ReadPureWrite<u32>, // 0x4
    /// GPIO 1 register (0x08)
    gp1: ReadPureWrite<u32>, // 0x8
    /// GPIO 2 register (0x0c)
    gp2: ReadPureWrite<u32>, // 0xc
    /// GPIO 3 register (0x10)
    gp3: ReadPureWrite<u32>, // 0x10
    /// GPIO 4 register (0x14)
    gp4: ReadPureWrite<u32>, // 0x14
    /// GPIO 5 register (0x18)
    gp5: ReadPureWrite<u32>, // 0x18
    /// GPIO 6 register (0x1c)
    gp6: ReadPureWrite<u32>, // 0x1c
    /// GPIO 7 register (0x20)
    gp7: ReadPureWrite<u32>, // 0x20
}

/// IO Bank 0 Pads object for managing the IO Bank 0 pads
pub struct PadsBank0 {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, PadsBank0Registers>
}

impl PadsBank0 {
    /// Creates a new `PadsBank0` object
    /// `base` is the base address of the IO Bank 0 pads memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the IO Bank 0 pads memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    /// Sets up GPIO 4 to 7 pads ready to be used by the UART
    pub fn set_pads_uart1(&mut self) {
        let val = gpio_register::DRIVE_4MA | 
            gpio_register::PULL_DOWN_ENABLE_MASK | 
            gpio_register::INPUT_ENABLE_MASK |
            gpio_register::SCHMITT_MASK;
        field!(self.registers, gp4).modify(|gp4| (gp4 & !gpio_register::VALID_MASK) | val);
        field!(self.registers, gp5).modify(|gp5| (gp5 & !gpio_register::VALID_MASK) | val);
        field!(self.registers, gp6).modify(|gp6| (gp6 & !gpio_register::VALID_MASK) | val);
        field!(self.registers, gp7).modify(|gp7| (gp7 & !gpio_register::VALID_MASK) | val);
    }
}

unsafe impl Send for PadsBank0 {}
unsafe impl Sync for PadsBank0 {}

/// Base address for the IO Bank 0 pads memory mapped registers
static PADS_BANK0_BASE: usize = 0x4001c000;

/// IO Bank 0 pads object
pub static PADS_BANK0: SpinIRQ<PadsBank0> = unsafe {
    SpinIRQ::new(PadsBank0::new(PADS_BANK0_BASE))
};

#[cfg(test)]
mod test {
    use super::*;
    use crate::{println, print, CS};

    #[test_case]
    fn test_setup_correct() {
        println!("Testing IO bank 0 pads setup");
        let cs = unsafe {
            CS::new()
        };
        let mut pads_bank0 = PADS_BANK0.lock(&cs);
        print!("Testing GPIO 4 ");
        let gp4 = field!(pads_bank0.registers, gp4).read();
        assert_eq!(gp4 & gpio_register::VALID_MASK, 0x56);
        println!("[ok]");
        print!("Testing GPIO 5 ");
        let gp5 = field!(pads_bank0.registers, gp5).read();
        assert_eq!(gp5 & gpio_register::VALID_MASK, 0x56);
        println!("[ok]");
        print!("Testing GPIO 6 ");
        let gp6 = field!(pads_bank0.registers, gp6).read();
        assert_eq!(gp6 & gpio_register::VALID_MASK, 0x56);
        println!("[ok]");
        print!("Testing GPIO 7 ");
        let gp7 = field!(pads_bank0.registers, gp7).read();
        assert_eq!(gp7 & gpio_register::VALID_MASK, 0x56);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing IO bank 0 pads register mask values");
        print!("Testing gpio register ");
        assert_eq!(gpio_register::VALID_MASK, 0xff);
        println!("[ok]");
    }
}
