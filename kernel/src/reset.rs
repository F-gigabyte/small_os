use core::{ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPureWrite, WriteOnly}};

use crate::{mmio::{REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS}, mutex::SpinIRQ};

/// Subsystem reset reset register masks and shifts
mod reset_register {
    /// Analog to Digital Converter reset shift
    pub const ADC_SHIFT: usize = 0;
    /// Bus control reset shift
    pub const BUSCTRL_SHIFT: usize = 1;
    /// DMA reset shift
    pub const DMA_SHIFT: usize = 2;
    /// I2C 0 reset shift
    pub const I2C0_SHIFT: usize = 3;
    /// I2C 1 reset shift
    pub const I2C1_SHIFT: usize = 4;
    /// IO Bank 0 reset shift
    pub const IO_BANK0_SHIFT: usize = 5;
    /// IO QSPI reset shift
    pub const IO_QSPI_SHIFT: usize = 6;
    /// JTAG reset shift
    pub const JTAG_SHIFT: usize = 7;
    /// Pads Bank 0 reset shift
    pub const PADS_BANK0_SHIFT: usize = 8;
    /// Pads QSPI reset shift
    pub const PADS_QSPI_SHIFT: usize = 9;
    /// Programmable IO 0 reset shift
    pub const PIO0_SHIFT: usize = 10;
    /// Programmable IO 1 reset shift
    pub const PIO1_SHIFT: usize = 11;
    /// System PLL reset shift
    pub const PLL_SYS_SHIFT: usize = 12;
    /// USB PLL reset shift
    pub const PLL_USB_SHIFT: usize = 13;
    /// Pulse width module reset shift
    pub const PWM_SHIFT: usize = 14;
    /// Real time clock reset shift
    pub const RTC_SHIFT: usize = 15;
    /// SPI 0 reset shift
    pub const SPI0_SHIFT: usize = 16;
    /// SPI 1 reset shift
    pub const SPI1_SHIFT: usize = 17;
    /// System Config reset shift
    pub const SYSCFG_SHIFT: usize = 18;
    /// System Info reset shift
    pub const SYSINFO_SHIFT: usize = 19;
    /// Testbench manager reset shift
    pub const TBMAN_SHIFT: usize = 20;
    /// Timer reset shift
    pub const TIMER_SHIFT: usize = 21;
    /// UART 0 reset shift
    pub const UART0_SHIFT: usize = 22;
    /// UART 1 reset shift
    pub const UART1_SHIFT: usize = 23;
    /// USB control reset shift
    pub const USBCTRL_SHIFT: usize = 24;

    /// Analog to Digital Converter reset mask
    pub const ADC_MASK: u32 = 1 << ADC_SHIFT;
    /// Bus control reset mask
    pub const BUSCTRL_MASK: u32 = 1 << BUSCTRL_SHIFT;
    /// DMA reset mask
    pub const DMA_MASK: u32 = 1 << DMA_SHIFT;
    /// I2C 0 reset mask
    pub const I2C0_MASK: u32 = 1 << I2C0_SHIFT;
    /// I2C 1 reset mask
    pub const I2C1_MASK: u32 = 1 << I2C1_SHIFT;
    /// IO Bank 0 reset mask
    pub const IO_BANK0_MASK: u32 = 1 << IO_BANK0_SHIFT;
    /// IO QSPI reset mask
    pub const IO_QSPI_MASK: u32 = 1 << IO_QSPI_SHIFT;
    /// JTAG reset mask
    pub const JTAG_MASK: u32 = 1 << JTAG_SHIFT;
    /// Pads Bank 0 reset mask
    pub const PADS_BANK0_MASK: u32 = 1 << PADS_BANK0_SHIFT;
    /// Pads QSPI reset mask
    pub const PADS_QSPI_MASK: u32 = 1 << PADS_QSPI_SHIFT;
    /// Programmable IO 0 reset mask
    pub const PIO0_MASK: u32 = 1 << PIO0_SHIFT;
    /// Programmable IO 1 reset mask
    pub const PIO1_MASK: u32 = 1 << PIO1_SHIFT;
    /// System PLL reset mask
    pub const PLL_SYS_MASK: u32 = 1 << PLL_SYS_SHIFT;
    /// USB PLL reset mask
    pub const PLL_USB_MASK: u32 = 1 << PLL_USB_SHIFT;
    /// Pulse width module reset mask
    pub const PWM_MASK: u32 = 1 << PWM_SHIFT;
    /// Real time clock reset mask
    pub const RTC_MASK: u32 = 1 << RTC_SHIFT;
    /// SPI 0 reset mask
    pub const SPI0_MASK: u32 = 1 << SPI0_SHIFT;
    /// SPI 1 reset mask
    pub const SPI1_MASK: u32 = 1 << SPI1_SHIFT;
    /// System Config reset mask
    pub const SYSCFG_MASK: u32 = 1 << SYSCFG_SHIFT;
    /// System Info reset mask
    pub const SYSINFO_MASK: u32 = 1 << SYSINFO_SHIFT;
    /// Testbench manager reset mask
    pub const TBMAN_MASK: u32 = 1 << TBMAN_SHIFT;
    /// Timer reset mask
    pub const TIMER_MASK: u32 = 1 << TIMER_SHIFT;
    /// UART 0 reset mask
    pub const UART0_MASK: u32 = 1 << UART0_SHIFT;
    /// UART 1 reset mask
    pub const UART1_MASK: u32 = 1 << UART1_SHIFT;
    /// USB control reset mask
    pub const USBCTRL_MASK: u32 = 1 << USBCTRL_SHIFT;

    /// Mask of all devices to reset  
    /// Doesn't include Pads QSPI or IO QSPI (as using flash as XIP)
    pub const ALL_RESET_MASK: u32 = ADC_MASK |
        DMA_MASK |
        BUSCTRL_MASK |
        I2C0_MASK |
        I2C1_MASK |
        IO_BANK0_MASK |
        JTAG_MASK |
        PADS_BANK0_MASK |
        PIO0_MASK |
        PIO1_MASK |
        PLL_SYS_MASK |
        PLL_USB_MASK |
        PWM_MASK |
        RTC_MASK |
        SPI0_MASK |
        SPI1_MASK |
        SYSCFG_MASK |
        SYSINFO_MASK |
        TBMAN_MASK |
        TIMER_MASK |
        UART0_MASK |
        UART1_MASK |
        USBCTRL_MASK;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ADC_MASK |
        BUSCTRL_MASK |
        DMA_MASK |
        I2C0_MASK |
        I2C1_MASK |
        IO_BANK0_MASK |
        IO_QSPI_MASK |
        JTAG_MASK |
        PADS_BANK0_MASK |
        PADS_QSPI_MASK |
        PIO0_MASK |
        PIO1_MASK |
        PLL_SYS_MASK |
        PLL_USB_MASK |
        PWM_MASK |
        RTC_MASK |
        SPI0_MASK |
        SPI1_MASK |
        SYSCFG_MASK |
        SYSINFO_MASK |
        TBMAN_MASK |
        TIMER_MASK |
        UART0_MASK |
        UART1_MASK |
        USBCTRL_MASK;
}

/// Subsystem Reset memory mapped registers
#[repr(C)]
struct ResetRegisters {
    /// Subsystem reset reset register (0x00)
    reset: ReadPureWrite<u32>, // 0x0
    _reserved0: u32, // 0x4
    /// Subsystem reset reset done register (0x08)
    reset_done: ReadOnly<u32> // 0x8
}

/// Subsystem Reset object for managing device resets
pub struct Reset {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, ResetRegisters>,
    /// Memory mapped registers where writing a bit clears the corresponding bit in `registers`
    clear_reg: UniqueMmioPointer<'static, ResetRegisters>,
    /// Memory mapped registers where writing a bit sets the corresponding bit in `registers`
    set_reg: UniqueMmioPointer<'static, ResetRegisters>
}

impl Reset {
    /// Creates a new `Reset` object  
    /// `base` is the base address of the Reset memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the Reset memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap())
            }
        }
    }

    /// Resets all devices that can be safely reset
    pub fn reset_all(&mut self) {
        field!(self.set_reg, reset).write(reset_register::ALL_RESET_MASK);
    }

    /// Pulls IO Bank 0 pads out of reset
    pub fn unreset_pads_bank0(&mut self) {
        field!(self.clear_reg, reset).write(reset_register::PADS_BANK0_MASK);
        while field!(self.registers, reset_done).read() & reset_register::IO_BANK0_MASK == 0 {}
    }

    /// Pulls IO Bank 0 out of reset
    pub fn unreset_iobank0(&mut self) {
        field!(self.clear_reg, reset).write(reset_register::IO_BANK0_MASK);
        while field!(self.registers, reset_done).read() & reset_register::IO_BANK0_MASK == 0 {}
    }

    /// Pulls the system PLL out of reset
    pub fn unreset_pll_sys(&mut self) {
        field!(self.clear_reg, reset).write(reset_register::PLL_SYS_MASK);
        while field!(self.registers, reset_done).read() & reset_register::PLL_SYS_MASK == 0 {}
    }
    
    /// Pulls the USB PLL out of reset
    pub fn unreset_pll_usb(&mut self) {
        field!(self.clear_reg, reset).write(reset_register::PLL_USB_MASK);
        while field!(self.registers, reset_done).read() & reset_register::PLL_USB_MASK == 0 {}
    }

    /// Pulls UART 1 out of reset
    pub fn unreset_uart1(&mut self) {
        field!(self.clear_reg, reset).write(reset_register::UART1_MASK);
        while field!(self.registers, reset_done).read() & reset_register::UART1_MASK == 0 {}
    }
}

unsafe impl Send for Reset {}
unsafe impl Sync for Reset {}

/// Base address for the Subsystem Reset memory mapped registers
static RESET_BASE: usize = 0x4000c000;

/// Subsystem Reset object
pub static RESET: SpinIRQ<Reset> = unsafe {
    SpinIRQ::new(Reset::new(RESET_BASE))
};

#[cfg(test)]
mod test {
    use crate::{inter::CS, print, println};

    use super::*;

    #[test_case]
    fn test_setup_correct() {
        println!("Testing reset setup");
        let cs = unsafe {
            CS::new()
        };
        let mut reset = RESET.lock(&cs);
        print!("Testing reset register ");
        let reset = field!(reset.registers, reset).read();
        // Only IO bank 0, QSPI, PLL Sys, PLL USB, UART1 as USB ctrl out of reset
        assert_eq!(reset & reset_register::VALID_MASK, 0x17fcc9f);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing reset register mask values");
        print!("Testing reset register ");
        assert_eq!(reset_register::VALID_MASK, 0x1ffffff);
        println!("[ok]");
    }
}
