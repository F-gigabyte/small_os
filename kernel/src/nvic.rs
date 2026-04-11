use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};

use crate::{mutex::IRQMutex};

/// NVIC Interrupt Set Enable memory mapped register
#[repr(C)]
struct InterSetEnable {
    /// The register (0xe100)
    register: ReadPureWrite<u32> // 0xe100 
}

/// NVIC Interrupt Clear Enable memory mapped register
#[repr(C)]
struct InterClearEnable {
    /// The register (0xe180)
    register: ReadPureWrite<u32> // 0xe180
}

/// NVIC Interrupt Set Pending memory mapped register
#[repr(C)]
struct InterSetPending {
    /// The register (0xe200)
    register: ReadPureWrite<u32> // 0xe200
}

/// NVIC Interrupt Clear Pending memory mapped register
#[repr(C)]
struct InterClearPending {
    /// The register (0xe280)
    register: ReadPureWrite<u32> // 0xe280
}

/// NVIC Interrupt Priority memory mapped registers
#[repr(C)]
struct InterPriorityRegisters {
    /// Interrupts priority registers
    inter_priority: [ReadPureWrite<u32>; 8], // 0xe400 -> 0xe41c
}

/// NVIC IRQs
pub mod irqs {
    /// Timer 0 interrupt
    pub const TIMER0: u8 = 0;
    /// Timer 1 interrupt
    pub const TIMER1: u8 = 1;
    /// Timer 2 interrupt
    pub const TIMER2: u8 = 2;
    /// Timer 3 interrupt
    pub const TIMER3: u8 = 3;
    /// Pulse width module wrap interrupt
    pub const PWM_WRAP: u8 = 4;
    /// USB control interrupt
    pub const USB_CTRL: u8 = 5;
    /// Execute in place interrupt
    pub const XIP: u8 = 6;
    /// Programmable IO 0 interrupt 0
    pub const PIO0_IRQ0: u8 = 7;
    /// Programmable IO 0 interrupt 1
    pub const PIO0_IRQ1: u8 = 8;
    /// Programmable IO 1 interrupt 0
    pub const PIO1_IRQ0: u8 = 9;
    /// Programmable IO 1 interrupt 1
    pub const PIO1_IRQ1: u8 = 10;
    /// IO Bank 0 interrupt
    pub const IO_BANK0: u8 = 11;
    /// IO QSPI interrupt
    pub const IO_QSPI: u8 = 12;
    /// Single-cycle IO processor 0 interrupt
    pub const SIO_PROC0: u8 = 13;
    /// Single-cycle IO processor 1 interrupt
    pub const SIO_PROC1: u8 = 14;
    /// Clocks interrupt
    pub const CLOCKS: u8 = 15;
    /// SPI 0 interrupt
    pub const SPI0: u8 = 16;
    /// SPI 1 interrupt
    pub const SPI1: u8 = 17;
    /// UART 0 interrupt
    pub const UART0: u8 = 18;
    /// UART 1 interrupt
    pub const UART1: u8 = 19;
    /// Analog to Digital Converter FIFO interrupt
    pub const ADC_FIFO: u8 = 20;
    /// I2C 0 interrupt
    pub const I2C0: u8 = 21;
    /// I2C 1 interrupt
    pub const I2C1: u8 = 22;
}

/// NVIC interrupt priority register masks and shifts
mod inter_priority_register {
    /// Shift for 1st interrupt priority
    pub const IP_0_SHIFT: usize = 6;
    /// Shift for 2nd interrupt priority
    pub const IP_1_SHIFT: usize = 14;
    /// Shift for 3rd interrupt priority
    pub const IP_2_SHIFT: usize = 22;
    /// Shift for 4th interrupt priority
    pub const IP_3_SHIFT: usize = 30;

    /// Mask for 1st interrupt priority
    pub const IP_0_MASK: u32 = 0x3 << IP_0_SHIFT;
    /// Mask for 2nd interrupt priority
    pub const IP_1_MASK: u32 = 0x3 << IP_1_SHIFT;
    /// Mask for 3rd interrupt priority
    pub const IP_2_MASK: u32 = 0x3 << IP_2_SHIFT;
    /// Mask for 4th interrupt priority
    pub const IP_3_MASK: u32 = 0x3 << IP_3_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = IP_0_MASK |
        IP_1_MASK |
        IP_2_MASK |
        IP_3_MASK;
}

/// NVIC object for managing the NVIC
pub struct NVIC {
    /// Interrupt set enable memory mapped register
    inter_set_enable_reg: UniqueMmioPointer<'static, InterSetEnable>,
    /// Interrupt clear enable memory mapped register
    inter_clear_enable_reg: UniqueMmioPointer<'static, InterClearEnable>,
    /// Interrupt set pending memory mapped register
    inter_set_pend_reg: UniqueMmioPointer<'static, InterSetPending>,
    /// Interrupt clear pending memory mapped register
    inter_clear_pend_reg: UniqueMmioPointer<'static, InterClearPending>,
    /// Interrupt priority memory mapped registers
    inter_priority_registers: UniqueMmioPointer<'static, InterPriorityRegisters>,
}

/// NVIC Interrupt Set Enable memory mapped register offset
const INTER_SET_ENABLE_OFFSET: usize = 0x0;
/// NVIC Interrupt Clear Enable memory mapped register offset
const INTER_CLEAR_ENABLE_OFFSET: usize = 0x80;
/// NVIC Interrupt Set Pending memory mapped register offset
const INTER_SET_PENDING_OFFSET: usize = 0x100;
/// NVIC Interrupt Clear Pending memory mapped register offset
const INTER_CLEAR_PENDING_OFFSET: usize = 0x180;
/// NVIC Interrupt Priority memory mapped registers offset
const INTER_PRIORITY_OFFSET: usize = 0x300;

impl NVIC {
    /// Creates a new `NVIC` object
    /// `base` is the base of the NVIC memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the NVIC memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                inter_set_enable_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + INTER_SET_ENABLE_OFFSET)).unwrap()),
                inter_clear_enable_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + INTER_CLEAR_ENABLE_OFFSET)).unwrap()),
                inter_set_pend_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + INTER_SET_PENDING_OFFSET)).unwrap()),
                inter_clear_pend_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + INTER_CLEAR_PENDING_OFFSET)).unwrap()),
                inter_priority_registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + INTER_PRIORITY_OFFSET)).unwrap()),
            }
        }
    }

    /// Enables the IRQ  
    /// `irq` is the IRQ to enable
    #[inline(always)]
    pub fn enable_irq(&mut self, irq: u8) {
        field!(self.inter_set_enable_reg, register).write(1 << irq);
    }

    /// Clears a pending IRQ
    /// `irq` is the IRQ to clear
    #[inline(always)]
    pub fn clear_pending_irq(&mut self, irq: u8) {
        field!(self.inter_clear_pend_reg, register).write(1 << irq);
    }

    /// Gets the state of pending IRQs
    #[inline(always)]
    pub fn get_pending(&mut self) -> u32 {
        field!(self.inter_set_pend_reg, register).read()
    }

    /// Disable an IRQ  
    /// `irq` is the IRQ to disable
    #[inline(always)]
    pub fn disable_irq(&mut self, irq: u8) {
        field!(self.inter_clear_enable_reg, register).write(1 << irq);
    }

    /// Disables all IRQs
    #[inline(always)]
    pub fn disable_all_irq(&mut self) -> u32 {
        let mask = field!(self.inter_clear_enable_reg, register).read();
        field!(self.inter_clear_enable_reg, register).write(mask);
        mask
    }

    /// Restores the IRQ context from `mask`
    #[cfg(test)]
    #[inline(always)]
    pub fn restore_irq(&mut self, mask: u32) {
        field!(self.inter_set_enable_reg, register).write(mask);
    }
}

unsafe impl Send for NVIC {}
unsafe impl Sync for NVIC {}

/// Base address for the NVIC memory mapped registers
static NVIC_BASE: usize = 0xe000e100;

// Safety
// NVIC implemented for both processors so there shouldn't be a data race
/// NVIC object
pub static NVIC: IRQMutex<NVIC> = unsafe {
    IRQMutex::new(NVIC::new(NVIC_BASE))
};

#[cfg(test)]
mod test {
    use crate::{print, println, CS};
    use super::*;

    #[test_case]
    fn test_setup_correct() {
        println!("Testing NVIC setup");
        let cs = unsafe {
            CS::new()
        };
        let mut nvic = NVIC.lock(&cs);
        // since no processes are running yet, all interrupts should be disabled
        print!("Testing no interrupts enabled ");
        let inter = field!(nvic.inter_set_enable_reg, register).read();
        assert_eq!(inter, 0);
        println!("[ok]");
        print!("Testing all priorities are 0 ");
        for p in field!(nvic.inter_priority_registers, inter_priority).as_slice() {
            assert_eq!(p.read(), 0);
        }
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing NVIC register mask values");
        print!("Testing inter priority regiter ");
        assert_eq!(inter_priority_register::VALID_MASK, 0xc0c0c0c0);
        println!("[ok]");
    }
}
