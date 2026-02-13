use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite}};

use crate::mutex::IRQMutex;

#[repr(C)]
struct InterSetEnable {
    register: ReadPureWrite<u32> // 0xe100 
}

#[repr(C)]
struct InterClearEnable {
    register: ReadPureWrite<u32> // 0xe180
}

#[repr(C)]
struct InterSetPending {
    register: ReadPureWrite<u32> // 0xe200
}

#[repr(C)]
struct InterClearPending {
    register: ReadPureWrite<u32> // 0xe280
}

#[repr(C)]
struct InterPriorityRegisters {
    inter_priority: [ReadPureWrite<u32>; 8], // 0xe400 -> 0xe41c
}

pub mod irqs {
    pub const TIMER0: u8 = 0;
    pub const TIMER1: u8 = 1;
    pub const TIMER2: u8 = 2;
    pub const TIMER3: u8 = 3;
    pub const PWM_WRAP: u8 = 4;
    pub const USB_CTRL: u8 = 5;
    pub const XIP: u8 = 6;
    pub const PIO0_IRQ0: u8 = 7;
    pub const PIO0_IRQ1: u8 = 8;
    pub const PIO1_IRQ0: u8 = 9;
    pub const PIO1_IRQ1: u8 = 10;
    pub const IO_BANK0: u8 = 11;
    pub const IO_QSPI: u8 = 12;
    pub const SIO_PROC0: u8 = 13;
    pub const SIO_PROC1: u8 = 14;
    pub const CLOCKS: u8 = 15;
    pub const SPI0: u8 = 16;
    pub const SPI1: u8 = 17;
    pub const UART0: u8 = 18;
    pub const UART1: u8 = 19;
    pub const ADC_FIFO: u8 = 20;
    pub const I2C0: u8 = 21;
    pub const I2C1: u8 = 22;
}

mod inter_priority_register {
    pub const IP_0_SHIFT: usize = 6;
    pub const IP_1_SHIFT: usize = 14;
    pub const IP_2_SHIFT: usize = 22;
    pub const IP_3_SHIFT: usize = 30;

    pub const IP_0_MASK: u32 = 0x3 << IP_0_SHIFT;
    pub const IP_1_MASK: u32 = 0x3 << IP_1_SHIFT;
    pub const IP_2_MASK: u32 = 0x3 << IP_2_SHIFT;
    pub const IP_3_MASK: u32 = 0x3 << IP_3_SHIFT;
}

pub struct NVIC {
    inter_set_enable_reg: UniqueMmioPointer<'static, InterSetEnable>,
    inter_clear_enable_reg: UniqueMmioPointer<'static, InterClearEnable>,
    inter_set_pend_reg: UniqueMmioPointer<'static, InterSetPending>,
    inter_clear_pend_reg: UniqueMmioPointer<'static, InterClearPending>,
    inter_priority_registers: UniqueMmioPointer<'static, InterPriorityRegisters>,
}

const INTER_SET_ENABLE_OFFSET: usize = 0x0;
const INTER_CLEAR_ENABLE_OFFSET: usize = 0x80;
const INTER_SET_PENDING_OFFSET: usize = 0x100;
const INTER_CLEAR_PENDING_OFFSET: usize = 0x180;
const INTER_PRIORITY_OFFSET: usize = 0x300;

impl NVIC {
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

    #[inline(always)]
    pub fn enable_irq(&mut self, irq: u8) {
        field!(self.inter_set_enable_reg, register).write(1 << irq);
    }

    #[inline(always)]
    pub fn clear_pending_irq(&mut self, irq: u8) {
        field!(self.inter_clear_pend_reg, register).write(1 << irq);
    }

    pub fn get_pending(&mut self) -> u32 {
        field!(self.inter_set_pend_reg, register).read()
    }
}

unsafe impl Send for NVIC {}
unsafe impl Sync for NVIC {}

static NVIC_BASE: usize = 0xe000e100;
// SAFETY
// NVIC implemented for both processors so there shouldn't be a data race
pub static NVIC: IRQMutex<NVIC> = unsafe {
    IRQMutex::new(NVIC::new(NVIC_BASE))
};
