use core::{ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPureWrite, WriteOnly}};

use crate::{mmio::{REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS}, mutex::SpinIRQ};

mod reset_register {
    pub const IO_BANK0_SHIFT: usize = 5;
    pub const PLL_SYS_SHIFT: usize = 12;
    pub const UART1_SHIFT: usize = 23;

    pub const IO_BANK0_MASK: u32 = 1 << IO_BANK0_SHIFT;
    pub const PLL_SYS_MASK: u32 = 1 << PLL_SYS_SHIFT;
    pub const UART1_MASK: u32 = 1 << UART1_SHIFT;
}


#[repr(C)]
struct ResetRegisters {
    reset: ReadPureWrite<u32>,
    _reserved0: u32,
    reset_done: ReadOnly<u32>
}

#[repr(C)]
struct ResetClearRegisters {
    reset: WriteOnly<u32>,
}

#[repr(C)]
struct ResetSetRegisters {
    reset: WriteOnly<u32>,
}

pub struct Reset {
    registers: UniqueMmioPointer<'static, ResetRegisters>,
    clear_reg: UniqueMmioPointer<'static, ResetClearRegisters>,
    set_reg: UniqueMmioPointer<'static, ResetSetRegisters>
}

impl Reset {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap())
            }
        }
    }

    pub fn reset_iobank0(&mut self) {
        field!(self.set_reg, reset).write(reset_register::IO_BANK0_MASK);
        field!(self.clear_reg, reset).write(reset_register::IO_BANK0_MASK);
        while field!(self.registers, reset_done).read() & reset_register::IO_BANK0_MASK == 0 {}
    }

    pub fn reset_pll_sys(&mut self) {
        field!(self.set_reg, reset).write(reset_register::PLL_SYS_MASK);
        field!(self.clear_reg, reset).write(reset_register::PLL_SYS_MASK);
        while field!(self.registers, reset_done).read() & reset_register::PLL_SYS_MASK == 0 {}
    }

    pub fn reset_uart1(&mut self) {
        field!(self.set_reg, reset).write(reset_register::UART1_MASK);
        field!(self.clear_reg, reset).write(reset_register::UART1_MASK);
        while field!(self.registers, reset_done).read() & reset_register::UART1_MASK == 0 {}
    }
}

unsafe impl Send for Reset {}
unsafe impl Sync for Reset {}

static RESET_BASE: usize = 0x4000c000;

pub static RESET: SpinIRQ<Reset> = unsafe {
    SpinIRQ::new(Reset::new(RESET_BASE))
};
