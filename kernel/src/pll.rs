use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};

use crate::{mmio::REG_ALIAS_CLR_BITS, mutex::SpinIRQ};

mod cs_register {
    pub const REF_DIV_SHIFT: usize = 0;
    pub const BYPASS_SHIFT: usize = 8;
    pub const LOCK_SHIFT: usize = 31;

    pub const REF_DIV_MASK: u32 = 0x1f << REF_DIV_SHIFT;
    pub const BYPASS_MASK: u32 = 1 << BYPASS_SHIFT;
    pub const LOCK_MASK: u32 = 1 << LOCK_SHIFT;
}

mod power_register {
    pub const PD_SHIFT: usize = 0;
    pub const DSMPD_SHIFT: usize = 2;
    pub const POSTDIVPD_SHIFT: usize = 3;
    pub const VCOPD_SHIFT: usize = 5;

    pub const PD_MASK: u32 = 1 << PD_SHIFT;
    pub const DSMPD_MASK: u32 = 1 << DSMPD_SHIFT;
    pub const POSTDIVPD_MASK: u32 = 1 << POSTDIVPD_SHIFT;
    pub const VCOPD_MASK: u32 = 1 << VCOPD_SHIFT;
}

mod fbdiv_int_register {
    pub const VALUE_SHIFT: usize = 0;

    pub const VALUE_MASK: u32 = 0xfff << VALUE_SHIFT;
}

mod prim_register {
    pub const POSTDIV2_SHIFT: usize = 12;
    pub const POSTDIV1_SHIFT: usize = 16;

    pub const POSTDIV2_MASK: u32 = 0x7 << POSTDIV2_SHIFT;
    pub const POSTDIV1_MASK: u32 = 0x7 << POSTDIV1_SHIFT;
}

#[repr(C)]
struct PLLRegisters {
    cs: ReadPureWrite<u32>, // 0x0
    power: ReadPureWrite<u32>, // 0x4
    fbdiv_int: ReadPureWrite<u32>, // 0x8
    prim: ReadPureWrite<u32>, // 0xc
}

pub struct PLL {
    registers: UniqueMmioPointer<'static, PLLRegisters>,
    clear_reg: UniqueMmioPointer<'static, PLLRegisters>
}

impl PLL {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap())
            }
        }
    }

    pub fn reset_sys(&mut self) {
        let ref_div = 1 << cs_register::REF_DIV_SHIFT;
        let fbdiv = 133 << fbdiv_int_register::VALUE_SHIFT;
        let pd1 = 6 << prim_register::POSTDIV1_SHIFT;
        let pd2 = 2 << prim_register::POSTDIV2_SHIFT;
        let prim = pd1 | pd2;
        if field!(self.registers, cs).read() & cs_register::LOCK_MASK != 0 && 
            field!(self.registers, cs).read() & cs_register::REF_DIV_MASK == ref_div && 
            field!(self.registers, fbdiv_int).read() & fbdiv_int_register::VALUE_MASK == fbdiv && 
            field!(self.registers, prim).read() & (prim_register::POSTDIV1_MASK | prim_register::POSTDIV2_MASK) == prim {
                return;
        }
        field!(self.registers, cs).write(ref_div);
        field!(self.registers, fbdiv_int).write(fbdiv);
        field!(self.clear_reg, power).write(power_register::PD_MASK | power_register::VCOPD_MASK);
        while field!(self.registers, cs).read() & cs_register::LOCK_MASK == 0 {}
        field!(self.registers, prim).write(prim);
        field!(self.clear_reg, power).write(power_register::POSTDIVPD_MASK);
    }
    
    pub fn reset_usb(&mut self) {
        let ref_div = 1 << cs_register::REF_DIV_SHIFT;
        let fbdiv = 120 << fbdiv_int_register::VALUE_SHIFT;
        let pd1 = 6 << prim_register::POSTDIV1_SHIFT;
        let pd2 = 5 << prim_register::POSTDIV2_SHIFT;
        let prim = pd1 | pd2;
        if field!(self.registers, cs).read() & cs_register::LOCK_MASK != 0 && 
            field!(self.registers, cs).read() & cs_register::REF_DIV_MASK == ref_div && 
            field!(self.registers, fbdiv_int).read() & fbdiv_int_register::VALUE_MASK == fbdiv && 
            field!(self.registers, prim).read() & (prim_register::POSTDIV1_MASK | prim_register::POSTDIV2_MASK) == prim {
                return;
        }
        field!(self.registers, cs).write(ref_div);
        field!(self.registers, fbdiv_int).write(fbdiv);
        field!(self.clear_reg, power).write(power_register::PD_MASK | power_register::VCOPD_MASK);
        while field!(self.registers, cs).read() & cs_register::LOCK_MASK == 0 {}
        field!(self.registers, prim).write(prim);
        field!(self.clear_reg, power).write(power_register::POSTDIVPD_MASK);
    }
}

unsafe impl Send for PLL {}
unsafe impl Sync for PLL {}

static PLL_SYS_BASE: usize = 0x40028000;
static PLL_USB_BASE: usize = 0x4002c000;

pub static PLL_SYS: SpinIRQ<PLL> = unsafe {
    SpinIRQ::new(PLL::new(PLL_SYS_BASE))
};

pub static PLL_USB: SpinIRQ<PLL> = unsafe {
    SpinIRQ::new(PLL::new(PLL_USB_BASE))
};
