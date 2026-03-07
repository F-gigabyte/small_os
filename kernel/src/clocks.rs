use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, ReadWrite}};

use crate::{mmio::{REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS}, mutex::SpinIRQ, wait};

#[repr(C)]
struct ClockRegisters {
    gpout0_ctrl: ReadPureWrite<u32>,
    gpout0_div: ReadPureWrite<u32>,
    gpout0_selected: ReadPure<u32>,
    gpout1_ctrl: ReadPureWrite<u32>,
    gpout1_div: ReadPureWrite<u32>,
    gpout1_selected: ReadPure<u32>,
    gpout2_ctrl: ReadPureWrite<u32>,
    gpout2_div: ReadPureWrite<u32>,
    gpout2_selected: ReadPure<u32>,
    gpout3_ctrl: ReadPureWrite<u32>,
    gpout3_div: ReadPureWrite<u32>,
    gpout3_selected: ReadPure<u32>,
    ref_ctrl: ReadPureWrite<u32>,
    ref_div: ReadPureWrite<u32>,
    ref_selected: ReadPure<u32>,
    sys_ctrl: ReadPureWrite<u32>,
    sys_div: ReadPureWrite<u32>,
    sys_selected: ReadPure<u32>,
    peri_ctrl: ReadPureWrite<u32>,
    peri_selected: ReadPure<u32>,
    usb_ctrl: ReadPureWrite<u32>,
    usb_div: ReadPureWrite<u32>,
    usb_selected: ReadPure<u32>,
    adc_ctrl: ReadPureWrite<u32>,
    adc_div: ReadPureWrite<u32>,
    adc_selected: ReadPure<u32>,
    rtc_ctrl: ReadPureWrite<u32>,
    rtc_div: ReadPureWrite<u32>,
    rtc_selected: ReadPure<u32>,
    sys_resus_ctrl: ReadPureWrite<u32>,
    sys_resus_status: ReadWrite<u32>,
}

mod gpout_ctrl_register {
    pub const AUXSRC_SHIFT: usize = 5;
    pub const KILL_SHIFT: usize = 10;
    pub const ENABLE_SHIFT: usize = 11;
    pub const DC50_SHIFT: usize = 12;
    pub const PHASE_SHIFT: usize = 16;
    pub const NUDGE_SHIFT: usize = 20;

    pub const AUXSRC_MASK: u32 = 0xf << AUXSRC_SHIFT;
    pub const KILL_MASK: u32 = 1 << KILL_SHIFT;
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const DC50_MASK: u32 = 1 << DC50_SHIFT;
    pub const PHASE_MASK: u32 = 0x3 << PHASE_SHIFT;
    pub const NUDGE_MASK: u32 = 1 << NUDGE_SHIFT;

    pub const AUXSRC_PLL_SYS: u32 = 0x0 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN0: u32 = 0x1 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN1: u32 = 0x2 << AUXSRC_SHIFT;
    pub const AUXSRC_PLL_USB: u32 = 0x3 << AUXSRC_SHIFT;
    pub const AUXSRC_ROSC: u32 = 0x4 << AUXSRC_SHIFT;
    pub const AUXSRC_XOSC: u32 = 0x5 << AUXSRC_SHIFT;
    pub const AUXSRC_SYS: u32 = 0x6 << AUXSRC_SHIFT;
    pub const AUXSRC_USB: u32 = 0x7 << AUXSRC_SHIFT;
    pub const AUXSRC_ADC: u32 = 0x8 << AUXSRC_SHIFT;
    pub const AUXSRC_RTC: u32 = 0x9 << AUXSRC_SHIFT;
    pub const AUXSRC_REF: u32 = 0xa << AUXSRC_SHIFT;
}

mod div_register {
    pub const FRAC_SHIFT: usize = 0;
    pub const INT_SHIFT: usize = 8;

    pub const INT_MASK: u32 = 0xffffff << INT_SHIFT;
    pub const FRAC_MASK: u32 = 0xff << FRAC_SHIFT;
}

mod int_div_register {
    pub const INT_SHIFT: usize = 8;

    pub const INT_MASK: u32 = 0x3 << INT_SHIFT;
}

mod ref_ctrl_register {
    pub const SRC_SHIFT: usize = 0;
    pub const AUXSRC_SHIFT: usize = 5;

    pub const SRC_MASK: u32 = 0x3 << SRC_SHIFT;
    pub const AUXSRC_MASK: u32 = 0x3 << AUXSRC_SHIFT;

    pub const SRC_ROSC: u32 = 0x0 << SRC_SHIFT;
    pub const SRC_REF_AUX: u32 = 0x1 << SRC_SHIFT;
    pub const SRC_XOSC: u32 = 0x2 << SRC_SHIFT;

    pub const AUXSRC_PLL_USB: u32 = 0x0 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN0: u32 = 0x1 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN1: u32 = 0x2 << AUXSRC_SHIFT;
}

mod sys_ctrl_register {
    pub const SRC_SHIFT: usize = 0;
    pub const AUXSRC_SHIFT: usize = 5;

    pub const SRC_MASK: u32 = 1 << SRC_SHIFT;
    pub const AUXSRC_MASK: u32 = 0x3 << AUXSRC_SHIFT;

    pub const SRC_REF: u32 = 0 << SRC_SHIFT;
    pub const SRC_SYS_AUX: u32 = 1 << SRC_SHIFT;

    pub const AUXSRC_PLL_SYS: u32 = 0x0 << AUXSRC_SHIFT;
    pub const AUXSRC_PLL_USB: u32 = 0x1 << AUXSRC_SHIFT;
    pub const AUXSRC_ROSC: u32 = 0x2 << AUXSRC_SHIFT;
    pub const AUXSRC_XOSC: u32 = 0x3 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN0: u32 = 0x4 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN1: u32 = 0x5 << AUXSRC_SHIFT;
}

mod peri_ctrl_register {
    pub const AUXSRC_SHIFT: usize = 5;
    pub const KILL_SHIFT: usize = 10;
    pub const ENABLE_SHIFT: usize = 11;

    pub const AUXSRC_MASK: u32 = 0x3 << AUXSRC_SHIFT;
    pub const KILL_MASK: u32 = 1 << KILL_SHIFT;
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;

    pub const AUXSRC_SYS: u32 = 0x0 << AUXSRC_SHIFT;
    pub const AUXSRC_PLL_SYS: u32 = 0x1 << AUXSRC_SHIFT;
    pub const AUXSRC_PLL_USB: u32 = 0x2 << AUXSRC_SHIFT;
    pub const AUXSRC_ROSC: u32 = 0x3 << AUXSRC_SHIFT;
    pub const AUXSRC_XOSC: u32 = 0x4 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN0: u32 = 0x5 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN1: u32 = 0x6 << AUXSRC_SHIFT;
}

mod ctrl_register {
    pub const AUXSRC_SHIFT: usize = 5;
    pub const KILL_SHIFT: usize = 10;
    pub const ENABLE_SHIFT: usize = 11;
    pub const PHASE_SHIFT: usize = 16;
    pub const NUDGE_SHIFT: usize = 20;

    pub const AUXSRC_MASK: u32 = 0x3 << AUXSRC_SHIFT;
    pub const KILL_MASK: u32 = 1 << KILL_SHIFT;
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const PHASE_MASK: u32 = 0x3 << PHASE_SHIFT;
    pub const NUDGE_MASK: u32 = 1 << NUDGE_SHIFT;

    pub const AUXSRC_PLL_USB: u32 = 0x0 << AUXSRC_SHIFT;
    pub const AUXSRC_PLL_SYS: u32 = 0x1 << AUXSRC_SHIFT;
    pub const AUXSRC_ROSC: u32 = 0x2 << AUXSRC_SHIFT;
    pub const AUXSRC_XOSC: u32 = 0x3 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN0: u32 = 0x4 << AUXSRC_SHIFT;
    pub const AUXSRC_GPIN1: u32 = 0x5 << AUXSRC_SHIFT;
}

mod sys_resus_ctrl_register {
    pub const TIMEOUT_SHIFT: usize = 0;
    pub const ENABLE_SHIFT: usize = 8;
    pub const FORCE_SHIFT: usize = 12;
    pub const CLEAR_SHIFT: usize = 16;

    pub const TIMEOUT_MASK: u32 = 0xff << TIMEOUT_SHIFT;
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const FORCE_MASK: u32 = 1 << FORCE_SHIFT;
    pub const CLEAR_MASK: u32 = 1 << CLEAR_SHIFT;
}

mod sys_resus_status_register {
    pub const RESUSSED_SHIFT: usize = 0;

    pub const RESUSSED_MASK: u32 = 1 << RESUSSED_SHIFT;
}

pub enum PeriAuxSrc {
    Sys = 0,
    PllSys = 1,
    PllUSB = 2,
    ROSC = 3,
    XOSC = 4,
    GPIN0 = 5,
    GPIN1 = 6
}

pub struct Clocks {
    registers: UniqueMmioPointer<'static, ClockRegisters>,
    set_reg: UniqueMmioPointer<'static, ClockRegisters>,
    clear_reg: UniqueMmioPointer<'static, ClockRegisters>
}

impl Clocks {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap())
            }
        }
    }

    pub fn disble_sys_resus(&mut self) {
        field!(self.registers, sys_resus_ctrl).write(0);
    }

    fn clock_bitmap(src: u32, shift: usize) -> u32 {
        1 << (src >> shift)
    }

    pub fn preinit_sys_ref(&mut self) {
        // ref clock
        let mut ref_ctrl = field!(self.registers, ref_ctrl).read();
        ref_ctrl &= !ref_ctrl_register::SRC_MASK;
        field!(self.registers, ref_ctrl).write(ref_ctrl | ref_ctrl_register::SRC_XOSC);
        while field!(self.registers, ref_selected).read() & Self::clock_bitmap(ref_ctrl_register::SRC_XOSC, ref_ctrl_register::SRC_SHIFT) == 0 {} 
        
        // sys clock
        let mut sys_ctrl = field!(self.registers, sys_ctrl).read();
        sys_ctrl &= !sys_ctrl_register::SRC_MASK;
        field!(self.registers, sys_ctrl).write(sys_ctrl | sys_ctrl_register::SRC_REF);
        while field!(self.registers, sys_selected).read() & Self::clock_bitmap(sys_ctrl_register::SRC_REF, sys_ctrl_register::SRC_SHIFT) == 0 {} 
    }

    pub fn setup_clocks(&mut self) {
        // sys clock
        let mut sys_ctrl = field!(self.registers, sys_ctrl).read();
        sys_ctrl &= !sys_ctrl_register::SRC_MASK;
        field!(self.registers, sys_ctrl).write(sys_ctrl);
        while field!(self.registers, sys_selected).read() & sys_ctrl_register::SRC_MASK != 1 {}
        let mut sys_ctrl = field!(self.registers, sys_ctrl).read();
        sys_ctrl &= !sys_ctrl_register::AUXSRC_MASK;
        sys_ctrl |= sys_ctrl_register::AUXSRC_PLL_SYS;
        field!(self.registers, sys_ctrl).write(sys_ctrl);
        let mut sys_ctrl = field!(self.registers, sys_ctrl).read();
        sys_ctrl &= !sys_ctrl_register::SRC_MASK;
        sys_ctrl |= sys_ctrl_register::SRC_SYS_AUX;
        field!(self.registers, sys_ctrl).write(sys_ctrl);
        while field!(self.registers, sys_selected).read() & Self::clock_bitmap(sys_ctrl_register::SRC_SYS_AUX, sys_ctrl_register::SRC_SHIFT) == 0 {}
        // peripheral clock
        field!(self.clear_reg, peri_ctrl).write(peri_ctrl_register::ENABLE_MASK);
        wait::wait_cycles(33);
        field!(self.registers, peri_ctrl).write(peri_ctrl_register::AUXSRC_XOSC);
        field!(self.set_reg, peri_ctrl).write(peri_ctrl_register::ENABLE_MASK);
        wait::wait_cycles(33);
    }
}

unsafe impl Send for Clocks {}
unsafe impl Sync for Clocks {}

static CLOCKS_BASE: usize = 0x40008000;

pub static CLOCKS: SpinIRQ<Clocks> = unsafe {
    SpinIRQ::new(Clocks::new(CLOCKS_BASE))
};
