use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite}};

use crate::mutex::IRQMutex;

#[repr(C)]
struct SystemRegisters {
    cpuid: ReadPure<u32>,
    inter_ctrl_state: ReadPureWrite<u32>,
    vtor: ReadPureWrite<u32>,
    app_inter_reset_ctrl: ReadPureWrite<u32>,
    sys_ctrl: ReadPureWrite<u32>,
    config_ctrl: ReadPure<u32>,
    _reserved0: u32,
    sys_handler_priority2: ReadPureWrite<u32>,
    sys_handler_priority3: ReadPureWrite<u32>,
    sys_handler_ctrl_state: ReadPureWrite<u32>
}

mod cpuid_register {
    pub const REVISION_SHIFT: usize = 0;
    pub const PARTNO_SHIFT: usize = 4;
    pub const ARCHITECTURE_SHIFT: usize = 16;
    pub const VARIANT_SHIFT: usize = 20;
    pub const IMPLEMENTER_SHIFT: usize = 24;

    pub const REVISION_MASK: u32 = 0xf << REVISION_SHIFT;
    pub const PARTNO_MASK: u32 = 0xfff << PARTNO_SHIFT;
    pub const ARCHITECTURE_MASK: u32 = 0xf << ARCHITECTURE_SHIFT;
    pub const VARIANT_MASK: u32 = 0xf << VARIANT_SHIFT;
    pub const IMPLEMENTER_MASK: u32 = 0xff << IMPLEMENTER_SHIFT;

    pub const REVISION_PATCH1: u32 = 1 << REVISION_SHIFT;
    pub const PARTNO_CORTEX_M0_PLUS: u32 = 0xc60 << PARTNO_SHIFT;
    pub const ARCHITECTURE_ARMV6M: u32 = 0xc << ARCHITECTURE_SHIFT;
    pub const VARIANT_REVISION0: u32 = 0 << REVISION_SHIFT;
    pub const IMPLEMENTER_ARM: u32 = 0x41 << IMPLEMENTER_SHIFT;
}

mod inter_ctrl_state_register {
    pub const VEC_ACTIVE_SHIFT: usize = 0;
    pub const VEC_PENDING_SHIFT: usize = 12;
    pub const ISR_PENDING_SHIFT: usize = 22;
    pub const SYS_TICK_CLEAR_SHIFT: usize = 25;
    pub const SYS_TICK_SET_SHIFT: usize = 26;
    pub const PEND_SV_CLEAR_SHIFT: usize = 27;
    pub const PEND_SV_SET_SHIFT: usize = 28;

    pub const VEC_ACTIVE_MASK: u32 = 0xff << VEC_ACTIVE_SHIFT;
    pub const VEC_PENDING_MASK: u32 = 0xff << VEC_PENDING_SHIFT;
    pub const ISR_PENDING_MASK: u32 = 1 << ISR_PENDING_SHIFT;
    pub const SYS_TICK_CLEAR_MASK: u32 = 1 << SYS_TICK_CLEAR_SHIFT;
    pub const SYS_TICK_SET_MASK: u32 = 1 << SYS_TICK_SET_SHIFT;
    pub const PEND_SV_CLEAR_MASK: u32 = 1 << PEND_SV_CLEAR_SHIFT;
    pub const PEND_SV_SET_MASK: u32 = 1 << PEND_SV_SET_SHIFT;
}

mod sys_ctrl_register {
    pub const SLEEP_ON_EXIT_SHIFT: usize = 1;
    pub const SLEEP_DEEP_SHIFT: usize = 2;
    pub const SEND_DISABLE_IRQ_SHIFT: usize = 4;

    pub const SLEEP_ON_EXIT_MASK: u32 = 1 << SLEEP_ON_EXIT_SHIFT;
    pub const SLEEP_DEEP_MASK: u32 = 1 << SLEEP_DEEP_SHIFT;
    pub const SEND_DISABLE_IRQ_MASK: u32 = 1 << SEND_DISABLE_IRQ_SHIFT;
}

pub struct System {
    registers: UniqueMmioPointer<'static, SystemRegisters>
}

#[derive(Debug, Clone, Copy)]
pub enum SysIRQ {
    SysTick,
    PendSV,
}

impl System {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    #[inline(always)]
    pub fn clear_irq(&mut self, irq: SysIRQ) {
        match irq {
            SysIRQ::PendSV => field!(self.registers, inter_ctrl_state).write(inter_ctrl_state_register::PEND_SV_CLEAR_MASK),
            SysIRQ::SysTick => field!(self.registers, inter_ctrl_state).write(inter_ctrl_state_register::SYS_TICK_CLEAR_MASK),
        }
    }

    #[inline(always)]
    pub fn set_vtor(&mut self, vtor: u32) {
        let vtor = vtor & !0xff;
        field!(self.registers, vtor).write(vtor);
    }

    #[inline(always)]
    pub fn send_pend(&mut self) {
        field!(self.registers, inter_ctrl_state).write(inter_ctrl_state_register::PEND_SV_SET_MASK);
    }

    #[inline(always)]
    pub fn get_vtor(&mut self) -> u32 {
        field!(self.registers, vtor).read()
    }
}

unsafe impl Send for System {}
unsafe impl Sync for System {}

static SYSTEM_BASE: usize =  0xe000ed00;

pub static SYSTEM: IRQMutex<System> = unsafe {
    IRQMutex::new(System::new(SYSTEM_BASE))
};
