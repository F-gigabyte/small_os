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
    sys_handler_ctrl_state: ReadPureWrite<u32>,
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

    pub const VALID_MASK: u32 = REVISION_MASK |
        PARTNO_MASK |
        ARCHITECTURE_MASK |
        VARIANT_MASK |
        IMPLEMENTER_MASK;
}

mod inter_ctrl_state_register {
    pub const VEC_ACTIVE_SHIFT: usize = 0;
    pub const VEC_PENDING_SHIFT: usize = 12;
    pub const ISR_PENDING_SHIFT: usize = 22;
    pub const ISR_PREEMPT_SHIFT: usize = 23;
    pub const SYS_TICK_CLEAR_SHIFT: usize = 25;
    pub const SYS_TICK_SET_SHIFT: usize = 26;
    pub const PEND_SV_CLEAR_SHIFT: usize = 27;
    pub const PEND_SV_SET_SHIFT: usize = 28;

    pub const VEC_ACTIVE_MASK: u32 = 0x1ff << VEC_ACTIVE_SHIFT;
    pub const VEC_PENDING_MASK: u32 = 0x1ff << VEC_PENDING_SHIFT;
    pub const ISR_PENDING_MASK: u32 = 1 << ISR_PENDING_SHIFT;
    pub const ISR_PREEMPT_MASK: u32 = 1 << ISR_PREEMPT_SHIFT;
    pub const SYS_TICK_CLEAR_MASK: u32 = 1 << SYS_TICK_CLEAR_SHIFT;
    pub const SYS_TICK_SET_MASK: u32 = 1 << SYS_TICK_SET_SHIFT;
    pub const PEND_SV_CLEAR_MASK: u32 = 1 << PEND_SV_CLEAR_SHIFT;
    pub const PEND_SV_SET_MASK: u32 = 1 << PEND_SV_SET_SHIFT;

    pub const VALID_MASK: u32 = VEC_ACTIVE_MASK |
        VEC_PENDING_MASK |
        ISR_PENDING_MASK |
        ISR_PREEMPT_MASK |
        SYS_TICK_CLEAR_MASK |
        SYS_TICK_SET_MASK |
        PEND_SV_CLEAR_MASK |
        PEND_SV_SET_MASK;
}

mod sys_ctrl_register {
    pub const SLEEP_ON_EXIT_SHIFT: usize = 1;
    pub const SLEEP_DEEP_SHIFT: usize = 2;
    pub const SEND_DISABLE_IRQ_SHIFT: usize = 4;

    pub const SLEEP_ON_EXIT_MASK: u32 = 1 << SLEEP_ON_EXIT_SHIFT;
    pub const SLEEP_DEEP_MASK: u32 = 1 << SLEEP_DEEP_SHIFT;
    pub const SEND_DISABLE_IRQ_MASK: u32 = 1 << SEND_DISABLE_IRQ_SHIFT;

    pub const VALID_MASK: u32 = SLEEP_ON_EXIT_MASK |
        SLEEP_DEEP_MASK |
        SEND_DISABLE_IRQ_MASK;
}

mod vtor_register {
    pub const TABLE_OFFSET_SHIFT: usize = 8;
    
    pub const TABLE_OFFSET_MASK: u32 = 0xffffff << TABLE_OFFSET_SHIFT;

    pub const VALID_MASK: u32 = TABLE_OFFSET_MASK;
}

mod sys_handler_priority2_register {
    pub const SV_CALL_SHIFT: usize = 30;

    pub const SV_CALL_MASK: u32 = 0x3 << SV_CALL_SHIFT;

    pub const VALID_MASK: u32 = SV_CALL_MASK;
}

mod sys_handler_priority3_register {
    pub const SYS_TICK_SHIFT: usize = 30;
    pub const PEND_SV_SHIFT: usize = 22;

    pub const SYS_TICK_MASK: u32 = 0x3 << SYS_TICK_SHIFT;
    pub const PEND_SV_MASK: u32 = 0x3 << PEND_SV_SHIFT;

    pub const VALID_MASK: u32 = SYS_TICK_MASK |
        PEND_SV_MASK;
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
        let vtor = vtor & vtor_register::TABLE_OFFSET_MASK;
        field!(self.registers, vtor).modify(|vtor_reg| (vtor_reg & !vtor_register::VALID_MASK) | vtor);
    }

    #[inline(always)]
    pub fn set_svc_priorities(&mut self) {
        // set sys call and sys pend priroities to 1 (after IRQ and sys tick)
        field!(self.registers, sys_handler_priority2).modify(|sys_handler| (sys_handler & !sys_handler_priority2_register::VALID_MASK) | (1 << sys_handler_priority2_register::SV_CALL_SHIFT));
        field!(self.registers, sys_handler_priority3).modify(|sys_handler| (sys_handler & !sys_handler_priority3_register::VALID_MASK) | (1 << sys_handler_priority3_register::PEND_SV_SHIFT));
    }

    #[inline(always)]
    pub fn send_pend(&mut self) {
        field!(self.registers, inter_ctrl_state).write(inter_ctrl_state_register::PEND_SV_SET_MASK);
    }

    #[inline(always)]
    pub fn get_vtor(&mut self) -> u32 {
        field!(self.registers, vtor).read() & vtor_register::TABLE_OFFSET_MASK
    }
}

unsafe impl Send for System {}
unsafe impl Sync for System {}

static SYSTEM_BASE: usize =  0xe000ed00;

pub static SYSTEM: IRQMutex<System> = unsafe {
    IRQMutex::new(System::new(SYSTEM_BASE))
};

#[cfg(test)]
mod test {
    use crate::{inter::CS, print, println};

    use super::*;

    #[test_case]
    fn test_setup_correct() {
        println!("Testing system setup");
        let cs = unsafe {
            CS::new()
        };
        let mut system = SYSTEM.lock(&cs);
        print!("Testing vtor ");
        let vtor = field!(system.registers, vtor).read();
        assert_eq!(vtor & vtor_register::VALID_MASK, 0x10000000 + 256);
        println!("[ok]");
        print!("Testing SVC priority "); 
        let svc = field!(system.registers, sys_handler_priority2).read();
        assert_eq!(svc & sys_handler_priority2_register::VALID_MASK, 0);
        println!("[ok]");
        print!("Testing sys tick and Pend SV priority "); 
        let psv_timer = field!(system.registers, sys_handler_priority3).read();
        assert_eq!(psv_timer & sys_handler_priority3_register::VALID_MASK, 0);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing system register mask values");
        print!("Testing cpuid register ");
        assert_eq!(cpuid_register::VALID_MASK, u32::MAX);
        println!("[ok]");
        print!("Testing inter ctrl state register ");
        assert_eq!(inter_ctrl_state_register::VALID_MASK, 0x1edff1ff);
        println!("[ok]");
        print!("Testing vtor register ");
        assert_eq!(vtor_register::VALID_MASK, 0xffffff00);
        println!("[ok]");
        print!("Testing sys handler priority 2 register ");
        assert_eq!(sys_handler_priority2_register::VALID_MASK, 0xc0000000);
        println!("[ok]");
        print!("Testing sys handler priority 3 register ");
        assert_eq!(sys_handler_priority3_register::VALID_MASK, 0xc0c00000);
        println!("[ok]");
    }
}
