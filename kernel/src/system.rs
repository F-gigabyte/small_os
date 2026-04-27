/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Kernel.
 *
 * The SmallOS Kernel is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Kernel is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS Kernel. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite}};

use crate::mutex::IRQMutex;

/// System memory mapped registers
#[repr(C)]
struct SystemRegisters {
    /// CPUID register (0xed00)
    cpuid: ReadPure<u32>, // 0xed00
    /// Interrupt control state register (0xed04)
    inter_ctrl_state: ReadPureWrite<u32>, // 0xed04
    /// Vector table offset register (0xed08)
    vtor: ReadPureWrite<u32>, // 0xed08
    /// Application interrupt and reset control register (0xed0c)
    app_inter_reset_ctrl: ReadPureWrite<u32>, // 0xed0c
    /// System control register (0xed10)
    sys_ctrl: ReadPureWrite<u32>, // 0xed10
    /// Configuration and control register (0xed14)
    config_ctrl: ReadPure<u32>, // 0xed14
    _reserved0: u32, // 0xed18
    /// System handler priority register 2 (0xed1c)
    sys_handler_priority2: ReadPureWrite<u32>, // 0xed1c
    /// System handler priority register 3 (0xed20)
    sys_handler_priority3: ReadPureWrite<u32>, // 0xed20
    /// System handler control and state register (0xed24)
    sys_handler_ctrl_state: ReadPureWrite<u32>, // 0xed24
}

/// System CPUID register masks and shifts
mod cpuid_register {
    /// Revision shift
    pub const REVISION_SHIFT: usize = 0;
    /// Part number shift
    pub const PARTNO_SHIFT: usize = 4;
    /// Architecture shift
    pub const ARCHITECTURE_SHIFT: usize = 16;
    /// Variant shift
    pub const VARIANT_SHIFT: usize = 20;
    /// Implementer shift
    pub const IMPLEMENTER_SHIFT: usize = 24;

    /// Revision shift
    pub const REVISION_MASK: u32 = 0xf << REVISION_SHIFT;
    /// Part number shift
    pub const PARTNO_MASK: u32 = 0xfff << PARTNO_SHIFT;
    /// Architecture shift
    pub const ARCHITECTURE_MASK: u32 = 0xf << ARCHITECTURE_SHIFT;
    /// Variant shift
    pub const VARIANT_MASK: u32 = 0xf << VARIANT_SHIFT;
    /// Implementer shift
    pub const IMPLEMENTER_MASK: u32 = 0xff << IMPLEMENTER_SHIFT;

    /// Patch 1
    pub const REVISION_PATCH1: u32 = 1 << REVISION_SHIFT;
    /// Cortex M0+
    pub const PARTNO_CORTEX_M0_PLUS: u32 = 0xc60 << PARTNO_SHIFT;
    /// ARMv6-M
    pub const ARCHITECTURE_ARMV6M: u32 = 0xc << ARCHITECTURE_SHIFT;
    /// Revision 0
    pub const VARIANT_REVISION0: u32 = 0 << REVISION_SHIFT;
    /// ARM
    pub const IMPLEMENTER_ARM: u32 = 0x41 << IMPLEMENTER_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = REVISION_MASK |
        PARTNO_MASK |
        ARCHITECTURE_MASK |
        VARIANT_MASK |
        IMPLEMENTER_MASK;
}

/// System interrupt control and state register masks and shifts
mod inter_ctrl_state_register {
    /// Active exception shift
    pub const VEC_ACTIVE_SHIFT: usize = 0;
    /// Pending exception shift
    pub const VEC_PENDING_SHIFT: usize = 12;
    /// External interrupt pending shift
    pub const ISR_PENDING_SHIFT: usize = 22;
    /// Pending interrupt will be taken in the next running cycle shift
    pub const ISR_PREEMPT_SHIFT: usize = 23;
    /// Clear pending state from system tick shift
    pub const SYS_TICK_CLEAR_SHIFT: usize = 25;
    /// Set pending state on system tick shift
    pub const SYS_TICK_SET_SHIFT: usize = 26;
    /// Clear pending state from Pend SV shift
    pub const PEND_SV_CLEAR_SHIFT: usize = 27;
    /// Set pending state on Pend SV shift
    pub const PEND_SV_SET_SHIFT: usize = 28;

    /// Active exception mask
    pub const VEC_ACTIVE_MASK: u32 = 0x1ff << VEC_ACTIVE_SHIFT;
    /// Pending exception mask
    pub const VEC_PENDING_MASK: u32 = 0x1ff << VEC_PENDING_SHIFT;
    /// External interrupt pending mask
    pub const ISR_PENDING_MASK: u32 = 1 << ISR_PENDING_SHIFT;
    /// Pending interrupt will be taken in the next running cycle mask
    pub const ISR_PREEMPT_MASK: u32 = 1 << ISR_PREEMPT_SHIFT;
    /// Clear pending state from system tick mask
    pub const SYS_TICK_CLEAR_MASK: u32 = 1 << SYS_TICK_CLEAR_SHIFT;
    /// Set pending state on system tick mask
    pub const SYS_TICK_SET_MASK: u32 = 1 << SYS_TICK_SET_SHIFT;
    /// Clear pending state from Pend SV mask
    pub const PEND_SV_CLEAR_MASK: u32 = 1 << PEND_SV_CLEAR_SHIFT;
    /// Set pending state on Pend SV mask
    pub const PEND_SV_SET_MASK: u32 = 1 << PEND_SV_SET_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = VEC_ACTIVE_MASK |
        VEC_PENDING_MASK |
        ISR_PENDING_MASK |
        ISR_PREEMPT_MASK |
        SYS_TICK_CLEAR_MASK |
        SYS_TICK_SET_MASK |
        PEND_SV_CLEAR_MASK |
        PEND_SV_SET_MASK;
}

/// System system control register shifts and masks
mod sys_ctrl_register {
    /// Shift for sleeping on exit
    pub const SLEEP_ON_EXIT_SHIFT: usize = 1;
    /// Shift for deep sleep on exit
    pub const SLEEP_DEEP_SHIFT: usize = 2;
    /// Shift for all interrupts and events able to wake the processor from sleep as opposed to only
    /// those activated
    pub const SEND_DISABLE_IRQ_SHIFT: usize = 4;

    /// Mask for sleeping on exit
    pub const SLEEP_ON_EXIT_MASK: u32 = 1 << SLEEP_ON_EXIT_SHIFT;
    /// Mask for deep sleep on exit
    pub const SLEEP_DEEP_MASK: u32 = 1 << SLEEP_DEEP_SHIFT;
    /// Mask for all interrupts and events able to wake the processor from sleep as opposed to only
    /// those activated
    pub const SEND_DISABLE_IRQ_MASK: u32 = 1 << SEND_DISABLE_IRQ_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SLEEP_ON_EXIT_MASK |
        SLEEP_DEEP_MASK |
        SEND_DISABLE_IRQ_MASK;
}

/// System vector table offset register shifts and masks
mod vtor_register {
    /// Table offset shift
    pub const TABLE_OFFSET_SHIFT: usize = 8;
    
    /// Table offset mask
    pub const TABLE_OFFSET_MASK: u32 = 0xffffff << TABLE_OFFSET_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = TABLE_OFFSET_MASK;
}

/// System system handler priority 2 register masks and shifts
mod sys_handler_priority2_register {
    /// SV Call shift
    pub const SV_CALL_SHIFT: usize = 30;

    /// SV Call mask
    pub const SV_CALL_MASK: u32 = 0x3 << SV_CALL_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SV_CALL_MASK;
}

/// System system handler priority 3 register masks and shifts
mod sys_handler_priority3_register {
    /// System Tick shift
    pub const SYS_TICK_SHIFT: usize = 30;
    /// Pend SV shift
    pub const PEND_SV_SHIFT: usize = 22;

    /// System Tick mask
    pub const SYS_TICK_MASK: u32 = 0x3 << SYS_TICK_SHIFT;
    /// Pend SV mask
    pub const PEND_SV_MASK: u32 = 0x3 << PEND_SV_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SYS_TICK_MASK |
        PEND_SV_MASK;
}

/// System object for managing the system
pub struct System {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, SystemRegisters>
}

/// System IRQs
#[derive(Debug, Clone, Copy)]
pub enum SysIRQ {
    /// System Tick IRQ
    SysTick,
    /// Pend SV IRQ
    PendSV,
}

impl System {
    /// Creates a new `System` object  
    /// `base` is the base address of the System memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the System memory mapped registers and not
    /// being used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    /// Clears `irq` IRQ  
    /// `irq` is the IRQ to clear
    #[inline(always)]
    pub fn clear_irq(&mut self, irq: SysIRQ) {
        match irq {
            SysIRQ::PendSV => field!(self.registers, inter_ctrl_state).write(inter_ctrl_state_register::PEND_SV_CLEAR_MASK),
            SysIRQ::SysTick => field!(self.registers, inter_ctrl_state).write(inter_ctrl_state_register::SYS_TICK_CLEAR_MASK),
        }
    }

    /// Sets the vector table offset  
    /// `vtor` is the base to set
    #[inline(always)]
    pub fn set_vtor(&mut self, vtor: u32) {
        let vtor = vtor & vtor_register::TABLE_OFFSET_MASK;
        field!(self.registers, vtor).modify(|vtor_reg| (vtor_reg & !vtor_register::VALID_MASK) | vtor);
    }

    /// Sets SV calls and Pend SVs priorities to after IRQs and system ticks
    #[inline(always)]
    pub fn set_svc_priorities(&mut self) {
        // set sys call and sys pend priroities to 1 (after IRQ and sys tick)
        field!(self.registers, sys_handler_priority2).modify(|sys_handler| (sys_handler & !sys_handler_priority2_register::VALID_MASK) | (1 << sys_handler_priority2_register::SV_CALL_SHIFT));
        field!(self.registers, sys_handler_priority3).modify(|sys_handler| (sys_handler & !sys_handler_priority3_register::VALID_MASK) | (1 << sys_handler_priority3_register::PEND_SV_SHIFT));
    }

    /// Sends a Pend SV interrupt
    #[inline(always)]
    pub fn send_pend(&mut self) {
        field!(self.registers, inter_ctrl_state).write(inter_ctrl_state_register::PEND_SV_SET_MASK);
    }

    /// Gets the vector table offset
    #[inline(always)]
    pub fn get_vtor(&mut self) -> u32 {
        field!(self.registers, vtor).read() & vtor_register::TABLE_OFFSET_MASK
    }
}

unsafe impl Send for System {}
unsafe impl Sync for System {}

/// Base address for the System memory mapped registers
static SYSTEM_BASE: usize =  0xe000ed00;

/// System object
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
