use core::{arch::asm, marker::PhantomData};

use crate::{println, timer::{TIMER, Timer, TimerIRQ}};

#[repr(C)]
pub struct StackTrace {
    xpsr: u32,
    ret_addr: u32,
    lr: u32,
    r12: u32,
    r3: u32,
    r2: u32,
    r1: u32,
    r0: u32
}

#[unsafe(no_mangle)]
pub extern "C" fn nmi() -> ! {
    println!("NMI");
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn hard_fault(trace: *const StackTrace) -> ! {
    let trace = unsafe {
        &*trace
    };
    println!("Hard Fault!");
    println!("Registers");
    println!("\tr0: {:x}", trace.r0);
    println!("\tr1: {:x}", trace.r1);
    println!("\tr2: {:x}", trace.r2);
    println!("\tr3: {:x}", trace.r3);
    println!("\tr12: {:x}", trace.r12);
    println!("\tlr: {:x}", trace.lr);
    println!("\tret_addr: {:x}", trace.ret_addr);
    println!("\txpsr: {:x}", trace.xpsr);
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn unimplemented_irq(trace: *const StackTrace) -> ! {
    let trace = unsafe {
        &*trace
    };
    println!("Unimplemented IRQ!");
    println!("Registers");
    println!("\tr0: {:x}", trace.r0);
    println!("\tr1: {:x}", trace.r1);
    println!("\tr2: {:x}", trace.r2);
    println!("\tr3: {:x}", trace.r3);
    println!("\tr12: {:x}", trace.r12);
    println!("\tlr: {:x}", trace.lr);
    println!("\tret_addr: {:x}", trace.ret_addr);
    println!("\txpsr: {:x}", trace.xpsr);
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn sys_call() -> ! {
    println!("Sys Call!");
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn sys_pend() -> ! {
    println!("Sys Pending!");
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn sys_tick() -> ! {
    println!("Sys Tick!");
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn timer_tick() {
    // SAFETY
    // In interrupt handler so interrupts are disabled
    let cs = unsafe {
        CS::new()
    };
    let mut timer = TIMER.lock(&cs);
    timer.handle_irq(TimerIRQ::Timer0);
}

// critical section
pub struct CS {
    _phantom: PhantomData<()>
}

impl CS {
    pub unsafe fn new() -> Self {
        Self {
            _phantom: PhantomData
        }
    }
}

pub unsafe fn disable_irq() {
    unsafe {
        asm!("cpsid i");
    }
}

pub unsafe fn enable_irq() {
    unsafe {
        asm!("cpsie i");
    }
}

#[inline(always)]
pub fn irq_enabled() -> bool {
    let mut r0: u32;
    unsafe {
        asm!("mrs {r0}, PRIMASK", r0 = out(reg) r0);
    }
    r0 & 1 == 0
}

pub fn without_inter<F: FnOnce(&CS)>(f: F) {
    let irq = irq_enabled();
    if irq {
        unsafe {
            disable_irq();
        }
    }
    // Explicity state scope so cs doesn't go beyond interrupt free boundaries
    {
        let cs = unsafe {
            CS::new()
        };
        f(&cs);
    }
    if irq {
        unsafe {
            enable_irq();
        }
    }
    // SAFETY
    // the state of IRQ before this function is the same as afterwards
}
