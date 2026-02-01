use core::{arch::asm, marker::PhantomData, ptr};

use crate::{nvic::NVIC, println, proc::Proc, scheduler::{QUANTUM_MICROS, get_current_proc, scheduler}, sys_tick::SYS_TICK, system::{SYSTEM, SysIRQ}, timer::{TIMER, TimerIRQ}};

unsafe extern "C" {
    fn schedule(proc: *mut Proc);
}

#[repr(C)]
pub struct StackTrace {
    r0: u32,
    r1: u32,
    r2: u32,
    r3: u32,
    r12: u32,
    lr: u32,
    ret_addr: u32,
    xpsr: u32,
}

#[repr(C)]
pub struct UpperStackTrace {
    r4: u32,
    r5: u32,
    r6: u32,
    r7: u32,
    r8: u32,
    r9: u32,
    r10: u32,
    r11: u32
}

#[unsafe(no_mangle)]
pub extern "C" fn nmi(trace: *mut StackTrace) -> ! {
    let trace = unsafe {
        &*trace
    };
    println!("NMI!");
    println!("Registers");
    println!("\tr0: {:x}", trace.r0);
    println!("\tr1: {:x}", trace.r1);
    println!("\tr2: {:x}", trace.r2);
    println!("\tr3: {:x}", trace.r3);
    println!("\tr12: {:x}", trace.r12);
    println!("\tlr: {:x}", trace.lr);
    println!("\tret_addr: {:x}", trace.ret_addr);
    println!("\txpsr: {:x}", trace.xpsr);
    panic!();
}

#[unsafe(no_mangle)]
pub extern "C" fn hard_fault(trace: *mut StackTrace) -> ! {
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
    panic!();
}

#[unsafe(no_mangle)]
pub extern "C" fn irq_handler(irq: u8, mode: u8) {
    let cs = unsafe {
        CS::new()
    };
    let mut nvic = NVIC.lock(&cs);
    nvic.clear_pending_irq(irq);
}

#[unsafe(no_mangle)]
pub extern "C" fn sys_call(service: u32, stack: *mut StackTrace) -> *mut Proc {
    let stack = unsafe {
        &mut *stack
    };
    let cs = unsafe {
        CS::new()
    };
    if service == 0 {
        let len = stack.r1 as usize;
        let text: &[u8] = unsafe {
            & *ptr::slice_from_raw_parts(ptr::with_exposed_provenance(stack.r0 as usize), len)
        };
        if let Ok(text) = str::from_utf8(text) {
            println!("{}", text);
            stack.r0 = 0;
        } else {
            stack.r0 = 1;
        }
    } else {
        stack.r0 = 2;
    }
    unsafe {
        get_current_proc()
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn pend_sv() -> *mut Proc {
    let cs = unsafe {
        CS::new()
    };
    {
        let mut sys = SYSTEM.lock(&cs);
        sys.clear_irq(SysIRQ::PendSV);
    }
    proc_switch(cs)
}

#[unsafe(no_mangle)]
pub extern "C" fn sys_tick() {
    let cs = unsafe {
        CS::new()
    };
    // Sys Tick shouldn't be enabled
    let mut sys_tick = SYS_TICK.lock(&cs);
    sys_tick.disable();
    let mut sys = SYSTEM.lock(&cs);
    sys.clear_irq(SysIRQ::SysTick);
}

fn proc_switch(cs: CS) -> *mut Proc {
    let current;
    {
        let mut timer = TIMER.lock(&cs);
        let mut scheduler = scheduler(&cs);
        scheduler.next_process();
        current = unsafe {
            get_current_proc()
        };
        timer.set_count(TimerIRQ::Timer0, QUANTUM_MICROS);
    }
    drop(cs);
    current
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn timer0() -> *mut Proc {
    // SAFETY
    // In interrupt handler so interrupts are disabled
    let cs = unsafe {
        CS::new()
    };
    {
        let mut timer = TIMER.lock(&cs);
        timer.clear_irq(TimerIRQ::Timer0);
    }
    proc_switch(cs)
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
