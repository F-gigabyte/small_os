use core::{arch::asm, marker::PhantomData, ptr};

use crate::{nvic::NVIC, println, proc::Proc, program::AccessAttr, scheduler::{QUANTUM_MICROS, get_current_proc, scheduler}, sys_tick::SYS_TICK, system::{SYSTEM, SysIRQ}, timer::{TIMER, TimerIRQ}};

// https://aticleworld.com/arm-function-call-stack-frame/ accessed 4/02/2026 mentions r0 to r3, r12
// and lr (r14) are caller saved so use r0 to r3 as arguments and r12 as sys call number so
// performing a sys call is like calling a function
// Inputs
// - r12 -> sys call number
// - r0 -> arg 1
// - r1 -> arg 2
// - r2 -> arg 3
// - r3 -> arg 4
// Outputs
// - r12 -> error code (0 for success)
// - r0 -> out 0
// - r1 -> out 1
// - r2 -> out 2
// - r3 -> out 3
#[derive(Debug)]
pub enum SysCall {
    // 0
    Yield {},
    // 1
    Exit {
        code: u32
    },
    // 2
    WaitIRQ {},
    // 3
    Send {
        endpoint: u32,
        tag: u32,
        // 0..16 -> send len, 16..32 -> receive len
        len: u32,
        data: *mut u8
    },
    // 4
    SendAsync {
        endpoint: u32,
        tag: u32,
        len: u32,
        data: *mut u8
    },
    // 5
    Header {
        queue: u32
    },
    // 6
    HeaderAsync {
        queue: u32
    },
    // 7
    HeaderNonBlocking {
        queue: u32,
    },
    // 8
    HeaderAsyncNonBlocking {
        queue: u32,
    },
    // 9
    Receive {
        queue: u32,
        len: u32,
        buffer: *mut u8
    },
    // 10
    ReceiveAsync {
        queue: u32,
        len: u32,
        buffer: *mut u8
    },
    // 11
    Reply {
        queue: u32,
        msg: u32,
        len: u32,
        buffer: *const u8
    },
}

pub enum IRQError {
    NoIRQ
}

impl From<IRQError> for u32 {
    fn from(value: IRQError) -> Self {
        match value {
            IRQError::NoIRQ => 0
        }
    }
}

impl TryFrom<&StackTrace> for SysCall {
    type Error = ();
    fn try_from(stack: &StackTrace) -> Result<Self, ()> {
        match stack.r12 {
            0 => Ok(SysCall::Yield {}),
            1 => Ok(SysCall::Exit { 
                code: stack.r0
            }),
            2 => Ok(SysCall::WaitIRQ {}),
            3 => Ok(SysCall::Send { 
                endpoint: stack.r0,
                tag: stack.r1,
                len: stack.r2,
                data: ptr::with_exposed_provenance_mut(stack.r3 as usize)
            }),
            4 => Ok(SysCall::SendAsync { 
                endpoint: stack.r0,
                tag: stack.r1,
                len: stack.r2,
                data: ptr::with_exposed_provenance_mut(stack.r3 as usize)
            }),
            5 => Ok(SysCall::Header { 
                queue: stack.r0,
            }),
            6 => Ok(SysCall::HeaderAsync { 
                queue: stack.r0 
            }),
            7 => Ok(SysCall::HeaderNonBlocking { 
                queue: stack.r0, 
            }),
            8 => Ok(SysCall::HeaderAsyncNonBlocking {
                queue: stack.r0,
            }),
            9 => Ok(SysCall::Receive { 
                queue: stack.r0, 
                len: stack.r1, 
                buffer: ptr::with_exposed_provenance_mut(stack.r2 as usize) 
            }),
            10 => Ok(SysCall::ReceiveAsync { 
                queue: stack.r0, 
                len: stack.r1, 
                buffer: ptr::with_exposed_provenance_mut(stack.r2 as usize) 
            }),
            11 => Ok(SysCall::Reply { 
                queue: stack.r0, 
                msg: stack.r1,
                len: stack.r2,
                buffer: ptr::with_exposed_provenance(stack.r3 as usize)
            }),
            _ => Err(())
        }
    }
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

const PROC_MASK: u32 = 0x8;

#[unsafe(no_mangle)]
pub extern "C" fn nmi(trace: *mut StackTrace, lr: u32) -> *mut Proc {
    if lr & PROC_MASK != 0 {
        // recoverable non-maskable interrupt in application
        let cs = unsafe {
            CS::new()
        };
        // reset current process and hope it doesn't die again
        let mut scheduler = scheduler(&cs);
        scheduler.reset_current();
        scheduler.next_process();
        let mut sys_tick = SYS_TICK.lock(&cs);
        sys_tick.reload();
        scheduler.get_current()
    } else {
        let trace = unsafe {
            &*trace
        };
        println!("Non Maskable Interrupt!");
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
}

#[unsafe(no_mangle)]
pub extern "C" fn hard_fault(trace: *mut StackTrace, lr: u32) -> *mut Proc {
    if lr & PROC_MASK != 0 {
        println!("Recoverable hard fault");
        // recoverable hard fault in application
        let cs = unsafe {
            CS::new()
        };
        // reset current process and hope it doesn't die again
        let mut scheduler = scheduler(&cs);
        scheduler.reset_current();
        scheduler.next_process();
        let mut sys_tick = SYS_TICK.lock(&cs);
        sys_tick.reload();
        scheduler.get_current()
    } else {
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
}

/// IRQ handler
/// `irq` is the IRQ to wake up all sleeping processes on
/// Interrupts are handled in a similar way to QNX with processes
/// registering to them and waiting for them to happen
/// https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.lib_ref/topic/i/interruptattachevent.html accessed 1/02/2026
#[unsafe(no_mangle)]
pub extern "C" fn irq_handler(irq: u8) {
    let cs = unsafe {
        CS::new()
    };
    let mut nvic = NVIC.lock(&cs);
    nvic.clear_pending_irq(irq);
    let mut scheduler = scheduler(&cs);
    scheduler.wake(irq);
}

fn do_sys_call(sys_call: SysCall, cs: &CS) -> Result<(), u32> {
    match sys_call {
        // kernel print
        SysCall::Yield {} => {
            let mut scheduler = scheduler(cs);
            scheduler.yield_current();
            Ok(())
        },
        // exit
        SysCall::Exit { 
            code: _ 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.terminate_current();
            Ok(())
        },
        // wait for IRQ
        SysCall::WaitIRQ {} => {
            let mut scheduler = scheduler(cs);
            let current = scheduler.get_current();
            let program = unsafe {
                & *(*current).program
            };

            if program.has_interrupt() {
                scheduler.sleep_irq();
                let mut nvic = NVIC.lock(&cs);
                // enable the IRQs for firing
                for inter in program.interrupts() {
                    if *inter < 32 {
                        nvic.enable_irq(*inter);
                    }
                }
                Ok(())
            } else {
                Err(u32::from(IRQError::NoIRQ))
            }
        },
        // send message
        SysCall::Send { 
            endpoint,  
            tag,
            len,
            data
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.send(endpoint, tag, len, data).map_err(|err| u32::from(err))
        },
        SysCall::SendAsync { 
            endpoint, 
            tag, 
            len, 
            data 
        } => {
            let mut scheduler = scheduler(cs);
            unsafe {
                scheduler.send_async(endpoint, tag, len, data).map_err(|err| u32::from(err))
            }
        },
        SysCall::Header { 
            queue 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.header(queue, true).map_err(|err| u32::from(err))
        },
        SysCall::HeaderAsync { 
            queue 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.header_async(queue, true).map_err(|err| u32::from(err))
        },
        SysCall::HeaderNonBlocking { 
            queue 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.header(queue, false).map_err(|err| u32::from(err))
        },
        SysCall::HeaderAsyncNonBlocking { 
            queue 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.header_async(queue, false).map_err(|err| u32::from(err))
        },
        SysCall::Receive { 
            queue, 
            len, 
            buffer 
        } => {
            let mut scheduler = scheduler(cs);
            unsafe {
                scheduler.receive_message(queue, len, buffer).map_err(|err| u32::from(err))
            }
        },
        SysCall::ReceiveAsync { 
            queue, 
            len, 
            buffer 
        } => {
            let mut scheduler = scheduler(cs);
            unsafe {
                scheduler.receive_message_async(queue, len, buffer).map_err(|err| u32::from(err))
            }
        },
        SysCall::Reply { 
            queue, 
            msg,
            len,
            buffer 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.reply(queue, msg, len, buffer).map_err(|err| u32::from(err))
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn sys_call(stack: *mut StackTrace) -> *mut Proc {
    let stack = unsafe {
        &mut *stack
    };
    let cs = unsafe {
        CS::new()
    };
    let prev = {
        let mut scheduler = scheduler(&cs);
        scheduler.get_current()
    };
    match SysCall::try_from(&*stack) {
        Ok(sys_call) => {
            match do_sys_call(sys_call, &cs) {
                Ok(_) => {
                    // success
                    let prev = unsafe {
                        &mut *prev
                    };
                    _ = prev.set_r12(0);
                },
                Err(err) => {
                    // failure
                    let prev = unsafe {
                        &mut *prev
                    };
                    _ = prev.set_r12(err + 2);
                }
            }
        },
        Err(_) => {
            // unknown sys call
            let prev = unsafe {
                &mut *prev
            };
            _ = prev.set_r12(1);
        }
    }
    let mut scheduler = scheduler(&cs);

    // only switch out if current is blocked or its time slice expired
    scheduler.next_current_process();
    let current = scheduler.get_current();
    // only reset time quantum if current process has switched
    if prev != current {
        let mut sys_tick = SYS_TICK.lock(&cs);
        sys_tick.reload();
    }
    current
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

/// performs a process switch and gets the next process
/// SAFETY
/// must be called without interrupts
#[unsafe(no_mangle)]
pub unsafe extern "C" fn do_proc_switch() -> *mut Proc {
    let cs = unsafe {
        CS::new()
    };
    proc_switch(cs)
}

fn proc_switch(cs: CS) -> *mut Proc {
    let current;
    {
        let mut scheduler = scheduler(&cs);
        scheduler.yield_current();
        scheduler.next_process();
        current = unsafe {
            get_current_proc()
        };
    }
    current
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn sys_tick() -> *mut Proc {
    // SAFETY
    // In interrupt handler so interrupts are disabled
    let cs = unsafe {
        CS::new()
    };
    {
        let mut sys_tick = SYS_TICK.lock(&cs);
        sys_tick.clear_count_flag();
        sys_tick.reload();
        let mut sys = SYSTEM.lock(&cs);
        sys.clear_irq(SysIRQ::SysTick);
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
