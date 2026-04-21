/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS kernel.
 *
 * The SmallOS kernel is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS kernel is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS kernel. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

use core::{arch::asm, marker::PhantomData, ptr, slice};

use crate::{message_queue::QueueError, print, println, proc::Proc, program::AccessAttr, scheduler::{QUANTUM_MICROS, get_current_proc, scheduler}, sys_tick::SYS_TICK, system::{SYSTEM, SysIRQ}};

/// SmallOS System Calls  
/// <https://aticleworld.com/arm-function-call-stack-frame/> accessed 4/02/2026 mentions r0 to r3, r12
/// and lr (r14) are caller saved so use r0 to r3 as arguments and r12 as sys call number so
/// performing a sys call is like calling a function  
/// Inputs  
/// - `r12` -> system call number
/// - `r0` -> arg 1
/// - `r1` -> arg 2
/// - `r2` -> arg 3
/// - `r3` -> arg 4  
/// Outputs  
/// - `r12` -> error code (0 for success)
/// - `r0` -> out 0
/// - `r1` -> out 1
/// - `r2` -> out 2
/// - `r3` -> out 3  
/// Not all inputs or outputs may be used, this depends on the system call
#[derive(Debug)]
pub enum SysCall {
    /// Yield (0)
    Yield,
    /// Exit (1)
    Exit {
        /// Exit code
        code: u32
    },
    /// Wait for an IRQ (2)
    WaitIRQ,
    /// Clear IRQ flags (3)
    ClearIRQ,
    /// Send a message to a synchronous queue (4)
    Send {
        /// Endpoint to use
        endpoint: u32,
        /// Message tag
        tag: u32,
        /// Message length
        /// - [0..15] -> send length
        /// - [16..31] -> reply length
        len: u32,
        /// Pointer to message data
        data: *mut u8
    },
    /// Send a message to an asynchronous queue (5)
    SendAsync {
        /// Endpoint to use
        endpoint: u32,
        /// Message tag
        tag: u32,
        /// Message length
        /// - [0..15] -> send length
        /// - [16..31] -> reply length
        len: u32,
        /// Pointer to message data
        data: *mut u8
    },
    /// Send a message from a synchronous queue to a notifier (6)
    NotifySend {
        /// Queue to use
        queue: u32,
        /// Notifier to use
        notifier: u32
    },
    /// Wait for a synchronous queue to contain data (7)
    WaitQueues {
        /// Mask of queues to wait on. A bit set at index n indicates wait on queue n
        queue_mask: u32
    },
    /// Wait for an asynchronous queue to contain data (8)
    WaitQueuesAsync {
        /// Mask of queues to wait on. A bit set at index n indicates wait on queue n
        queue_mask: u32
    },
    /// Wait for a synchronous queue to contain data or an interrupt to happen (9)
    WaitQueuesIRQ {
        /// Mask of queues to wait on. A bit set at index n indicates wait on queue n
        queue_mask: u32
    },
    /// Wait for an asynchronous queue to contain data or an interrupt to happen (10)
    WaitQueuesIRQAsync {
        /// Mask of queues to wait on. A bit set at index n indicates wait on queue n
        queue_mask: u32
    },
    /// Read the header from the first process in the synchronous queue  
    /// Blocks if the queue is empty (11)
    Header {
        /// Queue to use
        queue: u32
    },
    /// Read the header from the first message in the asynchronous queue  
    /// Blocks if the queue is empty (12)
    HeaderAsync {
        /// Queue to use
        queue: u32
    },
    /// Read the header from the first process in the synchronous queue  
    /// Returns an error if the queue is empty (13)
    HeaderNonBlocking {
        /// Queue to use
        queue: u32,
    },
    /// Read the header from the first message in the asynchronous queue  
    /// Returns an error if the queue is empty (14)
    HeaderAsyncNonBlocking {
        /// Queue to use
        queue: u32,
    },
    /// Read the header from the first process in the notifier queue  
    /// Returns an error if the queue is empty (15)
    NotifyHeader {
        /// Notifier to use
        notifier: u32
    },
    /// Reads the message data from the first processor in the synchronous queue and stores it in
    /// the provided buffer (16)
    Receive {
        /// Queue to use
        queue: u32,
        /// Buffer length
        len: u32,
        /// Buffer address
        buffer: *mut u8
    },
    /// Reads the message data from the first message in the asynchronous queue and stores it in
    /// the provided buffer (17)
    ReceiveAsync {
        /// Queue to use
        queue: u32,
        /// Buffer length
        len: u32,
        /// Buffer address
        buffer: *mut u8
    },
    /// Reads the message data from the first processor in the notifier queue and stores it in
    /// the provided buffer (18)
    NotifyReceive {
        /// Notifier to use
        notifier: u32,
        /// Buffer length
        len: u32,
        /// Buffer address
        buffer: *mut u8
    },
    /// Sends a reply to the first processor in the synchronous queue (19)
    Reply {
        /// Queue to use
        queue: u32,
        /// Message tag
        msg: u32,
        /// Message buffer length
        len: u32,
        /// Message buffer address
        buffer: *const u8
    },
    /// Sends a reply to the first processor in the notifier queue (20)
    NotifyReply {
        /// Notifier to use
        notifier: u32,
        /// Message tag
        msg: u32,
        /// Message buffer length
        len: u32,
        /// Message buffer address
        buffer: *const u8
    },
    /// Gets the kernel to print out a message (21)
    KPrint {
        /// Message length
        len: u32,
        /// Message buffer address of UTF8 characters
        buffer: *const u8
    },
}

/// Error returned from IRQ related system calls
#[derive(Debug)]
pub enum IRQError {
    /// Process doesn't have any registered IRQs
    NoIRQ
}

/// Error returned from IRQ and Queue related system calls
#[derive(Debug)]
pub enum QueueIRQError {
    /// An `IRQError` was returned
    IRQError(IRQError),
    /// A `QueueError` was returned
    QueueError(QueueError)
}

/// Converts an `IRQError` into a `u32` so it can be returned to the calling process
impl From<IRQError> for u32 {
    fn from(value: IRQError) -> Self {
        match value {
            IRQError::NoIRQ => 0
        }
    }
}

/// Converts an `QueueIRQError` into a `u32` so it can be returned to the calling process
impl From<QueueIRQError> for u32 {
    fn from(value: QueueIRQError) -> Self {
        match value {
            QueueIRQError::IRQError(irq) => u32::from(irq),
            QueueIRQError::QueueError(queue) => 1 + u32::from(queue)
        }
    }
}

/// Converts a `QueueError` into a `QueueIRQError`
impl From<QueueError> for QueueIRQError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// Converts a `IRQError` into a `QueueIRQError`
impl From<IRQError> for QueueIRQError {
    fn from(value: IRQError) -> Self {
        Self::IRQError(value)
    }
}

/// Error returned from the `KPrint` system call
pub enum KPrintError {
    /// Process refers to memory it doesn't own
    InvalidAccess,
    /// The message bytes and not valid UTF8 characters
    NotUTF8
}

/// Converts an `QueueIRQError` into a `u32` so it can be returned to the calling process
impl From<KPrintError> for u32 {
    fn from(value: KPrintError) -> Self {
        match value {
            KPrintError::InvalidAccess => 0,
            KPrintError::NotUTF8 => 1
        }
    }
}

/// Converts the register provided by the calling process into their corresponding system call
impl TryFrom<&StackTrace> for SysCall {
    type Error = ();
    fn try_from(stack: &StackTrace) -> Result<Self, ()> {
        // r12 holds the system call number
        match stack.r12 {
            0 => Ok(SysCall::Yield),
            1 => Ok(SysCall::Exit { 
                code: stack.r0
            }),
            2 => Ok(SysCall::WaitIRQ),
            3 => Ok(SysCall::ClearIRQ),
            4 => Ok(SysCall::Send { 
                endpoint: stack.r0,
                tag: stack.r1,
                len: stack.r2,
                data: ptr::with_exposed_provenance_mut(stack.r3 as usize)
            }),
            5 => Ok(SysCall::SendAsync { 
                endpoint: stack.r0,
                tag: stack.r1,
                len: stack.r2,
                data: ptr::with_exposed_provenance_mut(stack.r3 as usize)
            }),
            6 => Ok(SysCall::NotifySend { 
                queue: stack.r0, 
                notifier: stack.r1 
            }),
            7 => Ok(SysCall::WaitQueues { 
                queue_mask: stack.r0 
            }),
            8 => Ok(SysCall::WaitQueuesAsync { 
                queue_mask: stack.r0 
            }),
            9 => Ok(SysCall::WaitQueuesIRQ { 
                queue_mask: stack.r0 
            }),
            10 => Ok(SysCall::WaitQueuesIRQAsync { 
                queue_mask: stack.r0 
            }),
            11 => Ok(SysCall::Header { 
                queue: stack.r0,
            }),
            12 => Ok(SysCall::HeaderAsync { 
                queue: stack.r0 
            }),
            13 => Ok(SysCall::HeaderNonBlocking { 
                queue: stack.r0 
            }),
            14 => Ok(SysCall::HeaderAsyncNonBlocking {
                queue: stack.r0
            }),
            15 => Ok(SysCall::NotifyHeader { 
                notifier: stack.r0
            }),
            16 => Ok(SysCall::Receive { 
                queue: stack.r0, 
                len: stack.r1, 
                buffer: ptr::with_exposed_provenance_mut(stack.r2 as usize) 
            }),
            17 => Ok(SysCall::ReceiveAsync { 
                queue: stack.r0, 
                len: stack.r1, 
                buffer: ptr::with_exposed_provenance_mut(stack.r2 as usize) 
            }),
            18 => Ok(SysCall::NotifyReceive { 
                notifier: stack.r0, 
                len: stack.r1, 
                buffer: ptr::with_exposed_provenance_mut(stack.r2 as usize) 
            }),
            19 => Ok(SysCall::Reply { 
                queue: stack.r0, 
                msg: stack.r1,
                len: stack.r2,
                buffer: ptr::with_exposed_provenance(stack.r3 as usize)
            }),
            20 => Ok(SysCall::NotifyReply { 
                notifier: stack.r0, 
                msg: stack.r1, 
                len: stack.r2, 
                buffer: ptr::with_exposed_provenance(stack.r3 as usize) 
            }),
            21 => Ok(SysCall::KPrint { 
                len: stack.r0, 
                buffer: stack.r1 as *const u8 
            }),
            // no valid system call
            _ => Err(())
        }
    }
}

/// Holds the registers pushed onto the stack on exception entry
#[repr(C)]
pub struct StackTrace {
    /// Register `r0`
    r0: u32,
    /// Register `r1`
    r1: u32,
    /// Register `r2`
    r2: u32,
    /// Register `r3`
    r3: u32,
    /// Register `r12`
    r12: u32,
    /// Register `lr`
    lr: u32,
    /// Return address or register `pc`
    ret_addr: u32,
    // Register `xpsr`
    xpsr: u32,
}

/// Mask for if the exception was generated by the kernel or a process
const PROC_MASK: u32 = 0x8;

/// Non-Maskable Interrupt Handler  
/// `trace` is a pointer to the stack the exception arguments were pushed to on exception entry  
/// `lr` is the contents of the link register on exception entry  
/// Returns the next process to schedule
#[unsafe(no_mangle)]
pub extern "C" fn nmi(trace: *mut StackTrace, lr: u32) -> *mut Proc {
    if lr & PROC_MASK != 0 {
        // recoverable non-maskable interrupt in application
        let cs = unsafe {
            CS::new()
        };
        // reset current process and hope it doesn't die again
        let mut scheduler = scheduler(&cs);
        let pid = unsafe {
            (*scheduler.get_current()).get_pid()
        };
        #[cfg(feature = "radiation")]
        {
            println!("NMI for PID {}", pid);
            panic!();
        }
        #[cfg(not(feature = "radiation"))]
        {
            println!("Recoverable NMI for PID {}", pid);
            scheduler.reset_current();
            scheduler.next_process();
            // reload time quantum for next process
            let mut sys_tick = SYS_TICK.lock(&cs);
            sys_tick.reload();
            scheduler.get_current()
        }
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

/// Hard Fault Handler  
/// `trace` is a pointer to the stack the exception arguments were pushed to on exception entry  
/// `lr` is the contents of the link register on exception entry  
/// Returns the next process to schedule
#[unsafe(no_mangle)]
pub extern "C" fn hard_fault(trace: *mut StackTrace, lr: u32) -> *mut Proc {
    if lr & PROC_MASK != 0 {
        // recoverable hard fault in application
        let cs = unsafe {
            CS::new()
        };
        // reset current process and hope it doesn't die again
        let mut scheduler = scheduler(&cs);
        let pid = unsafe {
            (*scheduler.get_current()).get_pid()
        };
        #[cfg(feature = "radiation")]
        {
            println!("Hard fault for PID {}", pid);
            panic!();
        }
        #[cfg(not(feature = "radiation"))]
        {
            println!("Recoverable hard fault for PID {}", pid);
            scheduler.reset_current();
            scheduler.next_process();
            // reload time quantum for next process
            let mut sys_tick = SYS_TICK.lock(&cs);
            sys_tick.reload();
            scheduler.get_current()
        }
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
/// Interrupts are handled by processes
/// registering to them and waiting for them to happen  
/// This is based off the `InterruptWait` function in QNX
/// <https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.lib_ref/topic/i/interruptwait.html> accessed 1/02/2026  
/// Returns the next process to schedule
#[unsafe(no_mangle)]
pub extern "C" fn irq_handler(irq: u8) -> *mut Proc {
    let cs = unsafe {
        CS::new()
    };
    let mut scheduler = scheduler(&cs);
    let prev = scheduler.get_current();
    scheduler.wake(irq, &cs);
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

/// Performs the system call  
/// `sys_call` is the system call to perform  
/// `cs` is a token to show interrupts are disabled  
/// On success, returns nothing. On error, returns the error code converted to a `u32`
fn do_sys_call(sys_call: SysCall, cs: &CS) -> Result<(), u32> {
    match sys_call {
        // yield
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
            scheduler.sleep_irq(&cs).map_err(|err| u32::from(err))
        },
        // clear IRQ mask
        SysCall::ClearIRQ {} => {
            let mut scheduler = scheduler(cs);
            scheduler.clear_irq(cs).map_err(|err| u32::from(err))
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
        // send message async
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
        // notify send
        SysCall::NotifySend { 
            queue, 
            notifier 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.notify_send(queue, notifier).map_err(|err| u32::from(err))
        },
        // wait queues
        SysCall::WaitQueues {
            queue_mask 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.wait_queues(queue_mask, false, cs).map_err(|err| match err {
                QueueIRQError::QueueError(queue) => u32::from(queue),
                _ => unreachable!()
            })
        },
        // wait queues async
        SysCall::WaitQueuesAsync {
            queue_mask 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.wait_queues_async(queue_mask, false, cs).map_err(|err| match err {
                QueueIRQError::QueueError(queue) => u32::from(queue),
                _ => unreachable!()
            })
        },
        // wait queues IRQ
        SysCall::WaitQueuesIRQ {
            queue_mask 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.wait_queues(queue_mask, true, cs).map_err(|err| u32::from(err))
        },
        // wait queues async IRQ
        SysCall::WaitQueuesIRQAsync {
            queue_mask 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.wait_queues_async(queue_mask, true, cs).map_err(|err| u32::from(err))
        },
        // read header
        SysCall::Header { 
            queue 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.header(queue, true).map_err(|err| u32::from(err))
        },
        // read header async
        SysCall::HeaderAsync { 
            queue 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.header_async(queue, true).map_err(|err| u32::from(err))
        },
        // read header non-blocking
        SysCall::HeaderNonBlocking { 
            queue 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.header(queue, false).map_err(|err| u32::from(err))
        },
        // read header async non-blocking
        SysCall::HeaderAsyncNonBlocking { 
            queue 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.header_async(queue, false).map_err(|err| u32::from(err))
        },
        // read notifier header
        SysCall::NotifyHeader { 
            notifier 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.notify_header(notifier).map_err(|err| u32::from(err))
        },
        // receive data
        SysCall::Receive { 
            queue, 
            len, 
            buffer 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.receive_message(queue, len, buffer).map_err(|err| u32::from(err))
        },
        // receive data async
        SysCall::ReceiveAsync { 
            queue, 
            len, 
            buffer 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.receive_message_async(queue, len, buffer).map_err(|err| u32::from(err))
        },
        // notify receive data
        SysCall::NotifyReceive { 
            notifier, 
            len, 
            buffer 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.notify_receive_message(notifier, len, buffer).map_err(|err| u32::from(err))
        },
        // reply
        SysCall::Reply { 
            queue, 
            msg,
            len,
            buffer 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.reply(queue, msg, len, buffer).map_err(|err| u32::from(err))
        },
        // notify reply
        SysCall::NotifyReply { 
            notifier, 
            msg, 
            len, 
            buffer 
        } => {
            let mut scheduler = scheduler(cs);
            scheduler.notify_reply(notifier, msg, len, buffer).map_err(|err| u32::from(err))
        },
        // kprint
        SysCall::KPrint { 
            len, 
            buffer 
        } => {
            let mut scheduler = scheduler(cs);
            let current = unsafe {
                &mut *scheduler.get_current()
            };
            current.check_access(buffer as u32, len, AccessAttr::new(true, false, false)).map_err(|_| u32::from(KPrintError::InvalidAccess))?;
            let buffer = unsafe {
                slice::from_raw_parts(buffer, len as usize)
            };
            let buffer = str::from_utf8(buffer).map_err(|_| u32::from(KPrintError::NotUTF8))?;
            print!("{}", buffer);
            Ok(())
        },
    }
}

/// System Call Handler  
/// `stack` is a pointer to the process stack the exception arguments were pushed to on exception entry  
/// Returns the next process to schedule
#[unsafe(no_mangle)]
pub extern "C" fn sys_call(stack: *mut StackTrace) -> *mut Proc {
    let stack = unsafe {
        &mut *stack
    };
    let cs = unsafe {
        CS::new()
    };
    // get previous process
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

/// Pend SV Handler  
/// `stack` is a pointer to the process stack the exception arguments were pushed to on exception entry  
/// Returns the next process to schedule
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

/// Performs a process switch and gets the next process  
/// Returns the next process to schedule
/// #Safety
/// must be called without interrupts
#[unsafe(no_mangle)]
pub unsafe extern "C" fn do_proc_switch() -> *mut Proc {
    let cs = unsafe {
        CS::new()
    };
    proc_switch(cs)
}

/// Actually performs a process switch and gets the next process  
/// Returns the next process to schedule
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

/// Sys Tick Handler
/// Returns the next process to schedule
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sys_tick() -> *mut Proc {
    // Safety
    // In interrupt handler so interrupts are disabled
    let cs = unsafe {
        CS::new()
    };
    {
        // reload time quantum
        let mut sys_tick = SYS_TICK.lock(&cs);
        sys_tick.clear_count_flag();
        sys_tick.reload();
        let mut sys = SYSTEM.lock(&cs);
        sys.clear_irq(SysIRQ::SysTick);
    }
    // perform process switch
    proc_switch(cs)
}

/// Critical Section  
/// Signifies no interrupts can happen
pub struct CS {
    _phantom: PhantomData<()>
}

impl CS {
    /// Creates a `CS`
    /// # Safety
    /// Must be called in an environment where interrupts can't happen 
    /// and the resulting object must be destroyed before leaving this environment
    pub unsafe fn new() -> Self {
        Self {
            _phantom: PhantomData
        }
    }
}

/// Disables Interrupts
/// # Safety
/// Logic based on interrupts must be considered before calling this function
pub unsafe fn disable_irq() {
    unsafe {
        asm!("cpsid i");
    }
}

/// Enables Interrupts
/// # Safety
/// Logic based on interrupts must be considered before calling this function
pub unsafe fn enable_irq() {
    unsafe {
        asm!("cpsie i");
    }
}

/// Returns whether interrupts are enabled or not
#[inline(always)]
pub fn irq_enabled() -> bool {
    let mut r0: u32;
    unsafe {
        asm!("mrs {r0}, PRIMASK", r0 = out(reg) r0);
    }
    r0 & 1 == 0
}

/// Saves the interrupt context, disables interrupts, calls `f` and restores the interrupt context
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

#[cfg(test)]
mod test {
    use core::{assert_matches, ptr};
    use crate::{println, nvic::NVIC};
    use super::*;

    #[test_case]
    fn test_reg_to_sys_call() {
        println!("Testing registers to system call");
        let mut stack = StackTrace {
            r0: 1, // arg 0
            r1: 2, // arg 1
            r2: 3, // arg 2
            r3: 4, // arg 3
            r12: 0, // sys call number 
            lr: 0,
            ret_addr: 0,
            xpsr: 0
        };
        let r1_ptr: *const u8 = ptr::without_provenance(2);
        let r2_ptr: *mut u8 = ptr::without_provenance_mut(3);
        let r3_ptr: *mut u8 = ptr::without_provenance_mut(4);
        print!("Testing yield ");
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::Yield)
        );
        println!("[ok]");
        print!("Testing exit ");
        stack.r12 = 1;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::Exit { 
                code: 1 
            })
        );
        println!("[ok]");
        print!("Testing wait IRQ ");
        stack.r12 = 2;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::WaitIRQ)
        );
        println!("[ok]");
        print!("Testing clear IRQ ");
        stack.r12 = 3;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::ClearIRQ)
        );
        println!("[ok]");
        print!("Testing send ");
        stack.r12 = 4;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::Send { 
                endpoint: 1,
                tag: 2,
                len: 3,
                data: r3_ptr
            })
        );
        println!("[ok]");
        print!("Testing send async ");
        stack.r12 = 5;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::SendAsync { 
                endpoint: 1,
                tag: 2,
                len: 3,
                data: r3_ptr
            })
        );
        println!("[ok]");
        print!("Testing notify send ");
        stack.r12 = 6;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::NotifySend { 
                queue: 1,
                notifier: 2,
            })
        );
        println!("[ok]");
        print!("Testing wait queues ");
        stack.r12 = 7;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::WaitQueues { 
                queue_mask: 1
            })
        );
        println!("[ok]");
        print!("Testing wait queues async ");
        stack.r12 = 8;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::WaitQueuesAsync { 
                queue_mask: 1
            })
        );
        println!("[ok]");
        print!("Testing wait queues irq ");
        stack.r12 = 9;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::WaitQueuesIRQ { 
                queue_mask: 1
            })
        );
        println!("[ok]");
        print!("Testing wait queues irq async ");
        stack.r12 = 10;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::WaitQueuesIRQAsync { 
                queue_mask: 1
            })
        );
        println!("[ok]");
        print!("Testing header ");
        stack.r12 = 11;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::Header { 
                queue: 1
            })
        );
        println!("[ok]");
        print!("Testing header async ");
        stack.r12 = 12;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::HeaderAsync { 
                queue: 1
            })
        );
        println!("[ok]");
        print!("Testing header non-blocking ");
        stack.r12 = 13;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::HeaderNonBlocking { 
                queue: 1
            })
        );
        println!("[ok]");
        print!("Testing header async non-blocking ");
        stack.r12 = 14;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::HeaderAsyncNonBlocking { 
                queue: 1
            })
        );
        println!("[ok]");
        print!("Testing notify header ");
        stack.r12 = 15;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::NotifyHeader { 
                notifier: 1
            })
        );
        println!("[ok]");
        print!("Testing receive ");
        stack.r12 = 16;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::Receive { 
                queue: 1,
                len: 2,
                buffer: r2_ptr
            })
        );
        println!("[ok]");
        print!("Testing receive async ");
        stack.r12 = 17;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::ReceiveAsync { 
                queue: 1,
                len: 2,
                buffer: r2_ptr
            })
        );
        println!("[ok]");
        print!("Testing notify receive ");
        stack.r12 = 18;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::NotifyReceive { 
                notifier: 1,
                len: 2,
                buffer: r2_ptr
            })
        );
        println!("[ok]");
        print!("Testing reply ");
        stack.r12 = 19;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::Reply { 
                queue: 1,
                msg: 2,
                len: 3,
                buffer: r3_ptr
            })
        );
        println!("[ok]");
        print!("Testing notify reply ");
        stack.r12 = 20;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::NotifyReply { 
                notifier: 1,
                msg: 2,
                len: 3,
                buffer: r3_ptr
            })
        );
        println!("[ok]");
        print!("Testing kprint ");
        stack.r12 = 21;
        assert_matches!(
            SysCall::try_from(&stack), 
            Ok(SysCall::KPrint { 
                len: 1,
                buffer: r1_ptr
            })
        );
        println!("[ok]");
        print!("Testing invalid sys call ");
        stack.r12 = 22;
        assert_matches!(
            SysCall::try_from(&stack),
            Err(())
        );
        println!("[ok]");
    }

    #[test_case]
    fn test_irq_enable_disable() {
        println!("Testing IRQ enable / disable");
        let cs = unsafe {
            CS::new()
        };
        let mut nvic = NVIC.lock(&cs);
        // disable all IRQ at NVIC so when enabling IRQ for testing, no IRQ is fired
        let prev = nvic.disable_all_irq();
        print!("Testing enable IRQ ");
        let mut primask: u32;
        unsafe {
            enable_irq();
            asm!(
                "mrs r0, primask",
                out("r0") primask
            );
        }
        assert!(irq_enabled());
        assert_eq!(primask, 0);
        println!("[ok]");
        print!("Testing disable IRQ ");
        unsafe {
            disable_irq();
            asm!(
                "mrs r0, primask",
                out("r0") primask
            );
        }
        assert!(!irq_enabled());
        assert_eq!(primask, 1);
        println!("[ok]");
        print!("Testing without inter (interrupts previously disabled) ");
        without_inter(|_| {
            unsafe {
                asm!(
                    "mrs r0, primask",
                    out("r0") primask
                );
            }
            assert!(!irq_enabled());
            assert_eq!(primask, 1);
        });
        // test outside to check state restored
        unsafe {
            asm!(
                "mrs r0, primask",
                out("r0") primask
            );
        }
        assert!(!irq_enabled());
        assert_eq!(primask, 1);
        println!("[ok]");
        print!("Testing without inter (interrupts previously enabled) ");
        unsafe {
            enable_irq();
        }
        without_inter(|_| {
            unsafe {
                asm!(
                    "mrs r0, primask",
                    out("r0") primask
                );
            }
            assert!(!irq_enabled());
            assert_eq!(primask, 1);
        });
        // test outside to check state restored
        unsafe {
            asm!(
                "mrs r0, primask",
                out("r0") primask
            );
        }
        assert!(irq_enabled());
        assert_eq!(primask, 0);
        println!("[ok]");
        // restore IRQ state
        unsafe {
            disable_irq();
        }
        nvic.restore_irq(prev);
    }
}
