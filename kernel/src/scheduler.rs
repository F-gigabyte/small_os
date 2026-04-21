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

use core::{cell::UnsafeCell, ptr, slice};

use crate::{inter::{CS, IRQError, QueueIRQError}, message_queue::{AsyncMessageQueue, QueueError, SyncMessageQueue}, messages::MessageHeader, mutex::{IRQGuard, IRQMutex}, nvic::NVIC, proc::{Proc, ProcError, ProcState}, program::{__args, AccessAttr, Program}};

// If this is changed, must change Proc struct
/// Number of supported process priorities
const NUM_PRIORITIES: usize = 256;

/// Time quantum in micro seconds
#[cfg(debug_assertions)]
pub const QUANTUM_MICROS: u32 = 1000;
/// Time quantum in micro seconds
#[cfg(not(debug_assertions))]
pub const QUANTUM_MICROS: u32 = 100;

/// Scheduler object for handling the scheduling of processes
pub struct Scheduler {
    // processes managed by scheduler
    // have `UnsafeCell` as processes may be accessed through other means than the array and
    // `UnsafeCell` adds indirection for this
    /// All the processes managed by this scheduler
    processes: &'static mut [UnsafeCell<Proc>],
    /// Next process to be scheduled at the given priority
    start_proc: [*mut Proc; NUM_PRIORITIES],
    /// Last process to be scheduled at the given priority
    end_proc: [*mut Proc; NUM_PRIORITIES],
    /// Process currently running
    current: *mut Proc,
    /// IRQ events
    irq_events: [*mut Proc; 32],
}

impl Scheduler {
    /// Creates a new `Scheduler`
    pub const fn new() -> Self {
        Self { 
            start_proc: [ptr::null_mut(); NUM_PRIORITIES], 
            end_proc: [ptr::null_mut(); NUM_PRIORITIES],
            current: ptr::null_mut(),
            irq_events: [ptr::null_mut(); 32],
            processes: &mut [],
        }
    }

    /// Initialises the schedulers processes to those past in
    pub fn init(&mut self, processes: &'static mut [UnsafeCell<Proc>]) {
        self.processes = processes;
    }

    /// Creates a process  
    /// `program` is the program to create the process from  
    /// `args` are the arguments to pass to the process  
    /// Returns the PID on success or a `ProcError` with the corresponding error on failure
    /// # Safety
    /// It must be safe to create this process
    pub unsafe fn create_proc(
        &mut self, 
        program: &'static mut Program,
        args: &[u32]
        ) -> Result<u32, ProcError> {
        let pid = program.pid;
        if (pid as usize) >= self.processes.len() {
            return Err(ProcError::InvalidPID(pid));
        }
        let proc = self.processes[pid as usize].get_mut();
        if proc.get_state() == ProcState::Free {
            let has_inter = program.has_interrupt();
            let inter = program.inter;
            if has_inter {
                for inter in inter {
                    let inter = inter as usize;
                    if inter < self.irq_events.len() {
                        if !self.irq_events[inter].is_null() {
                            return Err(ProcError::IRQTaken);
                        }
                    }
                }
            }
            unsafe {
                proc.init(program, args)?;
            }
            if has_inter {
                for inter in inter {
                    let inter = inter as usize;
                    if inter < self.irq_events.len() {
                        self.irq_events[inter] = proc;
                        
                    }
                }
            }
            Ok(pid)
        } else {
            Err(ProcError::InvalidState)
        }
    }

    /// Switches out the current process so the next time schedule is called, a new process is
    /// scheduled
    fn switch_out_current(&mut self) {
        if !self.current.is_null() {
            let current = self.current;
            self.current = ptr::null_mut();
            unsafe {
                self.schedule_internal::<true>(current);
            }
        }
    }

    /// Sets up `proc` ready to be scheduled  
    /// If it preempts the current process, it becomes the next running process  
    /// If the current process is null or it doesn't preempt current, its placed in its priority's
    /// queue  
    /// If `LAST` is true, it's placed at the end of its queue or else it's placed at the start  
    /// `proc` is the process to be scheduled  
    /// # Safety
    /// Preconditions for `proc` must be upheld as though it was passed through `schedule_process`  
    /// `proc` must not be accessed from `self.processes`  
    /// `proc` must not be `self.current`
    unsafe fn schedule_internal<const LAST: bool>(&mut self, proc: *mut Proc) {
        let mut proc = unsafe {
            &mut *proc
        };
        // Check proc is still alive and if not free it
        // This is safe to do as proc isn't referenced anywhere else
        if proc.get_state() == ProcState::Dead {
            unsafe {
                Self::free_proc(&mut self.irq_events, proc);
            }
        }
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            // higher priority process preempts a lower priority one
            if current.priority() < proc.priority() {
                // swap current and proc since proc preempts current
                current.set_state(ProcState::Scheduled);
                proc.set_state(ProcState::Running);
                self.current = proc;
                proc = current;
            }
        }
        proc.set_state(ProcState::Scheduled);
        proc.next = ptr::null_mut();
        let priority = proc.priority() as usize;
        if LAST {
            // check null case
            if self.end_proc[priority].is_null() {
                self.start_proc[priority] = &raw mut *proc;
                self.end_proc[priority] = &raw mut *proc;
            } else {
                let last = unsafe {
                    &mut *self.end_proc[priority]
                };
                last.next = &raw mut *proc;
                self.end_proc[priority] = &raw mut *proc;
            }
        } else {
            // check null case
            if self.end_proc[priority].is_null() {
                self.start_proc[priority] = &raw mut *proc;
                self.end_proc[priority] = &raw mut *proc;
            } else {
                let start = self.start_proc[priority];
                proc.next = start;
                self.start_proc[priority] = &raw mut *proc;
            }
        }
    }

    /// Adds process to be scheduled
    /// `pid` is the process' PID to schedule  
    /// On error, returns the `ProcError` generated
    pub fn schedule_process(&mut self, pid: u32) -> Result<(), ProcError> {
        if (pid as usize) >= self.processes.len() {
            return Err(ProcError::InvalidPID(pid));
        }
        let state = self.processes[pid as usize].get_mut().get_state();
        if state == ProcState::Init || state.blocked() {
            let proc = self.processes[pid as usize].get();
            unsafe {
                self.schedule_internal::<true>(proc);
            }
            Ok(())
        } else {
            Err(ProcError::InvalidState)
        }
    }

    /// Gives up current process time  
    /// `pid` is the PID of the process that yields  
    /// On error, returns the `ProcError` generated
    pub fn yield_process(&mut self, pid: u32) -> Result<(), ProcError> {
        if (pid as usize) >= self.processes.len() {
            return Err(ProcError::InvalidPID(pid));
        }
        let proc = self.processes[pid as usize].get_mut();
        if proc.get_state() != ProcState::Running {
            Err(ProcError::InvalidState)
        } else {
            // must be current as only way it's running
            self.switch_out_current();
            self.schedule_process(pid).unwrap();
            Ok(())
        }
    }

    /// switches out the current process
    pub fn yield_current(&mut self) {
        self.switch_out_current();
    }

    /// Determines the next process to be scheduled not considering process' with the same priority
    /// as current  
    /// The next process can then be obtained through `get_current()`
    pub fn next_current_process(&mut self) {
        let priority = if self.current.is_null() {
            NUM_PRIORITIES
        } else {
            unsafe {
                (*self.current).priority() as usize
            }
        };
        self.next_process_internal(priority);
    }

    /// Determines the next process to be scheduled considering process' with the same priority as
    /// current  
    /// The next process can then be obtained through `get_current()`
    pub fn next_process(&mut self) {
        let priority;
        if self.current.is_null() {
            priority = NUM_PRIORITIES;
        } else {
            // SAFETY
            // not accesses processes array
            let current = unsafe {
                &mut *self.current
            };
            priority = current.priority() as usize + 1;
        }
        self.next_process_internal(priority);
    }

    /// Determines the next process to be scheduled  
    /// `priority` is one more than the maximum priority to consider  
    /// The next process can then be obtained through `get_current()`
    fn next_process_internal(&mut self, priority: usize) {
        // if process with same priority as priority - 1, select it to be scheduled next or else
        // stick with current
        for (start, end) in self.start_proc[..priority].iter_mut().zip(self.end_proc[..priority].iter_mut()) {
            // when going to start of loop, it's because the last selected process is dead
            while !start.is_null() {
                // SAFETY
                // entry is not null and proc should have been initialised
                let proc = unsafe {
                    &mut **start
                };
                *start = proc.next;
                // if removing last process, set end to null
                if *end == &raw mut *proc {
                    *end = ptr::null_mut();
                }
                // If process is dead, free it as it's not referenced by anything else
                if proc.get_state() == ProcState::Dead {
                    unsafe {
                        Self::free_proc(&mut self.irq_events, proc);
                    }
                    continue;
                }
                proc.set_state(ProcState::Running);
                self.switch_out_current();
                self.current = &raw mut *proc;
                return;
            }
        }
    }

    /// Terminates the current process
    pub fn terminate_current(&mut self) {
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            self.current = ptr::null_mut();
            unsafe {
                Self::free_proc(&mut self.irq_events, current);
            }
        }
    }

    /// Resets the current process to before it started executing  
    /// Swaps the current process out for a new one
    pub fn reset_current(&mut self) {
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            self.current = ptr::null_mut();
            unsafe {
                let program = current.program;
                current.program = ptr::null_mut();
                let program = &mut *program;
                let device = program.device() as usize;
                let mut args = [0; 7];
                let kernel_args = &__args;
                let mut arg_len = 0;
                if device != 0 {
                    args[arg_len] = program.regions[0].get_runtime_addr().unwrap();
                    arg_len += 1;
                    if device == 6 {
                        // IO Bank 0
                        args[arg_len] = kernel_args.pin_func[0];
                        args[arg_len + 1] = kernel_args.pin_func[1];
                        args[arg_len + 2] = kernel_args.pin_func[2];
                        args[arg_len + 3] = kernel_args.pin_func[3];
                        arg_len += 4;
                    } else if device == 8 {
                        // Pads bank 0
                        args[arg_len] = kernel_args.pads[0];
                        args[arg_len + 1] = kernel_args.pads[1];
                        arg_len += 2;
                    } else if device == 27 {
                        // Resets
                        args[arg_len] = kernel_args.resets;
                        arg_len += 1;
                    }
                }
                if program.pin_mask != 0 {
                    args[arg_len] = program.pin_mask;
                    arg_len += 1;
                }
                args[arg_len] = arg_len as u32;

                // reset region data
                for region in &mut program.regions {
                    if region.enabled() {
                        if region.should_zero() {
                            let runtime_addr = region.get_runtime_addr().unwrap();
                            let data: &mut [u8] = slice::from_raw_parts_mut(ptr::with_exposed_provenance_mut(runtime_addr as usize), region.len as usize);
                            data.fill(0);
                        } else if let Some(virt_addr) = region.get_virt() && let Some(phys_addr) = region.get_phys() {
                            let len = 1 << (region.get_len() + 1);
                            let virt: &mut [u8] = slice::from_raw_parts_mut(ptr::with_exposed_provenance_mut(virt_addr as usize), len);
                            let phys: &[u8] = slice::from_raw_parts(ptr::with_exposed_provenance(phys_addr as usize), len);
                            virt.copy_from_slice(phys);
                        }
                        region.encode_codes(program.block_len);
                        region.check_crc().unwrap_or(Ok(())).expect("Have uncorrectable flash error");
                    }
                }
                current.init(
                    &mut *program, 
                    &args[..arg_len + 1]
                ).unwrap();
                self.schedule_process(current.get_pid()).unwrap();
            }
        }
    }

    /// Suspends the current process to wait for IRQ number `irq`  
    /// `cs` is a token to signify interrupts won't happen  
    /// Returns an `IRQError` on error
    pub fn sleep_irq(&mut self, cs: &CS) -> Result<(), IRQError> {
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            let program = unsafe {
                & *current.program
            };
            let irq_mask = Self::get_proc_irq_mask_clear(current, cs)?;
            if irq_mask != 0 {
                _ = current.set_r0(irq_mask as u32);
            } else {
                let mut nvic = NVIC.lock(&cs);
                self.current = ptr::null_mut();
                current.set_state(ProcState::BlockedIRQ);
                // enable the IRQs for firing
                for inter in program.interrupts() {
                    if *inter < 32 {
                        nvic.enable_irq(*inter);
                    }
                }
            }
        }
        Ok(())
    }

    /// Wakes up process waiting on IRQ number `irq`
    /// `irq` is the IRQ to wake on  
    /// `cs` is a token to signify interrupts won't happen  
    /// Panics if `irq` is 32 or more
    pub fn wake(&mut self, irq: u8, cs: &CS) {
        let mut nvic = NVIC.lock(&cs);
        nvic.clear_pending_irq(irq);
        nvic.disable_irq(irq);
        let current = self.irq_events[irq as usize];
        if !current.is_null() {
            let proc = unsafe {
                &mut *current
            };
            if proc.get_state() == ProcState::BlockedIRQ {
                // fine to unwrap as this should be one of this processes IRQs
                // proc IRQ mask should be 0
                _ = proc.set_r0(1 << proc.index_irq(irq).unwrap());
                unsafe {
                    self.schedule_internal::<false>(proc);
                }
            } else if proc.get_state() == ProcState::BlockedQueuesIRQ {
                // proc IRQ mask should be 0
                _ = proc.set_r0(1);
                _ = proc.set_r1(1 << proc.index_irq(irq).unwrap());
                proc.wake_from_queues();
                unsafe {
                    self.schedule_internal::<false>(proc);
                }
            } else if proc.get_state() == ProcState::Dead {
                unsafe {
                    Self::free_proc(&mut self.irq_events, proc);
                }
            } else {
                proc.mask_in_irq(irq);
            }
        }
    }

    /// Clears IRQ state of pending IRQs from the current process and the NVIC
    /// `cs` is a token to signify interrupts won't happen  
    /// Returns an `IRQError` if an error happened
    pub fn clear_irq(&mut self, cs: &CS) -> Result<(), IRQError> {
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            Self::get_proc_irq_mask_clear(current, cs)?;
        }
        Ok(())
    }

    /// Frees process `proc`  
    /// `irq_events` are the scheduler's IRQ events  
    /// `proc` is the process to free
    unsafe fn free_proc(irq_events: &mut [*mut Proc], proc: *mut Proc) {
        let inter = unsafe {
            (*(*proc).program).inter
        };
        // use proc interrupts to remove process from IRQ events
        for irq in inter {
            if (irq as usize) < irq_events.len() {
                irq_events[irq as usize] = ptr::null_mut();
            }
        }
        let proc = unsafe {
            &mut *proc
        };
        proc.set_state(ProcState::Free);
    }

    /// Sends a message from the current process to another
    /// `endpoint` is the process' endpoint to activate  
    /// `tag` is the message's tag  
    /// `len` is the message's data length in bytes  
    /// `data` is the message data to be sent  
    /// Returns an error if it fails
    pub fn send(&mut self, endpoint: u32, tag: u32, len: u32, data: *mut u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if endpoint < program.num_sync_endpoints {
            // check access permissions
            let send_len = len & 0xffff;
            let reply_len = (len >> 16) & 0xffff;
            current.check_access(data.addr() as u32, send_len, AccessAttr::new(true, false, false)).map_err(|_| QueueError::InvalidMemoryAccess)?;
            current.check_access(data.addr() as u32, reply_len, AccessAttr::new(false, true, false)).map_err(|_| QueueError::InvalidMemoryAccess)?;
            let queue = unsafe {
                &mut **program.sync_endpoints.add(endpoint as usize)
            };
            // block current process
            current.set_state(ProcState::BlockedEndpoint);
            self.current = ptr::null_mut();
            if !queue.blocked.is_null() {
                let proc = unsafe {
                    &mut *queue.blocked
                };
                if proc.get_state() == ProcState::BlockedQueue {
                    // pid
                    _ = proc.set_r0(current.get_pid());
                    // pin mask
                    _ = proc.set_r1(current.get_pin_mask());
                    // device tag
                    _ = proc.set_r2(((current.get_device() as u32) << 16) | (tag & 0xffff));
                    // message length
                    _ = proc.set_r3(len);
                    queue.blocked = ptr::null_mut();
                    // wake up blocked receiver
                    unsafe {
                        self.schedule_internal::<false>(proc);
                    }
                } else {
                    let sync_queues = unsafe {
                        (*proc.program).sync_queues
                    };
                    let queue_num = unsafe { (queue as *mut SyncMessageQueue).offset_from_unsigned(sync_queues) } as u32;
                    if proc.get_state() == ProcState::BlockedQueues {
                        // queue that woke it
                        _ = proc.set_r0(queue_num);
                    } else {
                        // woken from queue
                        _ = proc.set_r0(0);
                        // queue that woke it
                        _ = proc.set_r1(queue_num);
                    }
                    proc.wake_from_sync_queues();
                    // wake up blocked receiver
                    unsafe {
                        self.schedule_internal::<false>(proc);
                    }
                }
            }
            // send current to queue
            unsafe {
                queue.send(&raw mut *current);
            }
            Ok(())
        } else {
            Err(QueueError::InvalidQueue(endpoint))
        }
    }

    /// Sends a message from one of the current process' queues to one of its notifier queues
    /// `queue` is the process' queue to send from
    /// `notifier` is the process' notifier queue to send to
    /// Returns an error if it fails
    pub fn notify_send(&mut self, queue: u32, notifier: u32) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_sync_queues() {
            let queue = unsafe {
                &mut *program.sync_queues.add(queue as usize)
            };
            if notifier < program.num_notifier_queues() {
                let notifier = unsafe {
                    &mut *program.notifiers.add(notifier as usize)
                };
                unsafe {
                    notifier.put(queue.take()?);
                }
                Ok(())
            } else {
                Err(QueueError::InvalidNotifer(notifier))
            }
        } else {
            Err(QueueError::InvalidQueue(queue))
        }
    }

    /// Sends a message from the current process to another
    /// `endpoint` is the process' asnchronous endpoint to activate  
    /// `tag` is the message's tag  
    /// `len` is the message's data length in bytes  
    /// `data` is the message data to be sent  
    /// Returns an error if it fails
    pub fn send_async(&mut self, endpoint: u32, tag: u32, len: u32, data: *mut u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if endpoint < program.num_async_endpoints {
            current.check_access(data.addr() as u32, len, AccessAttr::new(true, false, false)).map_err(|_| QueueError::InvalidMemoryAccess)?;
            let queue = unsafe {
                &mut **program.async_endpoints.add(endpoint as usize)
            };
            let data = unsafe {
                slice::from_raw_parts(data, len as usize)
            };
            queue.send(&current, (tag & 0xffff) as u16, data)?;
            if !queue.blocked.is_null() {
                let proc = unsafe {
                    &mut *queue.blocked
                };
                if proc.get_state() == ProcState::BlockedQueue {
                    // pid
                    _ = proc.set_r0(current.get_pid());
                    // pin mask
                    _ = proc.set_r1(current.get_pin_mask());
                    // device tag
                    _ = proc.set_r2(((current.get_device() as u32) << 16) | (tag & 0xffff));
                    // message length
                    _ = proc.set_r3(len);
                    queue.blocked = ptr::null_mut();
                    // wake up blocked receiver
                    unsafe {
                        self.schedule_internal::<false>(proc);
                    }
                } else {
                    let async_queues = unsafe {
                        (*proc.program).async_queues
                    };
                    let queue_num = unsafe { (queue as *mut AsyncMessageQueue).offset_from_unsigned(async_queues) } as u32;
                    if proc.get_state() == ProcState::BlockedQueues {
                        // queue that woke it
                        _ = proc.set_r0(queue_num);
                    } else {
                        // woken from queue
                        _ = proc.set_r0(0);
                        // queue that woke it
                        _ = proc.set_r1(queue_num);
                    }
                    proc.wake_from_async_queues();
                    // wake up blocked receiver
                    unsafe {
                        self.schedule_internal::<false>(proc);
                    }
                }
            }
            Ok(())
        } else {
            Err(QueueError::InvalidQueue(endpoint))
        }
    }

    /// Gets the header of from the next process in the synchronous queue `queue` and writes the
    /// content to the correct registers in `current`  
    /// `current` is the process who's registers should be updated  
    /// `queue` is the queue to obtain the next message from  
    /// Returns a `QueueError` on error
    fn sync_header_internal(&mut self, current: &mut Proc, queue: &mut SyncMessageQueue) -> Result<(), QueueError> {
        let header = queue.read_header()?;
        _ = current.set_r0(header.pid);
        _ = current.set_r1(header.pin_mask);
        _ = current.set_r2(header.device_tag);
        _ = current.set_r3(header.len);
        Ok(())
    }

    /// Gets the state of the process' IRQ state along with the state of any pending IRQs and clears
    /// both of these states
    /// `proc` is the process to get the IRQ state from  
    /// `cs` is a token to signify no interrupts can happen  
    /// Returns a mask of the IRQs on success or an `IRQError` on failure
    fn get_proc_irq_mask_clear(proc: &mut Proc, cs: &CS) -> Result<u8, IRQError> {
        let mut nvic = NVIC.lock(cs);
        let program = unsafe {
            & *proc.program
        };
        let pending = nvic.get_pending();
        let mut mask = proc.get_irq_mask();
        let mut has_inter = false;
        if program.device() != 0 {
            for (i, inter) in program.interrupts().iter().enumerate() {
                if *inter < 32 {
                    has_inter = true;
                    if pending & (1 << *inter) != 0 {
                        mask |= 1 << i;
                        nvic.clear_pending_irq(*inter);
                    }
                }
            }
        }
        proc.clear_irqs();
        if has_inter {
            Ok(mask)
        } else {
            Err(IRQError::NoIRQ)
        }
    }

    /// Gets the header of from the next process in the current process' queue `queue`
    /// `queue` is the queue to obtain the next message from  
    /// `block` is whether to block if the queue is empty or return an error  
    /// Returns a `QueueError` on error
    pub fn header(&mut self, queue: u32, block: bool) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_sync_queues() {
            let queue = unsafe {
                &mut *program.sync_queues.add(queue as usize)
            };
            if let Err(err) = self.sync_header_internal(current, queue) {
                if err == QueueError::QueueEmpty && block {
                    // block current process
                    current.set_state(ProcState::BlockedQueue);
                    queue.blocked = &raw mut *current;
                    self.current = ptr::null_mut();
                } else {
                    return Err(err);
                }
            }
            Ok(())
        } else {
            Err(QueueError::InvalidQueue(queue))
        }
    }

    /// Gets the header of from the next process in the current process' notifier queue `notifier`
    /// `nofifer` is the notifier queue to obtain the next message from  
    /// Returns a `QueueError` on error
    pub fn notify_header(&mut self, notifier: u32) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if notifier < program.num_notifier_queues() {
            let notifier = unsafe {
                &mut *program.notifiers.add(notifier as usize)
            };
            self.sync_header_internal(current, notifier)
        } else {
            Err(QueueError::InvalidQueue(notifier))
        }
    }
    
    /// Gets the header of from the next process in the current process' asynchronous queue `queue`
    /// `queue` is the asynchronous queue to obtain the next message from  
    /// `block` is whether to block if the queue is empty or return an error  
    /// Returns a `QueueError` on error
    pub fn header_async(&mut self, queue: u32, block: bool) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_async_queues() {
            let queue = unsafe {
                &mut *program.async_queues.add(queue as usize)
            };
            match queue.read_header() {
                Ok(header) => {
                    _ = current.set_r0(header.pid);
                    _ = current.set_r1(header.pin_mask);
                    _ = current.set_r2(header.device_tag);
                    _ = current.set_r3(header.len);
                    Ok(())
                },
                Err(err) => {
                    if block && let QueueError::QueueEmpty = err {
                        // block current process
                        current.set_state(ProcState::BlockedQueue);
                        queue.blocked = &raw mut *current;
                        self.current = ptr::null_mut();
                        Ok(())
                    } else {
                        Err(err)
                    }
                }
            }
        } else {
            Err(QueueError::InvalidQueue(queue))
        }
    }

    /// Waits on multiple queues from the current process for a process to arrive  
    /// `queue_mask` is a bitmap of each queue to wait on  
    /// `irq` is whether to also wait on IRQ events as well  
    /// `cs` is a token signifying interrupts can't happen  
    /// Returns a `QueueIRQError` on failure
    pub fn wait_queues(&mut self, queue_mask: u32, irq: bool, cs: &CS) -> Result<(), QueueIRQError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue_mask == 0 {
            return Err(QueueIRQError::QueueError(QueueError::NoQueueMask));
        }
        if program.num_sync_queues() == 0 {
            return Err(QueueIRQError::QueueError(QueueError::InvalidQueue(0)));
        }
        let sync_queues = unsafe {
            slice::from_raw_parts_mut(program.sync_queues, program.num_sync_queues() as usize)
        };
        if irq {
            let irq_mask = Self::get_proc_irq_mask_clear(current, cs)?;
            if irq_mask != 0 {
                _ = current.set_r0(1);
                _ = current.set_r1(irq_mask as u32);
                return Ok(());
            }
        }
        let mut index = 0;
        let mut mask = queue_mask;
        while mask != 0 {
            if index >= sync_queues.len() {
                return Err(QueueIRQError::QueueError(QueueError::InvalidQueue(index as u32)));
            }
            if mask & 1 != 0 {
                if !sync_queues[index].is_empty() {
                    if irq {
                        _ = current.set_r0(0);
                        _ = current.set_r1(index as u32);
                    } else {
                        _ = current.set_r0(index as u32);
                    }
                    return Ok(())
                }
            }
            index += 1;
            mask >>= 1;
        }
        let mut index = 0;
        let mut mask = queue_mask;
        while mask != 0 {
            if mask & 1 != 0 {
                sync_queues[index].blocked = current;
            }
            index += 1;
            mask >>= 1;
        }
        if irq {
            let mut nvic = NVIC.lock(cs);
            // enable the IRQs for firing
            for inter in program.interrupts() {
                if *inter < 32 {
                    nvic.enable_irq(*inter);
                }
            }
            current.set_state(ProcState::BlockedQueuesIRQ);
        } else {
            current.set_state(ProcState::BlockedQueues);
        }
        self.current = ptr::null_mut();
        Ok(())
    }

    /// Waits on multiple asynchronous queues from the current process for a process to arrive  
    /// `queue_mask` is a bitmap of each asynchronous queue to wait on  
    /// `irq` is whether to also wait on IRQ events as well  
    /// `cs` is a token signifying interrupts can't happen  
    /// Returns a `QueueIRQError` on failure
    pub fn wait_queues_async(&mut self, queue_mask: u32, irq: bool, cs: &CS) -> Result<(), QueueIRQError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue_mask == 0 {
            return Err(QueueIRQError::QueueError(QueueError::NoQueueMask));
        }
        if program.num_async_queues() == 0 {
            return Err(QueueIRQError::QueueError(QueueError::InvalidQueue(0)));
        }
        let async_queues = unsafe {
            slice::from_raw_parts_mut(program.async_queues, program.num_async_queues() as usize)
        };
        if irq {
            let irq_mask = Self::get_proc_irq_mask_clear(current, cs)?;
            if irq_mask != 0 {
                _ = current.set_r0(1);
                _ = current.set_r1(irq_mask as u32);
                return Ok(());
            }
        }
        let mut index = 0;
        let mut mask = queue_mask;
        while mask != 0 {
            if mask & 1 != 0 {
                if index >= async_queues.len() {
                    return Err(QueueIRQError::QueueError(QueueError::InvalidQueue(index as u32)));
                }
                if !async_queues[index].is_empty() {
                    if irq {
                        _ = current.set_r0(0);
                        _ = current.set_r1(index as u32);
                    } else {
                        _ = current.set_r0(index as u32);
                    }
                    return Ok(())
                }
            }
            index += 1;
            mask >>= 1;
        }
        let mut index = 0;
        let mut mask = queue_mask;
        while mask != 0 {
            if mask & 1 != 0 {
                async_queues[index].blocked = current;
            }
            index += 1;
            mask >>= 1;
        }
        if irq {
            let mut nvic = NVIC.lock(cs);
            // enable the IRQs for firing
            for inter in program.interrupts() {
                if *inter < 32 {
                    nvic.enable_irq(*inter);
                }
            }
            current.set_state(ProcState::BlockedQueuesIRQ);
        } else {
            current.set_state(ProcState::BlockedQueues);
        }
        self.current = ptr::null_mut();
        Ok(())
    }

    /// Receives message data from a synchronous queue and writes it into `buffer`
    /// `current` is the process whose buffer should be written into  
    /// `queue` is the synchronous queue to be read from  
    /// `len` is the number of bytes to read  
    /// `buffer` is the buffer to write into  
    /// Returns a `QueueError` on failure
    fn sync_receive_message_internal(&mut self, current: &mut Proc, queue: &mut SyncMessageQueue, len: u32, buffer: *mut u8) -> Result<(), QueueError> {
        let data = queue.read_data(len as usize)?;
        let len = data.len();
        // check access is valid
        current.write_bytes(buffer as u32, data).map_err(|_| QueueError::InvalidMemoryAccess)?;
        _ = current.set_r0(len as u32);
        Ok(())
    }

    /// Receives message data from a queue and writes it into `buffer` in the current process
    /// `queue` is the queue to be read from  
    /// `len` is the number of bytes to read  
    /// `buffer` is the buffer to write into  
    /// Returns a `QueueError` on failure
    pub fn receive_message(&mut self, queue: u32, len: u32, buffer: *mut u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_sync_queues() {
            let queue = unsafe {
                &mut *program.sync_queues.add(queue as usize)
            };
            self.sync_receive_message_internal(current, queue, len, buffer)
        } else {
            Err(QueueError::InvalidQueue(queue))
        }
    }

    /// Receives message data from a notifier queue and writes it into `buffer` in the current process
    /// `notifier` is the notifier to be read from  
    /// `len` is the number of bytes to read  
    /// `buffer` is the buffer to write into  
    /// Returns a `QueueError` on failure
    pub fn notify_receive_message(&mut self, notifier: u32, len: u32, buffer: *mut u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if notifier < program.num_notifier_queues() {
            let notifier = unsafe {
                &mut *program.notifiers.add(notifier as usize)
            };
            self.sync_receive_message_internal(current, notifier, len, buffer)
        } else {
            Err(QueueError::InvalidQueue(notifier))
        }
    }

    
    /// Receives message data from an asynchronous queue and writes it into `buffer` in the current process
    /// `queue` is the asynchronous queue to be read from  
    /// `len` is the number of bytes to read  
    /// `buffer` is the buffer to write into  
    /// Returns a `QueueError` on failure
    pub fn receive_message_async(&mut self, queue: u32, len: u32, buffer: *mut u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_async_queues() {
            let queue = unsafe {
                &mut *program.async_queues.add(queue as usize)
            };
            let data = queue.read_data(len as usize)?;
            let len = data.len();
            current.write_bytes(buffer as u32, data).map_err(|_| QueueError::InvalidMemoryAccess)?;
            _ = current.set_r0(len as u32);
            Ok(())
        } else {
            Err(QueueError::InvalidQueue(queue))
        }
    }

    /// Sends a reply from process `current` to the process at the front of the queue `queue`
    /// `current` is the process sending the reply  
    /// `queue` is the synchronous queue to reply to  
    /// `msg` is the reply tag  
    /// `len` is the number of bytes to write  
    /// `buffer` is the buffer to write the reply from  
    /// Returns a `QueueError` on failure
    fn sync_reply_internal(&mut self, current: &mut Proc, queue: &mut SyncMessageQueue, msg: u32, len: u32, buffer: *const u8) -> Result<(), QueueError> {
        let buffer = if len > 0 {
            Some(current.read_bytes(buffer as u32, len as usize).map_err(|_| QueueError::InvalidMemoryAccess)?)
        } else {
            None
        };
        // wake up sender and send reply
        match queue.reply(msg, buffer) {
            Ok(sender) => {
                unsafe {
                    self.schedule_internal::<false>(&raw mut *sender);
                }
                Ok(())
            },
            Err((proc, err)) => {
                if proc.is_null() {
                    Err(err)
                } else {
                    assert!(err == QueueError::SenderInvalidMemoryAccess);
                    let proc = unsafe {
                        &mut *proc
                    };
                    _ = proc.set_r12(u32::from(QueueError::InvalidMemoryAccess) + 1);
                    unsafe {
                        self.schedule_internal::<true>(proc);
                    }
                    Err(err)
                }
            }
        }
    }

    /// Sends a reply from the current process to the process at the front of the queue `queue`
    /// `queue` is the queue to reply to  
    /// `msg` is the reply tag  
    /// `len` is the number of bytes to write  
    /// `buffer` is the buffer to write the reply from  
    /// Returns a `QueueError` on failure
    pub fn reply(&mut self, queue: u32, msg: u32, len: u32, buffer: *const u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_sync_queues() {
            let queue = unsafe {
                &mut *program.sync_queues.add(queue as usize)
            };
            self.sync_reply_internal(current, queue, msg, len, buffer)
        } else {
            Err(QueueError::InvalidQueue(queue))
        }
    }

    /// Sends a reply from the current process to the process at the front of the notifier queue `queue`
    /// `notifier` is the notifier queue to reply to  
    /// `msg` is the reply tag  
    /// `len` is the number of bytes to write  
    /// `buffer` is the buffer to write the reply from  
    /// Returns a `QueueError` on failure
    pub fn notify_reply(&mut self, notifier: u32, msg: u32, len: u32, buffer: *const u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if notifier < program.num_notifier_queues() {
            let notifier = unsafe {
                &mut *program.notifiers.add(notifier as usize)
            };
            self.sync_reply_internal(current, notifier, msg, len, buffer)
        } else {
            Err(QueueError::InvalidQueue(notifier))
        }
    }

    /// Gets the current process
    pub fn get_current(&mut self) -> *mut Proc {
        self.current
    }

    /// Updates the error codes of the current process
    pub fn update_current_codes(&mut self) {
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            current.update_codes();
        }
    }

    /// Kills the process with PID `pid`  
    /// `pid` is the PID of the process to kill  
    /// Returns a `ProcError` on failure
    pub fn kill(&mut self, pid: u32) -> Result<(), ProcError> {
        if (pid as usize) >= self.processes.len() {
            return Err(ProcError::InvalidPID(pid));
        }
        let proc = self.processes[pid as usize].get_mut();
        let state = proc.get_state();
        if matches!(state, ProcState::BlockedEndpoint) || state == ProcState::Scheduled {
            // set proc to dead as may be referenced by other processes
            proc.set_state(ProcState::Dead);
            Ok(())
        } else if matches!(state, ProcState::BlockedQueue | ProcState::BlockedQueues | ProcState::BlockedQueuesIRQ) {
            proc.wake_from_queues();
            unsafe {
                Self::free_proc(&mut self.irq_events, proc);
            }
            Ok(())
        } else if matches!(state, ProcState::Running | ProcState::Init) {
            // free proc as not referenced anywhere else
            unsafe {
                Self::free_proc(&mut self.irq_events, proc);
            }
            self.current = ptr::null_mut();
            Ok(())
        } else {
            Err(ProcError::InvalidState)
        }
    }
}

unsafe impl Send for Scheduler {}
unsafe impl Sync for Scheduler {}


/// Gets the cores CPUID (0 for core 0 and 1 for core 1)
#[inline(always)]
pub fn get_cpuid() -> u32 {
    let cpuid_reg: *const u32 = ptr::with_exposed_provenance(0xd0000000);
    unsafe {
        cpuid_reg.read_volatile()
    }
}

/// Gets the scheduler for this core  
/// core 0 gets scheduler 0 while core 1 gets scheduler 1  
/// `cs` is a token to signify interrupts can't happen
pub fn scheduler<'a, 'b>(cs: &'b CS) -> IRQGuard<'a, 'b, Scheduler> {
    static SCHEDULER0: IRQMutex<Scheduler> = unsafe {
        IRQMutex::new(Scheduler::new())
    };
    static SCHEDULER1: IRQMutex<Scheduler> = unsafe {
        IRQMutex::new(Scheduler::new())
    };
    let cpuid = get_cpuid();
    if cpuid == 0 {
        SCHEDULER0.lock(cs)
    } else if cpuid == 1 {
        SCHEDULER1.lock(cs)
    } else {
        unreachable!();
    }
}

/// Gets the current process in the scheduler
/// # Safety
/// This must be called without interrupts
#[unsafe(no_mangle)]
pub unsafe fn get_current_proc() -> *mut Proc {
    let cs = unsafe {
        CS::new()
    };
    scheduler(&cs).get_current()
}

/// Corrects errors in `proc`'s memory  
/// `proc` is the process whose errors should be corrected  
/// Returns `proc`
/// # Safety
/// `proc` must be a valid process
#[unsafe(no_mangle)]
pub unsafe fn correct_errors(proc: *mut Proc) -> *mut Proc {
    let proc = unsafe {
        &mut *proc
    };
    proc.correct_errors().expect("Unable to correct process errors");
    proc
}

/// Updates `proc`'s error codes   
/// `proc` is the process whose error codes should be updated   
/// Returns `proc`
/// # Safety
/// `proc` must be a valid process
#[unsafe(no_mangle)]
pub unsafe fn update_codes(proc: *mut Proc) -> *mut Proc {
    let proc = unsafe {
        &mut *proc
    };
    proc.update_codes();
    proc
}

#[cfg(test)]
mod test {
    use core::mem::{self, MaybeUninit};

    use crate::{print, println};

    use super::*;

    static mut PROCS: [UnsafeCell<Proc>; 10] = [const { UnsafeCell::new(Proc::new()) }; 10];
    static mut SCHEDULER: Scheduler = Scheduler::new();

    unsafe fn reset_procs() {
        for i in 0..unsafe { (* &raw const PROCS).len() } {
            let proc = unsafe {
                &mut *((&raw mut PROCS) as *mut Proc).add(i)
            };
            *proc = Proc::new();
        }
    }

    unsafe fn reset_scheduler() -> &'static mut Scheduler {
        unsafe {
            reset_procs();
        }
        let scheduler = unsafe {
            &mut * &raw mut SCHEDULER
        };
        for proc in scheduler.start_proc.iter_mut() {
            *proc = ptr::null_mut();
        }
        for proc in scheduler.end_proc.iter_mut() {
            *proc = ptr::null_mut();
        }
        for proc in scheduler.irq_events.iter_mut() {
            *proc = ptr::null_mut();
        }
        scheduler.current = ptr::null_mut();
        unsafe {
            scheduler.init(&mut * &raw mut PROCS);
        }
        scheduler
    }

    #[test_case]
    fn test_priority() {
        println!("Testing scheduler priority");
        let scheduler = unsafe {
            reset_scheduler()
        };
        let mut stack: [u32; _] = [0; 64 * 10];
        let stack_addr = stack.as_mut_ptr() as u32;
        for i in 0..4 {
            let prog = unsafe {
                Program::get_test_prog(i)
            };
            prog.pid = i as u32;
            prog.regions[0].virt_addr = stack_addr + 256 * i as u32;
            prog.regions[0].actual_len = 256;
            prog.regions[0].len = 0x70023;
            prog.flags = 1;
            unsafe {
                let pid = scheduler.create_proc(prog, &[]).unwrap();
                scheduler.schedule_process(pid).unwrap();
            }
        }
        for i in 4..9 {
            let prog = unsafe {
                Program::get_test_prog(i)
            };
            prog.pid = i as u32;
            prog.regions[0].virt_addr = stack_addr + 256 * i as u32;
            prog.regions[0].actual_len = 256;
            prog.regions[0].len = 0x70023;
            prog.flags = i as u32;
            unsafe {
                let pid = scheduler.create_proc(prog, &[]).unwrap();
                scheduler.schedule_process(pid).unwrap();
            }
        }
        print!("Testing round robin ");
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 0);
        }
        scheduler.yield_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 1);
        }
        scheduler.yield_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 2);
        }
        scheduler.yield_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 3);
        }
        scheduler.yield_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 0);
        }
        println!("[ok]");
        print!("Testing preemptive high priority task ");
        {
            let prog = unsafe {
                Program::get_test_prog(9)
            };
            prog.pid = 9;
            prog.regions[0].virt_addr = stack_addr + 256 * 9;
            prog.regions[0].actual_len = 256;
            prog.regions[0].len = 0x70023;
            unsafe {
                let pid = scheduler.create_proc(prog, &[]).unwrap();
                scheduler.schedule_process(pid).unwrap();
            }
        }
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 9);
        }
        println!("[ok]");
        print!("Testing terminating current ");
        scheduler.terminate_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 1);
        }
        println!("[ok]");
    }

    #[test_case]
    fn test_schedule_irq() {
        println!("Testing scheduling around IRQ");
        let scheduler = unsafe {
            reset_scheduler()
        };
        let prog = unsafe {
            Program::get_test_prog(0)
        };
        let cs = unsafe {
            CS::new()
        };
        let mut stack: [u32; _] = [0; 64];
        let stack_addr = stack.as_mut_ptr() as u32;
        prog.pid = 1;
        prog.regions[0].virt_addr = stack_addr;
        prog.regions[0].actual_len = 256;
        prog.regions[0].len = 0x70023;
        prog.flags = 0x10001;
        prog.inter[0] = 5;
        let pid = unsafe {
            scheduler.create_proc(prog, &[]).unwrap()
        };
        scheduler.schedule_process(pid).unwrap();
        print!("Testing proc runnable no irq ");
        scheduler.next_current_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 1);
        }
        println!("[ok]");
        print!("Testing sleep IRQ ");
        scheduler.sleep_irq(&cs).unwrap();
        assert!(scheduler.current.is_null());
        println!("[ok]");
        print!("Testing wake IRQ ");
        scheduler.wake(5, &cs);
        scheduler.next_current_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 1);
            assert_eq!((*scheduler.current).get_r0().unwrap(), 1);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
        }
        println!("[ok]");
        print!("Testing clear IRQ ");
        unsafe {
            (*scheduler.current).mask_in_irq(5);
            assert_eq!((*scheduler.current).get_irq_mask(), 1);
        }
        scheduler.clear_irq(&cs).unwrap();
        unsafe {
            assert_eq!((*scheduler.current).get_irq_mask(), 0);
        }
        println!("[ok]");
    }

    #[test_case]
    fn test_sync_queue() {
        println!("Testing scheduling with synchronous queues");
        let scheduler = unsafe {
            reset_scheduler()
        };
        let prog0 = unsafe {
            Program::get_test_prog(0)
        };
        let prog1 = unsafe {
            Program::get_test_prog(1)
        };
        let mut queue = unsafe {
            SyncMessageQueue::new()
        };
        let cs = unsafe {
            CS::new()
        };
        let endpoint: *mut SyncMessageQueue = &mut queue;
        let mut stack: [u32; _] = [0; 64 * 2];
        stack[2] = 120;
        stack[3] = 240;
        stack[64] = 10;
        stack[65] = 12;
        let stack = stack.as_mut_ptr();
        let stack_addr = stack as u32;
        prog0.pid = 0;
        prog0.sync_queues = &mut queue;
        prog0.num_queues = 0x1;
        prog0.regions[0].virt_addr = stack_addr; 
        prog0.regions[0].actual_len = 256;
        prog0.regions[0].len = 0x70023;
        prog1.pid = 1;
        prog1.sync_endpoints = &endpoint;
        prog1.num_sync_endpoints = 1;
        prog1.regions[0].virt_addr = stack_addr + 256; 
        prog1.regions[0].actual_len = 256;
        prog1.regions[0].len = 0x70023;
        let proc0 = unsafe {
            scheduler.create_proc(prog0, &[]).unwrap()
        };
        let proc1 = unsafe {
            scheduler.create_proc(prog1, &[]).unwrap()
        };
        scheduler.schedule_process(proc1).unwrap();
        scheduler.schedule_process(proc0).unwrap();
        scheduler.next_process();
        print!("Testing proc1 is first ");
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc1);
        }
        println!("[ok]");
        print!("Testing send ");
        let (r0, r1, r2, r3) = unsafe {
            (*scheduler.current).set_r0(0).unwrap();
            (*scheduler.current).set_r1(1).unwrap();
            (*scheduler.current).set_r2((2 * 4) | ((2 * 4) << 16)).unwrap();
            (*scheduler.current).set_r3(stack_addr + 256).unwrap();
            ((*scheduler.current).get_r0().unwrap(), (*scheduler.current).get_r1().unwrap(), (*scheduler.current).get_r2().unwrap(), (*scheduler.current).get_r3().unwrap())
        };
        scheduler.send(r0, r1, r2, ptr::with_exposed_provenance_mut(r3 as usize)).unwrap();
        scheduler.next_current_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
        }
        println!("[ok]");
        print!("Testing wait queues ");
        scheduler.wait_queues(1, false, &cs).unwrap();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), 0);
        }
        println!("[ok]");
        print!("Testing header ");
        scheduler.header(0, true).unwrap();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), proc1);
            assert_eq!((*scheduler.current).get_r1().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r2().unwrap(), 1);
            assert_eq!((*scheduler.current).get_r3().unwrap(), (2 * 4) | ((2 * 4) << 16));
        }
        println!("[ok]");
        print!("Testing receive ");
        scheduler.receive_message(0, 2 * 4, ptr::with_exposed_provenance_mut(stack_addr as usize)).unwrap();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), 2 * 4);
            assert_eq!(stack.add(0).read(), 10);
            assert_eq!(stack.add(1).read(), 12);
        }
        println!("[ok]");
        print!("Testing reply ");
        scheduler.reply(0, 67, 2 * 4, ptr::with_exposed_provenance_mut(stack_addr as usize + 2 * 4)).unwrap();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc1);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), 67);
            assert_eq!(stack.add(64).read(), 120);
            assert_eq!(stack.add(65).read(), 240);
        }
        println!("[ok]");
    }

    #[test_case]
    fn test_notifier_queue() {
        println!("Testing scheduling with notifier queues");
        let scheduler = unsafe {
            reset_scheduler()
        };
        let prog0 = unsafe {
            Program::get_test_prog(0)
        };
        let prog1 = unsafe {
            Program::get_test_prog(1)
        };
        let mut queue = unsafe {
            SyncMessageQueue::new()
        };
        let mut notifier = unsafe {
            SyncMessageQueue::new()
        };
        let endpoint: *mut SyncMessageQueue = &mut queue;
        let mut stack: [u32; _] = [0; 64 * 2];
        stack[2] = 120;
        stack[3] = 240;
        stack[64] = 10;
        stack[65] = 12;
        let stack = stack.as_mut_ptr();
        let stack_addr = stack as u32;
        prog0.pid = 0;
        prog0.sync_queues = &mut queue;
        prog0.num_queues = 0x10001;
        prog0.notifiers = &mut notifier;
        prog0.regions[0].virt_addr = stack_addr; 
        prog0.regions[0].actual_len = 256;
        prog0.regions[0].len = 0x70023;
        prog1.pid = 1;
        prog1.sync_endpoints = &endpoint;
        prog1.num_sync_endpoints = 1;
        prog1.regions[0].virt_addr = stack_addr + 256; 
        prog1.regions[0].actual_len = 256;
        prog1.regions[0].len = 0x70023;
        let proc0 = unsafe {
            scheduler.create_proc(prog0, &[]).unwrap()
        };
        let proc1 = unsafe {
            scheduler.create_proc(prog1, &[]).unwrap()
        };
        scheduler.schedule_process(proc1).unwrap();
        scheduler.schedule_process(proc0).unwrap();
        scheduler.next_process();
        print!("Testing proc1 is first ");
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc1);
        }
        println!("[ok]");
        print!("Testing send ");
        let (r0, r1, r2, r3) = unsafe {
            (*scheduler.current).set_r0(0).unwrap();
            (*scheduler.current).set_r1(1).unwrap();
            (*scheduler.current).set_r2((2 * 4) | ((2 * 4) << 16)).unwrap();
            (*scheduler.current).set_r3(stack_addr + 256).unwrap();
            ((*scheduler.current).get_r0().unwrap(), (*scheduler.current).get_r1().unwrap(), (*scheduler.current).get_r2().unwrap(), (*scheduler.current).get_r3().unwrap())
        };
        scheduler.send(r0, r1, r2, ptr::with_exposed_provenance_mut(r3 as usize)).unwrap();
        scheduler.next_current_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
        }
        println!("[ok]");
        print!("Testing notify send ");
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
        }
        scheduler.notify_send(0, 0).unwrap();
        println!("[ok]");
        print!("Testing notify header ");
        scheduler.notify_header(0).unwrap();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), proc1);
            assert_eq!((*scheduler.current).get_r1().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r2().unwrap(), 1);
            assert_eq!((*scheduler.current).get_r3().unwrap(), (2 * 4) | ((2 * 4) << 16));
        }
        println!("[ok]");
        print!("Testing notify receive ");
        scheduler.notify_receive_message(0, 2 * 4, ptr::with_exposed_provenance_mut(stack_addr as usize)).unwrap();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), 2 * 4);
            assert_eq!(stack.add(0).read(), 10);
            assert_eq!(stack.add(1).read(), 12);
        }
        println!("[ok]");
        print!("Testing notify reply ");
        scheduler.notify_reply(0, 67, 2 * 4, ptr::with_exposed_provenance_mut(stack_addr as usize + 2 * 4)).unwrap();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc1);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), 67);
            assert_eq!(stack.add(64).read(), 120);
            assert_eq!(stack.add(65).read(), 240);
        }
        println!("[ok]");
    }

    #[test_case]
    fn test_async_queue() {
        println!("Testing scheduling with asynchronous queues");
        let scheduler = unsafe {
            reset_scheduler()
        };
        let prog0 = unsafe {
            Program::get_test_prog(0)
        };
        let prog1 = unsafe {
            Program::get_test_prog(1)
        };
        let cs = unsafe {
            CS::new()
        };
        struct Message {
            _header: MessageHeader,
            _data: [u8; 200]
        }
        static mut MESSAGES: [MaybeUninit<Message>; 10] = [const { MaybeUninit::uninit() }; 10];
        let mut queue = unsafe {
            AsyncMessageQueue::new(&raw mut MESSAGES as *mut MessageHeader, 10, mem::size_of::<Message>() as u32)
        };
        let endpoint: *mut AsyncMessageQueue = &mut queue;
        let mut stack: [u32; _] = [0; 64 * 2];
        stack[2] = 120;
        stack[3] = 240;
        stack[64] = 10;
        stack[65] = 12;
        let stack = stack.as_mut_ptr();
        let stack_addr = stack as u32;
        prog0.pid = 0;
        prog0.async_queues = &mut queue;
        prog0.num_queues = 0x100;
        prog0.regions[0].virt_addr = stack_addr; 
        prog0.regions[0].actual_len = 256;
        prog0.regions[0].len = 0x70023;
        prog1.pid = 1;
        prog1.async_endpoints = &endpoint;
        prog1.num_async_endpoints = 1;
        prog1.regions[0].virt_addr = stack_addr + 256; 
        prog1.regions[0].actual_len = 256;
        prog1.regions[0].len = 0x70023;
        let proc0 = unsafe {
            scheduler.create_proc(prog0, &[]).unwrap()
        };
        let proc1 = unsafe {
            scheduler.create_proc(prog1, &[]).unwrap()
        };
        scheduler.schedule_process(proc1).unwrap();
        scheduler.schedule_process(proc0).unwrap();
        scheduler.next_process();
        print!("Testing proc1 is first ");
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc1);
        }
        println!("[ok]");
        print!("Testing send async ");
        unsafe {
            scheduler.send_async(0, 1, 2 * 4, ptr::with_exposed_provenance_mut(stack_addr as usize + 256)).unwrap();
        }
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc1);
        }
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
        }
        println!("[ok]");
        print!("Testing wait queues async ");
        scheduler.wait_queues_async(1, false, &cs).unwrap();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), 0);
        }
        println!("[ok]");
        print!("Testing header async ");
        scheduler.header_async(0, true).unwrap();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), proc1);
            assert_eq!((*scheduler.current).get_r1().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r2().unwrap(), 1);
            assert_eq!((*scheduler.current).get_r3().unwrap(), 2 * 4);
        }
        println!("[ok]");
        print!("Testing receive async ");
        scheduler.receive_message_async(0, 2 * 4, ptr::with_exposed_provenance_mut(stack_addr as usize)).unwrap();
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), proc0);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r0().unwrap(), 2 * 4);
            assert_eq!(stack.add(0).read(), 10);
            assert_eq!(stack.add(1).read(), 12);
        }
        println!("[ok]");
    }

    #[test_case]
    fn test_reset_current() {
        let scheduler = unsafe {
            reset_scheduler()
        };
        let mut stack: [u32; _] = [0; 64];
        let stack_addr = stack.as_mut_ptr() as u32;
        let prog = unsafe {
            Program::get_test_prog(0)
        };
        prog.pid = 1;
        prog.regions[0].virt_addr = stack_addr;
        prog.regions[0].actual_len = 256;
        prog.regions[0].len = 0x70023;
        prog.flags = 1;
        prog.entry = 0x9000;
        unsafe {
            let pid = scheduler.create_proc(prog, &[]).unwrap();
            scheduler.schedule_process(pid).unwrap();
        }
        scheduler.next_process();
        println!("Testing reset current process");
        unsafe {
            assert_eq!((*scheduler.current).get_pid(), 1);
            (*scheduler.current).set_r0(20).unwrap();
            (*scheduler.current).set_r1(20).unwrap();
            (*scheduler.current).set_r2(20).unwrap();
            (*scheduler.current).set_r3(20).unwrap();
            (*scheduler.current).r4 = 20;
            (*scheduler.current).r5 = 20;
            (*scheduler.current).r6 = 20;
            (*scheduler.current).r7 = 20;
            (*scheduler.current).r8 = 20;
            (*scheduler.current).r9 = 20;
            (*scheduler.current).r10 = 20;
            (*scheduler.current).r11 = 20;
            (*scheduler.current).set_r12(20).unwrap();
            (*scheduler.current).set_lr(0x1000).unwrap();
        }
        print!("Testing reset current ");
        scheduler.reset_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).get_r0().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r1().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r2().unwrap(), 0);
            assert_eq!((*scheduler.current).get_r3().unwrap(), 0);
            assert_eq!((*scheduler.current).r4, 0);
            assert_eq!((*scheduler.current).r5, 0);
            assert_eq!((*scheduler.current).r6, 0);
            assert_eq!((*scheduler.current).r7, 0);
            assert_eq!((*scheduler.current).r8, 0);
            assert_eq!((*scheduler.current).r9, 0);
            assert_eq!((*scheduler.current).r10, 0);
            assert_eq!((*scheduler.current).r11, 0);
            assert_eq!((*scheduler.current).get_r12().unwrap(), 0);
            assert_eq!((*scheduler.current).get_lr().unwrap(), 0x9000);
        }
        println!("[ok]");
    }

    #[test_case]
    fn test_update_codes() {
        let scheduler = unsafe {
            reset_scheduler()
        };
        let mut stack: [u32; _] = [0; 64];
        let stack_addr = stack.as_mut_ptr() as u32;
        let prog = unsafe {
            Program::get_test_prog(0)
        };
        prog.pid = 1;
        prog.regions[0].virt_addr = stack_addr;
        prog.regions[0].actual_len = 256;
        prog.regions[0].len = 0x70023;
        prog.flags = 1;
        prog.entry = 0x9000;
        unsafe {
            let pid = scheduler.create_proc(prog, &[]).unwrap();
            scheduler.schedule_process(pid).unwrap();
        }
        scheduler.next_current_process();
        println!("Testing update current process codes");
        print!("Testing update codes ");
        scheduler.update_current_codes();
        unsafe {
            (*scheduler.current).correct_errors().unwrap()
        };
        println!("[ok]")
    }
}
