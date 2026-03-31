use core::{cell::UnsafeCell, ptr, slice};

use crate::{inter::{CS, IRQError}, message_queue::{AsyncMessageQueue, QueueError, SyncMessageQueue}, messages::MessageHeader, mutex::{IRQGuard, IRQMutex}, nvic::NVIC, proc::{Proc, ProcError, ProcState}, program::{__args, AccessAttr, Program}};

// If this is changed, must change Proc struct
const NUM_PRIORITIES: usize = 256;

#[cfg(debug_assertions)]
pub const QUANTUM_MICROS: u32 = 1000;
#[cfg(not(debug_assertions))]
pub const QUANTUM_MICROS: u32 = 100;

pub struct Scheduler {
    // processes managed by scheduler
    // have `UnsafeCell` as processes may be accessed through other means than the array and
    // `UnsafeCell` adds indirection for this
    processes: &'static mut [UnsafeCell<Proc>],
    // next process to be scheduled at priority
    start_proc: [*mut Proc; NUM_PRIORITIES],
    // last process to be scheduled at priority
    end_proc: [*mut Proc; NUM_PRIORITIES],
    // process currently running
    current: *mut Proc,
    // IRQ events
    irq_events: [*mut Proc; 32],
}

impl Scheduler {
    pub const fn new() -> Self {
        Self { 
            start_proc: [ptr::null_mut(); NUM_PRIORITIES], 
            end_proc: [ptr::null_mut(); NUM_PRIORITIES],
            current: ptr::null_mut(),
            irq_events: [ptr::null_mut(); 32],
            processes: &mut [],
        }
    }

    pub fn init(&mut self, processes: &'static mut [UnsafeCell<Proc>]) {
        self.processes = processes;
    }

    /// Creates a process 
    /// SAFETY
    /// arguments supplied must be valid for creating a process
    /// On success, returns the pid of the created process. On error, returns `InvalidPID` if the
    /// pid doesn't refer to a process or `InvalidState` if the process is not in the `Free` state
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
            let inter = program.inter;
            for inter in inter {
                let inter = inter as usize;
                if inter < self.irq_events.len() {
                    if !self.irq_events[inter].is_null() {
                        return Err(ProcError::IRQTaken);
                    }
                }
            }
            unsafe {
                proc.init(program, args)?;
            }
            for inter in inter {
                let inter = inter as usize;
                if inter < self.irq_events.len() {
                    self.irq_events[inter] = proc;
                    
                }
            }
            Ok(pid)
        } else {
            Err(ProcError::InvalidState)
        }
    }

    fn switch_out_current(&mut self) {
        if !self.current.is_null() {
            let current = self.current;
            self.current = ptr::null_mut();
            unsafe {
                self.schedule_internal::<true>(current);
            }
        }
    }

    /// SAFETY
    /// preconditions for proc must be upheld as though it was passed through `schedule_process`
    /// proc must not be accessed from `self.processes`
    /// proc must not be `self.current`
    unsafe fn schedule_internal<const LAST: bool>(&mut self, proc: *mut Proc) {
        let proc = unsafe {
            &mut *proc
        };
        // Check proc is still alive and if not free it
        // This is safe to do as proc isn't referenced anywhere else
        if proc.get_state() == ProcState::Dead {
            Self::free_proc(&mut self.irq_events, proc);
        }
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            // higher priority process preempts a lower priority one
            if current.priority() < proc.priority() {
                self.switch_out_current();
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
    /// On success returns nothing. On error, returns `InvalidPID` if the `pid` doesn't
    /// refer to a process or `InvalidState` if the process is not in the `Init` or a blocked state
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
    /// On success returns nothing. On error, returns `InvalidPID` if the `pid` doesn't
    /// refer to a process or `InvalidState` if the process is not in the `Running` state
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

    /// Obtains the next process to be scheduled
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
                    Self::free_proc(&mut self.irq_events, proc);
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
            Self::free_proc(&mut self.irq_events, current);
        }
    }

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
                let driver = program.driver() as usize;
                let mut args = [0; 6];
                let kernel_args = &__args;
                let mut arg_len = 0;
                if driver != 0 {
                    args[arg_len + 1] = program.regions[0].get_runtime_addr().unwrap();
                    arg_len += 1;
                    if driver == 6 {
                        // IO Bank 0
                        args[arg_len + 1] = kernel_args.pin_func[0];
                        args[arg_len + 2] = kernel_args.pin_func[1];
                        args[arg_len + 3] = kernel_args.pin_func[2];
                        args[arg_len + 4] = kernel_args.pin_func[3];
                        arg_len += 4;
                    } else if driver == 8 {
                        // Pads bank 0
                        args[arg_len + 1] = kernel_args.pads[0];
                        args[arg_len + 2] = kernel_args.pads[1];
                        arg_len += 2;
                    } else if driver == 27 {
                        // Resets
                        args[arg_len + 1] = kernel_args.resets;
                        arg_len += 1;
                    }
                }
                if program.pin_mask != 0 {
                    args[arg_len + 1] = program.pin_mask;
                    arg_len += 1;
                }
                args[0] = arg_len as u32;

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
    /// `irq` must be a valid IRQ number
    pub fn sleep_irq(&mut self, cs: &CS) -> Result<(), IRQError> {
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            let program = unsafe {
                & *current.program
            };
            if !program.has_interrupt() {
                return Err(IRQError::NoIRQ)
            }
            let mut nvic = NVIC.lock(&cs);
            let irq_mask = Self::get_proc_irq_mask_clear(current, cs);
            if irq_mask != 0 {
                _ = current.set_r0(irq_mask as u32);
            } else {
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
    /// `irq` must be a valid IRQ number
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
                Self::free_proc(&mut self.irq_events, proc);
            } else {
                proc.mask_in_irq(irq);
            }
        }
    }

    pub fn clear_irq(&mut self, cs: &CS) -> Result<(), IRQError> {
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            Self::get_proc_irq_mask_clear(current, cs);
        }
        Ok(())
    }

    fn free_proc(irq_events: &mut [*mut Proc], proc: *mut Proc) {
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
                    // driver tag
                    _ = proc.set_r2(((current.get_driver() as u32) << 16) | (tag & 0xffff));
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
                notifier.put(queue.take()?);
                Ok(())
            } else {
                Err(QueueError::InvalidNotifer(notifier))
            }
        } else {
            Err(QueueError::InvalidQueue(queue))
        }
    }

    /// SAFETY
    /// `data` contains a pointer to a `u8` buffer of length `len` 
    pub unsafe fn send_async(&mut self, endpoint: u32, tag: u32, len: u32, data: *mut u8) -> Result<(), QueueError> {
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
            let header = MessageHeader {
                pid: current.get_pid(),
                pin_mask: current.get_pin_mask(),
                driver_tag: ((current.get_driver() as u32) << 16) | (tag & 0xffff),
                len
            };
            queue.send(header, data)?;
            if !queue.blocked.is_null() {
                let proc = unsafe {
                    &mut *queue.blocked
                };
                if proc.get_state() == ProcState::BlockedQueue {
                    // pid
                    _ = proc.set_r0(current.get_pid());
                    // pin mask
                    _ = proc.set_r1(current.get_pin_mask());
                    // driver tag
                    _ = proc.set_r2(((current.get_driver() as u32) << 16) | (tag & 0xffff));
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

    fn sync_header_internal(&mut self, current: &mut Proc, queue: &mut SyncMessageQueue) -> Result<(), QueueError> {
        let header = queue.read_header()?;
        _ = current.set_r0(header.pid);
        _ = current.set_r1(header.pin_mask);
        _ = current.set_r2(header.driver_tag);
        _ = current.set_r3(header.len);
        Ok(())
    }

    fn get_proc_irq_mask_clear(proc: &mut Proc, cs: &CS) -> u8 {
        let mut nvic = NVIC.lock(cs);
        let program = unsafe {
            & *proc.program
        };
        let pending = nvic.get_pending();
        let mut mask = proc.get_irq_mask();
        for (i, inter) in program.interrupts().iter().enumerate() {
            if *inter < 32 && pending & (1 << *inter) != 0 {
                mask |= 1 << i;
                nvic.clear_pending_irq(*inter);
            }
        }
        let pending = nvic.get_pending();
        proc.clear_irqs();
        mask
    }

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
                    _ = current.set_r2(header.driver_tag);
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

    pub fn wait_queues(&mut self, queue_mask: u32, irq: bool, cs: &CS) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        let sync_queues = unsafe {
            slice::from_raw_parts_mut(program.sync_queues, program.num_sync_queues() as usize)
        };
        if queue_mask == 0 {
            return Err(QueueError::NoQueueMask);
        }
        if irq {
            let irq_mask = Self::get_proc_irq_mask_clear(current, cs);
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
                return Err(QueueError::InvalidQueue(index as u32));
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

    pub fn wait_queues_async(&mut self, queue_mask: u32, irq: bool, cs: &CS) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        let async_queues = unsafe {
            slice::from_raw_parts_mut(program.async_queues, program.num_async_queues() as usize)
        };
        if queue_mask == 0 {
            _ = current.set_r0(0);
            return Ok(());
        }
        if irq {
            let irq_mask = Self::get_proc_irq_mask_clear(current, cs);
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
                    return Err(QueueError::InvalidQueue(index as u32));
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

    unsafe fn sync_receive_message_internal(&mut self, current: &mut Proc, queue: &mut SyncMessageQueue, len: u32, buffer: *mut u8) -> Result<(), QueueError> {
        let data = queue.read_data(len as usize)?;
        let len = data.len();
        // check access is valid
        current.write_bytes(buffer as u32, data).map_err(|_| QueueError::InvalidMemoryAccess)?;
        _ = current.set_r0(len as u32);
        Ok(())
    }

    /// SAFETY
    /// `data` points to a buffer of length `len` 
    pub unsafe fn receive_message(&mut self, queue: u32, len: u32, buffer: *mut u8) -> Result<(), QueueError> {
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
            unsafe {
                self.sync_receive_message_internal(current, queue, len, buffer)
            }
        } else {
            Err(QueueError::InvalidQueue(queue))
        }
    }

    /// SAFETY
    /// `data` points to a buffer of length `len` 
    pub unsafe fn notify_receive_message(&mut self, notifier: u32, len: u32, buffer: *mut u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if notifier < program.num_notifier_queues() {
            let notifier = unsafe {
                &mut *program.sync_queues.add(notifier as usize)
            };
            unsafe {
                self.sync_receive_message_internal(current, notifier, len, buffer)
            }
        } else {
            Err(QueueError::InvalidQueue(notifier))
        }
    }

    
    /// SAFETY
    /// `data` points to a buffer of length `len` 
    pub unsafe fn receive_message_async(&mut self, queue: u32, len: u32, buffer: *mut u8) -> Result<(), QueueError> {
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

    pub fn get_current(&mut self) -> *mut Proc {
        self.current
    }

    pub fn update_current_codes(&mut self) {
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            current.update_codes();
        }
    }

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
            Self::free_proc(&mut self.irq_events, proc);
            Ok(())
        } else if matches!(state, ProcState::Running | ProcState::Init) {
            // free proc as not referenced anywhere else
            Self::free_proc(&mut self.irq_events, proc);
            self.current = ptr::null_mut();
            Ok(())
        } else {
            Err(ProcError::InvalidState)
        }
    }
}

unsafe impl Send for Scheduler {}
unsafe impl Sync for Scheduler {}


#[inline(always)]
pub fn get_cpuid() -> u32 {
    let cpuid_reg: *const u32 = ptr::with_exposed_provenance(0xd0000000);
    unsafe {
        cpuid_reg.read_volatile()
    }
}

/// Gets the scheduler for this core
/// core 0 gets scheduler 0 while core 1 gets scheduler 1
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
/// SAFETY
/// This must be called without interrupts
#[unsafe(no_mangle)]
pub unsafe fn get_current_proc() -> *mut Proc {
    let cs = unsafe {
        CS::new()
    };
    scheduler(&cs).get_current()
}

#[unsafe(no_mangle)]
pub unsafe fn correct_errors(proc: *mut Proc) -> *mut Proc {
    let proc = unsafe {
        &mut *proc
    };
    proc.correct_errors().expect("Unable to correct process errors");
    proc
}

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
    use crate::{print, println};

    use super::*;
        
    static mut PROCS: [UnsafeCell<Proc>; 10] = [const { UnsafeCell::new(Proc::new()) }; 10];

    #[test_case]
    fn test_priority() {
        println!("Testing scheduler priority");
        let mut scheduler = unsafe {
            Scheduler::new(&mut * &raw mut PROCS)
        };
        let mut stack = [0; 8 * 10];
        for i in 0..4 {
            unsafe {
                let pid = scheduler.create_proc(i as u32, 0, stack.as_mut_ptr().add(8 * i + 8) as u32, 1 as u8, None, None, None, None).unwrap();
                scheduler.schedule_process(pid).unwrap();
            }
        }
        for i in 4..9 {
            unsafe {
                let pid = scheduler.create_proc(i as u32, 0, stack.as_mut_ptr().add(8 * i + 8) as u32, i as u8, None, None, None, None).unwrap();
                scheduler.schedule_process(pid).unwrap();
            }
        }
        print!("Testing round robin ");
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).pid, 0);
        }
        scheduler.yield_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).pid, 1);
        }
        scheduler.yield_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).pid, 2);
        }
        scheduler.yield_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).pid, 3);
        }
        scheduler.yield_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).pid, 0);
        }
        println!("[ok]");
        print!("Testing preemptive high priority task ");
        unsafe {
            let pid = scheduler.create_proc(9, 0, stack.as_mut_ptr().add(80) as u32, 0, None, None, None, None).unwrap();
            scheduler.schedule_process(pid).unwrap();
        }
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).pid, 9);
        }
        println!("[ok]");
        print!("Testing terminating current ");
        scheduler.terminate_current();
        scheduler.next_process();
        unsafe {
            assert_eq!((*scheduler.current).pid, 1);
        }
        println!("[ok]");
    }
}
