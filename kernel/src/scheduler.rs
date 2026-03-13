use core::{cell::UnsafeCell, ptr, slice};

use crate::{inter::CS, message_queue::QueueError, messages::MessageHeader, mutex::{IRQGuard, IRQMutex}, proc::{Proc, ProcError, ProcState}, program::{AccessAttr, Program}};

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
        pid: u32, 
        program: &'static mut Program,
        r0: u32
        ) -> Result<u32, ProcError> {
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
                proc.init(pid, program, r0)?;
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
                self.schedule_internal(current);
            }
        }
    }

    /// SAFETY
    /// preconditions for proc must be upheld as though it was passed through `schedule_process`
    /// proc must not be accessed from `self.processes`
    /// proc must not be `self.current`
    unsafe fn schedule_internal(&mut self, proc: *mut Proc) {
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
    }

    /// Adds process to be scheduled
    /// On success returns nothing. On error, returns `InvalidPID` if the `pid` doesn't
    /// refer to a process or `InvalidState` if the process is not in the `Init`, `Blocked` or
    /// `Running` states
    pub fn schedule_process(&mut self, pid: u32) -> Result<(), ProcError> {
        if (pid as usize) >= self.processes.len() {
            return Err(ProcError::InvalidPID(pid));
        }
        let state = self.processes[pid as usize].get_mut().get_state();
        if state == ProcState::Init || state == ProcState::Blocked {
            let proc = self.processes[pid as usize].get();
            unsafe {
                self.schedule_internal(proc);
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
        if self.current.is_null() {
            self.next_process()
        }
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
            priority = current.priority() as usize;
        }
        // if process with same priority as current, select it to be scheduled next or else
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
                let r0 = if driver != 0 {
                    program.regions[0].get_runtime_addr().unwrap()
                } else {
                    0
                };
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
                    current.pid, 
                    &mut *program, 
                    r0
                ).unwrap();
                self.schedule_process(current.pid).unwrap();
            }
        }
    }

    /// Suspends the current process to wait for IRQ number `irq`
    /// `irq` must be a valid IRQ number
    pub fn sleep_irq(&mut self) {
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            self.current = ptr::null_mut();
            current.set_state(ProcState::BlockedIRQ);
        }
    }

    /// Wakes up all processes waiting on IRQ number `irq`
    /// `irq` must be a valid IRQ number
    pub fn wake(&mut self, irq: u8) {
        let irq = irq as usize;
        let current = self.irq_events[irq];
        if !current.is_null() {
            let proc = unsafe {
                &mut *current
            };
            if proc.get_state() == ProcState::BlockedIRQ {
                // fine to unwrap as this should be one of this processes IRQs
                _ = proc.set_r0(proc.index_irq(irq as u8).unwrap() as u32);
                proc.set_state(ProcState::Running);
                unsafe {
                    self.schedule_internal(proc);
                }
            } else if proc.get_state() == ProcState::Dead {
                Self::free_proc(&mut self.irq_events, proc);
            }
        }
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
            current.check_access(data.addr() as u32, len, AccessAttr::new(true, false, false)).map_err(|_| QueueError::InvalidMemoryAccess)?;
            let queue = unsafe {
                &mut **program.sync_endpoints.add(endpoint as usize)
            };
            // block current process
            current.set_state(ProcState::Blocked);
            self.current = ptr::null_mut();
            if !queue.blocked.is_null() {
                let proc = unsafe {
                    &mut *queue.blocked
                };
                // pid
                _ = proc.set_r0(current.pid);
                // tag
                _ = proc.set_r1(tag);
                // message length
                _ = proc.set_r2(len);
                // wake up blocked receiver
                unsafe {
                    self.schedule_internal(queue.blocked);
                }
                queue.blocked = ptr::null_mut();
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
                pid: current.pid,
                tag,
                len
            };
            queue.send(header, data)?;
            if !queue.blocked.is_null() {
                let proc = unsafe {
                    &mut *queue.blocked
                };
                // set blocked processes message header
                // pid
                _ = proc.set_r0(current.pid);
                // tag
                _ = proc.set_r1(tag);
                // message length
                _ = proc.set_r2(len);
                // wake up blocked receiver
                unsafe {
                    self.schedule_internal(queue.blocked);
                }
                queue.blocked = ptr::null_mut();
            }
            Ok(())
        } else {
            Err(QueueError::InvalidQueue(endpoint))
        }
    }

    pub fn header(&mut self, queue: u32, block: bool) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_sync_queues {
            let queue = unsafe {
                &mut *program.sync_queues.add(queue as usize)
            };
            match queue.read_header() {
                Ok(header) => {
                    _ = current.set_r0(header.pid);
                    _ = current.set_r1(header.tag);
                    _ = current.set_r2(header.len);
                    Ok(())
                },
                Err(err) => {
                    if block && let QueueError::QueueEmpty = err {
                        // block current process
                        current.set_state(ProcState::Blocked);
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
    
    pub fn header_async(&mut self, queue: u32, block: bool) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_async_queues {
            let queue = unsafe {
                &mut *program.async_queues.add(queue as usize)
            };
            match queue.read_header() {
                Ok(header) => {
                    _ = current.set_r0(header.pid);
                    _ = current.set_r1(header.tag);
                    _ = current.set_r2(header.len);
                    Ok(())
                },
                Err(err) => {
                    if block && let QueueError::QueueEmpty = err {
                        // block current process
                        current.set_state(ProcState::Blocked);
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

    /// SAFETY
    /// `data` points to a buffer of length `len` 
    pub unsafe fn receive_message(&mut self, queue: u32, len: u32, buffer: *mut u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_sync_queues {
            let queue = unsafe {
                &mut *program.sync_queues.add(queue as usize)
            };
            let data = queue.read_data(len as usize)?;
            let len = data.len();
            // check access is valid
            current.write_bytes(buffer as u32, data).map_err(|_| QueueError::InvalidMemoryAccess)?;
            _ = current.set_r0(len as u32);
            Ok(())
        } else {
            Err(QueueError::InvalidQueue(queue))
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
        if queue < program.num_async_queues {
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

    pub fn reply(&mut self, queue: u32, msg: u32, len: u32, buffer: *const u8) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        let program = unsafe {
            & *current.program
        };
        if queue < program.num_sync_queues {
            let queue = unsafe {
                &mut *program.sync_queues.add(queue as usize)
            };
            let buffer = if len > 0 {
                Some(current.read_bytes(buffer as u32, len as usize).map_err(|_| QueueError::InvalidMemoryAccess)?)
            } else {
                None
            };
            // wake up sender and send reply
            let sender = queue.reply(msg, buffer)?;
            unsafe {
                self.schedule_internal(&raw mut *sender);
            }
            Ok(())
        } else {
            Err(QueueError::InvalidQueue(queue))
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
        if matches!(state, ProcState::Blocked | ProcState::Scheduled) {
            // set proc to dead as may be referenced by other processes
            proc.set_state(ProcState::Dead);
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
