use core::{cell::UnsafeCell, ptr, slice};

use crate::{inter::CS, message_queue::{AsyncMessageQueue, Endpoints, MessageQueue, QueueError, SyncMessageQueue}, messages::{AsyncMessage, MESSAGE_DIRECT_LEN, Message}, mutex::{IRQGuard, IRQMutex}, proc::{Proc, ProcState}};

pub const NUM_PROCESSES: usize = 3;

// If this is changed, must change Proc struct
const NUM_PRIORITIES: usize = 256;

pub const QUANTUM_MICROS: u32 = 1000;

#[derive(Debug)]
pub enum ProcError {
    InvalidPID(u32),
    InvalidState
}

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
    pub const fn new(processes: &'static mut [UnsafeCell<Proc>]) -> Self {
        Self { 
            start_proc: [ptr::null_mut(); NUM_PRIORITIES], 
            end_proc: [ptr::null_mut(); NUM_PRIORITIES],
            current: ptr::null_mut(),
            irq_events: [ptr::null_mut(); 32],
            processes
        }
    }

    /// Creates a process 
    /// SAFETY
    /// arguments supplied must be valid for creating a process
    /// On success, returns the pid of the created process. On error, returns `InvalidPID` if the
    /// pid doesn't refer to a process or `InvalidState` if the process is not in the `Free` state
    pub unsafe fn create_proc(
        &mut self, 
        pid: u32, 
        entry: u32, 
        sp: u32, 
        priority: u8, 
        sync_queues: Option<&'static mut [SyncMessageQueue]>, 
        sync_endpoints: Option<&'static Endpoints<SyncMessageQueue>>,
        async_queues: Option<&'static mut [AsyncMessageQueue]>, 
        async_endpoints: Option<&'static Endpoints<AsyncMessageQueue>>
        ) -> Result<u32, ProcError> {
        if (pid as usize) >= self.processes.len() {
            return Err(ProcError::InvalidPID(pid));
        }
        let proc = self.processes[pid as usize].get_mut();
        if proc.get_state() == ProcState::Free {
            unsafe {
                proc.init(pid, entry, sp, priority, sync_queues, sync_endpoints, async_queues, async_endpoints);
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
            proc.set_state(ProcState::Free);
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
            // when goin to start of loop, it's because the last selected process is dead
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
                    proc.set_state(ProcState::Free);
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
            current.set_state(ProcState::Free);
        }
    }

    /// Suspends the current process to wait for IRQ number `irq`
    /// `irq` must be a valid IRQ number
    pub fn sleep_irq(&mut self, irq: u8) {
        if !self.current.is_null() {
            let irq = irq as usize;
            let current = unsafe {
                &mut *self.current
            };
            self.current = ptr::null_mut();
            current.set_state(ProcState::Blocked);
            current.next = self.irq_events[irq];
            self.irq_events[irq] = self.current;
        }
    }

    /// Wakes up all processes waiting on IRQ number `irq`
    /// `irq` must be a valid IRQ number
    pub fn wake(&mut self, irq: u8) {
        let irq = irq as usize;
        let mut current = self.irq_events[irq];
        while !current.is_null() {
            let proc = unsafe {
                &mut *current
            };
            let next = proc.next;
            if proc.get_state() == ProcState::Blocked {
                proc.set_r0(irq as u32);
                proc.set_state(ProcState::Running);
                unsafe {
                    self.schedule_internal(proc);
                }
            } else if proc.get_state() == ProcState::Dead {
                proc.set_state(ProcState::Free);
            }
            current = next;
        }
        self.irq_events[irq] = ptr::null_mut();
    }

    /// SAFETY
    /// either `len` <= 4 or `data` contains a pointer to a `u8` buffer of length `len` 
    pub unsafe fn send(&mut self, endpoint: u32, tag: u32, len: u32, data: u32) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        if endpoint < current.num_sync_endpoints {
            let queue = unsafe {
                &mut **current.sync_endpoints.add(endpoint as usize)
            };
            if (len as usize) < MESSAGE_DIRECT_LEN {
                queue.send(Message::direct(&raw mut *current, tag, data, len).unwrap())?;
            } else {
                let data = unsafe {
                    slice::from_raw_parts(ptr::with_exposed_provenance_mut(data as usize), len as usize)
                };
                queue.send(Message::indirect(&raw mut *current, tag, data))?;
            };
            // block current process
            current.set_state(ProcState::Blocked);
            self.current = ptr::null_mut();
            if !queue.blocked.is_null() {
                let proc = unsafe {
                    &mut *queue.blocked
                };
                // set blocked processes message header
                proc.set_r0(current.pid);
                proc.set_r1(tag);
                proc.set_r2(len);
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

    /// SAFETY
    /// `data` contains a pointer to a `u8` buffer of length `len` 
    pub unsafe fn send_async(&mut self, endpoint: u32, tag: u32, len: u32, data: u32) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        if endpoint < current.num_async_endpoints {
            let queue = unsafe {
                &mut **current.async_endpoints.add(endpoint as usize)
            };
            let data = unsafe {
                slice::from_raw_parts(ptr::with_exposed_provenance_mut(data as usize), len as usize)
            };
            if let Some(msg) = AsyncMessage::new(current.pid, tag, data) {
                queue.send(msg)?;
                if !queue.blocked.is_null() {
                    let proc = unsafe {
                        &mut *queue.blocked
                    };
                    // set blocked processes message header
                    proc.set_r0(current.pid);
                    proc.set_r1(tag);
                    proc.set_r2(len);
                    // wake up blocked receiver
                    unsafe {
                        self.schedule_internal(queue.blocked);
                    }
                    queue.blocked = ptr::null_mut();
                }
                Ok(())
            } else {
                Err(QueueError::BufferTooLarge)
            }
        } else {
            Err(QueueError::InvalidQueue(endpoint))
        }
    }

    pub fn header(&mut self, queue: u32, block: bool) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        if queue < current.num_sync_queues {
            let queue = unsafe {
                &mut *current.sync_queues.add(queue as usize)
            };
            match queue.read_header() {
                Ok(header) => {
                    let sender = unsafe {
                        & *header.sender.proc
                    };
                    current.set_r0(sender.pid);
                    current.set_r1(header.tag);
                    current.set_r2(header.len);
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
        if queue < current.num_async_queues {
            let queue = unsafe {
                &mut *current.async_queues.add(queue as usize)
            };
            match queue.read_header() {
                Ok(header) => {
                    let pid = unsafe {
                        header.sender.pid
                    };
                    current.set_r0(pid);
                    current.set_r1(header.tag);
                    current.set_r2(header.len);
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
        if queue < current.num_sync_endpoints {
            let queue = unsafe {
                &mut *current.sync_queues.add(queue as usize)
            };
            let buffer = unsafe {
                slice::from_raw_parts_mut(buffer, len as usize)
            };
            current.set_r0(queue.read_data(buffer)? as u32);
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
        if queue < current.num_async_endpoints {
            let queue = unsafe {
                &mut *current.async_queues.add(queue as usize)
            };
            let buffer = unsafe {
                slice::from_raw_parts_mut(buffer, len as usize)
            };
            current.set_r0(queue.read_data(buffer)? as u32);
            Ok(())
        } else {
            Err(QueueError::InvalidQueue(queue))
        }
    }

    pub fn reply(&mut self, queue: u32, msg: u32) -> Result<(), QueueError> {
        let current = unsafe {
            &mut *self.current
        };
        if queue < current.num_sync_queues {
            let queue = unsafe {
                &mut *current.sync_queues.add(queue as usize)
            };
            // wake up sender and send reply
            let sender = queue.reply()?;
            let sender = unsafe {
                &mut *sender
            };
            sender.set_r0(msg);
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
        } else if state == ProcState::Running {
            // free proc as not referenced anywhere else
            proc.set_state(ProcState::Init);
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
    static mut PROCESSES0: [UnsafeCell<Proc>; NUM_PROCESSES] = [const { UnsafeCell::new(Proc::new()) }; NUM_PROCESSES];
    static mut PROCESSES1: [UnsafeCell<Proc>; 0] = [const { UnsafeCell::new(Proc::new()) }; 0];
    static SCHEDULER0: IRQMutex<Scheduler> = unsafe {
        IRQMutex::new(Scheduler::new(&mut *(&raw mut PROCESSES0)))
    };
    static SCHEDULER1: IRQMutex<Scheduler> = unsafe {
        IRQMutex::new(Scheduler::new(&mut *(&raw mut PROCESSES1)))
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

mod test {
    use super::*;

}
