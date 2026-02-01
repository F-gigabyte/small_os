use core::ptr;

use crate::{inter::CS, mutex::{IRQGuard, IRQMutex, SpinIRQ}, proc::Proc};

// If this is changed, must change Proc struct
const NUM_PRIORITIES: usize = 256;

pub const QUANTUM_MICROS: u32 = 1000;

pub struct Scheduler {
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
            irq_events: [ptr::null_mut(); 32]
        }
    }

    /// Adds process to be scheduled
    /// SAFETY
    /// `proc` must be initialised and be non null
    /// `proc` must not be accessed by the rest of the program once passed in
    pub unsafe fn add_process(&mut self, proc: *mut Proc) {
        let mut proc = unsafe {
            &mut *proc
        };
        if !self.current.is_null() {
            let current = unsafe {
                &mut *self.current
            };
            // higher priority process preempts a lower priority one
            if current.priority() < proc.priority() {
                self.current = &raw mut *proc;
                proc = current;
            }
        }
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

    /// Obtains the next process to be scheduled
    pub fn next_process(&mut self) {
        let priority;
        if self.current.is_null() {
            priority = NUM_PRIORITIES - 1;
        } else {
            let current = unsafe {
                &mut *self.current
            };
            current.set_running(false);
            priority = current.priority() as usize;
        }
        // if process with same priority as current, select it to be scheduled next or else
        // stick with current
        for (start, end) in self.start_proc[..priority + 1].iter_mut().zip(self.end_proc[..priority + 1].iter_mut()) {
            if !start.is_null() {
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
                proc.set_running(true);
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
            current.set_running(false);
        }
        self.current = ptr::null_mut();
        self.next_process();
    }

    pub fn sleep_irq(&mut self, irq: u8) {
        if !self.current.is_null() {
            let irq = irq as usize;
            let current = unsafe {
                &mut *self.current
            };
            current.sleep();
            current.next = self.irq_events[irq];
            self.irq_events[irq] = self.current;
            self.current = ptr::null_mut();
            self.next_process();
        }
    }

    pub fn wake(&mut self, irq: u8) {
        let irq = irq as usize;
        let mut current = self.irq_events[irq];
        while !current.is_null() {
            let proc = unsafe {
                &mut *current
            };
            let next = proc.next;
            if !proc.runnable() {
                proc.set_r0(irq as u32);
                proc.wake();
                unsafe {
                    self.add_process(current);
                }
            }
            current = next;
        }
        self.irq_events[irq] = ptr::null_mut();
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
    scheduler(&cs).current
}
