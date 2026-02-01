use core::ptr;

const NUM_PROCESSES: usize = 4;

mod proc_flags {
    pub const PRIORITY_SHIFT: usize = 0;
    pub const RUNNING_SHIFT: usize = 8;
    pub const RUNNABLE_SHIFT: usize = 9;

    pub const PRIORITY_MASK: u32 = 0xff << PRIORITY_SHIFT;
    pub const RUNNING_MASK: u32 = 1 << RUNNING_SHIFT;
    pub const RUNNABLE_MASK: u32 = 1 << RUNNABLE_SHIFT;

}

#[repr(C)]
pub struct Proc {
    pub r4: u32,
    pub r5: u32,
    pub r6: u32,
    pub r7: u32,
    pub r8: u32,
    pub r9: u32,
    pub r10: u32,
    pub r11: u32,
    pub pid: u32,
    pub psp: u32,
    pub flags: u32,
    pub next: *mut Proc,
    pub wait_queue: *mut Proc
}

impl Proc {
    pub const fn new() -> Proc {
        Proc { 
            r4: 0, // r4 (0x0)
            r5: 0, // r5 (0x4)
            r6: 0, // r6 (0x8)
            r7: 0, // r7 (0xc)
            r8: 0, // r8 (0x10)
            r9: 0, // r9 (0x14)
            r10: 0, // r10 (0x18)
            r11: 0, // r11 (0x1c)
            pid: 0, // pid (0x20)
            psp: 0, // sp (0x24)
            flags: 0, // (0x28) 
            next: ptr::null_mut(), // (0x2c)
            wait_queue: ptr::null_mut()  // (0x30)
        }
    }

    pub fn set_running(&mut self, running: bool) {
        if running {
            self.flags |= proc_flags::RUNNING_MASK;
        } else {
            self.flags &= !proc_flags::RUNNING_MASK;
        }
    }

    pub fn running(&self) -> bool {
        self.flags & proc_flags::RUNNING_MASK != 0
    }

    pub fn sleep(&mut self) {
        self.flags &= !proc_flags::RUNNABLE_MASK
    }
    
    pub fn wake(&mut self) {
        self.flags |= proc_flags::RUNNABLE_MASK
    }

    pub fn runnable(&self) -> bool {
        self.flags & proc_flags::RUNNABLE_MASK != 0
    }

    pub fn priority(&self) -> u8 {
        ((self.flags & proc_flags::PRIORITY_MASK) >> proc_flags::PRIORITY_SHIFT) as u8
    }

    pub fn set_r0(&mut self, r0: u32) {
        let stack: *mut u32 = ptr::with_exposed_provenance_mut(self.psp as usize);
        unsafe {
            *stack = r0;
        }
    }
    
    pub fn set_r1(&mut self, r1: u32) {
        let stack: *mut u32 = ptr::with_exposed_provenance_mut(self.psp as usize);
        unsafe {
            *stack.add(1) = r1;
        }
    }
    
    pub fn set_r2(&mut self, r2: u32) {
        let stack: *mut u32 = ptr::with_exposed_provenance_mut(self.psp as usize);
        unsafe {
            *stack.add(2) = r2;
        }
    }
    
    pub fn set_r3(&mut self, r3: u32) {
        let stack: *mut u32 = ptr::with_exposed_provenance_mut(self.psp as usize);
        unsafe {
            *stack.add(3) = r3;
        }
    }
    
    pub fn set_r12(&mut self, r12: u32) {
        let stack: *mut u32 = ptr::with_exposed_provenance_mut(self.psp as usize);
        unsafe {
            *stack.add(4) = r12;
        }
    }
    
    pub fn set_lr(&mut self, lr: u32) {
        let stack: *mut u32 = ptr::with_exposed_provenance_mut(self.psp as usize);
        unsafe {
            *stack.add(5) = lr;
        }
    }

    pub fn init(&mut self, entry: u32, mut sp: u32, priority: u8) {
        sp -= 8 * 4;
        let stack: &mut [u32; 8] = unsafe {
            // reserve 8 registers for initial setup
            &mut *ptr::with_exposed_provenance_mut(sp as usize)
        };
        stack[6] = entry;
        stack[7] = 0x01000000; // set thumb mode but nothing else
        self.psp = sp;
        self.flags = (priority as u32) << proc_flags::PRIORITY_SHIFT | proc_flags::RUNNABLE_MASK;
    }
}

unsafe impl Send for Proc {}
unsafe impl Sync for Proc {}

pub static mut PROCESSES: [Proc; NUM_PROCESSES] = [const { Proc::new() }; NUM_PROCESSES];
