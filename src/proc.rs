use core::ptr;

use crate::message_queue::{AsyncMessageQueue, Endpoints, MessageQueue, SyncMessageQueue};

mod proc_flags {
    pub const PRIORITY_SHIFT: usize = 0;
    pub const STATE_SHIFT: usize = 8;

    pub const PRIORITY_MASK: u32 = 0xff << PRIORITY_SHIFT;
    pub const STATE_MASK: u32 = 0x7 << STATE_SHIFT;
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum ProcState {
    Free = 0,
    Init = 1,
    Scheduled = 2,
    Running = 3,
    Blocked = 4,
    Dead = 5
}

impl TryFrom<u32> for ProcState {
    type Error = u32;
    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ProcState::Free),
            1 => Ok(ProcState::Init),
            2 => Ok(ProcState::Scheduled),
            3 => Ok(ProcState::Running),
            4 => Ok(ProcState::Blocked),
            5 => Ok(ProcState::Dead),
            _ => Err(value)
        }
    }
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
    pub num_sync_queues: u32,
    pub num_sync_endpoints: u32,
    pub sync_queues: *mut SyncMessageQueue,
    pub sync_endpoints: *const *mut SyncMessageQueue,
    pub num_async_queues: u32,
    pub num_async_endpoints: u32,
    pub async_queues: *mut AsyncMessageQueue,
    pub async_endpoints: *const *mut AsyncMessageQueue
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
            num_sync_queues: 0, // (0x30)
            num_sync_endpoints: 0, // (0x34)
            sync_queues: ptr::null_mut(), // (0x38)
            sync_endpoints: ptr::null(), // (0x3c)
            num_async_queues: 0, // (0x40)
            num_async_endpoints: 0, // (0x44)
            async_queues: ptr::null_mut(), // (0x48)
            async_endpoints: ptr::null() // (0x4c)
        }
    }

    pub fn set_state(&mut self, state: ProcState) {
        self.flags &= !proc_flags::STATE_MASK;
        self.flags |= (state as u32) << proc_flags::STATE_SHIFT;
    }
    
    /// gets the state
    pub fn get_state(&self) -> ProcState {
        // On error, free the process
        ProcState::try_from((self.flags & proc_flags::STATE_MASK) >> proc_flags::STATE_SHIFT).unwrap_or(ProcState::Free)
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

    // SAFETY
    // `entry` must be a valid execution point, `sp` must be a valid stack, `queues` must be a pointer to
    // an array of `num_queues` queues
    pub unsafe fn init(
        &mut self, 
        pid: u32, 
        entry: u32, 
        mut sp: u32, 
        priority: u8, 
        sync_queues: &'static mut [SyncMessageQueue], 
        sync_endpoints: &'static Endpoints<SyncMessageQueue>,
        async_queues: &'static mut [AsyncMessageQueue],
        async_endpoints: &'static Endpoints<AsyncMessageQueue>,
        ) {
        sp -= 8 * 4;
        let stack: &mut [u32; 8] = unsafe {
            // reserve 8 registers for initial setup
            &mut *ptr::with_exposed_provenance_mut(sp as usize)
        };
        stack[6] = entry;
        stack[7] = 0x01000000; // set thumb mode but nothing else
        self.pid = pid;
        self.psp = sp;
        self.flags = (priority as u32) << proc_flags::PRIORITY_SHIFT | ((ProcState::Init as u32) << proc_flags::STATE_SHIFT);
        self.sync_queues = sync_queues.as_mut_ptr();
        self.num_sync_queues = sync_queues.len() as u32;
        self.sync_endpoints = sync_endpoints.as_ptr();
        self.num_sync_endpoints = sync_endpoints.len() as u32;
        self.async_queues = async_queues.as_mut_ptr();
        self.num_async_queues = async_queues.len() as u32;
        self.async_endpoints = async_endpoints.as_ptr();
        self.num_async_endpoints = async_endpoints.len() as u32;
    }
}

unsafe impl Send for Proc {}
unsafe impl Sync for Proc {}
