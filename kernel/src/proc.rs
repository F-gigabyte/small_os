use core::{mem, ptr, slice};

use crate::{program::{AccessAttr, Program}};

mod proc_flags {
    pub const PRIORITY_SHIFT: usize = 0;
    pub const STATE_SHIFT: usize = 8;
    pub const IRQ_SHIFT: usize = 12;

    pub const PRIORITY_MASK: u32 = 0xff << PRIORITY_SHIFT;
    pub const STATE_MASK: u32 = 0xf << STATE_SHIFT;
    pub const IRQ_MASK: u32 = 0xf << IRQ_SHIFT;
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum ProcState {
    Free = 0,
    Init = 1,
    Scheduled = 2,
    Running = 3,
    BlockedQueue = 4,
    BlockedQueues = 5,
    BlockedIRQ = 6,
    BlockedQueuesIRQ = 7,
    BlockedEndpoint = 8,
    Dead = 9
}

impl ProcState {
    pub fn blocked(&self) -> bool {
        matches!(self, Self::BlockedQueues | Self::BlockedQueue | Self::BlockedQueuesIRQ | Self::BlockedIRQ)
    }
}

impl TryFrom<u32> for ProcState {
    type Error = u32;
    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ProcState::Free),
            1 => Ok(ProcState::Init),
            2 => Ok(ProcState::Scheduled),
            3 => Ok(ProcState::Running),
            4 => Ok(ProcState::BlockedQueue),
            5 => Ok(ProcState::BlockedQueues),
            6 => Ok(ProcState::BlockedIRQ),
            7 => Ok(ProcState::BlockedQueuesIRQ),
            8 => Ok(ProcState::BlockedEndpoint),
            9 => Ok(ProcState::Dead),
            _ => Err(value)
        }
    }
}

#[derive(Debug)]
pub enum ProcError {
    InvalidState,
    StackTooSmall,
    NotOnStack,
    ErrorCode,
    IRQTaken,
    InvalidPID(u32)
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
    pub psp: u32,
    pub flags: u32,
    pub program: *mut Program,
    pub next: *mut Proc,
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
            psp: 0, // sp (0x20)
            flags: 0, // (0x24) 
            program: ptr::null_mut(), // (0x28)
            next: ptr::null_mut() // (0x2c)
        }
    }

    pub fn set_state(&mut self, state: ProcState) {
        self.flags &= !proc_flags::STATE_MASK;
        self.flags |= (state as u32) << proc_flags::STATE_SHIFT;
    }

    pub fn index_irq(&self, irq: u8) -> Option<usize> {
        let prog = unsafe {
            &*self.program
        };
        prog.index_irq(irq)
    }
    
    /// gets the state
    pub fn get_state(&self) -> ProcState {
        // On error, panic
        ProcState::try_from((self.flags & proc_flags::STATE_MASK) >> proc_flags::STATE_SHIFT).expect("Have invalid process state.")
    }

    pub fn priority(&self) -> u8 {
        ((self.flags & proc_flags::PRIORITY_MASK) >> proc_flags::PRIORITY_SHIFT) as u8
    }

    fn set_stack_offset(&mut self, offset: usize, word: u32) -> Result<(), ProcError> {
        let program = unsafe {
            &mut *self.program
        };
        program.write_word(self.psp + (offset * mem::size_of::<u32>()) as u32, word).map_err(|_| ProcError::NotOnStack)?;
        Ok(())
    }
    
    fn get_stack_offset(&mut self, offset: usize) -> Result<u32, ProcError> {
        let program = unsafe {
            &mut *self.program
        };
        let word = program.read_word(self.psp + (offset * mem::size_of::<u32>()) as u32).map_err(|_| ProcError::NotOnStack)?;
        Ok(word)
    }

    pub fn set_r0(&mut self, r0: u32) -> Result<(), ProcError> {
        self.set_stack_offset(0, r0)
    }
    
    pub fn set_r1(&mut self, r1: u32) -> Result<(), ProcError> {
        self.set_stack_offset(1, r1)
    }
    
    pub fn set_r2(&mut self, r2: u32) -> Result<(), ProcError> {
        self.set_stack_offset(2, r2)
    }
    
    pub fn set_r3(&mut self, r3: u32) -> Result<(), ProcError> {
        self.set_stack_offset(3, r3)
    }
    
    pub fn set_r12(&mut self, r12: u32) -> Result<(), ProcError> {
        self.set_stack_offset(4, r12)
    }
    
    pub fn get_r0(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(0)
    }
    
    pub fn get_r1(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(1)
    }
    
    pub fn get_r2(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(2)
    }
    
    pub fn get_r3(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(3)
    }
    
    pub fn get_r12(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(4)
    }

    pub fn set_lr(&mut self, lr: u32) -> Result<(), ProcError> {
        self.set_stack_offset(5, lr)
    }
    
    pub fn get_lr(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(5)
    }

    // SAFETY
    // `entry` must be a valid execution point, `sp` must be a valid stack, `queues` must be a pointer to
    // an array of `num_queues` queues
    pub unsafe fn init(
        &mut self, 
        program: &'static mut Program,
        args: &[u32]
        ) -> Result<(), ProcError> {
        let sp_region = &program.regions[program.sp as usize];
        let mut sp = sp_region.get_runtime_addr().unwrap() + sp_region.actual_len;
        sp -= (8 + (args.len() as u32).saturating_sub(4)) * 4;
        for (i, arg) in args.iter().enumerate() {
            if i < 4 {
                program.write_word(sp + (i * mem::size_of::<u32>()) as u32, *arg).map_err(|_| ProcError::StackTooSmall)?;
            } else {
                program.write_word(sp + ((i + 4) * mem::size_of::<u32>()) as u32, *arg).map_err(|_| ProcError::StackTooSmall)?;
            }
        }
        program.write_word(sp + 6 * mem::size_of::<u32>() as u32, program.entry).map_err(|_| ProcError::StackTooSmall)?;
        // set thumb mode but nothing else
        program.write_word(sp + 7 * mem::size_of::<u32>() as u32, 0x1000000).map_err(|_| ProcError::StackTooSmall)?;
        self.psp = sp;
        self.flags = (program.priority() as u32) << proc_flags::PRIORITY_SHIFT | ((ProcState::Init as u32) << proc_flags::STATE_SHIFT);
        self.program = program;
        Ok(())
    }

    pub fn correct_errors(&mut self) -> Result<(), ()> {
        let prog = unsafe {
            &mut *self.program
        };
        prog.correct_errors()
    }
    
    pub fn update_codes(&mut self) {
        let prog = unsafe {
            &mut *self.program
        };
        prog.update_codes();
    }

    pub fn write_word(&mut self, addr: u32, word: u32) -> Result<(), ()> {
        let prog = unsafe {
            &mut *self.program
        };
        prog.write_word(addr, word)
    }
    
    pub fn write_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), ()> {
        let prog = unsafe {
            &mut *self.program
        };
        prog.write_bytes(addr, data)
    }
    
    pub fn read_word(&mut self, addr: u32) -> Result<u32, ()> {
        let prog = unsafe {
            &mut *self.program
        };
        prog.read_word(addr)
    }
    
    pub fn read_bytes(&mut self, addr: u32, len: usize) -> Result<&[u8], ()> {
        let prog = unsafe {
            &mut *self.program
        };
        prog.read_bytes(addr, len)
    }

    #[inline(always)]
    pub fn get_pid(&self) -> u32 {
        let prog = unsafe {
            & *self.program
        };
        prog.pid
    }

    #[inline(always)]
    pub fn get_driver(&self) -> u16 {
        let prog = unsafe {
            & *self.program
        };
        prog.driver()
    }

    #[inline(always)]
    pub fn get_pin_mask(&self) -> u32 {
        let prog = unsafe {
            & *self.program
        };
        prog.pin_mask
    }

    #[inline(always)]
    pub fn get_irq_mask(&self) -> u8 {
        ((self.flags & proc_flags::IRQ_MASK) >> proc_flags::IRQ_SHIFT) as u8
    }
    
    #[inline(always)]
    pub fn clear_irqs(&mut self) {
        self.flags &= !proc_flags::IRQ_MASK;
    }

    #[inline(always)]
    pub fn mask_in_irq(&mut self, irq: u8) {
        self.flags |= (1 << self.index_irq(irq).unwrap()) << proc_flags::IRQ_SHIFT;
    }

    pub fn check_access(&self, addr: u32, len: u32, attr: AccessAttr) -> Result<(), ()> {
        let prog = unsafe {
            & *self.program
        };
        prog.check_access(addr, len, attr)
    }

    pub fn wake_from_async_queues(&mut self) {
        let program = unsafe {
            &mut *self.program
        };
        if program.async_queues.is_null() {
            return;
        }
        let async_queues = unsafe {
            slice::from_raw_parts_mut(program.async_queues, program.num_async_queues() as usize)
        };
        for queue in async_queues {
            queue.blocked = ptr::null_mut();
        }
    }

    pub fn wake_from_sync_queues(&mut self) {
        let program = unsafe {
            &mut *self.program
        };
        if program.sync_queues.is_null() {
            return;
        }
        let sync_queues = unsafe {
            slice::from_raw_parts_mut(program.sync_queues, program.num_sync_queues() as usize)
        };
        for queue in sync_queues {
            queue.blocked = ptr::null_mut();
        }
    }

    pub fn wake_from_queues(&mut self) {
        self.wake_from_sync_queues();
        self.wake_from_async_queues();
    }
}

unsafe impl Send for Proc {}
unsafe impl Sync for Proc {}
