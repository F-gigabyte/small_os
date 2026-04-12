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

use core::{mem, ptr, slice};

use crate::{program::{AccessAttr, Program}};

/// Process flags masks and shifts
mod proc_flags {
    /// Process priority shift
    pub const PRIORITY_SHIFT: usize = 0;
    /// Process state shift
    pub const STATE_SHIFT: usize = 8;
    /// Process IRQ state shift
    pub const IRQ_SHIFT: usize = 12;

    /// Process priority mask
    pub const PRIORITY_MASK: u32 = 0xff << PRIORITY_SHIFT;
    /// Process state mask
    pub const STATE_MASK: u32 = 0xf << STATE_SHIFT;
    /// Process IRQ state mask
    pub const IRQ_MASK: u32 = 0xf << IRQ_SHIFT;
}

/// Process States
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum ProcState {
    /// Process is free, either because it hasn't been initialised yet or because it has been freed
    /// after dying
    Free = 0,
    /// Process is initialised and ready to be scheduled
    Init = 1,
    /// Process is in scheduler and ready to be run
    Scheduled = 2,
    /// Process is current running
    Running = 3,
    /// Process is blocked waiting on a queue
    BlockedQueue = 4,
    /// Process is blocked waiting on multiple queues
    BlockedQueues = 5,
    /// Process is blocked waitig on an IRQ
    BlockedIRQ = 6,
    /// Process is blocked waiting on multiple queues and on an IRQ
    BlockedQueuesIRQ = 7,
    /// Process is blocked waiting on an endpoint
    BlockedEndpoint = 8,
    /// Process has died but its resources haven't been freed yet
    Dead = 9
}

impl ProcState {
    /// Process is in a blocked state
    pub fn blocked(&self) -> bool {
        matches!(self, Self::BlockedQueues | Self::BlockedQueue | Self::BlockedQueuesIRQ | Self::BlockedIRQ)
    }
}

/// Converts a `u32` to a `ProcState`
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

/// Errors related to interacting with a process
#[derive(Debug)]
pub enum ProcError {
    /// Process is in an invalid state for the action performed
    InvalidState,
    /// Process' stack is too small
    StackTooSmall,
    /// 
    NotOnStack,
    /// Process' memory has been altered in an unrecoverable way
    ErrorCode,
    IRQTaken,
    /// Invalid process PID
    InvalidPID(u32)
}

/// A process
#[repr(C)]
pub struct Proc {
    /// Register `r4`
    pub r4: u32,
    /// Register `r5`
    pub r5: u32,
    /// Register `r6`
    pub r6: u32,
    /// Register `r7`
    pub r7: u32,
    /// Register `r8`
    pub r8: u32,
    /// Register `r9`
    pub r9: u32,
    /// Register `r10`
    pub r10: u32,
    /// Register `r11`
    pub r11: u32,
    /// Process stack pointer
    pub psp: u32,
    /// Process flags
    pub flags: u32,
    /// Process' program
    pub program: *mut Program,
    /// Next process (if in scheduler, the next process to be sheduled after this while in queues is
    /// the process after this one)
    pub next: *mut Proc,
}

impl Proc {
    /// Creates a new process (0s out process memory ready for it to be initialised)
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

    /// Sets the process' state  
    /// `state` is the state to set this process' state to
    pub fn set_state(&mut self, state: ProcState) {
        self.flags &= !proc_flags::STATE_MASK;
        self.flags |= (state as u32) << proc_flags::STATE_SHIFT;
    }

    /// Index a process' program's IRQ to find the index of that IRQ  
    /// `irq` is the IRQ number to get the index of  
    /// Returns the index if it's found or else `None`
    pub fn index_irq(&self, irq: u8) -> Option<usize> {
        let prog = unsafe {
            &*self.program
        };
        prog.index_irq(irq)
    }
    
    /// Gets the process' state
    pub fn get_state(&self) -> ProcState {
        // On error, panic
        ProcState::try_from((self.flags & proc_flags::STATE_MASK) >> proc_flags::STATE_SHIFT).expect("Have invalid process state.")
    }

    /// Gets the process' priority
    pub fn priority(&self) -> u8 {
        ((self.flags & proc_flags::PRIORITY_MASK) >> proc_flags::PRIORITY_SHIFT) as u8
    }

    /// Writes a 32 bit integer to the process' stack offset by `offset`  
    /// `offset` is the offset from the stack pointer  
    /// `word` is the integer to write  
    /// On success returns nothing and on error returns a `ProcError` explaining the error
    fn set_stack_offset(&mut self, offset: usize, word: u32) -> Result<(), ProcError> {
        assert!(!self.program.is_null());
        let program = unsafe {
            &mut *self.program
        };
        program.write_word(self.psp + (offset * mem::size_of::<u32>()) as u32, word).map_err(|_| ProcError::NotOnStack)?;
        Ok(())
    }
    
    /// Reads a 32 bit integer to the process' stack offset by `offset`  
    /// `offset` is the offset from the stack pointer  
    /// On success returns the integer at the offset and on error returns a `ProcError` explaining the error
    fn get_stack_offset(&mut self, offset: usize) -> Result<u32, ProcError> {
        assert!(!self.program.is_null());
        let program = unsafe {
            &mut *self.program
        };
        let word = program.read_word(self.psp + (offset * mem::size_of::<u32>()) as u32).map_err(|_| ProcError::NotOnStack)?;
        Ok(word)
    }

    /// Sets the process' r0 register which is saved on the stack
    pub fn set_r0(&mut self, r0: u32) -> Result<(), ProcError> {
        self.set_stack_offset(0, r0)
    }
    
    /// Sets the process' r1 register which is saved on the stack
    pub fn set_r1(&mut self, r1: u32) -> Result<(), ProcError> {
        self.set_stack_offset(1, r1)
    }
    
    /// Sets the process' r2 register which is saved on the stack
    pub fn set_r2(&mut self, r2: u32) -> Result<(), ProcError> {
        self.set_stack_offset(2, r2)
    }
    
    /// Sets the process' r3 register which is saved on the stack
    pub fn set_r3(&mut self, r3: u32) -> Result<(), ProcError> {
        self.set_stack_offset(3, r3)
    }
    
    /// Sets the process' r12 register which is saved on the stack
    pub fn set_r12(&mut self, r12: u32) -> Result<(), ProcError> {
        self.set_stack_offset(4, r12)
    }
    
    /// Returns the process' r0 register which is saved on the stack
    pub fn get_r0(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(0)
    }
    
    /// Returns the process' r1 register which is saved on the stack
    pub fn get_r1(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(1)
    }
    
    /// Returns the process' r2 register which is saved on the stack
    pub fn get_r2(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(2)
    }
    
    /// Returns the process' r3 register which is saved on the stack
    pub fn get_r3(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(3)
    }
    
    /// Returns the process' r12 register which is saved on the stack
    pub fn get_r12(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(4)
    }

    /// Sets the process' link register which is saved on the stack
    pub fn set_lr(&mut self, lr: u32) -> Result<(), ProcError> {
        self.set_stack_offset(5, lr)
    }
    
    /// Returns the process' link register which is saved on the stack
    pub fn get_lr(&mut self) -> Result<u32, ProcError> {
        self.get_stack_offset(5)
    }

    /// Initialises a process  
    /// `program` is the progam image the process should be created from    
    /// `args` is a list of initial arguments that are pushed to the process' stack and accessed by
    /// its program through the `args()` function in the OS library  
    /// The process' registers are initialised to 0 except for the stack pointer which points to the
    /// end of the processes stack (and is decremented by an initial amount to push the initial
    /// registers and arguments) as well as the program counter which points to the entry address
    /// along with the link register and the XPSR register which is set up to enable thumb mode
    /// # Safety
    /// `program` must have been initialised correctly
    pub unsafe fn init(
        &mut self, 
        program: &'static mut Program,
        args: &[u32]
        ) -> Result<(), ProcError> {
        let sp_region = &program.regions[program.sp as usize];
        let mut sp = sp_region.get_runtime_addr().unwrap() + sp_region.actual_len;
        sp -= (8 + (args.len() as u32)) * 4;
        // zero registers r0 to r12
        for i in 0..5 {
            program.write_word(sp + (i * mem::size_of::<u32>()) as u32, 0).map_err(|_| ProcError::StackTooSmall)?;
        }
        // lr is entry
        program.write_word(sp + 5 * mem::size_of::<u32>() as u32, program.entry).map_err(|_| ProcError::StackTooSmall)?;
        // program entry
        program.write_word(sp + 6 * mem::size_of::<u32>() as u32, program.entry).map_err(|_| ProcError::StackTooSmall)?;
        // set thumb mode but nothing else
        program.write_word(sp + 7 * mem::size_of::<u32>() as u32, 0x1000000).map_err(|_| ProcError::StackTooSmall)?;
        for (i, arg) in args.iter().enumerate() {
            program.write_word(sp + ((i + 8) * mem::size_of::<u32>()) as u32, *arg).map_err(|_| ProcError::StackTooSmall)?;
        }
        self.psp = sp;
        self.flags = (program.priority() as u32) << proc_flags::PRIORITY_SHIFT | ((ProcState::Init as u32) << proc_flags::STATE_SHIFT);
        self.program = program;
        self.r4 = 0;
        self.r5 = 0;
        self.r6 = 0;
        self.r7 = 0;
        self.r8 = 0;
        self.r9 = 0;
        self.r10 = 0;
        self.r11 = 0;
        Ok(())
    }

    /// Corrects the process' errors and returns if it was successful or not
    pub fn correct_errors(&mut self) -> Result<(), ()> {
        assert!(!self.program.is_null());
        let prog = unsafe {
            &mut *self.program
        };
        prog.correct_errors()
    }
    
    /// Updates the process' error codes based on changes to its memory
    pub fn update_codes(&mut self) {
        assert!(!self.program.is_null());
        let prog = unsafe {
            &mut *self.program
        };
        prog.update_codes();
    }

    /// Writes a 32 bit integer into the process' address space, updating its error codes in the
    /// process
    /// `addr` is the address to write to  
    /// `word` is the integer to write  
    /// Returns if the operation was successful or not
    pub fn write_word(&mut self, addr: u32, word: u32) -> Result<(), ()> {
        assert!(!self.program.is_null());
        let prog = unsafe {
            &mut *self.program
        };
        prog.write_word(addr, word)
    }
    
    /// Writes a series of bytes into the process' address space, updating its error codes in the
    /// process   
    /// `addr` is the address to write to  
    /// `data` is the series of bytes to write  
    /// Returns if the operation was successful or not
    pub fn write_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), ()> {
        assert!(!self.program.is_null());
        let prog = unsafe {
            &mut *self.program
        };
        prog.write_bytes(addr, data)
    }
    
    /// Reads a 32 bit integer from the process' address space
    /// `addr` is the address to read from  
    /// Returns the integer on success or indicates a failure on error
    pub fn read_word(&mut self, addr: u32) -> Result<u32, ()> {
        assert!(!self.program.is_null());
        let prog = unsafe {
            &mut *self.program
        };
        prog.read_word(addr)
    }
    
    /// Reads a series of bytes from the process' address space
    /// `addr` is the address to read from  
    /// `len` is the number of bytes to read
    /// Returns a slice to the requested memory on success or indicates a failure on error
    pub fn read_bytes(&mut self, addr: u32, len: usize) -> Result<&[u8], ()> {
        assert!(!self.program.is_null());
        let prog = unsafe {
            &mut *self.program
        };
        prog.read_bytes(addr, len)
    }

    /// Returns the process' PID
    #[inline(always)]
    pub fn get_pid(&self) -> u32 {
        assert!(!self.program.is_null());
        let prog = unsafe {
            & *self.program
        };
        prog.pid
    }

    /// Returns the process' driver
    #[inline(always)]
    pub fn get_driver(&self) -> u16 {
        assert!(!self.program.is_null());
        let prog = unsafe {
            & *self.program
        };
        prog.driver()
    }

    /// Returns the process' pin mask
    #[inline(always)]
    pub fn get_pin_mask(&self) -> u32 {
        assert!(!self.program.is_null());
        let prog = unsafe {
            & *self.program
        };
        prog.pin_mask
    }

    /// Returns the process' IRQ state mask
    #[inline(always)]
    pub fn get_irq_mask(&self) -> u8 {
        ((self.flags & proc_flags::IRQ_MASK) >> proc_flags::IRQ_SHIFT) as u8
    }
    
    /// Clears the process' IRQ state mask
    #[inline(always)]
    pub fn clear_irqs(&mut self) {
        self.flags &= !proc_flags::IRQ_MASK;
    }

    /// Masks in IRQ `irq` to process' IRQ state mask  
    /// `irq` is the IRQ to mask in  
    /// Panics if `irq` is not one of this process' program's IRQs
    #[inline(always)]
    pub fn mask_in_irq(&mut self, irq: u8) {
        self.flags |= (1 << self.index_irq(irq).unwrap()) << proc_flags::IRQ_SHIFT;
    }

    /// Checks if a memory access is valid for this process
    /// `addr` is the address to access  
    /// `len` is the number of bytes from address to access  
    /// `attr` are the access attributes used when accessing this memory region  
    /// Returns if the access was successful or not
    pub fn check_access(&self, addr: u32, len: u32, attr: AccessAttr) -> Result<(), ()> {
        assert!(!self.program.is_null());
        let prog = unsafe {
            & *self.program
        };
        prog.check_access(addr, len, attr)
    }

    /// Wakes the process from all of its asynchronous queues
    pub fn wake_from_async_queues(&mut self) {
        assert!(!self.program.is_null());
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

    /// Wakes the process from all of its synchronous queues
    pub fn wake_from_sync_queues(&mut self) {
        assert!(!self.program.is_null());
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

    /// Wakes the process from all of its queues
    pub fn wake_from_queues(&mut self) {
        self.wake_from_sync_queues();
        self.wake_from_async_queues();
    }
}

unsafe impl Send for Proc {}
unsafe impl Sync for Proc {}

#[cfg(test)]
mod test {
    use core::assert_matches;
    use crate::{print, println};

    use super::*;

    #[test_case]
    fn test_proc_state() {
        println!("Testing u32 to proc state");
        print!("Testing 0 ");
        assert_matches!(ProcState::try_from(0), Ok(ProcState::Free));
        println!("[ok]");
        print!("Testing 1 ");
        assert_matches!(ProcState::try_from(1), Ok(ProcState::Init));
        println!("[ok]");
        print!("Testing 2 ");
        assert_matches!(ProcState::try_from(2), Ok(ProcState::Scheduled));
        println!("[ok]");
        print!("Testing 3 ");
        assert_matches!(ProcState::try_from(3), Ok(ProcState::Running));
        println!("[ok]");
        print!("Testing 4 ");
        assert_matches!(ProcState::try_from(4), Ok(ProcState::BlockedQueue));
        println!("[ok]");
        print!("Testing 5 ");
        assert_matches!(ProcState::try_from(5), Ok(ProcState::BlockedQueues));
        println!("[ok]");
        print!("Testing 6 ");
        assert_matches!(ProcState::try_from(6), Ok(ProcState::BlockedIRQ));
        println!("[ok]");
        print!("Testing 7 ");
        assert_matches!(ProcState::try_from(7), Ok(ProcState::BlockedQueuesIRQ));
        println!("[ok]");
        print!("Testing 8 ");
        assert_matches!(ProcState::try_from(8), Ok(ProcState::BlockedEndpoint));
        println!("[ok]");
        print!("Testing 9 ");
        assert_matches!(ProcState::try_from(9), Ok(ProcState::Dead));
        println!("[ok]");
        print!("Testing 10 ");
        assert_matches!(ProcState::try_from(10), Err(10u32));
        println!("[ok]");
    }

    #[test_case]
    fn test_init() {
        println!("Testing proc init");
        let program = unsafe {
            Program::get_test_prog(0)
        };
        let mut stack: [u32; _] = [0; 64]; 
        let stack_addr = stack.as_mut_ptr() as u32;
        program.pid = 7;
        program.regions[0].virt_addr = stack_addr;
        program.regions[0].actual_len = 256;
        program.regions[0].len = 0x70023;
        program.flags = 1;
        program.entry = 0xc0dedec0;
        let stack = stack.as_mut_ptr();
        let mut proc = Proc::new();
        unsafe {
            proc.init(program, &[1, 2, 3, 4, 5]).unwrap();
        }
        print!("Testing r4 ");
        assert_eq!(proc.r4, 0);
        println!("[ok]");
        print!("Testing r5 ");
        assert_eq!(proc.r5, 0);
        println!("[ok]");
        print!("Testing r6 ");
        assert_eq!(proc.r6, 0);
        println!("[ok]");
        print!("Testing r7 ");
        assert_eq!(proc.r7, 0);
        println!("[ok]");
        print!("Testing r8 ");
        assert_eq!(proc.r8, 0);
        println!("[ok]");
        print!("Testing r9 ");
        assert_eq!(proc.r9, 0);
        println!("[ok]");
        print!("Testing r10 ");
        assert_eq!(proc.r10, 0);
        println!("[ok]");
        print!("Testing r11 ");
        assert_eq!(proc.r11, 0);
        println!("[ok]");
        print!("Testing psp ");
        assert_eq!(proc.psp, stack_addr + 256 - 4 * 13);
        println!("[ok]");
        let index = (256 - 4 * 13) / 4;
        unsafe {
            print!("Testing r0 ");
            assert_eq!(stack.add(index).read(), 0);
            println!("[ok]");
            print!("Testing r1 ");
            assert_eq!(stack.add(index + 1).read(), 0);
            println!("[ok]");
            print!("Testing r2 ");
            assert_eq!(stack.add(index + 2).read(), 0);
            println!("[ok]");
            print!("Testing r3 ");
            assert_eq!(stack.add(index + 3).read(), 0);
            println!("[ok]");
            print!("Testing r12 ");
            assert_eq!(stack.add(index + 4).read(), 0);
            println!("[ok]");
            print!("Testing lr ");
            assert_eq!(stack.add(index + 5).read(), 0xc0dedec0);
            println!("[ok]");
            print!("Testing pc ");
            assert_eq!(stack.add(index + 6).read(), 0xc0dedec0);
            println!("[ok]");
            print!("Testing xpsr ");
            assert_eq!(stack.add(index + 7).read(), 0x1000000);
            println!("[ok]");
            print!("Testing arg0 ");
            assert_eq!(stack.add(index + 8).read(), 1);
            println!("[ok]");
            print!("Testing arg1 ");
            assert_eq!(stack.add(index + 9).read(), 2);
            println!("[ok]");
            print!("Testing arg2 ");
            assert_eq!(stack.add(index + 10).read(), 3);
            println!("[ok]");
            print!("Testing arg3 ");
            assert_eq!(stack.add(index + 11).read(), 4);
            println!("[ok]");
            print!("Testing arg4 ");
            assert_eq!(stack.add(index + 12).read(), 5);
            println!("[ok]");
        }
        print!("Testing flags ");
        assert_eq!(proc.flags, 0x101);
        println!("[ok]");
    }
}
