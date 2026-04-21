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

use crate::{
    messages::MessageHeader,
    proc::{Proc, ProcState},
};

/// Message Queue trait  
/// Inicates the struct implementing this is a message queue
pub trait MessageQueue {}

/// Synchronous Message Queue
#[repr(C)]
pub struct SyncMessageQueue {
    /// Process at front of queue
    pub front: *mut Proc,
    /// Process at back of queue
    pub back: *mut Proc,
    /// Process waiting on queue to be non-empty
    pub blocked: *mut Proc,
}

/// Errors returned when completing queue operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueError {
    /// Queue is empty
    QueueEmpty,
    /// Provided buffer is too small
    BufferTooSmall,
    /// Queue is full
    QueueFull,
    /// Invalid queue was specified
    InvalidQueue(u32),
    /// Invalid notifier was specified
    InvalidNotifer(u32),
    /// Provided buffer is too large
    BufferTooLarge,
    /// Process died while in queue
    Died,
    /// Invalid memory access performed in queue
    InvalidMemoryAccess,
    /// Sender performed an invalid memory access
    SenderInvalidMemoryAccess,
    /// A queue mask with no queues was provided
    NoQueueMask,
}

/// Converts a `QueueError` to a `u32` so it can be returned to the calling process
impl From<QueueError> for u32 {
    fn from(value: QueueError) -> Self {
        match value {
            QueueError::InvalidQueue(_) => 0,
            QueueError::InvalidNotifer(_) => 1,
            QueueError::QueueEmpty => 2,
            QueueError::QueueFull => 3,
            QueueError::BufferTooSmall => 4,
            QueueError::BufferTooLarge => 5,
            QueueError::Died => 6,
            QueueError::InvalidMemoryAccess => 7,
            QueueError::SenderInvalidMemoryAccess => 8,
            QueueError::NoQueueMask => 9,
        }
    }
}

impl SyncMessageQueue {
    /// Creates a new synchronous message queue
    pub const fn new() -> Self {
        Self {
            front: ptr::null_mut(),
            back: ptr::null_mut(),
            blocked: ptr::null_mut(),
        }
    }

    /// Puts `proc` at the back of this queue  
    /// # Safety
    /// Assumes `proc` is a valid process
    unsafe fn push_back(&mut self, proc: *mut Proc) {
        assert!(!proc.is_null());
        if self.back.is_null() {
            self.front = proc;
            self.back = proc;
        } else {
            let back = unsafe { &mut *self.back };
            back.next = proc;
            self.back = proc;
        }
    }

    /// Removes the front process `front` from the front of the queue  
    /// Panics if `front` is not the front process
    fn pop_front<'a>(&mut self, front: &'a mut Proc) -> &'a mut Proc {
        assert!(self.front == front);
        self.front = front.next;
        if self.front.is_null() {
            self.back = ptr::null_mut();
        }
        front
    }

    /// Sends a message to the queue  
    /// `proc` is the process sending the message
    /// # Safety
    /// `proc` must be a valid process and blocked
    pub unsafe fn send(&mut self, proc: *mut Proc) {
        let proc = unsafe { &mut *proc };
        proc.next = ptr::null_mut();
        unsafe {
            self.push_back(proc);
        }
    }

    /// Reads the header of the process at the front of the queue  
    /// On success returns the header. On error returns a `QueueError` containing what went wrong
    pub fn read_header(&mut self) -> Result<MessageHeader, QueueError> {
        while !self.front.is_null() {
            let proc = unsafe { &mut *self.front };
            // skip and free dead processes
            if proc.get_state() == ProcState::Dead {
                proc.set_state(ProcState::Free);
                self.front = proc.next;
            } else {
                break;
            }
        }
        if self.front.is_null() {
            // queue is empty
            return Err(QueueError::QueueEmpty);
        }
        let front = unsafe { &mut *self.front };
        Ok(MessageHeader {
            pid: front.get_pid(),
            device_tag: ((front.get_device() as u32) << 16)
                | (front
                    .get_r1()
                    .map_err(|_| QueueError::SenderInvalidMemoryAccess)?
                    & 0xffff), // r1 contains message tag
            pin_mask: front.get_pin_mask(),
            len: front
                .get_r2()
                .map_err(|_| QueueError::SenderInvalidMemoryAccess)?, // r2 contains message length
        })
    }

    /// Reads the message content of the process at the front of the queue  
    /// `buffer_len` is the byte length of the reader's buffer  
    /// On success returns a slice to the data. On error returns a `QueueError` containing what went wrong
    pub fn read_data(&mut self, buffer_len: usize) -> Result<&[u8], QueueError> {
        if self.front.is_null() {
            return Err(QueueError::QueueEmpty);
        }
        let front = unsafe { &mut *self.front };
        if front.get_state() == ProcState::Dead {
            // if front died, remove from queue and free it
            front.set_state(ProcState::Free);
            self.pop_front(front);
            return Err(QueueError::Died);
        }
        let len = front
            .get_r2()
            .map_err(|_| QueueError::SenderInvalidMemoryAccess)?;
        // mask out send length
        let len = (len & 0xffff) as usize;
        let data = front
            .get_r3()
            .map_err(|_| QueueError::SenderInvalidMemoryAccess)?;
        if len > buffer_len {
            // buffer too small
            return Err(QueueError::BufferTooSmall);
        }
        // check access is valid
        // write message
        front
            .read_bytes(data, len)
            .map_err(|_| QueueError::SenderInvalidMemoryAccess)
    }

    /// Sends a reply to the process at the front of the queue  
    /// `msg` is the reply's tag  
    /// `buffer` is the reply's message content  
    /// On success, the front process is removed from the queue and returned  
    /// On error, if the front process is still valid, it is returned with its corresponding error
    /// or else a null pointer with the corresponding error is returned
    pub fn reply(
        &mut self,
        msg: u32,
        buffer: Option<&[u8]>,
    ) -> Result<*mut Proc, (*mut Proc, QueueError)> {
        if self.front.is_null() {
            return Err((ptr::null_mut(), QueueError::QueueEmpty));
        }
        let front = unsafe { &mut *self.front };
        if front.get_state() == ProcState::Dead {
            // if front died, remove from queue and free it
            front.set_state(ProcState::Free);
            self.pop_front(front);
            return Err((ptr::null_mut(), QueueError::Died));
        }
        let (len, data) = {
            let len = front.get_r2().map_err(|_| {
                self.pop_front(front);
                (front as *mut _, QueueError::SenderInvalidMemoryAccess)
            })?;
            let data = front.get_r3().map_err(|_| {
                self.pop_front(front);
                (front as *mut _, QueueError::SenderInvalidMemoryAccess)
            })?;
            (len, data)
        };
        // mask out reply len
        let len = ((len >> 16) & 0xffff) as usize;
        let reply_len = if let Some(buffer) = buffer {
            if buffer.len() > len {
                return Err((ptr::null_mut(), QueueError::BufferTooLarge));
            }
            if len > 0 {
                // check access is valid
                // write reply
                front.write_bytes(data, buffer).map_err(|_| {
                    self.pop_front(front);
                    (front as *mut _, QueueError::SenderInvalidMemoryAccess)
                })?;
            }
            buffer.len() as u32
        } else {
            0
        };
        _ = front.set_r0(msg);
        _ = front.set_r1(reply_len);
        self.pop_front(front);
        Ok(front)
    }

    /// Removes the next process in the queue from the front of the queue
    pub fn take(&mut self) -> Result<*mut Proc, QueueError> {
        if self.front.is_null() {
            return Err(QueueError::QueueEmpty);
        }
        let front = unsafe { &mut *self.front };
        if front.get_state() == ProcState::Dead {
            // if front died, remove from queue and free it
            front.set_state(ProcState::Free);
            self.pop_front(front);
            return Err(QueueError::Died);
        }
        Ok(self.pop_front(front))
    }

    /// Puts `proc` at the back of the queue
    /// # Safety
    /// `proc` must be a valid process
    pub unsafe fn put(&mut self, proc: *mut Proc) {
        assert!(!proc.is_null());
        unsafe {
            self.push_back(proc);
        }
    }

    /// Is the queue empty?
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.front.is_null()
    }
}

impl MessageQueue for SyncMessageQueue {}

unsafe impl Send for SyncMessageQueue {}
unsafe impl Sync for SyncMessageQueue {}

/// Asynchronous Message Queue
#[repr(C)]
pub struct AsyncMessageQueue {
    /// Buffer holding message data
    buffer: *mut MessageHeader,
    /// Buffer length in messages
    buffer_len: u32,
    /// Message length in bytes
    message_len: u32,
    /// Pointer to start of circular queue
    start: u32,
    /// Pointer to end of circular queue
    end: u32,
    /// Current queue length in messages
    len: u32,
    /// Process waiting on queue to be non-empty
    pub blocked: *mut Proc,
}

impl AsyncMessageQueue {
    /// Creates an asynchronous message queue  
    /// `buffer` is the buffer to hold all the message data  
    /// `buffer_len` is the length of the buffer in bytes  
    /// `message_len` is the length of a message in bytes  
    /// # Safety
    /// `buffer` must point to a memory region of length `buffer_len` * `message_len` bytes
    pub const unsafe fn new(buffer: *mut MessageHeader, buffer_len: u32, message_len: u32) -> Self {
        Self {
            buffer,
            buffer_len,
            message_len,
            start: 0,
            end: 0,
            len: 0,
            blocked: ptr::null_mut(),
        }
    }

    /// Queue length
    #[inline(always)]
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Queue capacity
    #[inline(always)]
    pub fn capacity(&self) -> usize {
        self.buffer_len as usize
    }

    /// Message size in bytes (is increased to make sure the header following it is aligned to a
    /// `MessageHeader` alignment)
    #[inline(always)]
    fn message_size(&self) -> usize {
        let align = mem::align_of::<MessageHeader>();
        let res = (self.message_len as usize + mem::size_of::<MessageHeader>() + align - 1) / align;
        res * align
    }


    /// Sends a message to the queue  
    /// `proc` is the process sending the message
    /// `tag` is the message tag being sent
    /// `data` is the data being sent
    /// On success returns nothing. On error returns a `QueueError` containing what went wrong
    pub fn send(&mut self, proc: &Proc, tag: u16, data: &[u8]) -> Result<(), QueueError> {
        if data.len() > self.message_len as usize {
            // buffer too large
            return Err(QueueError::BufferTooLarge);
        }
        // check there is space in the queue
        if self.len < self.buffer_len {
            // calculate the header offset
            let index = (self.end as usize * self.message_size()) / mem::size_of::<MessageHeader>();
            unsafe {
                // write header
                self.buffer.add(index).write(MessageHeader {
                    pid: proc.get_pid(),
                    pin_mask: proc.get_pin_mask(),
                    device_tag: ((proc.get_device() as u32) << 16) | (tag as u32),
                    len: data.len() as u32,
                });
            }
            // write body
            let body: &mut [u8] = unsafe {
                let addr = self.buffer.add(index).addr() + mem::size_of::<MessageHeader>();
                slice::from_raw_parts_mut(ptr::with_exposed_provenance_mut(addr), data.len())
            };
            body.copy_from_slice(data);
            // update queue length and pointers
            self.len += 1;
            self.end = (self.end + 1) % self.buffer_len;
            Ok(())
        } else {
            // queue full
            Err(QueueError::QueueFull)
        }
    }

    /// Reads the header of the message at the front of the queue  
    /// On success returns the header. On error returns a `QueueError` containing what went wrong
    pub fn read_header(&self) -> Result<MessageHeader, QueueError> {
        if self.len > 0 {
            // calculate header offset
            let index = (self.start as usize * self.message_size()) / mem::size_of::<MessageHeader>();
            // read header
            let res = unsafe { self.buffer.add(index).read() };
            Ok(res)
        } else {
            // queue empty
            Err(QueueError::QueueEmpty)
        }
    }

    /// Reads the message content of the message at the front of the queue  
    /// `buffer_len` is the byte length of the reader's buffer  
    /// On success returns a slice to the data. On error returns a `QueueError` containing what went wrong
    pub fn read_data(&mut self, buffer_len: usize) -> Result<&[u8], QueueError> {
        if self.len > 0 {
            let index = self.start as usize * self.message_size() / mem::size_of::<MessageHeader>();
            let header = unsafe { self.buffer.add(index).read() };
            // check buffer is large enough
            if buffer_len < header.len as usize {
                Err(QueueError::BufferTooSmall)
            } else {
                // read data
                let body: &[u8] = unsafe {
                    let addr = self.buffer.add(index).addr() + mem::size_of::<MessageHeader>();
                    slice::from_raw_parts(ptr::with_exposed_provenance(addr), header.len as usize)
                };
                // update length and pointers
                self.start = (self.start + 1) % self.buffer_len;
                self.len -= 1;
                Ok(body)
            }
        } else {
            // queue empty
            Err(QueueError::QueueEmpty)
        }
    }

    /// Is the queue empty?
    #[inline(always)]
    pub fn is_empty(&mut self) -> bool {
        self.len == 0
    }
}

impl MessageQueue for AsyncMessageQueue {}

unsafe impl Send for AsyncMessageQueue {}
unsafe impl Sync for AsyncMessageQueue {}

#[cfg(test)]
mod test {
    use core::{assert_matches, mem::MaybeUninit};

    use crate::{print, println, program::Program};

    use super::*;

    #[test_case]
    fn test_sync() {
        println!("Testing synchronous queues");

        let mut queue = unsafe { SyncMessageQueue::new() };
        let mut stack: [u32; 64] = [0; _];
        stack[0] = u32::from_le_bytes([1, 2, 3, 4]);
        let stack_addr = stack.as_mut_ptr() as u32;

        let prog = unsafe { Program::get_test_prog(0) };
        prog.pid = 1;
        prog.regions[0].virt_addr = stack_addr;
        prog.regions[0].actual_len = 256;
        prog.regions[0].len = 0x70023;

        let mut proc = Proc::new();
        unsafe {
            proc.init(prog, &[]).unwrap();
        }

        print!("Testing send message ");
        proc.set_r1(15).unwrap();
        proc.set_r2(4).unwrap();
        proc.set_r3(stack_addr).unwrap();
        unsafe {
            queue.send(&raw mut proc);
        }
        assert_eq!(queue.front, &raw mut proc);
        assert_eq!(queue.back, &raw mut proc);
        println!("[ok]");

        print!("Testing read header ");
        let header = queue.read_header().unwrap();
        assert_eq!(header.pid, proc.get_pid());
        assert_eq!(header.len, 4);
        assert_eq!(header.device_tag & 0xffff, 15);
        println!("[ok]");

        print!("Testing read body ");
        let buffer = queue.read_data(4).unwrap();
        assert_eq!(buffer.as_ptr() as u32, stack_addr);
        assert_eq!(buffer.len(), 4);
        assert_eq!(buffer[0], 1);
        assert_eq!(buffer[1], 2);
        assert_eq!(buffer[2], 3);
        assert_eq!(buffer[3], 4);
        println!("[ok]");

        print!("Testing reply ");
        let proc_ptr = queue.reply(20, None).unwrap();
        assert_eq!(proc_ptr, &raw mut proc);
        assert_eq!(queue.front, ptr::null_mut());
        assert_eq!(queue.back, ptr::null_mut());
        assert_eq!(proc.get_r0().unwrap(), 20);
        println!("[ok]");

        print!("Testing read empty queue ");
        assert!(queue.read_header().is_err());
        assert_matches!(queue.read_header().err().unwrap(), QueueError::QueueEmpty);
        println!("[ok]");

        print!("Testing read empty queue data ");
        assert!(queue.read_data(4).is_err());
        assert_matches!(queue.read_data(4).err().unwrap(), QueueError::QueueEmpty);
        println!("[ok]");
    }

    #[test_case]
    fn test_async() {
        println!("Testing asynchronous queues");
        struct Message {
            _header: MessageHeader,
            _data: [u8; 200],
        }
        static mut MESSAGES: [MaybeUninit<Message>; 10] = [const { MaybeUninit::uninit() }; 10];

        let mut queue = unsafe {
            AsyncMessageQueue::new(
                &raw mut MESSAGES as *mut MessageHeader,
                10,
                mem::size_of::<Message>() as u32,
            )
        };

        print!("Checking capacity ");
        assert_eq!(queue.capacity(), 10);
        println!("[ok]");
        print!("Checking len ");
        assert_eq!(queue.len(), 0);
        assert_eq!(queue.len(), queue.len as usize);
        println!("[ok]");

        let mut stack: [u32; 64] = [0; _];
        let stack_addr = stack.as_mut_ptr() as u32;

        let prog = unsafe { Program::get_test_prog(0) };
        prog.pid = 1;
        prog.regions[0].virt_addr = stack_addr;
        prog.regions[0].actual_len = 256;
        prog.regions[0].len = 0x70023;

        let mut proc = Proc::new();
        unsafe {
            proc.init(prog, &[]).unwrap();
        }

        print!("Testing send message ");

        let msg = [2, 4, 6, 8, 10];
        queue.send(&proc, 5, &msg).unwrap();
        assert_eq!(queue.len, 1);
        assert_eq!(queue.end, 1);
        assert_eq!(queue.start, 0);
        println!("[ok]");

        print!("Testing read header ");
        let header = queue.read_header().unwrap();
        assert_eq!(header.pid, proc.get_pid());
        assert_eq!(header.len, 5);
        assert_eq!(header.device_tag & 0xffff, 5);
        println!("[ok]");

        print!("Testing read body ");
        let buffer = queue.read_data(5).unwrap();
        assert_eq!(buffer.len(), 5);
        assert_eq!(buffer[0], 2);
        assert_eq!(buffer[1], 4);
        assert_eq!(buffer[2], 6);
        assert_eq!(buffer[3], 8);
        assert_eq!(buffer[4], 10);
        assert_eq!(queue.start, 1);
        assert_eq!(queue.end, 1);
        assert_eq!(queue.len, 0);
        println!("[ok]");

        print!("Testing read empty queue ");
        assert!(queue.read_header().is_err());
        assert_matches!(queue.read_header().err().unwrap(), QueueError::QueueEmpty);
        println!("[ok]");

        print!("Testing read empty queue data ");
        assert!(queue.read_data(1).is_err());
        assert_matches!(queue.read_data(1).err().unwrap(), QueueError::QueueEmpty);
        println!("[ok]");
    }
}
