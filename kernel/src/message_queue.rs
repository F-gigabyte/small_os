use core::{mem, ptr, slice};

use crate::{messages::MessageHeader, proc::{Proc, ProcState}};

pub trait MessageQueue {}

#[repr(C)]
pub struct SyncMessageQueue {
    pub front: *mut Proc,
    pub back: *mut Proc,
    pub blocked: *mut Proc,
}


#[derive(Debug, Clone, Copy)]
pub enum QueueError {
    QueueEmpty,
    BufferTooSmall,
    QueueFull,
    InvalidQueue(u32),
    BufferTooLarge,
    Died,
    InvalidMemoryAccess,
    SenderInvalidMemoryAccess
}

impl From<QueueError> for u32 {
    fn from(value: QueueError) -> Self {
        match value {
            QueueError::InvalidQueue(_) => 0,
            QueueError::QueueEmpty => 1,
            QueueError::QueueFull => 2,
            QueueError::BufferTooSmall => 3,
            QueueError::BufferTooLarge => 4,
            QueueError::Died => 5,
            QueueError::InvalidMemoryAccess => 6,
            QueueError::SenderInvalidMemoryAccess => 7,
        }
    }
}

impl SyncMessageQueue {
    pub const unsafe fn new() -> Self {
        Self {
            front: ptr::null_mut(),
            back: ptr::null_mut(),
            blocked: ptr::null_mut()
        }
    }
    
    /// SAFETY
    /// `proc` must be a valid process and blocked
    pub unsafe fn send(&mut self, proc: *mut Proc) {
        let proc = unsafe {
            &mut *proc
        };
        proc.next = ptr::null_mut();
        if self.back.is_null() {
            self.front = proc;
            self.back = proc;
        } else {
            let back = unsafe {
                &mut *self.back
            };
            back.next = proc;
        }
    }

    pub fn read_header(&mut self) -> Result<MessageHeader, QueueError> {
        while !self.front.is_null() {
            let proc = unsafe {
                &mut *self.front
            };
            // skip and free dead processes
            if proc.get_state() == ProcState::Dead {
                proc.set_state(ProcState::Free);
                self.front = proc.next;
            } else {
                break;
            }
        }
        if self.front.is_null() {
            return Err(QueueError::QueueEmpty);
        }
        let front = unsafe {
            &mut *self.front
        };
        Ok(MessageHeader { 
            pid: front.pid, 
            tag: front.get_r1().map_err(|_| QueueError::SenderInvalidMemoryAccess)?, // r1 contains message tag 
            len: front.get_r2().map_err(|_| QueueError::SenderInvalidMemoryAccess)? // r2 contains message length
        })
    }

    pub fn read_data(&mut self, buffer_len: usize) -> Result<&[u8], QueueError> {
        if self.front.is_null() {
            return Err(QueueError::QueueEmpty);
        }
        let front = unsafe {
            &mut *self.front
        };
        if front.get_state() == ProcState::Dead {
            // if front died, remove from queue and free it
            front.set_state(ProcState::Free);
            self.front = front.next;
            if self.front.is_null() {
                self.back = ptr::null_mut();
            }
            return Err(QueueError::Died);
        }
        let len = front.get_r2().map_err(|_| QueueError::SenderInvalidMemoryAccess)?;
        // mask out send length
        let len = (len & 0xffff) as usize;
        let data = front.get_r3().map_err(|_| QueueError::SenderInvalidMemoryAccess)?;
        if len > buffer_len {
            return Err(QueueError::BufferTooSmall);
        }
        // check access is valid
        // write message
        front.read_bytes(data, buffer_len).map_err(|_| QueueError::SenderInvalidMemoryAccess)
    }
    
    pub fn reply(&mut self, msg: u32, buffer: Option<&[u8]>) -> Result<*mut Proc, QueueError> {
        if self.front.is_null() {
            return Err(QueueError::QueueEmpty);
        }
        let front = unsafe {
            &mut *self.front
        };
        if front.get_state() == ProcState::Dead {
            // if front died, remove from queue and free it
            front.set_state(ProcState::Free);
            self.front = front.next;
            if self.front.is_null() {
                self.back = ptr::null_mut();
            }
            return Err(QueueError::Died);
        }
        let (len, data) = {
            let len = front.get_r2().map_err(|_| QueueError::SenderInvalidMemoryAccess)?;
            let data = front.get_r3().map_err(|_| QueueError::SenderInvalidMemoryAccess)?;
            (len, data)
        };
        // mask out reply len
        let len = ((len >> 16) & 0xffff) as usize;
        let reply_len = if let Some(buffer) = buffer {
            if buffer.len() > len {
                return Err(QueueError::BufferTooLarge);
            }
            if len > 0 {
                // check access is valid
                // write reply
                front.write_bytes(data, buffer).map_err(|_| QueueError::SenderInvalidMemoryAccess)?;
            }
            buffer.len() as u32
        } else {
            0
        };
        _ = front.set_r0(msg);
        _ = front.set_r1(reply_len);
        self.front = front.next;
        if self.front.is_null() {
            self.back = ptr::null_mut();
        }
        Ok(front)
    }
}

impl MessageQueue for SyncMessageQueue {}

unsafe impl Send for SyncMessageQueue {}
unsafe impl Sync for SyncMessageQueue {}


#[repr(C)]
pub struct AsyncMessageQueue {
    buffer: *mut MessageHeader,
    buffer_len: u32,
    message_len: u32,
    start: u32,
    end: u32,
    len: u32,
    pub blocked: *mut Proc,
}

impl AsyncMessageQueue {
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
    

    #[inline(always)]
    pub fn len(&self) -> usize {
        self.len as usize
    }

    #[inline(always)]
    pub fn capacity(&self) -> usize {
        self.buffer_len as usize
    }

    #[inline(always)]
    fn message_size(&self) -> usize {
        self.message_len as usize + mem::size_of::<MessageHeader>()
    }
    
    pub fn send(&mut self, mut header: MessageHeader, data: &[u8]) -> Result<(), QueueError> {
        header.len = data.len() as u32;
        if header.len > self.message_len {
            return Err(QueueError::BufferTooLarge);
        }
        if self.len < self.buffer_len {
            let index = self.end as usize * self.message_size() / mem::size_of::<MessageHeader>();
            unsafe {
                self.buffer.add(index).write(header);
            }
            let body: &mut [u8] = unsafe { 
                let addr = self.buffer.add(index).addr() + mem::size_of::<MessageHeader>();
                slice::from_raw_parts_mut(ptr::with_exposed_provenance_mut(addr), data.len())
            };
            body.copy_from_slice(data);
            self.len += 1;
            self.end = (self.end + 1) % self.buffer_len;
            Ok(())
        } else {
            Err(QueueError::QueueFull)
        }
    }

    pub fn read_header(&self) -> Result<MessageHeader, QueueError> {
        if self.len > 0 {
            let index = self.start as usize * self.message_size() / mem::size_of::<MessageHeader>();
            let res = unsafe {
                self.buffer.add(index).read()
            };
            Ok(res)
        } else {
            Err(QueueError::QueueEmpty)
        }
    }

    pub fn read_data(&mut self, buffer_len: usize) -> Result<&[u8], QueueError> {
        if self.len > 0 {
            let index = self.start as usize * self.message_size() / mem::size_of::<MessageHeader>();
            let header = unsafe {
                self.buffer.add(index).read()
            };
            if buffer_len < header.len as usize {
                Err(QueueError::BufferTooSmall)
            } else {
                let body: &[u8] = unsafe {
                    let addr = self.buffer.add(index).addr() + mem::size_of::<MessageHeader>();
                    slice::from_raw_parts(ptr::with_exposed_provenance(addr), header.len as usize)
                };
                self.start = (self.start + 1) % self.buffer_len;
                self.len -= 1;
                Ok(body)
            }
        } else {
            Err(QueueError::QueueEmpty)
        }
    }
}

impl MessageQueue for AsyncMessageQueue {}

unsafe impl Send for AsyncMessageQueue {}
unsafe impl Sync for AsyncMessageQueue {}


#[cfg(test)]
mod test {
    use core::assert_matches::assert_matches;

    use crate::{print, println};

    use super::*;

    #[test_case]
    fn test_sync() {
        println!("Testing synchronous queues");

        let mut queue = unsafe {
            SyncMessageQueue::new()
        };
        let stack: [u32; 8] = [0; _];

        let mut proc = Proc::new();
        unsafe {
            proc.init(
                1, 
                0, 
                stack.as_ptr().add(stack.len()) as u32, 
                0, 
                None, 
                None, 
                None, 
                None
            );
        }

        print!("Testing send message ");
        proc.set_r1(15).unwrap();
        proc.set_r2(4).unwrap();
        proc.set_r3(10).unwrap();
        unsafe {
            queue.send(&raw mut proc);
        }
        assert_eq!(queue.front, &raw mut proc);
        assert_eq!(queue.back, &raw mut proc);
        println!("[ok]");

        print!("Testing read header ");
        let header = queue.read_header().unwrap();
        assert_eq!(header.pid, proc.pid);
        assert_eq!(header.len, 4);
        assert_eq!(header.tag, 15);
        println!("[ok]");
        
        print!("Testing read body ");
        let mut buffer = [0; 4];
        let read = queue.read_data(&mut buffer).unwrap();
        assert_eq!(read, 4);
        assert_eq!(buffer[0], 0);
        assert_eq!(buffer[1], 0);
        assert_eq!(buffer[2], 0);
        assert_eq!(buffer[3], 10);
        println!("[ok]");

        print!("Testing reply ");
        let proc_ptr = queue.reply(20).unwrap();
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
        assert!(queue.read_data(&mut buffer).is_err());
        assert_matches!(queue.read_data(&mut buffer).err().unwrap(), QueueError::QueueEmpty);
        println!("[ok]");
    }

    #[test_case]
    fn test_async() {
        println!("Testing asynchronous queues");
        static mut MESSAGES: [MaybeUninit<Message>; 10] = [const { MaybeUninit::uninit() }; 10];

        let mut queue = unsafe {
            AsyncMessageQueue::new(&mut * &raw mut MESSAGES)
        };

        print!("Checking capacity ");
        assert_eq!(queue.capacity(), 10);
        println!("[ok]");
        print!("Checking len ");
        assert_eq!(queue.len(), 0);
        assert_eq!(queue.len(), queue.len as usize);
        println!("[ok]");
    
        let mut proc = Proc::new();
        proc.pid = 1;

        print!("Testing send message ");
        let msg = Message::new(proc.pid, 15, &[2, 4, 6, 8, 10]).unwrap();
        queue.send(msg).unwrap();
        assert_eq!(queue.len, 1);
        assert_eq!(queue.end, 1);
        assert_eq!(queue.start, 0);
        println!("[ok]");

        print!("Testing read header ");
        let header = queue.read_header().unwrap();
        assert_eq!(header.pid, proc.pid);
        assert_eq!(header.len, 5);
        assert_eq!(header.tag, 15);
        println!("[ok]");
        
        print!("Testing read body ");
        let mut buffer = [0; 5];
        let read = queue.read_data(&mut buffer).unwrap();
        assert_eq!(read, 5);
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
        assert!(queue.read_data(&mut buffer).is_err());
        assert_matches!(queue.read_data(&mut buffer).err().unwrap(), QueueError::QueueEmpty);
        println!("[ok]");
    }
}
