use core::{mem::{self, MaybeUninit}, ptr, slice};

use crate::{messages::{AsyncMessage, MESSAGE_DIRECT_LEN, Message, MessageHeader}, proc::Proc};

pub trait MessageQueue {
    type MessageType;
    fn send(&mut self, msg: Self::MessageType) -> Result<(), QueueError>;
    fn read_header(&self) -> Result<MessageHeader, QueueError>;
    fn read_data(&mut self, buffer: &mut [u8]) -> Result<usize, QueueError>;
}

#[repr(C)]
pub struct SyncMessageQueue {
    buffer: &'static mut [MaybeUninit<Message>],
    start: u32,
    end: u32,
    len: u32,
    pub blocked: *mut Proc,
}


#[derive(Debug, Clone, Copy)]
pub enum QueueError {
    QueueEmpty,
    BufferTooSmall,
    QueueFull,
    InvalidQueue(u32),
    BufferTooLarge
}

impl From<QueueError> for u32 {
    fn from(value: QueueError) -> Self {
        match value {
            QueueError::InvalidQueue(_) => 0,
            QueueError::QueueEmpty => 1,
            QueueError::QueueFull => 2,
            QueueError::BufferTooSmall => 3,
            QueueError::BufferTooLarge => 4
        }
    }
}

impl SyncMessageQueue {
    pub const unsafe fn new(buffer: &'static mut [MaybeUninit<Message>]) -> Self {
        Self {
            buffer,
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
    pub const fn capacity(&self) -> usize {
        self.buffer.len()
    }

    pub fn reply(&mut self) -> Result<*mut Proc, QueueError> {
        if self.len > 0 {
            let msg = unsafe {
                mem::replace(&mut self.buffer[self.start as usize], MaybeUninit::uninit()).assume_init()
            };
            self.len -= 1;
            self.start = (self.start + 1) % (self.buffer.len() as u32);
            unsafe {
                Ok(msg.header.sender.proc)
            }
        } else {
            Err(QueueError::QueueEmpty)
        }
    }
}

impl MessageQueue for SyncMessageQueue {
    type MessageType = Message;

    fn send(&mut self, msg: Self::MessageType) -> Result<(), QueueError> {
        if (self.len as usize) < self.buffer.len() {
            self.buffer[self.end as usize].write(msg);
            self.len += 1;
            self.end = (self.end + 1) % self.buffer.len() as u32;
            Ok(())
        } else {
            Err(QueueError::QueueFull)
        }
    }

    fn read_header(&self) -> Result<MessageHeader, QueueError> {
        if self.len > 0 {
            let res = unsafe {
                self.buffer[self.start as usize].assume_init_ref()
            };
            Ok(res.clone_header())
        } else {
            Err(QueueError::QueueEmpty)
        }
    }

    fn read_data(&mut self, buffer: &mut [u8]) -> Result<usize, QueueError> {
        if self.len > 0 {
            let msg = unsafe {
                self.buffer[self.start as usize].assume_init_ref()
            };
            if buffer.len() < msg.header.len as usize {
                Err(QueueError::BufferTooSmall)
            } else {
                let data = if msg.header.len as usize <= MESSAGE_DIRECT_LEN {
                    unsafe {
                        & msg.body.direct[..msg.header.len as usize]
                    }
                } else {
                    unsafe {
                        slice::from_raw_parts(msg.body.indirect, msg.header.len as usize)
                    }
                };
                let len = data.len();
                buffer[..len].copy_from_slice(data);
                Ok(len)
            }
        } else {
            Err(QueueError::QueueEmpty)
        }
    }
}

unsafe impl Send for SyncMessageQueue {}
unsafe impl Sync for SyncMessageQueue {}


#[repr(C)]
pub struct AsyncMessageQueue {
    buffer: &'static mut [MaybeUninit<AsyncMessage>],
    start: u32,
    end: u32,
    len: u32,
    pub blocked: *mut Proc,
}

impl AsyncMessageQueue {
    pub const unsafe fn new(buffer: &'static mut [MaybeUninit<AsyncMessage>]) -> Self {
        Self {
            buffer,
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
    pub const fn capacity(&self) -> usize {
        self.buffer.len()
    }
}

impl MessageQueue for AsyncMessageQueue {
    type MessageType = AsyncMessage;
    
    fn send(&mut self, msg: Self::MessageType) -> Result<(), QueueError> {
        if (self.len as usize) < self.buffer.len() {
            self.buffer[self.end as usize].write(msg);
            self.len += 1;
            self.end = (self.end + 1) % self.buffer.len() as u32;
            Ok(())
        } else {
            Err(QueueError::QueueFull)
        }
    }

    fn read_header(&self) -> Result<MessageHeader, QueueError> {
        if self.len > 0 {
            let res = unsafe {
                self.buffer[self.start as usize].assume_init_ref()
            };
            Ok(res.clone_header())
        } else {
            Err(QueueError::QueueEmpty)
        }
    }

    fn read_data(&mut self, buffer: &mut [u8]) -> Result<usize, QueueError> {
        if self.len > 0 {
            let msg = unsafe {
                self.buffer[self.start as usize].assume_init_ref()
            };
            if buffer.len() < msg.header.len as usize {
                Err(QueueError::BufferTooSmall)
            } else {
                let data = msg.body;
                let len = msg.header.len as usize;
                buffer[..len].copy_from_slice(&data[..len]);
                unsafe {
                    self.buffer[self.start as usize].assume_init_drop();
                }
                self.start = (self.start + 1) % self.buffer.len() as u32;
                self.len -= 1;
                Ok(len)
            }
        } else {
            Err(QueueError::QueueEmpty)
        }
    }
}

unsafe impl Send for AsyncMessageQueue {}
unsafe impl Sync for AsyncMessageQueue {}


pub struct Endpoints<QUEUE: MessageQueue + 'static> {
    endpoints: &'static [*mut QUEUE],
}

impl<QUEUE: MessageQueue + 'static> Endpoints<QUEUE> {
    pub const fn len(&self) -> usize {
        self.endpoints.len()
    }

    pub const fn as_ptr(&self) -> *const *mut QUEUE {
        self.endpoints.as_ptr()
    }
}

unsafe impl<QUEUE: MessageQueue + 'static> Send for Endpoints<QUEUE> {}
unsafe impl<QUEUE: MessageQueue + 'static> Sync for Endpoints<QUEUE> {}

static mut QUEUE1_MESSAGES: [MaybeUninit<Message>; 10] = [const { MaybeUninit::uninit() }; 10];
static mut QUEUE2_MESSAGES: [MaybeUninit<Message>; 10] = [const { MaybeUninit::uninit() }; 10];

pub static mut SYNC_QUEUES1: [SyncMessageQueue; 1] = unsafe { [SyncMessageQueue::new(&mut *&raw mut QUEUE1_MESSAGES)] };
pub static mut SYNC_QUEUES2: [SyncMessageQueue; 1] = unsafe { [SyncMessageQueue::new(&mut *&raw mut QUEUE2_MESSAGES)] };

pub static SYNC_ENDPOINTS1: Endpoints<SyncMessageQueue> = Endpoints { endpoints: &[unsafe { (* &raw mut SYNC_QUEUES2).as_mut_ptr().add(0) }] };

pub static SYNC_ENDPOINTS2: Endpoints<SyncMessageQueue> = Endpoints { endpoints: & [unsafe { (* &raw mut SYNC_QUEUES1).as_mut_ptr().add(0) }] };


#[cfg(test)]
mod test {
    use core::assert_matches::assert_matches;

    use crate::{print, println};

    use super::*;

    #[test_case]
    fn test_sync() {
        println!("Testing synchronous queues");
        static mut MESSAGES: [MaybeUninit<Message>; 10] = [const { MaybeUninit::uninit() }; 10];

        let mut queue = unsafe {
            SyncMessageQueue::new(&mut * &raw mut MESSAGES)
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
        let small_msg = Message::direct(&raw mut proc, 15, 10, 4).unwrap();
        queue.send(small_msg).unwrap();
        assert_eq!(queue.len, 1);
        assert_eq!(queue.end, 1);
        assert_eq!(queue.start, 0);
        println!("[ok]");

        print!("Testing read header ");
        let header = queue.read_header().unwrap();
        unsafe {
            assert_eq!(header.sender.proc, &raw mut proc);
        }
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
        let proc_ptr = queue.reply().unwrap();
        assert_eq!(proc_ptr, &raw mut proc);
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

    #[test_case]
    fn test_async() {
        println!("Testing asynchronous queues");
        static mut MESSAGES: [MaybeUninit<AsyncMessage>; 10] = [const { MaybeUninit::uninit() }; 10];

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
        let msg = AsyncMessage::new(proc.pid, 15, &[2, 4, 6, 8, 10]).unwrap();
        queue.send(msg).unwrap();
        assert_eq!(queue.len, 1);
        assert_eq!(queue.end, 1);
        assert_eq!(queue.start, 0);
        println!("[ok]");

        print!("Testing read header ");
        let header = queue.read_header().unwrap();
        unsafe {
            assert_eq!(header.sender.pid, proc.pid);
        }
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
