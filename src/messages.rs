use core::slice;

use crate::proc::Proc;

// Based on https://wiki.osdev.org/Message_Passing_Tutorial accessed 2/02/2026
// Based on https://wiki.osdev.org/Message_Passing accessed 2/02/2026
// Based on https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.getting_started/topic/s1_msg_Message_handling.html accessed 2/02/2026
// Based on https://sel4.systems/Info/Docs/seL4-manual-latest.pdf accessed 2/02/2026
// Based on https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/02-Queues-mutexes-and-semaphores/01-Queues accessed 2/02/2026


pub const MESSAGE_DIRECT_LEN: usize = 4;

pub const MAX_ASYNC_MESSAGE_LEN: usize = 128;

pub union MessageBody {
    pub direct: [u8; MESSAGE_DIRECT_LEN],
    pub indirect: *const u8
}

pub union MessageSender {
    pub proc: *mut Proc,
    pub pid: u32
}

pub struct MessageHeader {
    pub sender: MessageSender,
    pub tag: u32,
    pub len: u32,
}

#[repr(C)]
pub struct Message {
    pub header: MessageHeader,
    pub body: MessageBody
}

impl Message {
    pub fn direct(sender: *mut Proc, tag: u32, r3: u32, len: u32) -> Option<Self> {
        if len > 8 {
            None
        } else {
            let mut direct = [0; MESSAGE_DIRECT_LEN];
            for (i, byte) in r3.to_be_bytes().iter().enumerate() {
                if i >= len as usize {
                    break;
                }
                direct[i] = *byte;
            }
            Some(Message { 
                header: MessageHeader { 
                    sender: MessageSender{
                        proc: sender
                    }, 
                    tag, 
                    len 
                },
                body: MessageBody {
                    direct
                }
            })
        }
    }

    pub fn indirect(sender: *mut Proc, tag: u32, data: &[u8]) -> Self {
        if data.len() < MESSAGE_DIRECT_LEN as usize {
            let mut direct = [0; MESSAGE_DIRECT_LEN as usize];
            direct.copy_from_slice(data);
            Message { 
                header: MessageHeader { 
                    sender: MessageSender {
                        proc: sender
                    }, 
                    tag, 
                    len: data.len() as u32 
                }, 
                body: MessageBody {
                    direct
                }
            }
        } else {
            Message { 
                header: MessageHeader { 
                    sender: MessageSender {
                        proc: sender
                    }, 
                    tag, 
                    len: data.len() as u32 
                }, 
                body: MessageBody {
                    indirect: data.as_ptr()
                }
            }
        }
    }

    pub fn clone_header(&self) -> MessageHeader {
        let proc = unsafe {
            self.header.sender.proc
        };
        MessageHeader { 
            sender: MessageSender {
                proc

            }, 
            tag: self.header.tag, 
            len: self.header.len 
        }
    }
}

pub struct AsyncMessage {
    pub header: MessageHeader,
    pub body: [u8; MAX_ASYNC_MESSAGE_LEN]
}

impl AsyncMessage {
    pub fn new(pid: u32, tag: u32, data: &[u8]) -> Option<Self> {
        if data.len() > MAX_ASYNC_MESSAGE_LEN {
            None
        } else {
            let mut body = [0; MAX_ASYNC_MESSAGE_LEN];
            body[..data.len()].copy_from_slice(data);
            Some(Self {
                header: MessageHeader { 
                    sender: MessageSender {
                        pid
                    }, 
                    tag, 
                    len: data.len() as u32 
                },
                body
            })
        }
    }

    pub fn clone_header(&self) -> MessageHeader {
        let pid = unsafe {
            self.header.sender.pid
        };
        MessageHeader { 
            sender: MessageSender {
                pid
            }, 
            tag: self.header.tag, 
            len: self.header.len 
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    use crate::print;
    use crate::println;
    use crate::proc::Proc;

    #[test_case]
    fn test_message_create() {
        let mut proc = Proc::new();
        proc.pid = 1;
        print!("Testing direct message creation ");
        let msg = Message::direct(&raw mut proc, 0, 8, 4); 
        assert!(msg.is_some());
        let msg = msg.unwrap();
        unsafe {
            assert_eq!(msg.header.sender.proc, &raw mut proc);
        }
        assert_eq!(msg.header.len, 4);
        assert_eq!(msg.header.tag, 0);
        unsafe {
            assert_eq!(msg.body.direct.len(), 4);
            // should be 0, 0, 0, 8 bytes for big endian
            assert_eq!(msg.body.direct[0], 0);
            assert_eq!(msg.body.direct[1], 0);
            assert_eq!(msg.body.direct[2], 0);
            assert_eq!(msg.body.direct[3], 8);
        }
        println!("[ok]");
        print!("Testing direct message creation (fails) ");
        assert!(Message::direct(&raw mut proc, 0, 8, 20).is_none());
        println!("[ok]");
    }
    
    #[test_case]
    fn test_large_message_create() {
        let mut proc = Proc::new();
        proc.pid = 1;
        print!("Testing indirect message creation ");
        let data = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100];
        let msg = Message::indirect(&raw mut proc, 0, &data);
        unsafe {
            assert_eq!(msg.header.sender.proc, &raw mut proc);
        }
        assert_eq!(msg.header.len, 10);
        assert_eq!(msg.header.tag, 0);
        let body = unsafe {
            slice::from_raw_parts(msg.body.indirect, 10)
        };
        assert_eq!(body[0], 10);
        assert_eq!(body[1], 20);
        assert_eq!(body[2], 30);
        assert_eq!(body[3], 40);
        assert_eq!(body[4], 50);
        assert_eq!(body[5], 60);
        assert_eq!(body[6], 70);
        assert_eq!(body[7], 80);
        assert_eq!(body[8], 90);
        assert_eq!(body[9], 100);
        println!("[ok]");
    }
    
    #[test_case]
    fn test_async_message_create() {
        let mut proc = Proc::new();
        proc.pid = 1;
        print!("Testing async message creation ");
        let msg = AsyncMessage::new(proc.pid, 0, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]); 
        assert!(msg.is_some());
        let msg = msg.unwrap();
        unsafe {
            assert_eq!(msg.header.sender.pid, proc.pid);
        }
        assert_eq!(msg.header.len, 10);
        assert_eq!(msg.header.tag, 0);
        assert_eq!(msg.body.len(), MAX_ASYNC_MESSAGE_LEN);
        assert_eq!(msg.body[0], 1);
        assert_eq!(msg.body[1], 2);
        assert_eq!(msg.body[2], 3);
        assert_eq!(msg.body[3], 4);
        assert_eq!(msg.body[4], 5);
        assert_eq!(msg.body[5], 6);
        assert_eq!(msg.body[6], 7);
        assert_eq!(msg.body[7], 8);
        assert_eq!(msg.body[8], 9);
        assert_eq!(msg.body[9], 10);
        println!("[ok]");
        print!("Testing async message creation (fails) ");
        assert!(AsyncMessage::new(proc.pid, 0, &[0; MAX_ASYNC_MESSAGE_LEN + 1]).is_none());
        println!("[ok]");
    }
}
