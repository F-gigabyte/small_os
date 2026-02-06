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

#[derive(Debug, Clone)]
pub struct MessageHeader {
    pub sender: *mut Proc,
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
                direct[i + 4] = *byte;
            }
            Some(Message { 
                header: MessageHeader { 
                    sender, 
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
                    sender, 
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
                    sender, 
                    tag, 
                    len: data.len() as u32 
                }, 
                body: MessageBody {
                    indirect: data.as_ptr()
                }
            }
        }
    }
}

pub struct AsyncMessage {
    pub header: MessageHeader,
    pub body: [u8; MESSAGE_DIRECT_LEN]
}

impl AsyncMessage {
    pub fn new(sender: *mut Proc, tag: u32, data: &[u8]) -> Option<Self> {
        if data.len() > MESSAGE_DIRECT_LEN {
            None
        } else {
            let mut body = [0; MESSAGE_DIRECT_LEN];
            body[..data.len()].copy_from_slice(data);
            Some(Self {
                header: MessageHeader { 
                    sender, 
                    tag, 
                    len: data.len() as u32 
                },
                body
            })
        }
    }
}
