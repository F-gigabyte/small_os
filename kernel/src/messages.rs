// Based on https://wiki.osdev.org/Message_Passing_Tutorial accessed 2/02/2026
// Based on https://wiki.osdev.org/Message_Passing accessed 2/02/2026
// Based on https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.getting_started/topic/s1_msg_Message_handling.html accessed 2/02/2026
// Based on https://sel4.systems/Info/Docs/seL4-manual-latest.pdf accessed 2/02/2026
// Based on https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/02-Queues-mutexes-and-semaphores/01-Queues accessed 2/02/2026

#[repr(C)]
#[derive(Debug, Clone)]
pub struct MessageHeader {
    pub pid: u32,
    pub pin_mask: u32,
    pub driver_tag: u32,
    pub len: u32,
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
        print!("Testing message creation ");
        let msg = Message::new(proc.pid, 0, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]); 
        assert!(msg.is_some());
        let msg = msg.unwrap();
        assert_eq!(msg.header.pid, proc.pid);
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
        assert!(Message::new(proc.pid, 0, &[0; MAX_ASYNC_MESSAGE_LEN + 1]).is_none());
        println!("[ok]");
    }
}
