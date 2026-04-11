/// Message Header
/// Contains metadata about the message sent  
/// Based on
/// - <https://wiki.osdev.org/Message_Passing_Tutorial> accessed 2/02/2026
/// - <https://wiki.osdev.org/Message_Passing> accessed 2/02/2026
/// - <https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.getting_started/topic/s1_msg_Message_handling.html> accessed 2/02/2026
/// - <https://sel4.systems/Info/Docs/seL4-manual-latest.pdf> accessed 2/02/2026
/// - <https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/02-Queues-mutexes-and-semaphores/01-Queues> accessed 2/02/2026
#[repr(C)]
#[derive(Debug, Clone)]
pub struct MessageHeader {
    /// The PID of the calling process
    pub pid: u32,
    /// The calling process' pin mask
    pub pin_mask: u32,
    /// The driver of the calling process and the message tag
    /// - [0..15] -> tag
    /// - [16..31] -> driver
    pub driver_tag: u32,
    /// The message length  
    /// For synchronous messages this is
    /// - [0..15] -> send length
    /// - [16..31] -> reply length
    /// For asynchronous messages this is the message length
    pub len: u32,
}
