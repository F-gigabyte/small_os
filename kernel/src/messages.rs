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
