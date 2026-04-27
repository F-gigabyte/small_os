/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Kermit Driver.
 *
 * The SmallOS Kermit Driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Kermit Driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS Kermit Driver. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

/* Based off the kermit protocol as described at the online book
 * https://www.kermitproject.org/onlinebooks/kermitbook.pdf (pages 211 to 268, accessed 31/03/2026)
 */

// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use small_os_lib::{HeaderError, QueueError, args, check_critical, check_header_len, read_header, receive, reply_empty, send, send_empty};


/// UART driver endpoint
const UART_QUEUE: u32 = 0;

/// Different transmission states of the kermit protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum KermitState {
    /// Send Init Packet
    Init,
    /// Send File Header Packet
    FileHeader,
    /// Send File Data Packet
    FileData,
    /// Send End Of File Packet
    EOF,
    /// Send Break Packet
    Break,
    /// Wait for Send Init Packet
    WaitInit,
    /// Wait for File Header Packet
    WaitFileHeader,
    /// Wait for File Data Packet
    WaitFileData,
    /// Send Complete Packet
    Complete,
    /// Send Abort Packet
    Abort
}

/// Kermit packet types
#[derive(Debug, Clone, Copy)]
enum PacketType {
    /// Data packet
    Data,
    /// Acknowledge packet
    Ack,
    /// Not acknowledge packet
    Nak,
    /// Init packet
    Init,
    /// Break packet
    Break,
    /// Header packet
    Header,
    /// End of file packet
    EOF,
    /// Error packet
    PacketError
}

/// Gets the packet type code from `PacketType`
impl From<PacketType> for u8 {
    fn from(value: PacketType) -> Self {
        match value {
            PacketType::Data => b'D',
            PacketType::Ack => b'Y',
            PacketType::Nak => b'N',
            PacketType::Init => b'S',
            PacketType::Break => b'B',
            PacketType::Header => b'F',
            PacketType::EOF => b'Z',
            PacketType::PacketError => b'E'
        }
    }
}

/// Casts from packet type code to `PacketType`
impl TryFrom<u8> for PacketType {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            b'D' => Ok(Self::Data),
            b'Y' => Ok(Self::Ack),
            b'N' => Ok(Self::Nak),
            b'S' => Ok(Self::Init),
            b'B' => Ok(Self::Break),
            b'F' => Ok(Self::Header),
            b'Z' => Ok(Self::EOF),
            b'E' => Ok(Self::PacketError),
            _ => Err(value)
        }
    }
}

/// Transforms a byte in the range 0 to 94 into an ASCII character  
/// `x` is the byte to convert  
/// Panics if x is greater than 94
fn to_char(x: u8) -> u8 {
    assert!(x <= 94);
    x + 32
}

/// Transforms an ASCII character into a byte in the range 0 to 94  
/// 'x' is the ASCII character to convert  
/// Returns the converted byte on success or an error containing `x` on failure
fn unchar(x: u8) -> Result<u8, u8> {
    if x < 32 {
        Err(x)
    } else {
        Ok(x - 32)
    }
}

/// Converts between ASCII control characters and their printable representations
fn ctrl(x: u8) -> u8 {
    x ^ 64
}

/// Clears the UART RX fifo
fn clear_all_rx() {
    send_empty(UART_QUEUE, 3, &[]).unwrap();
}

/// Object to manage the Kermit protocol
pub struct Kermit {
    /// The current state of the kermit protocol
    state: KermitState,
    /// The current packet sequence number
    seq: u8,
    /// The maximum packet length in ASCII characters
    packet_len: u8,
}

/// A Kermit data packet
pub struct KermitPacket {
    /// The packet length
    data_len: usize,
    /// The packet data buffer
    data: [u8; 91]
}

impl Kermit {
    /// Creates a new `Kermit` object
    pub fn new() -> Self {
        Self {
            state: KermitState::Init,
            seq: 0,
            packet_len: 80
        }
    }

    /// Sends a Kermit packet  
    /// `packet_type` is the type of packet to send  
    /// `data` is the packet contents
    fn send_packet(&mut self, packet_type: PacketType, data: &[u8]) {
        assert!(data.len() <= (self.packet_len - 3) as usize);
        let mut packet = [0; 96];
        packet[0] = 0x1; // ctrl-A
        packet[1] = to_char(data.len() as u8 + 3);
        packet[2] = to_char(self.seq);
        packet[3] = u8::from(packet_type);
        packet[4..4 + data.len()].copy_from_slice(data);
        packet[4 + data.len()] = Self::calc_packet_data(&packet[..4], data);
        let num_calls = (5 + data.len() + 31) / 32;
        for i in 0..num_calls {
            let offset = i * 32;
            let len = (5 + data.len() - offset).min(32);
            send_empty(UART_QUEUE, 0, &packet[offset..offset + len]).unwrap();
        }
    }
    
    /// Calculates the packet checksum  
    /// `header` is the packet's header  
    /// `data` is the packet's content
    fn calc_packet_data(header: &[u8], data: &[u8]) -> u8 {
        let s = header[1..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        let s = data.iter().fold(s, |a, b| a.wrapping_add(*b));
        to_char((s.wrapping_add((s & 192) / 64)) & 63)
    }

    /// Checks the packet's checksum against that calculated for its content  
    /// `header` is the packet's header  
    /// `data` is the packet's content  
    /// `check` is the packet checksum to compare against  
    /// Returns success if `check` matches the checksum calculated or else returns an error
    fn check_packet_data(header: &[u8], data: &[u8], check: u8) -> Result<(), ()> {
        if Self::calc_packet_data(header, data) == check {
            Ok(())
        } else {
            Err(())
        }
    }

    /// Gets the header of the next packet in the UART RX fifo
    fn get_packet_header(&mut self) -> [u8; 4] {
        let mut byte = [0];
        while byte[0] != 1 {
            send(UART_QUEUE, 1, &mut byte, 0, 1).unwrap();
        }
        let mut header = [byte[0], 0, 0, 0];
        send(UART_QUEUE, 1, &mut header[1..], 0, 3).unwrap();
        header
    }

    /// Receives a packet of data  
    /// `perform_check` is whether the packet's checksum should be checked  
    /// Returns the kermit packet if a valid packet is available, `None` if no valid packet is
    /// available or an error if an invalid packet type is received
    fn receive_packet(&mut self, perform_check: bool) -> Result<Option<KermitPacket>, KermitReplyError> {
        let packet_header = self.get_packet_header();
        if packet_header[3] == u8::from(PacketType::Nak) {
            return Ok(None);
        } else if packet_header[3] == u8::from(PacketType::Ack) {
            let len = match unchar(packet_header[1]) {
                Ok(len) => len,
                Err(_) => {
                    return Ok(None);
                }
            };
            let seq = match unchar(packet_header[2]) {
                Ok(seq) => seq,
                Err(_) => return Ok(None)
            };
            if len < 3 {
                return Ok(None);
            }
            if seq != self.seq {
                return Ok(None);
            }
            let data_len = len as usize - 3;
            let mut packet_data = [0; 91];
            let num_calls = (data_len + 31) / 32;
            for i in 0..num_calls {
                let offset = i * 32;
                let len = (data_len - offset).min(32);
                send(UART_QUEUE, 1, &mut packet_data[offset..offset + len], 0, len).unwrap();
            }
            let mut check = [0];
            send(UART_QUEUE, 1, &mut check, 0, 1).unwrap();
            let check = check[0];
            if !perform_check || Self::check_packet_data(&packet_header, &packet_data, check).is_ok() {
                Ok(Some(KermitPacket { 
                    data_len,
                    data: packet_data
                }))
            } else {
                Ok(None)
            }
        } else {
            Err(KermitReplyError::ProtocolError)
        }
    }

    /// Sends a packet and waits for it to be acknowledged  
    /// `packet_type` is the type of packet to send  
    /// `data` is the packet contents  
    /// `perform_check` is whether to check received packet checksums  
    /// Returns the received packet on success or an error on failure
    fn send_packet_ack(&mut self, packet_type: PacketType, data: &[u8], perform_check: bool) -> Result<KermitPacket, KermitReplyError> {
        loop {
            self.send_packet(packet_type, data);
            if let Some(packet) = self.receive_packet(perform_check)? {
                self.seq = (self.seq + 1) % 64;
                return Ok(packet);
            }
        }
    }

    /// Sends an error packet
    fn send_error(&mut self) {
        self.send_packet(PacketType::PacketError, &[]);
    }

    /// Starts a kermit transaction  
    /// The state must be the `Init` state or else an error is returned and nothing happens  
    /// This sends the init packet and on success will change the state to the `FileHeader` state
    /// Otherwise, the state remains in the `Init` state
    pub fn start_transaction(&mut self) -> Result<(), KermitReplyError> {
        if self.state != KermitState::Init {
            return Err(KermitReplyError::InvalidState);
        }
        self.seq = 0;
        clear_all_rx();
        let init = match self.send_packet_ack(PacketType::Init, &[
            to_char(94), // max length
            to_char(94), // timeout of 94s
            to_char(0), // no padding
            ctrl(0), // NULL padding
            to_char(b'\r'), // Line feed terminating packet
            b'#', // # precedes ctrl characters
            b'N', // no 8 bit quoting
            to_char(1), // single char checksum
            to_char(0), // no repeat char
            to_char(0), // no extra capabilities
        ], false) {
            Ok(packet) => packet,
            Err(err) => {
                self.send_error();
                return Err(err);
            }
        };
        if init.data_len > 0 && let Ok(data_len) = unchar(init.data[0]) && data_len == 94 {
            self.packet_len = 94;
        }
        self.state = KermitState::FileHeader;
        Ok(())
    }

    /// Starts sending a file  
    /// `request` is a `Packet` containing the file name  
    /// The state must be the `FileHeader` state or else an error is returned and nothing happens  
    /// This sends the file header packet and on success will change the state to the `FileData` state  
    /// On error, the state transitions back to the `Init` state
    pub fn start_file(&mut self, request: Packet) -> Result<(), KermitReplyError> {
        if self.state != KermitState::FileHeader {
            return Err(KermitReplyError::InvalidState);
        }
        clear_all_rx();
        match self.send_packet_ack(PacketType::Header, &request.data[..request.len as usize], true) {
            Ok(_) => {
                self.state = KermitState::FileData;
                Ok(())
            },
            Err(err) => {
                self.send_error();
                self.state = KermitState::Init;
                Err(err)
            }
        }
    }

    /// Sends a block of file data  
    /// `request` is a `Packet` containing the file data, which is correctly escaped where needed  
    /// The state must be the `FileData` state or else an error is returned and nothing happens  
    /// This sends a data packet  
    /// On error, the state transitions back to the `Init` state
    pub fn write_data(&mut self, request: Packet) -> Result<(), KermitReplyError> {
        if self.state != KermitState::FileData {
            return Err(KermitReplyError::InvalidState);
        }
        clear_all_rx();
        match self.send_packet_ack(PacketType::Data, &request.data[..request.len as usize], true) {
            Ok(_) => {
                Ok(())
            },
            Err(err) => {
                self.send_error();
                self.state = KermitState::Init;
                Err(err)
            }
        }
    }

    /// Ends a file  
    /// The state must be the `FileData` state or else an error is returned and nothing happens  
    /// This sends an end of file packet and on success will change the state to the `FileHeader` state  
    /// On error, the state transitions back to the `Init` state
    pub fn end_file(&mut self) -> Result<(), KermitReplyError> {
        if self.state != KermitState::FileData {
            return Err(KermitReplyError::InvalidState);
        }
        clear_all_rx();
        match self.send_packet_ack(PacketType::EOF, &[], true) {
            Ok(_) => {
                self.state = KermitState::FileHeader;
                Ok(())
            },
            Err(err) => {
                self.send_error();
                self.state = KermitState::Init;
                Err(err)
            }
        }
    }

    /// Ends a transaction  
    /// The state must be the `FileHeader` state or else an error is returned and nothing happens  
    /// This sends a break packet and on success will change the state to the `Init` state  
    /// On error, the state transitions back to the `Init` state
    pub fn end_transaction(&mut self) -> Result<(), KermitReplyError> {
        if self.state != KermitState::FileHeader {
            return Err(KermitReplyError::InvalidState);
        }
        clear_all_rx();
        match self.send_packet_ack(PacketType::Break, &[], true) {
            Ok(_) => {
                self.state = KermitState::Init;
                Ok(())
            },
            Err(err) => {
                self.send_error();
                self.state = KermitState::Init;
                Err(err)
            }
        }
    }
}

/// Kermit reply errors
pub enum KermitReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// An invalid file name was provided
    InvalidFileName,
    /// An operation was attempted on an invalid state
    InvalidState,
    /// The packet length is too large
    InvalidPacketLen,
    /// An invalid packet type was received
    ProtocolError,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Converts from a `HeaderError` to a `KermitReplyError`
impl From<HeaderError> for KermitReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

/// Converts from a `KermitReplyError` to a `u32`
impl From<KermitReplyError> for u32 {
    fn from(value: KermitReplyError) -> Self {
        match value {
            KermitReplyError::SendError => 1,
            KermitReplyError::InvalidRequest => 2,
            KermitReplyError::InvalidFileName => 3,
            KermitReplyError::InvalidState => 4,
            KermitReplyError::InvalidPacketLen => 5,
            KermitReplyError::ProtocolError => 6,
            KermitReplyError::InvalidSendBuffer => 7,
            KermitReplyError::InvalidReplyBuffer => 8
        }
    }
}

/// Kermit Errors
pub enum KermitError {
    /// Error with the request
    ReplyError(KermitReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts from a `KermitReplyError` to a `KermitError`
impl From<KermitReplyError> for KermitError {
    fn from(value: KermitReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts from a `QueueError` to a `KermitError`
impl From<QueueError> for KermitError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// Converts from a `HeaderError` to a `KermitError`
impl From<HeaderError> for KermitError {
    fn from(value: HeaderError) -> Self {
        Self::from(KermitReplyError::from(value))
    }
}

/// Kermit packet data
pub struct Packet {
    /// The data to be sent
    data: [u8; 90],
    /// The packet length
    len: u8
}

/// Kermit request
pub enum Request {
    /// Start a Kermit transaction
    StartTransaction,
    /// Start sending a file
    StartFile(Packet),
    /// Send some file data
    SendFileData(Packet),
    /// Finish sending a file
    FinishFile,
    /// End a Kermit transaction
    EndTransaction
}

impl Request {
    /// Parses the next request  
    /// Returns the request on success or a `KermitError` on failure
    pub fn parse(packet_len: usize) -> Result<Self, KermitError> {
        assert!(packet_len / 2 <= 45);
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // Start Transaction
                check_header_len(&header, 0, 0)?;
                Ok(Self::StartTransaction)
            },
            1 => {
                // Start File
                let mut data = [0; 90];
                if header.send_len as usize > packet_len / 2 {
                    return Err(KermitError::ReplyError(KermitReplyError::InvalidSendBuffer));
                }
                if header.reply_len != 0 {
                    return Err(KermitError::ReplyError(KermitReplyError::InvalidReplyBuffer));
                }
                _ = receive(0, &mut data)?;
                let mut num_dot = 0;
                for i in 0..header.send_len as usize {
                    if !data[i].is_ascii_digit() && !data[i].is_ascii_alphabetic() {
                        if data[i] == b'.' {
                            if num_dot == 1 || i == 0 || i == header.send_len as usize - 1 {
                                return Err(KermitError::ReplyError(KermitReplyError::InvalidFileName));
                            }
                            num_dot += 1;
                        } else {
                            return Err(KermitError::ReplyError(KermitReplyError::InvalidFileName));
                        }
                    }
                }
                Ok(Self::StartFile(Packet { 
                    data, 
                    len: header.send_len as u8 
                }))
            },
            2 => {
                // Send Data
                let mut data = [0; 90];
                if header.send_len as usize > (packet_len - 3) / 2 {
                    return Err(KermitError::ReplyError(KermitReplyError::InvalidSendBuffer));
                }
                if header.reply_len != 0 {
                    return Err(KermitError::ReplyError(KermitReplyError::InvalidReplyBuffer));
                }
                _ = receive(0, &mut data)?;
                let mut inc = 0;
                for i in 0..header.send_len as usize {
                    let lower = data[i] & !0x80;
                    if lower == 127 || lower < 32 || lower == b'#' {
                        inc += 1;
                    }
                }
                let len = header.send_len + inc as u16;
                if inc != 0 {
                    for i in (0..header.send_len as usize).rev() {
                        if inc == 0 {
                            break;
                        }
                        let upper_bit = data[i] & 0x80;
                        let lower = data[i] & !0x80;
                        if lower == 127 || lower < 32 {
                            data[i + inc] = ctrl(lower) | upper_bit;
                            data[i + inc - 1] = b'#';
                            inc -= 1;
                        } else if lower == b'#' {
                            data[i + inc] = lower | upper_bit;
                            data[i + inc - 1] = b'#';
                            inc -= 1;
                        } else {
                            data[i + inc] = data[i];
                        }
                    }
                }
                Ok(Self::SendFileData(Packet { 
                    data, 
                    len: len as u8 
                }))
            },
            3 => {
                // Finish File
                check_header_len(&header, 0, 0)?;
                Ok(Self::FinishFile)
            },
            4 => {
                // End Transaction
                check_header_len(&header, 0, 0)?;
                Ok(Self::EndTransaction)
            },
            _ => Err(KermitError::ReplyError(KermitReplyError::InvalidRequest))
        }
    }
}

/// Driver entry point
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 0);
    #[cfg(test)]
    test_main();
    let mut kermit = Kermit::new();
    loop {
        match Request::parse(kermit.packet_len as usize) {
            Ok(request) => {
                match request {
                    Request::StartTransaction => {
                        match kermit.start_transaction() {
                            Ok(()) => check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap(),
                            Err(err) => check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap()
                        }
                    },
                    Request::StartFile(file) => {
                        match kermit.start_file(file) {
                            Ok(()) => check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap(),
                            Err(err) => check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap()
                        }
                    },
                    Request::SendFileData(data) => {
                        match kermit.write_data(data) {
                            Ok(()) => check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap(),
                            Err(err) => check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap()
                        }
                    },
                    Request::FinishFile => {
                        match kermit.end_file() {
                            Ok(()) => check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap(),
                            Err(err) => check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap()
                        }
                    },
                    Request::EndTransaction => {
                        match kermit.end_transaction() {
                            Ok(()) => check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap(),
                            Err(err) => check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap()
                        }
                    }
                }
            },
            Err(err) => {
                match err {
                    KermitError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    KermitError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(KermitReplyError::SendError))).unwrap_or(Ok(())).unwrap();
                            },
                            _ => {
                                panic!("{:?}", err);
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::kprintln;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for kermit", tests.len());
        for test in tests {
            test();
        }
    }
}
