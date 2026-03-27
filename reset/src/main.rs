#![no_std]
#![no_main]

use core::{ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPureWrite, WriteOnly}};
use small_os_lib::{HeaderError, QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, check_critical, check_header_len, do_yield, read_header, reply_empty};

#[repr(C)]
struct ResetRegisters {
    reset: ReadPureWrite<u32>,
    _reserved0: u32,
    reset_done: ReadOnly<u32>
}

#[repr(C)]
struct ResetClearRegisters {
    reset: WriteOnly<u32>,
}

#[repr(C)]
struct ResetSetRegisters {
    reset: WriteOnly<u32>,
}

pub struct Reset {
    registers: UniqueMmioPointer<'static, ResetRegisters>,
    clear_reg: UniqueMmioPointer<'static, ResetClearRegisters>,
    set_reg: UniqueMmioPointer<'static, ResetSetRegisters>,
    bitmap: u32
}

impl Reset {
    unsafe fn new(base: usize, bitmap: u32) -> Self {
        let mut res = unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap()),
                bitmap
            }
        };
        res.reset_all();
        res
    }

    pub fn reset_all(&mut self) {
        // mask is all the devices that need reseting
        let mask = (!field!(self.registers, reset).read()) & self.bitmap;
        if mask != 0 {
            // reset all masked devices not already reset
            field!(self.set_reg, reset).write(mask);
        }
        // deassert reset on all masked devices
        field!(self.clear_reg, reset).write(self.bitmap);
        // wait until devices ready to be used
        while field!(self.registers, reset_done).read() & self.bitmap != self.bitmap {
            do_yield().unwrap();
        }
    }

    pub fn reset_device(&mut self, device: u8) {
        assert!(device <= 24);
        let mask = 1 << device;
        if field!(self.registers, reset).read() & mask == 0 {
            field!(self.set_reg, reset).write(mask);
        }
        field!(self.clear_reg, reset).write(mask);
        while field!(self.registers, reset_done).read() & mask == 0 {
            do_yield().unwrap();
        }
    }
}

pub enum ResetReplyError {
    SendError,
    InvalidRequest,
    InvalidDevice,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<HeaderError> for ResetReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

impl From<ResetReplyError> for u32 {
    fn from(value: ResetReplyError) -> Self {
        match value {
            ResetReplyError::SendError => 1,
            ResetReplyError::InvalidRequest => 2,
            ResetReplyError::InvalidDevice => 3,
            ResetReplyError::InvalidSendBuffer => 4,
            ResetReplyError::InvalidReplyBuffer => 5
        }
    }
}

pub enum ResetError {
    ReplyError(ResetReplyError),
    QueueError(QueueError)
}

impl From<ResetReplyError> for ResetError {
    fn from(value: ResetReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for ResetError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

impl From<HeaderError> for ResetError {
    fn from(value: HeaderError) -> Self {
        Self::from(ResetReplyError::from(value))
    }
}

pub enum Request {
    Finished,
    ResetDevice(u8)
}

impl Request {
    pub fn parse(bitmap: u32) -> Result<Self, ResetError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // Finished request
                check_header_len(&header, 0, 0)?;
                Ok(Self::Finished)
            },
            1 => {
                // Reset Device request
                check_header_len(&header, 0, 0)?; 
                if header.driver >= 1 && header.driver <= 22 {
                    let extra_shift = if header.driver > 21 {
                        // skip UART1, TBMAN and JTAG
                        3
                    } else if header.driver > 19 {
                        // skip TBMAN and JTAG
                        2
                    } else if header.driver > 7 {
                        // skip JTAG
                        1
                    } else {
                        0
                    };
                    let device = (header.driver + extra_shift - 1) as u8;
                    if (1 << device) & bitmap != 0 {
                        Ok(Self::ResetDevice(device))
                    } else {
                        Err(ResetError::ReplyError(ResetReplyError::InvalidDevice))
                    }
                } else {
                    Err(ResetError::ReplyError(ResetReplyError::InvalidDevice))
                }
            },
            _ => Err(ResetError::ReplyError(ResetReplyError::InvalidRequest))
        }
    }
}

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(num_args: usize, reset_base: usize, bitmap: u32) {
    assert!(num_args == 2);
    let mut reset = unsafe {
        Reset::new(reset_base, bitmap)
    };
    loop {
        match Request::parse(reset.bitmap) {
            Ok(request) => {
                match request {
                    Request::Finished => {},
                    Request::ResetDevice(device) => reset.reset_device(device),
                }
                check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
            },
            Err(err) => {
                match err {
                    ResetError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    ResetError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(ResetReplyError::SendError))).unwrap_or(Ok(())).unwrap();
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
