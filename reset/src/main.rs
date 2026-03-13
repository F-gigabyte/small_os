#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{intrinsics::abort, panic::PanicInfo};
use core::{ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPureWrite, WriteOnly}};
use small_os_lib::{QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, check_critical, do_yield, read_header, reply_empty};

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
    set_reg: UniqueMmioPointer<'static, ResetSetRegisters>
}

impl Reset {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap())
            }
        }
    }

    pub fn handle_request(&mut self, request: &mut Request) {
        let device = request.device as u32;
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

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

pub enum ResetReplyError {
    SendError,
    InvalidDevice,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<ResetReplyError> for u32 {
    fn from(value: ResetReplyError) -> Self {
        match value {
            ResetReplyError::SendError => 1,
            ResetReplyError::InvalidDevice => 2,
            ResetReplyError::InvalidSendBuffer => 3,
            ResetReplyError::InvalidReplyBuffer => 4
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

#[derive(Clone, Copy, Debug)]
pub enum ResetDevice {
    ADC = 0,
    BusCtrl = 1,
    DMA = 2,
    I2C0 = 3,
    I2C1 = 4,
    IOBank0 = 5,
    IOQSPI = 6,
    JTAG = 7,
    PadsIOBank0 = 8,
    PadsQSPI = 9,
    PIO0 = 10,
    PIO1 = 11,
    PllSys = 12,
    PllUSB = 13,
    PWM = 14,
    RTC = 15,
    SPI0 = 16,
    SPI1 = 17,
    SysCfg = 18,
    SysInfo = 19,
    Timer = 21,
    Uart0 = 22,
    Uart1 = 23,
    USBCtrl = 24
}

impl TryFrom<u32> for ResetDevice {
    type Error = ResetReplyError;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::ADC),
            1 => Ok(Self::BusCtrl),
            2 => Ok(Self::DMA),
            3 => Ok(Self::I2C0),
            4 => Ok(Self::I2C1),
            5 => Ok(Self::IOBank0),
            6 => Ok(Self::IOQSPI),
            7 => Ok(Self::JTAG),
            8 => Ok(Self::PadsIOBank0),
            9 => Ok(Self::PadsQSPI),
            10 => Ok(Self::PIO0),
            11 => Ok(Self::PIO1),
            12 => Ok(Self::PllSys),
            13 => Ok(Self::PllUSB),
            14 => Ok(Self::PWM),
            15 => Ok(Self::RTC),
            16 => Ok(Self::SPI0),
            17 => Ok(Self::SPI1),
            18 => Ok(Self::SysCfg),
            19 => Ok(Self::SysInfo),
            21 => Ok(Self::Timer),
            22 => Ok(Self::Uart0),
            23 => Ok(Self::Uart1),
            24 => Ok(Self::USBCtrl),
            _ => Err(ResetReplyError::InvalidDevice)
        }
    }
}

pub struct Request {
    device: ResetDevice
}

impl Request {
    pub fn parse() -> Result<Self, ResetError> {
        let header = read_header(0)?;
        let device = ResetDevice::try_from(header.tag)?;
        if header.reply_len != 0 {
            return Err(ResetError::ReplyError(ResetReplyError::InvalidReplyBuffer));
        }
        if header.send_len != 0 {
            return Err(ResetError::ReplyError(ResetReplyError::InvalidSendBuffer));
        }
        Ok(Self { 
            device
        })
    }
}

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(reset_base: usize) {
    let mut reset = unsafe {
        Reset::new(reset_base)
    };
    loop {
        match Request::parse() {
            Ok(mut request) => {
                reset.handle_request(&mut request);
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
