#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{intrinsics::abort, panic::PanicInfo, ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPure, ReadPureWrite, ReadWrite}};
use small_os_lib::{QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, check_critical, do_yield, read_header, reply, reply_empty, send, wait_irq};

#[repr(C)]
struct ADCRegisters {
    cs: ReadPureWrite<u32>, // 0x0
    result: ReadOnly<u32>, // 0x4
    fifo_ctrl_status: ReadPureWrite<u32>, // 0x8
    fifo: ReadWrite<u32>, // 0xc
    div: ReadPureWrite<u32>, // 0x10
    raw_inter: ReadPure<u32>, // 0x14
    inter_enable: ReadPureWrite<u32>, // 0x18
    inter_force: ReadPureWrite<u32>, // 0x1c
    inter_status: ReadPure<u32> // 0x20
}

mod cs_register {
    pub const ENABLE_SHIFT: usize = 0;
    pub const TEMP_ENABLE_SHIFT: usize = 1;
    pub const START_ONCE_SHIFT: usize = 2;
    pub const START_MANY_SHIFT: usize = 3;
    pub const READY_SHIFT: usize = 8;
    pub const ERROR_SHIFT: usize = 9;
    pub const ERROR_STICKY_SHIFT: usize = 10;
    pub const ANALOG_MUX_SHIFT: usize = 12;
    pub const ROUND_ROBIN_SHIFT: usize = 16;

    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const TEMP_ENABLE_MASK: u32 = 1 << TEMP_ENABLE_SHIFT;
    pub const START_ONCE_MASK: u32 = 1 << START_ONCE_SHIFT;
    pub const START_MANY_MASK: u32 = 1 << START_MANY_SHIFT;
    pub const READY_MASK: u32 = 1 << READY_SHIFT;
    pub const ERROR_MASK: u32 = 1 << ERROR_SHIFT;
    pub const ERROR_STICKY_MASK: u32 = 1 << ERROR_STICKY_SHIFT;
    pub const ANALOG_MUX_MASK: u32 = 0x7 << ANALOG_MUX_SHIFT;
    pub const ROUND_ROBIN_MASK: u32 = 0x1f << ROUND_ROBIN_SHIFT;
}

mod result_register {
    pub const RESULT_SHIFT: usize = 0;

    pub const RESULT_MASK: u32 = 0xfff << RESULT_SHIFT;
}

mod fifo_ctrl_status_register {
    pub const ENABLE_SHIFT: usize = 0;
    pub const SHIFT_SHIFT: usize = 1;
    pub const ERROR_SHIFT: usize = 2;
    pub const DMA_SHIFT: usize = 3;
    pub const EMPTY_SHIFT: usize = 8;
    pub const FULL_SHIFT: usize = 9;
    pub const UNDERFLOW_SHIFT: usize = 10;
    pub const OVERFLOW_SHIFT: usize = 11;
    pub const LEVEL_SHIFT: usize = 16;
    pub const THRESHOLD_SHIFT: usize = 24;

    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const SHIFT_MASK: u32 = 1 << SHIFT_SHIFT;
    pub const ERROR_MASK: u32 = 1 << ERROR_SHIFT;
    pub const DMA_MASK: u32 = 1 << DMA_SHIFT;
    pub const EMPTY_MASK: u32 = 1 << EMPTY_SHIFT;
    pub const FULL_MASK: u32 = 1 << FULL_SHIFT;
    pub const UNDERFLOW_MASK: u32 = 1 << UNDERFLOW_SHIFT;
    pub const OVERFLOW_MASK: u32 = 1 << OVERFLOW_SHIFT;
    pub const LEVEL_MASK: u32 = 0xf << LEVEL_SHIFT;
    pub const THRESHOLD_MASK: u32 = 0xf << THRESHOLD_SHIFT;
}

mod fifo_register {
    pub const VALUE_SHIFT: usize = 0;
    pub const ERROR_SHIFT: usize = 15;

    pub const VALUE_MASK: u32 = 0xfff << VALUE_SHIFT;
    pub const ERROR_MASK: u32 = 1 << ERROR_SHIFT;
}

mod div_register {
    pub const FRAC_SHIFT: usize = 0;
    pub const INT_SHIFT: usize = 8;

    pub const FRAC_MASK: u32 = 0xff << FRAC_SHIFT;
    pub const INT_MASK: u32 = 0xffff << INT_SHIFT;
}

mod inter_register {
    pub const FIFO_SHIFT: usize = 0;
    
    pub const FIFO_MASK: u32 = 1 << FIFO_SHIFT;
}

pub struct ADC {
    registers: UniqueMmioPointer<'static, ADCRegisters>,
    set_reg: UniqueMmioPointer<'static, ADCRegisters>,
    clear_reg: UniqueMmioPointer<'static, ADCRegisters>
}

const MAX_SOURCE: u8 = 4;
const FIFO_SIZE: usize = 8;

impl ADC {
    pub unsafe fn new(adc_base: usize) -> Self {
        let mut res = unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(adc_base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(adc_base + REG_ALIAS_SET_BITS)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(adc_base + REG_ALIAS_CLR_BITS)).unwrap()),
            }
        };
        field!(res.registers, cs).write(
            cs_register::ENABLE_MASK | 
            cs_register::TEMP_ENABLE_MASK
        );
        field!(res.registers, div).write(0);
        field!(res.registers, inter_enable).write(inter_register::FIFO_MASK);
        while field!(res.registers, cs).read() & cs_register::READY_MASK == 0 {
            do_yield().unwrap();
        }
        res
    }

    /// Reads buffer.len() / 2 samples into buffer
    pub fn read_samples(&mut self, buffer: &mut [u8], src: u8) {
        assert!(buffer.len() & 1 == 0);
        assert!(src <= MAX_SOURCE);
        let mut len = buffer.len() / 2;
        let mut buffer_offset = 0;
        while len > FIFO_SIZE {
            field!(self.registers, fifo_ctrl_status).write(
                fifo_ctrl_status_register::ENABLE_MASK | 
                fifo_ctrl_status_register::ERROR_MASK | 
                fifo_ctrl_status_register::UNDERFLOW_MASK | 
                fifo_ctrl_status_register::OVERFLOW_MASK |
                ((FIFO_SIZE as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
            );
            field!(self.registers, cs).write(
                cs_register::ENABLE_MASK |
                cs_register::TEMP_ENABLE_MASK |
                cs_register::START_MANY_MASK |
                ((src as u32) << cs_register::ANALOG_MUX_SHIFT)
            );
            wait_irq().unwrap();
            field!(self.clear_reg, cs).write(cs_register::START_MANY_MASK);
            assert!(buffer_offset + FIFO_SIZE * 2 < buffer.len());
            for _ in 0..FIFO_SIZE {
                let sample = field!(self.registers, fifo).read();
                let error = sample & fifo_register::ERROR_MASK != 0;
                let sample = (sample & fifo_register::VALUE_MASK) >> fifo_register::VALUE_SHIFT;
                let sample = (sample << 1) | (error as u32);
                let sample = (sample as u16).to_le_bytes();
                buffer[buffer_offset] = sample[0];
                buffer[buffer_offset + 1] = sample[1];
                buffer_offset += 2;
            }
            len -= FIFO_SIZE;
        }
        field!(self.registers, fifo_ctrl_status).write(
            fifo_ctrl_status_register::ENABLE_MASK | 
            fifo_ctrl_status_register::ERROR_MASK | 
            fifo_ctrl_status_register::UNDERFLOW_MASK | 
            fifo_ctrl_status_register::OVERFLOW_MASK |
            ((len as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
        );
        field!(self.registers, cs).write(
            cs_register::ENABLE_MASK |
            cs_register::TEMP_ENABLE_MASK |
            cs_register::START_MANY_MASK |
            ((src as u32) << cs_register::ANALOG_MUX_SHIFT)
        );
        wait_irq().unwrap();
        field!(self.clear_reg, cs).write(cs_register::START_MANY_MASK);
        assert!(buffer_offset + len * 2 == buffer.len());
        for _ in 0..len {
            let sample = field!(self.registers, fifo).read();
            let error = sample & fifo_register::ERROR_MASK != 0;
            let sample = (sample & fifo_register::VALUE_MASK) >> fifo_register::VALUE_SHIFT;
            let sample = (sample << 1) | (error as u32);
            let sample = sample.to_le_bytes();
            buffer[buffer_offset] = sample[0];
            buffer[buffer_offset + 1] = sample[1];
            buffer_offset += 2;
        }
    }
}

pub enum ADCReplyError {
    SendError,
    InvalidSource,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<ADCReplyError> for u32 {
    fn from(value: ADCReplyError) -> Self {
        match value {
            ADCReplyError::SendError => 1,
            ADCReplyError::InvalidSource => 2,
            ADCReplyError::InvalidSendBuffer => 3,
            ADCReplyError::InvalidReplyBuffer => 4,
        }
    }
}

pub enum ADCError {
    ReplyError(ADCReplyError),
    QueueError(QueueError)
}

impl From<ADCReplyError> for ADCError {
    fn from(value: ADCReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for ADCError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

pub struct Request {
    source: u8,
    buffer: [u8; 32],
    len: usize
}

impl Request {
    pub fn parse() -> Result<Self, ADCError> {
        let buffer = [0; 32];
        let header = read_header(0)?;
        if header.tag != 2 && header.tag != 4 {
            return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
        }
        if header.send_len != 0 {
            return Err(ADCError::ReplyError(ADCReplyError::InvalidSendBuffer));
        }
        if (header.reply_len & 1) != 0 || (header.reply_len as usize) > buffer.len() {
            return Err(ADCError::ReplyError(ADCReplyError::InvalidReplyBuffer));
        }
        Ok(Self { 
            source: header.tag as u8, 
            buffer, 
            len: header.reply_len as usize
        })
    }
}

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

const RESET_QUEUE: u32 = 0;
const RESET_ADC: u32 = 0;

const PADS_QUEUE: u32 = 1;
const PADS_GPIO: u32 = 0;
const PADS_GPIO28: u8 = 28;
const PADS_ANALOG: u8 = 2;

#[unsafe(no_mangle)]
pub extern "C" fn main(adc_base: usize) {
    send(RESET_QUEUE, RESET_ADC, &mut [], 0, 0).unwrap();
    send(PADS_QUEUE, PADS_GPIO, &mut [PADS_GPIO28, PADS_ANALOG], 2, 0).unwrap();
    let mut adc = unsafe {
        ADC::new(adc_base)
    };
    loop {
        match Request::parse() {
            Ok(mut request) => {
                adc.read_samples(&mut request.buffer[..request.len], request.source);
                check_critical(reply(0, 0, &request.buffer[..request.len])).unwrap_or(Ok(())).unwrap();
            },
            Err(err) => {
                match err {
                    ADCError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    ADCError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(ADCReplyError::SendError))).unwrap_or(Ok(())).unwrap();
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
