// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPure, ReadPureWrite, ReadWrite}};
use small_os_lib::{HeaderError, QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, args, check_critical, check_header_len, do_yield, read_header, receive, reply, reply_empty, send_empty, wait_irq};

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

    pub const VALID_MASK: u32 = ENABLE_MASK |
        TEMP_ENABLE_MASK |
        START_ONCE_MASK |
        START_MANY_MASK |
        READY_MASK |
        ERROR_MASK |
        ERROR_STICKY_MASK |
        ANALOG_MUX_MASK |
        ROUND_ROBIN_MASK;
}

mod result_register {
    pub const RESULT_SHIFT: usize = 0;

    pub const RESULT_MASK: u32 = 0xfff << RESULT_SHIFT;

    pub const VALID_MASK: u32 = RESULT_MASK;
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

    pub const VALID_MASK: u32 = ENABLE_MASK |
        SHIFT_MASK |
        ERROR_MASK |
        DMA_MASK |
        EMPTY_MASK |
        FULL_MASK |
        UNDERFLOW_MASK |
        OVERFLOW_MASK |
        LEVEL_MASK |
        THRESHOLD_MASK;
}

mod fifo_register {
    pub const VALUE_SHIFT: usize = 0;
    pub const ERROR_SHIFT: usize = 15;

    pub const VALUE_MASK: u32 = 0xfff << VALUE_SHIFT;
    pub const ERROR_MASK: u32 = 1 << ERROR_SHIFT;

    pub const VALID_MASK: u32 = VALUE_MASK |
        ERROR_MASK;
}

mod div_register {
    pub const FRAC_SHIFT: usize = 0;
    pub const INT_SHIFT: usize = 8;

    pub const FRAC_MASK: u32 = 0xff << FRAC_SHIFT;
    pub const INT_MASK: u32 = 0xffff << INT_SHIFT;

    pub const VALID_MASK: u32 = FRAC_MASK |
        INT_MASK;
}

mod inter_register {
    pub const FIFO_SHIFT: usize = 0;
    
    pub const FIFO_MASK: u32 = 1 << FIFO_SHIFT;

    pub const VALID_MASK: u32 = FIFO_MASK;
}

pub struct ADC {
    registers: UniqueMmioPointer<'static, ADCRegisters>,
    clear_reg: UniqueMmioPointer<'static, ADCRegisters>,
    bitmap: u32
}

const MAX_SOURCE: u8 = 4;
const FIFO_SIZE: usize = 8;

impl ADC {
    pub unsafe fn new(adc_base: usize, bitmap: u32) -> Self {
        let mut res = unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(adc_base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(adc_base + REG_ALIAS_CLR_BITS)).unwrap()),
                bitmap
            }
        };
        field!(res.registers, cs).modify(|cs|
            (cs & !cs_register::VALID_MASK) |
            cs_register::ENABLE_MASK | 
            cs_register::TEMP_ENABLE_MASK
        );
        field!(res.registers, div).modify(|div| div & !div_register::VALID_MASK);
        field!(res.registers, inter_enable).modify(|inter_enable| 
            (inter_enable & !inter_register::VALID_MASK) |
            inter_register::FIFO_MASK
        );
        while field!(res.registers, cs).read() & cs_register::READY_MASK == 0 {
            do_yield().unwrap();
        }
        res
    }

    fn src_to_gpio(src: u8) -> Option<u8> {
        assert!(src <= MAX_SOURCE);
        if src < 4 {
            Some(src + 26)
        } else {
            None
        }
    }

    fn wait_busy(&mut self) {
        while field!(self.registers, cs).read() & cs_register::READY_MASK == 0 {
            do_yield().unwrap();
        }
    }

    /// Reads buffer.len() / 2 samples into buffer
    pub fn read_samples(&mut self, buffer: &mut [u8], src: u8) {
        assert!(buffer.len() & 1 == 0);
        assert!(src <= MAX_SOURCE);
        self.wait_busy();
        let mut len = buffer.len() / 2;
        let mut buffer_offset = 0;
        while len > FIFO_SIZE {
            field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
                (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
                fifo_ctrl_status_register::ENABLE_MASK | 
                fifo_ctrl_status_register::ERROR_MASK | 
                fifo_ctrl_status_register::UNDERFLOW_MASK | 
                fifo_ctrl_status_register::OVERFLOW_MASK |
                ((FIFO_SIZE as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
            );
            field!(self.registers, cs).modify(|cs|
                (cs & !cs_register::VALID_MASK) |
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
        field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
            (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
            fifo_ctrl_status_register::ENABLE_MASK | 
            fifo_ctrl_status_register::ERROR_MASK | 
            fifo_ctrl_status_register::UNDERFLOW_MASK | 
            fifo_ctrl_status_register::OVERFLOW_MASK |
            ((len as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
        );
        field!(self.registers, cs).modify(|cs|
            (cs & !cs_register::VALID_MASK) |
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

    /// Reads buffer.len() / 2 samples into buffer
    pub fn read_samples_round_robin(&mut self, buffer: &mut [u8], samples: &[u8], mut start: usize) {
        assert!(samples.len() < 5);
        assert!(buffer.len() & 1 == 0);
        assert!(start < samples.len());
        let mut bitmap = 0;
        for sample in samples {
            assert!(*sample <= MAX_SOURCE);
            bitmap |= 1 << sample;
        }
        if bitmap == 0 {
            // empty bitmap means no reading
            return;
        }
        self.wait_busy();
        let mut len = buffer.len() / 2;
        let mut buffer_offset = 0;
        while len > FIFO_SIZE {
            field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
                (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
                fifo_ctrl_status_register::ENABLE_MASK | 
                fifo_ctrl_status_register::ERROR_MASK | 
                fifo_ctrl_status_register::UNDERFLOW_MASK | 
                fifo_ctrl_status_register::OVERFLOW_MASK |
                ((FIFO_SIZE as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
            );
            field!(self.registers, cs).modify(|cs|
                (cs & !cs_register::VALID_MASK) |
                cs_register::ENABLE_MASK |
                cs_register::TEMP_ENABLE_MASK |
                cs_register::START_MANY_MASK |
                ((samples[start] as u32) << cs_register::ANALOG_MUX_SHIFT) |
                ((bitmap as u32) << cs_register::ROUND_ROBIN_SHIFT)
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
            start = (start + FIFO_SIZE) % (samples.len());
            len -= FIFO_SIZE;
        }
        field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
            (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
            fifo_ctrl_status_register::ENABLE_MASK | 
            fifo_ctrl_status_register::ERROR_MASK | 
            fifo_ctrl_status_register::UNDERFLOW_MASK | 
            fifo_ctrl_status_register::OVERFLOW_MASK |
            ((len as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
        );
        field!(self.registers, cs).modify(|cs|
            (cs & !cs_register::VALID_MASK) |
            cs_register::ENABLE_MASK |
            cs_register::TEMP_ENABLE_MASK |
            cs_register::START_MANY_MASK |
            ((samples[start] as u32) << cs_register::ANALOG_MUX_SHIFT) |
            ((bitmap as u32) << cs_register::ROUND_ROBIN_SHIFT)
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

    pub fn read_sample(&mut self, src: u8) -> u16 {
        assert!(src <= MAX_SOURCE);
        self.wait_busy();
        field!(self.registers, fifo_ctrl_status).modify(|fifo_ctrl_status|
            (fifo_ctrl_status & !fifo_ctrl_status_register::VALID_MASK) |
            fifo_ctrl_status_register::ENABLE_MASK | 
            fifo_ctrl_status_register::ERROR_MASK | 
            fifo_ctrl_status_register::UNDERFLOW_MASK | 
            fifo_ctrl_status_register::OVERFLOW_MASK |
            ((1 as u32) << fifo_ctrl_status_register::THRESHOLD_SHIFT)
        );
        field!(self.registers, cs).modify(|cs|
            (cs & !cs_register::VALID_MASK) |
            cs_register::ENABLE_MASK |
            cs_register::TEMP_ENABLE_MASK |
            cs_register::START_ONCE_MASK |
            ((src as u32) << cs_register::ANALOG_MUX_SHIFT)
        );
        wait_irq().unwrap();
        let sample = field!(self.registers, fifo).read();
        let error = sample & fifo_register::ERROR_MASK != 0;
        let sample = (sample & fifo_register::VALUE_MASK) >> fifo_register::VALUE_SHIFT;
        let sample = (sample << 1) | (error as u32);
        sample as u16
    }
}

pub enum ADCReplyError {
    SendError,
    InvalidRequest,
    InvalidSource,
    InvalidSendBuffer,
    InvalidReplyBuffer,
    InvalidStart
}

impl From<ADCReplyError> for u32 {
    fn from(value: ADCReplyError) -> Self {
        match value {
            ADCReplyError::SendError => 1,
            ADCReplyError::InvalidRequest => 2,
            ADCReplyError::InvalidSource => 3,
            ADCReplyError::InvalidSendBuffer => 4,
            ADCReplyError::InvalidReplyBuffer => 5,
            ADCReplyError::InvalidStart => 6
        }
    }
}

impl From<HeaderError> for ADCReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer,
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer
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

impl From<HeaderError> for ADCError {
    fn from(value: HeaderError) -> Self {
        Self::from(ADCReplyError::from(value))
    }
}

impl From<QueueError> for ADCError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

const BUFFER_LEN: usize = 32;

pub struct ReadBuffered {
    buffer: [u8; BUFFER_LEN],
    len: usize
}

pub struct ReadMultiple {
    src: u8,
    buffer: ReadBuffered
}

pub struct ReadRoundRobin {
    start: usize,
    bitmap: u8,
    buffer: ReadBuffered
}

pub enum Request {
    ReadSingle(u8),
    ReadMultiple(ReadMultiple),
    ReadRoundRobin(ReadRoundRobin),
}

impl Request {
    pub fn parse(bitmap: u32) -> Result<Self, ADCError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // Read single
                check_header_len(&header, 1, 2)?;
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let src = buffer[0];
                if src > MAX_SOURCE {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                if let Some(gpio) = ADC::src_to_gpio(src) && (1 << gpio) & bitmap == 0 {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                Ok(Request::ReadSingle(src))
            },
            1 => {
                // Read multiple
                if header.send_len != 1 {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSendBuffer));
                }
                if !header.reply_len.is_multiple_of(2) || header.reply_len as usize > BUFFER_LEN {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSendBuffer));
                }
                let mut buffer = [0; 1];
                _ = receive(0, &mut buffer)?;
                let src = buffer[0];
                let len = header.reply_len as usize;
                if src > MAX_SOURCE {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                if let Some(gpio) = ADC::src_to_gpio(src) && (1 << gpio) & bitmap == 0 {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                Ok(Request::ReadMultiple(ReadMultiple { 
                    src, 
                    buffer: ReadBuffered {
                        buffer: [0; BUFFER_LEN],
                        len
                    }
                }))
            },
            2 => {
                // Read round robin
                if header.send_len != 2 {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSendBuffer));
                }
                if !header.reply_len.is_multiple_of(2) || header.reply_len as usize > BUFFER_LEN {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSendBuffer));
                }
                let mut buffer = [0; 2];
                _ = receive(0, &mut buffer)?;
                let mask = buffer[0];
                let start = buffer[1] as usize;
                let len = header.reply_len as usize;
                if mask > 0x1f {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                }
                let mut m = mask;
                let mut src = 0;
                while m > 0 {
                    if m & 1 != 0 {
                        if let Some(gpio) = ADC::src_to_gpio(src) && (1 << gpio) & bitmap == 0 {
                            return Err(ADCError::ReplyError(ADCReplyError::InvalidSource));
                        }
                    }
                    m >>= 1;
                    src += 1;
                }
                if start >= mask.count_ones() as usize {
                    return Err(ADCError::ReplyError(ADCReplyError::InvalidStart));
                }
                Ok(Request::ReadRoundRobin(ReadRoundRobin { 
                    start, 
                    bitmap: mask, 
                    buffer: ReadBuffered { 
                        buffer: [0; BUFFER_LEN], 
                        len 
                    }
                }))
            },
            _ => {
                Err(ADCError::ReplyError(ADCReplyError::InvalidRequest))
            }
        }
    }
}

const IO_BANK0_QUEUE: u32 = 0;

#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 2);
    // check func sel has been set up
    send_empty(IO_BANK0_QUEUE, 0, &[]).unwrap();
    #[cfg(test)]
    {
        test_main();
    }
    let adc_base = args[0] as usize;
    let bitmap = args[1];
    let mut adc = unsafe {
        ADC::new(adc_base, bitmap)
    };
    loop {
        match Request::parse(adc.bitmap) {
            Ok(request) => {
                match request {
                    Request::ReadSingle(src) => {
                        let sample = adc.read_sample(src).to_le_bytes();
                        check_critical(reply(0, 0, &sample)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::ReadMultiple(mut read_multiple) => {
                        adc.read_samples(&mut read_multiple.buffer.buffer[..read_multiple.buffer.len], read_multiple.src);
                        check_critical(reply(0, 0, &read_multiple.buffer.buffer[..read_multiple.buffer.len])).unwrap_or(Ok(())).unwrap();
                    },
                    Request::ReadRoundRobin(mut round_robin) => {
                        let mut samples = [0; 5];
                        let mut sample_len = 0;
                        let mut mask = round_robin.bitmap;
                        let mut sample: u8 = 0;
                        while mask > 0 {
                            if mask & 1 != 0 {
                                samples[sample_len] = sample;
                                sample_len += 1;
                            }
                            sample += 1;
                            mask >>= 1;
                        }
                        adc.read_samples_round_robin(&mut round_robin.buffer.buffer[..round_robin.buffer.len], &samples[..sample_len], round_robin.start);
                        check_critical(reply(0, 0, &round_robin.buffer.buffer[..round_robin.buffer.len])).unwrap_or(Ok(())).unwrap();
                    }
                }
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

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {

    use small_os_lib::{kprint, kprintln};

    use super::*;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for ADC", tests.len());
        for test in tests {
            test();
        }
    }

    #[test_case]
    fn test_setup() {
        let args = args();
        let adc_base = args[0] as usize;
        let bitmap = args[1];
        let mut adc = unsafe {
            ADC::new(adc_base, bitmap)
        };
        kprintln!("Testing ADC setup");
        kprint!("Testing cs register ");
        let cs = field!(adc.registers, cs).read();
        assert_eq!(cs & cs_register::VALID_MASK, 0x103);
        kprintln!("[ok]");
        kprint!("Testing div register ");
        let div = field!(adc.registers, div).read();
        assert_eq!(div & div_register::VALID_MASK, 0);
        kprintln!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        kprintln!("Testing ADC register mask values");
        kprint!("Testing cs register ");
        assert_eq!(cs_register::VALID_MASK, 0x1f770f);
        kprintln!("[ok]");
        kprint!("Testing result register ");
        assert_eq!(result_register::VALID_MASK, 0xfff);
        kprintln!("[ok]");
        kprint!("Testing fifo ctrl status register ");
        assert_eq!(fifo_ctrl_status_register::VALID_MASK, 0xf0f0f0f);
        kprintln!("[ok]");
        kprint!("Testing fifo register ");
        assert_eq!(fifo_register::VALID_MASK, 0x8fff);
        kprintln!("[ok]");
        kprint!("Testing div register ");
        assert_eq!(div_register::VALID_MASK, 0xffffff);
        kprintln!("[ok]");
        kprint!("Testing interrupt register ");
        assert_eq!(inter_register::VALID_MASK, 0x1);
        kprintln!("[ok]");
    }
}
