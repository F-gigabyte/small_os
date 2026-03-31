#![no_std]
#![no_main]

use small_os_lib::{HeaderError, QueueError, WakeSrc, check_critical, check_header_len, clear_irq, notify_reply_empty, notify_send, read_header, receive, reply, reply_empty, send_empty, wait_irq, wait_queues_irq};
use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, WriteOnly}};

mod armed_register {
    pub const ARMED_SHIFT: usize = 0;

    pub const ARMED_MASK: u32 = 0xf << ARMED_SHIFT;

    pub const ARMED1: u32 = 1 << ARMED_SHIFT;
    pub const ARMED2: u32 = 1 << (ARMED_SHIFT + 1);
    pub const ARMED3: u32 = 1 << (ARMED_SHIFT + 2);
    pub const ARMED4: u32 = 1 << (ARMED_SHIFT + 3);
}

mod debug_pause_register {
    pub const DEBUG0_SHIFT: usize = 1;
    pub const DEBUG1_SHIFT: usize = 2;

    pub const DEBUG0_MASK: u32 = 1 << DEBUG0_SHIFT;
    pub const DEBUG1_MASK: u32 = 1 << DEBUG1_SHIFT;
}

mod interrupt_register {
    pub const ALARM0_SHIFT: usize = 0;
    pub const ALARM1_SHIFT: usize = 1;
    pub const ALARM2_SHIFT: usize = 2;
    pub const ALARM3_SHIFT: usize = 3;

    pub const ALARM0_MASK: u32 = 1 << ALARM0_SHIFT;
    pub const ALARM1_MASK: u32 = 1 << ALARM1_SHIFT;
    pub const ALARM2_MASK: u32 = 1 << ALARM2_SHIFT;
    pub const ALARM3_MASK: u32 = 1 << ALARM3_SHIFT;

    pub const ALL_MASK: u32 = ALARM0_MASK | ALARM1_MASK | ALARM2_MASK | ALARM3_MASK;
}

struct TimerRegisters {
    timehw: WriteOnly<u32>, // 0x0
    timelw: WriteOnly<u32>, // 0x4
    timehr: ReadPure<u32>, // 0x8
    timelr: ReadPure<u32>, // 0xc
    alarm0: ReadPureWrite<u32>, // 0x10
    alarm1: ReadPureWrite<u32>, // 0x14
    alarm2: ReadPureWrite<u32>, // 0x18
    alarm3: ReadPureWrite<u32>, // 0x1c
    armed: ReadPureWrite<u32>, // 0x20
    timerawh: ReadPure<u32>, // 0x24
    timerawl: ReadPure<u32>, // 0x28
    debug_pause: ReadPureWrite<u32>, // 0x2c
    pause: ReadPureWrite<u32>, // 0x30
    int_raw: ReadPureWrite<u32>, // 0x34
    int_enable: ReadPureWrite<u32>, // 0x38
    int_force: ReadPureWrite<u32>, // 0x3c
    int_status: ReadPure<u32> // 0x40
}

pub struct Timer {
    registers: UniqueMmioPointer<'static, TimerRegisters>,
    set_reg: UniqueMmioPointer<'static, TimerRegisters>,
    clear_reg: UniqueMmioPointer<'static, TimerRegisters>,
    notifiers: [bool; 4]
}

impl Timer {
    pub unsafe fn new(timer_base: usize) -> Self {
        let mut res = unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(timer_base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(timer_base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(timer_base)).unwrap()),
                notifiers: [false; 4]
            }
        };
        field!(res.registers, debug_pause).write(0);
        res
    }

    pub fn read_time(&mut self) -> u64 {
        let lower = field!(self.registers, timelr).read();
        let upper = field!(self.registers, timehr).read();
        ((upper as u64) << 32) | (lower as u64)
    }
    
    #[inline(always)]
    pub fn enable_irq(&mut self, timer: u8) { 
        assert!(timer < 4);
        field!(self.set_reg, int_enable).write(1 << timer);
    }
    
    #[inline(always)]
    pub fn disable_irq(&mut self, timer: u8) { 
        assert!(timer < 4);
        field!(self.clear_reg, int_enable).write(1 << timer);
    }
    
    #[inline(always)]
    pub fn clear_irq(&mut self, timer: u8) {
        assert!(timer < 4);
        field!(self.set_reg, int_raw).write(1 << timer);
    }

    pub fn set_timer0(&mut self, count: u32) {
        // do this step so we lock the time registers in the middle
        let lower = field!(self.registers, timelr).read();
        let time = lower.wrapping_add(count);
        field!(self.registers, alarm0).write(time);
        field!(self.registers, timehr).read();
        self.enable_irq(0);
    }
    
    pub fn set_timer1(&mut self, count: u32) {
        // do this step so we lock the time registers in the middle
        let lower = field!(self.registers, timelr).read();
        let time = lower.wrapping_add(count);
        field!(self.registers, alarm1).write(time);
        field!(self.registers, timehr).read();
        self.enable_irq(1);
    }
    
    pub fn set_timer2(&mut self, count: u32) {
        // do this step so we lock the time registers in the middle
        let lower = field!(self.registers, timelr).read();
        let time = lower.wrapping_add(count);
        field!(self.registers, alarm2).write(time);
        field!(self.registers, timehr).read();
        self.enable_irq(2);
    }
    
    pub fn set_timer3(&mut self, count: u32) {
        // do this step so we lock the time registers in the middle
        let lower = field!(self.registers, timelr).read();
        let time = lower.wrapping_add(count);
        field!(self.registers, alarm3).write(time);
        field!(self.registers, timehr).read();
        self.enable_irq(3);
    }

    pub fn get_slot(&mut self) -> Option<u8> {
        if !self.notifiers[0] {
            Some(0)
        } else if !self.notifiers[1] {
            Some(1)
        } else if !self.notifiers[2] {
            Some(2)
        } else if !self.notifiers[3] {
            Some(3)
        } else {
            None
        }
    }

    pub fn set_timer(&mut self, slot: u8, time: u32) {
        assert!(slot < 4);
        match slot {
            0 => {
                self.set_timer0(time);
                self.notifiers[0] = true;
            },
            1 => {
                self.set_timer1(time);
                self.notifiers[1] = true;
            },
            2 => {
                self.set_timer2(time);
                self.notifiers[2] = true;
            },
            3 => {
                self.set_timer3(time);
                self.notifiers[3] = true;
            },
            _ => unreachable!()
        }
    }

    pub fn handle_irqs(&mut self, mut mask: u32) {
        let mut index = 0;
        while mask > 0 && index < 4 {
            if mask & 1 != 0 {
                self.clear_irq(index as u8);
                if self.notifiers[index] {
                    check_critical(notify_reply_empty(index as u32, 0)).unwrap_or(Ok(())).unwrap();
                    self.notifiers[index] = false;
                }
                index += 1;
                mask >>= 1;
            }
        }
        clear_irq().unwrap();
    }
}

pub enum TimerReplyError {
    SendError,
    InvalidRequest,
    TooLarge,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<TimerReplyError> for u32 {
    fn from(value: TimerReplyError) -> Self {
        match value {
            TimerReplyError::SendError => 1,
            TimerReplyError::InvalidRequest => 2,
            TimerReplyError::TooLarge => 3,
            TimerReplyError::InvalidSendBuffer => 4,
            TimerReplyError::InvalidReplyBuffer => 5,
        }
    }
}

impl From<HeaderError> for TimerReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

pub enum TimerError {
    ReplyError(TimerReplyError),
    QueueError(QueueError)
}

impl From<TimerReplyError> for TimerError {
    fn from(value: TimerReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for TimerError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

impl From<HeaderError> for TimerError {
    fn from(value: HeaderError) -> Self {
        Self::from(TimerReplyError::from(value))
    }
}

pub enum Request {
    Sleep(u32),
    ReadTime
}

impl Request {
    pub fn parse() -> Result<Self, TimerError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // wait micros
                let mut buffer = [0; 4];
                check_header_len(&header, 4, 0)?;
                _ = receive(0, &mut buffer)?;
                let micros = u32::from_le_bytes(buffer);
                Ok(Request::Sleep(micros))
            },
            1 => {
                // wait millis
                let mut buffer = [0; 4];
                check_header_len(&header, 4, 0)?;
                _ = receive(0, &mut buffer)?;
                let millis = u32::from_le_bytes(buffer);
                if millis > u32::MAX / 1000 {
                    return Err(TimerError::ReplyError(TimerReplyError::TooLarge));
                }
                Ok(Request::Sleep(millis * 1000))
            },
            2 => {
                check_header_len(&header, 0, 8)?;
                // get time
                Ok(Request::ReadTime)
            },
            _ => Err(TimerError::ReplyError(TimerReplyError::InvalidRequest))
        }
    }
}

const RESET_QUEUE: u32 = 0;

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(num_args: u32, timer_base: usize) {
    assert!(num_args == 1);
    send_empty(RESET_QUEUE, 0, &[]).unwrap();
    let mut timer = unsafe {
        Timer::new(timer_base)
    };
    loop {
        match wait_queues_irq(1).unwrap() {
            WakeSrc::IRQ(irq) => {
                timer.handle_irqs(irq);
            },
            WakeSrc::Queue(_) => {
                match Request::parse() {
                    Ok(request) => {
                        match request {
                            Request::Sleep(micros) => {
                                let slot = loop {
                                    if let Some(slot) = timer.get_slot() {
                                        break slot;
                                    } else {
                                        let irq = wait_irq().unwrap();
                                        timer.handle_irqs(irq);
                                    }
                                };
                                notify_send(0, slot as u32).unwrap();
                                timer.set_timer(slot, micros);
                            },
                            Request::ReadTime => {
                                check_critical(reply(0, 0, &timer.read_time().to_le_bytes())).unwrap_or(Ok(())).unwrap();
                            }
                        }
                    },
                    Err(err) => {
                        match err {
                            TimerError::ReplyError(err) => {
                                check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                            },
                            TimerError::QueueError(err) => {
                                match err {
                                    QueueError::Died => {},
                                    QueueError::SenderInvalidMemoryAccess => {
                                        check_critical(reply_empty(0, u32::from(TimerReplyError::SendError))).unwrap_or(Ok(())).unwrap();
                                    },
                                    _ => {
                                        panic!("{:?}", err);
                                    }
                                }
                            }
                        }
                    }
                }
            },
            src => {
                panic!("Have unknown source {:?}", src);
            }
        }
    }
}
