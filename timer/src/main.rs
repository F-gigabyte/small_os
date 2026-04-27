/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Timer Driver.
 *
 * The SmallOS Timer Driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Timer Driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS Timer Driver. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use small_os_lib::{HeaderError, QueueError, WakeSrc, args, check_critical, check_header_len, clear_irq, notify_reply_empty, notify_send, read_header, receive, reply, reply_empty, send_empty, wait_irq, wait_queues_irq};
use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, WriteOnly}};

/// Timer armed register masks and shifts
mod armed_register {
    /// Shift for whether the timer is armed
    pub const ARMED_SHIFT: usize = 0;

    /// Mask for whether the timer is armed
    pub const ARMED_MASK: u32 = 0xf << ARMED_SHIFT;

    /// Timer 0 is armed
    pub const ARMED0: u32 = 1 << ARMED_SHIFT;
    /// Timer 1 is armed
    pub const ARMED1: u32 = 1 << (ARMED_SHIFT + 1);
    /// Timer 2 is armed
    pub const ARMED2: u32 = 1 << (ARMED_SHIFT + 2);
    /// Timer 3 is armed
    pub const ARMED3: u32 = 1 << (ARMED_SHIFT + 3);

    /// Mask of all non-reserved reset register bits
    pub const VALID_MASK: u32 = ARMED_MASK;
}

/// Timer debug pause register masks and shifts
mod debug_pause_register {
    /// Shift to pause the timer when processor 0 is in debug mode
    pub const DEBUG0_SHIFT: usize = 1;
    /// Shift to pause the timer when processor 1 is in debug mode
    pub const DEBUG1_SHIFT: usize = 2;

    /// Mask to pause timer when processor 0 is in debug mode
    pub const DEBUG0_MASK: u32 = 1 << DEBUG0_SHIFT;
    /// Mask to pause the timer when processor 1 is in debug mode
    pub const DEBUG1_MASK: u32 = 1 << DEBUG1_SHIFT;

    /// Mask of all non-reserved reset register bits
    pub const VALID_MASK: u32 = DEBUG0_MASK |
        DEBUG1_MASK;
}

/// Timer interrupt registers masks and shifts
mod interrupt_register {
    ///  Timer 0 interrupt shift
    pub const ALARM0_SHIFT: usize = 0;
    ///  Timer 1 interrupt shift
    pub const ALARM1_SHIFT: usize = 1;
    ///  Timer 2 interrupt shift
    pub const ALARM2_SHIFT: usize = 2;
    ///  Timer 3 interrupt shift
    pub const ALARM3_SHIFT: usize = 3;

    ///  Timer 0 interrupt mask
    pub const ALARM0_MASK: u32 = 1 << ALARM0_SHIFT;
    ///  Timer 1 interrupt mask
    pub const ALARM1_MASK: u32 = 1 << ALARM1_SHIFT;
    ///  Timer 2 interrupt mask
    pub const ALARM2_MASK: u32 = 1 << ALARM2_SHIFT;
    ///  Timer 3 interrupt mask
    pub const ALARM3_MASK: u32 = 1 << ALARM3_SHIFT;

    /// Mask of all non-reserved reset register bits
    pub const VALID_MASK: u32 = ALARM0_MASK | 
        ALARM1_MASK | 
        ALARM2_MASK | 
        ALARM3_MASK;
}

/// Timer memory mapped registers
struct TimerRegisters {
    /// Write to upper time bits (should write to lower first) (0x00)
    timehw: WriteOnly<u32>, // 0x0
    /// Write to lower time bits (should write to upper next) (0x04)
    timelw: WriteOnly<u32>, // 0x4
    /// Read from upper time bits (should read from lower first) (0x08)
    timehr: ReadPure<u32>, // 0x8
    /// Read from lower time bits (should read from upper next) (0x0c)
    timelr: ReadPure<u32>, // 0xc
    /// Arm timer 0 register (0x10)
    alarm0: ReadPureWrite<u32>, // 0x10
    /// Arm timer 1 register (0x14)
    alarm1: ReadPureWrite<u32>, // 0x14
    /// Arm timer 2 register (0x18)
    alarm2: ReadPureWrite<u32>, // 0x18
    /// Arm timer 3 register (0x1c)
    alarm3: ReadPureWrite<u32>, // 0x1c
    /// Armed register (0x20)
    armed: ReadPureWrite<u32>, // 0x20
    /// Read from upper time bits with no side effects (0x24)
    timerawh: ReadPure<u32>, // 0x24
    /// Read from lower time bits with no side effects (0x28)
    timerawl: ReadPure<u32>, // 0x28
    /// Debug pause register (0x2c)
    debug_pause: ReadPureWrite<u32>, // 0x2c
    /// Pause register (0x30)
    pause: ReadPureWrite<u32>, // 0x30
    /// Raw interrupt register (0x34)
    int_raw: ReadPureWrite<u32>, // 0x34
    /// Interrupt enable register (0x38)
    int_enable: ReadPureWrite<u32>, // 0x38
    /// Interrupt force register (0x3c)
    int_force: ReadPureWrite<u32>, // 0x3c
    /// Interrupt status register (0x40)
    int_status: ReadPure<u32> // 0x40
}

/// Timer object for managing the timer device
pub struct Timer {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, TimerRegisters>,
    /// Memory mapped registers where writing a bit sets the corresponding bit in `registers`
    set_reg: UniqueMmioPointer<'static, TimerRegisters>,
    /// Memory mapped registers where writing a bit clears the corresponding bit in `registers`
    clear_reg: UniqueMmioPointer<'static, TimerRegisters>,
    /// Which notifier queues are in use
    notifiers: [bool; 4]
}

impl Timer {
    /// Creates a new `Timer` object and initialises it  
    /// `timer_base` is the base address of the Timer memory mapped registers
    /// # Safety
    /// `timer_base` must be a valid address which points to the Timer memory mapped registers and not
    /// being used by anything else
    pub unsafe fn new(timer_base: usize) -> Self {
        let mut res = unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(timer_base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(timer_base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(timer_base)).unwrap()),
                notifiers: [false; 4]
            }
        };
        field!(res.registers, debug_pause).modify(|debug_pause| debug_pause & !debug_pause_register::VALID_MASK);
        res
    }

    /// Reads the current time since boot
    pub fn read_time(&mut self) -> u64 {
        let lower = field!(self.registers, timelr).read();
        let upper = field!(self.registers, timehr).read();
        ((upper as u64) << 32) | (lower as u64)
    }
    
    /// Enables a timer IRQ  
    /// `timer` is which timer IRQ should be enabled  
    /// Panics if `timer` is 4 or more
    #[inline(always)]
    pub fn enable_irq(&mut self, timer: u8) { 
        assert!(timer < 4);
        field!(self.set_reg, int_enable).write(1 << timer);
    }
    
    /// Disables a timer IRQ  
    /// `timer` is which timer IRQ should be disabled  
    /// Panics if `timer` is 4 or more
    #[inline(always)]
    pub fn disable_irq(&mut self, timer: u8) { 
        assert!(timer < 4);
        field!(self.clear_reg, int_enable).write(1 << timer);
    }
    
    /// Clears a timer IRQ  
    /// `timer` is which timer IRQ should be disabled  
    /// Panics if `timer` is 4 or more
    #[inline(always)]
    pub fn clear_irq(&mut self, timer: u8) {
        assert!(timer < 4);
        field!(self.set_reg, int_raw).write(1 << timer);
    }

    /// Sets the timer 0 register, arming the timer  
    /// `count` is the number of microseconds into the future for when the interrupt should fire
    pub fn set_timer0(&mut self, count: u32) {
        // do this step so we lock the time registers in the middle
        let lower = field!(self.registers, timelr).read();
        let time = lower.wrapping_add(count);
        field!(self.registers, alarm0).write(time);
        field!(self.registers, timehr).read();
        self.enable_irq(0);
    }
    
    /// Sets the timer 1 register, arming the timer  
    /// `count` is the number of microseconds into the future for when the interrupt should fire
    pub fn set_timer1(&mut self, count: u32) {
        // do this step so we lock the time registers in the middle
        let lower = field!(self.registers, timelr).read();
        let time = lower.wrapping_add(count);
        field!(self.registers, alarm1).write(time);
        field!(self.registers, timehr).read();
        self.enable_irq(1);
    }
    
    /// Sets the timer 2 register, arming the timer  
    /// `count` is the number of microseconds into the future for when the interrupt should fire
    pub fn set_timer2(&mut self, count: u32) {
        // do this step so we lock the time registers in the middle
        let lower = field!(self.registers, timelr).read();
        let time = lower.wrapping_add(count);
        field!(self.registers, alarm2).write(time);
        field!(self.registers, timehr).read();
        self.enable_irq(2);
    }
    
    /// Sets the timer 3 register, arming the timer  
    /// `count` is the number of microseconds into the future for when the interrupt should fire
    pub fn set_timer3(&mut self, count: u32) {
        // do this step so we lock the time registers in the middle
        let lower = field!(self.registers, timelr).read();
        let time = lower.wrapping_add(count);
        field!(self.registers, alarm3).write(time);
        field!(self.registers, timehr).read();
        self.enable_irq(3);
    }

    /// Finds a notifier slot to put the next request in  
    /// Returns `None` if a slot can't be found
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

    /// Sets a timer register, arming the timer  
    /// `slot` is which timer to arm  
    /// `time` is the number of microseconds into the future for when the interrupt should fire  
    /// Panics if slot is 4 or more
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

    /// Handles any timer IRQs that fired  
    /// `mask` is the mask of timer IRQs that fired
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
        // clear IRQ at process and NVIC level or else IRQ lingers
        clear_irq().unwrap();
    }
}

/// Timer reply errors
pub enum TimerReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// Time request was too large
    TooLarge,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Converts from a `TimerReplyError` to a `u32`
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

/// Converts from a `HeaderError` to a `TimerReplyError`
impl From<HeaderError> for TimerReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

/// Timer Errors
pub enum TimerError {
    /// Error with the request
    ReplyError(TimerReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts from a `TimerReplyError` to a `TimerError`
impl From<TimerReplyError> for TimerError {
    fn from(value: TimerReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts from a `QueueError` to a `TimerError`
impl From<QueueError> for TimerError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// Converts from a `HeaderError` to a `TimerError`
impl From<HeaderError> for TimerError {
    fn from(value: HeaderError) -> Self {
        Self::from(TimerReplyError::from(value))
    }
}

/// Timer request
pub enum Request {
    /// Sleep micros
    Sleep(u32),
    /// Read time
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

/// Subsystem reset driver endpoint
const RESET_QUEUE: u32 = 0;

/// Driver entry point
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 1);
    send_empty(RESET_QUEUE, 0, &[]).unwrap();
    #[cfg(test)]
    test_main();
    let timer_base = args[0] as usize;
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

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::{kprint, kprintln};

    use super::*;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for timer", tests.len());
        for test in tests {
            test();
        }
    }

    #[test_case]
    fn test_valid() {
        kprintln!("Testing timer register mask values");
        kprint!("Testing armed register ");
        assert_eq!(armed_register::VALID_MASK, 0xf);
        kprintln!("[ok]");
        kprint!("Testing debug pause register ");
        assert_eq!(debug_pause_register::VALID_MASK, 0x6);
        kprintln!("[ok]");
        kprint!("Testing interrupt register ");
        assert_eq!(interrupt_register::VALID_MASK, 0xf);
        kprintln!("[ok]");
    }

    #[test_case]
    fn test_setup() {
        let args = args();
        let timer_base = args[0] as usize;
        let mut timer = unsafe {
            Timer::new(timer_base)
        };
        kprintln!("Testing timer setup");
        kprint!("Testing debug pause register ");
        let debug_pause = field!(timer.registers, debug_pause).read();
        assert_eq!(debug_pause & debug_pause_register::VALID_MASK, 0);
        kprintln!("[ok]");
    }
}
