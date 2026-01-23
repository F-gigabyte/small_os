use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, WriteOnly}};

use crate::{mmio::{REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS}, mutex::Mutex, println};

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

pub struct Timer {
    registers: UniqueMmioPointer<'static, TimerRegisters>,
    set_reg: UniqueMmioPointer<'static, TimerRegisters>,
    clear_reg: UniqueMmioPointer<'static, TimerRegisters>
}

#[derive(Clone, Copy)]
pub enum TimerIRQ {
    Timer0 = 0,
    Timer1 = 1,
    Timer2 = 2,
    Timer3 = 3
}

impl Timer {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap())
            }
        }
    }

    pub fn remove_debug_pause(&mut self) {
        field!(self.registers, debug_pause).write(0);
    }

    pub fn read_time(&mut self) -> u64 {
        let lower = field!(self.registers, timerawl).read();
        let upper = field!(self.registers, timerawh).read();
        (upper as u64) << 32 | (lower as u64)
    }

    pub fn set_count0(&mut self, micros: u32) {
        let time = (self.read_time() & (u32::MAX as u64)) as u32;
        let time = time.wrapping_add(micros);
        field!(self.registers, alarm0).write(time);
    }
    
    pub fn set_count1(&mut self, micros: u32) {
        let time = (self.read_time() & (u32::MAX as u64)) as u32;
        let time = time.wrapping_add(micros);
        field!(self.registers, alarm1).write(time);
    }
    
    pub fn set_count2(&mut self, micros: u32) {
        let time = (self.read_time() & (u32::MAX as u64)) as u32;
        let time = time.wrapping_add(micros);
        field!(self.registers, alarm2).write(time);
    }
    
    pub fn set_count3(&mut self, micros: u32) {
        let time = (self.read_time() & (u32::MAX as u64)) as u32;
        let time = time.wrapping_add(micros);
        field!(self.registers, alarm3).write(time);
    }

    pub fn enable_irq(&mut self, timer: TimerIRQ) { 
        field!(self.set_reg, int_enable).write(1 << (timer as usize));
    }
    
    pub fn disable_irq(&mut self, timer: TimerIRQ) { 
        field!(self.clear_reg, int_enable).write(1 << (timer as usize));
    }
    
    pub fn clear_irq(&mut self, timer: TimerIRQ) {
        field!(self.registers, int_raw).write(1 << (timer as usize));
    }

    pub fn get_irq(&mut self) -> u32 {
        field!(self.registers, int_status).read()
    }
}

unsafe impl Send for Timer {}
unsafe impl Sync for Timer {}

static TIMER_BASE: usize =  0x40054000;

pub static TIMER: Mutex<Timer> = unsafe {
    Mutex::new(Timer::new(TIMER_BASE))
};
