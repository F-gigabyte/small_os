use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite}};

use crate::mutex::SpinIRQ;

#[repr(C)]
struct ROSCRegisters {
    ctrl: ReadPureWrite<u32>, // 0x0
    freqa: ReadPureWrite<u32>, // 0x4
    freqb: ReadPureWrite<u32>, // 0x8
    dormant: ReadPureWrite<u32>, // 0xc
    div: ReadPureWrite<u32>, // 0x10
    phase: ReadPureWrite<u32>, // 0x14
    status: ReadPureWrite<u32>, // 0x18
    random_bit: ReadPure<u32>, //0x1c
    count: ReadPureWrite<u32>, // 0x20
}

mod ctrl_register {
    pub const FREQ_RANGE_SHIFT: usize = 0;
    pub const ENABLE_SHIFT: usize = 12;

    pub const FREQ_RANGE_MASK: u32 = 0xfff << FREQ_RANGE_SHIFT;
    pub const ENABLE_MASK: u32 = 0xfff << ENABLE_SHIFT;

    pub const FREQ_LOW: u32 = 0xfa4 << FREQ_RANGE_SHIFT;
    pub const FREQ_MEDIUM: u32 = 0xfa5 << FREQ_RANGE_SHIFT;
    pub const FREQ_HIGH: u32 = 0xfa7 << FREQ_RANGE_SHIFT;
    pub const FREQ_TOOHIGH: u32 = 0xfa6 << FREQ_RANGE_SHIFT;

    pub const ENABLE_DISABLE: u32 = 0xd1e << ENABLE_SHIFT;
    pub const ENABLE_ENABLE: u32 = 0xfab << ENABLE_SHIFT;
    
    pub const VALID_MASK: u32 = FREQ_RANGE_MASK |
        ENABLE_MASK;
}

mod freqa_register {
    pub const DS0_SHIFT: usize = 0;
    pub const DS1_SHIFT: usize = 4;
    pub const DS2_SHIFT: usize = 8;
    pub const DS3_SHIFT: usize = 12;
    pub const PASSWD_SHIFT: usize = 16;

    pub const DS0_MASK: u32 = 0x7 << DS0_SHIFT;
    pub const DS1_MASK: u32 = 0x7 << DS1_SHIFT;
    pub const DS2_MASK: u32 = 0x7 << DS2_SHIFT;
    pub const DS3_MASK: u32 = 0x7 << DS3_SHIFT;
    pub const PASSWD_MASK: u32 = 0xffff << PASSWD_SHIFT;
    
    pub const PASSWD_PASS: u32 = 0x9696 << PASSWD_SHIFT;

    pub const VALID_MASK: u32 = DS0_MASK |
        DS1_MASK |
        DS2_MASK |
        DS3_MASK |
        PASSWD_MASK;
}

mod freqb_register {
    pub const DS4_SHIFT: usize = 0;
    pub const DS5_SHIFT: usize = 4;
    pub const DS6_SHIFT: usize = 8;
    pub const DS7_SHIFT: usize = 12;
    pub const PASSWD_SHIFT: usize = 16;

    pub const DS4_MASK: u32 = 0x7 << DS4_SHIFT;
    pub const DS5_MASK: u32 = 0x7 << DS5_SHIFT;
    pub const DS6_MASK: u32 = 0x7 << DS6_SHIFT;
    pub const DS7_MASK: u32 = 0x7 << DS7_SHIFT;
    pub const PASSWD_MASK: u32 = 0xffff << PASSWD_SHIFT;
    
    pub const PASSWD_PASS: u32 = 0x9696 << PASSWD_SHIFT;

    pub const VALID_MASK: u32 = DS4_MASK |
        DS5_MASK |
        DS6_MASK |
        DS7_MASK |
        PASSWD_MASK;
}

mod dormant_register {
    pub const VALUE_SHIFT: usize = 0;
    
    pub const VALUE_MASK: u32 = 0xffffffff << VALUE_SHIFT;

    pub const DORMANT: u32 = 0x636f6d61;
    pub const WAKE: u32 = 0x77616b65;

    pub const VALID_MASK: u32 = VALUE_MASK;
}

mod div_register {
    pub const DIV_SHIFT: usize = 0;

    pub const DIV_MASK: u32 = 0xfff << DIV_SHIFT;

    pub const DIV_MIN: u32 = 0xaa0 << DIV_SHIFT;
    pub const DIV_MAX: u32 = DIV_MIN + (0x31 << DIV_SHIFT);

    pub const VALID_MASK: u32 = DIV_MASK;
}

mod phase_register {
    pub const SHIFT_SHIFT: usize = 0;
    pub const FLIP_SHIFT: usize = 2;
    pub const ENABLE_SHIFT: usize = 3;
    pub const PASSWD_SHIFT: usize = 4;

    pub const SHIFT_MASK: u32 = 0x3 << SHIFT_SHIFT;
    pub const FLIP_MASK: u32 = 1 << FLIP_SHIFT;
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const PASSWD_MASK: u32 = 0xff << PASSWD_SHIFT;

    pub const PASSWD_PASS: u32 = 0xaa << PASSWD_SHIFT;

    pub const VALID_MASK: u32 = SHIFT_MASK |
        FLIP_MASK |
        ENABLE_MASK |
        PASSWD_MASK;
}

mod status_register {
    pub const ENABLED_SHIFT: usize = 12;
    pub const DIV_RUNNING_SHIFT: usize = 16;
    pub const BADWRITE_SHIFT: usize = 24;
    pub const STABLE_SHIFT: usize = 31;

    pub const ENABLED_MASK: u32 = 1 << ENABLED_SHIFT;
    pub const DIV_RUNNING_MASK: u32 = 1 << DIV_RUNNING_SHIFT;
    pub const BADWRITE_MASK: u32 = 1 << BADWRITE_SHIFT;
    pub const STABLE_MASK: u32 = 1 << STABLE_SHIFT;

    pub const VALID_MASK: u32 = ENABLED_MASK |
        DIV_RUNNING_MASK |
        BADWRITE_MASK |
        STABLE_MASK;
}

mod random_bit_register {
    pub const RANDOM_BIT_SHIFT: usize = 0;

    pub const RANDOM_BIT_MASK: u32 = 1 << RANDOM_BIT_SHIFT;

    pub const VALID_MASK: u32 = RANDOM_BIT_MASK;
}

mod count_register {
    pub const COUNT_SHIFT: usize = 0;

    pub const COUNT_MASK: u32 = 0xff << COUNT_SHIFT;

    pub const VALID_MASK: u32 = COUNT_MASK;
}

pub struct ROSC {
    registers: UniqueMmioPointer<'static, ROSCRegisters>
}

impl ROSC {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    #[cfg(feature = "random")]
    pub fn get_random(&mut self, data: &mut [u8], bits: usize) {
        assert!(data.len() * 8 >= bits);
        for i in 0..bits / 8 {
            let mut byte = 0;
            for _ in 0..8 {
                byte |= ((field!(self.registers, random_bit).read() & random_bit_register::RANDOM_BIT_MASK) >> random_bit_register::RANDOM_BIT_SHIFT) as u8;
                byte <<= 1;
            }
            data[i] = byte;
        }
        let mut final_byte = 0;
        for j in 0..bits % 8 {
            final_byte |= ((field!(self.registers, random_bit).read() & random_bit_register::RANDOM_BIT_MASK) >> random_bit_register::RANDOM_BIT_SHIFT) as u8;
            final_byte <<= 1;
        }
        if bits % 8 != 0 {
            data[bits / 8] = final_byte;
        }
    }

    pub fn enable(&mut self) {
        field!(self.registers, phase).modify(|phase| 
            (phase & !phase_register::VALID_MASK) |
            phase_register::PASSWD_PASS |
            phase_register::ENABLE_MASK
        );
        field!(self.registers, div).modify(|div|
            (div & !div_register::VALID_MASK) |
            ((0xaa0 + 16) << div_register::DIV_SHIFT)
        );
        field!(self.registers, freqa).modify(|freqa|
            (freqa & !freqa_register::VALID_MASK) |
            freqa_register::PASSWD_PASS
        );
        field!(self.registers, freqb).modify(|freqb|
            (freqb & !freqa_register::VALID_MASK) |
            freqb_register::PASSWD_PASS
        );
        field!(self.registers, ctrl).modify(|ctrl|
            (ctrl & !ctrl_register::VALID_MASK) |
            ctrl_register::ENABLE_ENABLE |
            ctrl_register::FREQ_LOW
        );
        while field!(self.registers, status).read() & status_register::STABLE_MASK == 0 {}
    }

    pub fn disable(&mut self) {
        field!(self.registers, ctrl).modify(|ctrl| (ctrl & !ctrl_register::VALID_MASK) | ctrl_register::ENABLE_DISABLE);
    }
}

unsafe impl Send for ROSC {}
unsafe impl Sync for ROSC {}

static ROSC_BASE: usize = 0x40060000;

pub static ROSC: SpinIRQ<ROSC> = unsafe {
    SpinIRQ::new(ROSC::new(ROSC_BASE))
};

#[cfg(test)]
mod test {
    use crate::{CS, print, println};

    use super::*;

    #[test_case]
    fn test_setup_correct() {
        #[cfg(not(feature = "random"))]
        {
            println!("Testing ROSC setup");
            let cs = unsafe {
                CS::new()
            };
            let mut rosc = ROSC.lock(&cs);
            print!("Testing ctrl register ");
            let ctrl = field!(rosc.registers, ctrl).read();
            // ROSC disabled
            assert_eq!(ctrl & ctrl_register::ENABLE_MASK, 0xd1e000);
            println!("[ok]");
        }
    }

    #[test_case]
    fn test_valid() {
        println!("Testing ROSC register mask values");
        print!("Testing ctrl register ");
        assert_eq!(ctrl_register::VALID_MASK, 0xffffff);
        println!("[ok]");
        print!("Testing freqa register ");
        assert_eq!(freqa_register::VALID_MASK, 0xffff7777);
        println!("[ok]");
        print!("Testing freqb register ");
        assert_eq!(freqb_register::VALID_MASK, 0xffff7777);
        println!("[ok]");
        print!("Testing dormant register ");
        assert_eq!(dormant_register::VALID_MASK, 0xffffffff);
        println!("[ok]");
        print!("Testing div register ");
        assert_eq!(div_register::VALID_MASK, 0xfff);
        println!("[ok]");
        print!("Testing phase register ");
        assert_eq!(phase_register::VALID_MASK, 0xfff);
        println!("[ok]");
        print!("Testing status register ");
        assert_eq!(status_register::VALID_MASK, 0x81011000);
        println!("[ok]");
        print!("Testing random bit register ");
        assert_eq!(random_bit_register::VALID_MASK, 0x1);
        println!("[ok]");
        print!("Testing count register ");
        assert_eq!(count_register::VALID_MASK, 0xff);
        println!("[ok]");
    }
}
