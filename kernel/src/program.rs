use core::{mem, ptr, slice};

use crate::{mutex::Spin, println};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegionAttr {
    R = 0b100,
    RW = 0b110,
    RX = 0b101
}


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AccessAttr {
    attr: u8
}

impl AccessAttr {
    pub const READ_SHIFT: usize = 2;
    pub const WRITE_SHIFT: usize = 1;
    pub const EXEC_SHIFT: usize = 0;
    pub const READ_MASK: u8 = 1 << Self::READ_SHIFT;
    pub const WRITE_MASK: u8 = 1 << Self::WRITE_SHIFT;
    pub const EXEC_MASK: u8 = 1 << Self::EXEC_SHIFT;

    pub fn new(read: bool, write: bool, exec: bool) -> Self {
        let mut attr = 0;
        if read {
            attr |= Self::READ_MASK;
        }
        if write {
            attr |= Self::WRITE_MASK;
        }
        if exec {
            attr |= Self::EXEC_MASK;
        }
        Self { 
            attr 
        }
    }

    pub fn read(&self) -> bool {
        self.attr & Self::READ_MASK != 0
    }

    pub fn write(&self) -> bool {
        self.attr & Self::WRITE_MASK != 0
    }
    
    pub fn exec(&self) -> bool {
        self.attr & Self::EXEC_MASK != 0
    }

    pub fn set_read(&mut self, read: bool) {
        if read {
            self.attr |= Self::READ_MASK;
        } else {
            self.attr &= !Self::READ_MASK;
        }
    }
    
    pub fn set_write(&mut self, write: bool) {
        if write {
            self.attr |= Self::WRITE_MASK;
        } else {
            self.attr &= !Self::WRITE_MASK;
        }
    }
    
    pub fn set_exec(&mut self, exec: bool) {
        if exec {
            self.attr |= Self::EXEC_MASK;
        } else {
            self.attr &= !Self::EXEC_MASK;
        }
    }
}

impl RegionAttr {
    pub fn access_valid(&self, access: AccessAttr) -> bool {
        if access.write() && !self.write() {
            false
        } else if access.read() && !self.read() {
            false
        } else if access.exec() && !self.exec() {
            false
        } else {
            true
        }
    }

    #[inline(always)]
    pub fn read(&self) -> bool {
        true
    }

    #[inline(always)]
    pub fn write(&self) -> bool {
        match self {
            Self::RW => true,
            _ => false,
        }
    }

    #[inline(always)]
    pub fn exec(&self) -> bool {
        match self {
            Self::RX => true,
            _ => false,
        }
    }
}

#[repr(C)]
#[derive(Clone)]
pub struct Region {
    pub phys_addr: u32, // 0x0
    pub virt_addr: u32, // 0x4
    pub len: u32 // 0x8
}

impl Region {
    pub const ENABLE_SHIFT: usize = 0;
    pub const PERM_SHIFT: usize = 1;
    pub const DEVICE_SHIFT: usize = 4;

    pub const ENABLE_MASK: u32 = 1 << Self::ENABLE_SHIFT;
    pub const DEVICE_MASK: u32 = 1 << Self::DEVICE_SHIFT;
    pub const PERM_MASK: u32 = 0x7 << Self::PERM_SHIFT;
    pub const ADDR_MASK: u32 = 0xffffff00;

    pub const fn default() -> Self {
        Self {
            phys_addr: 0,
            virt_addr: 0,
            len: 0
        }
    }

    pub fn map_device(&mut self, addr: u32, len: u32) -> Result<(), ()> {
        if !len.is_power_of_two() || len < 256 || !addr.is_multiple_of(len) {
            return Err(());
        }
        self.virt_addr = addr | 
            Self::ENABLE_MASK | 
            Self::DEVICE_MASK | 
            ((RegionAttr::RW as u32) << Self::PERM_SHIFT);
        self.phys_addr = addr;
        Ok(())
    }

    #[inline(always)]
    pub fn get_virt(&self) -> u32 {
        self.virt_addr & Self::ADDR_MASK
    }

    #[inline(always)]
    pub fn get_phys(&self) -> u32 {
        self.phys_addr & Self::ADDR_MASK
    }

    #[inline(always)]
    pub fn get_attr(&self) -> Result<RegionAttr, u32> {
        match (self.virt_addr & Self::PERM_MASK) >> Self::PERM_SHIFT {
            0b100 => Ok(RegionAttr::R),
            0b110 => Ok(RegionAttr::RW),
            0b101 => Ok(RegionAttr::RX),
            val => Err(val)
        }
    }

    #[inline(always)]
    pub fn is_device(&self) -> bool {
        self.virt_addr & Self::DEVICE_MASK != 0
    }

    #[inline(always)]
    pub fn enabled(&self) -> bool {
        self.virt_addr & Self::ENABLE_MASK != 0
    }
}

pub enum ProgramDriver {

}

#[repr(C)]
pub struct Program {
    pub flags: u32,
    pub sp: u32,
    pub entry: u32,
    pub regions: [Region; 8],
}

impl Program {
    const PRIORITY_SHIFT: usize = 0;
    const DRIVER_SHIFT: usize = 16;

    const PRIORITY_MASK: u32 = 0xff << Self::PRIORITY_SHIFT;
    const DRIVER_MASK: u32 = 0xffff << Self::DRIVER_SHIFT;

    pub fn priority(&self) -> u8 {
        ((self.flags & Self::PRIORITY_MASK) >> Self::PRIORITY_SHIFT) as u8
    }
    
    pub fn driver(&self) -> u16 {
        ((self.flags & Self::DRIVER_MASK) >> Self::DRIVER_SHIFT) as u16
    }
}

pub struct ProgramTable {
    table: &'static [Program]
}

impl ProgramTable {
    pub unsafe fn new(base: usize) -> Self {
        let table_len: u32 = unsafe {
            *ptr::with_exposed_provenance(base)
        };
        println!("Base is 0x{:x}, need alignment of 0x{:x} and have len 0x{:x}", base + 4, mem::align_of::<Program>(), table_len);
        let table = unsafe {
            slice::from_raw_parts(ptr::with_exposed_provenance(base + 4), table_len as usize)
        };
        Self { 
            table 
        }
    }

    #[inline(always)]
    pub fn table(&self) -> &'static [Program] {
        self.table
    }
}

pub static PROGRAM_TABLE: Spin<Option<ProgramTable>> = Spin::new(None);
