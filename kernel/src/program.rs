use core::{cell::UnsafeCell, mem, ptr, slice};

use crate::{inter::CS, message_queue::{AsyncMessageQueue, SyncMessageQueue}, println, proc::Proc, scheduler::scheduler};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegionAttr {
    R = 0b100,
    RW = 0b110,
    RX = 0b101
}

unsafe extern "C" {
    static mut __program_table: u8;
    static __procs_virt_start: u8;
    static __procs_virt_end: u8;
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
    pub const ZERO_SHIFT: usize = 5;

    pub const ENABLE_MASK: u32 = 1 << Self::ENABLE_SHIFT;
    pub const DEVICE_MASK: u32 = 1 << Self::DEVICE_SHIFT;
    pub const ZERO_MASK: u32 = 1 << Self::ZERO_SHIFT;
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
    pub fn should_zero(&self) -> bool {
        self.virt_addr & Self::ZERO_MASK != 0
    }

    #[inline(always)]
    pub fn enabled(&self) -> bool {
        self.virt_addr & Self::ENABLE_MASK != 0
    }
}

#[repr(C)]
pub struct Program {
    pub flags: u32,
    pub sp: u32,
    pub entry: u32,
    pub regions: [Region; 8],
    pub num_sync_queues: u32,
    pub num_sync_endpoints: u32,
    pub sync_queues: *mut SyncMessageQueue,
    pub sync_endpoints: *const *mut SyncMessageQueue,
    pub num_async_queues: u32,
    pub num_async_endpoints: u32,
    pub async_queues: *mut AsyncMessageQueue,
    pub async_endpoints: *const *mut AsyncMessageQueue
}

impl Program {
    const PRIORITY_SHIFT: usize = 0;
    const DRIVER_SHIFT: usize = 16;
    const INTERRUPT_SHIFT: usize = 8;

    const PRIORITY_MASK: u32 = 0xff << Self::PRIORITY_SHIFT;
    const DRIVER_MASK: u32 = 0xffff << Self::DRIVER_SHIFT;
    const INTERRUPT_MASK: u32 = 0xff << Self::INTERRUPT_SHIFT;
    const INTERRUPT_NONE: u8 = 0xff;

    pub fn priority(&self) -> u8 {
        ((self.flags & Self::PRIORITY_MASK) >> Self::PRIORITY_SHIFT) as u8
    }
    
    pub fn driver(&self) -> u16 {
        ((self.flags & Self::DRIVER_MASK) >> Self::DRIVER_SHIFT) as u16
    }

    pub fn interrupt(&self) -> Option<u8> {
        let inter = ((self.flags & Self::INTERRUPT_MASK) >> Self::INTERRUPT_SHIFT) as u8;
        if inter >= 32 {
            None
        } else {
            Some(inter)
        }
    }

    pub fn add_region(&mut self) -> Option<&mut Region> {
        for i in 0..self.regions.len() {
            if !self.regions[i].enabled() {
                return Some(&mut self.regions[i])
            }
        }
        None
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

/// SAFETY
/// this function is non-reentrant
pub unsafe fn init_processes(cs: &CS) {
    let num_programs: u32 = unsafe {
        *ptr::with_exposed_provenance((&raw const __program_table) as usize)
    };
    let programs: &mut[Program] = unsafe {
        slice::from_raw_parts_mut(ptr::without_provenance_mut(((&raw mut __program_table) as usize) + 4), num_programs as usize)
    };
    let mut scheduler = scheduler(&cs);
    let procs_start = &raw const __procs_virt_start as usize;
    let procs_end = &raw const __procs_virt_end as usize;
    let len = (procs_end - procs_start) / mem::size_of::<Proc>();
    let processes: &mut [UnsafeCell<Proc>] = unsafe {
        slice::from_raw_parts_mut(ptr::with_exposed_provenance_mut(procs_start), len)
    };
    scheduler.init(processes);
    let mut current_pid = 0;
    let mut current_driver = 0;
    for program in programs {
        println!("Driver {}", program.driver());
        if program.driver() <= current_driver && current_driver != 0 {
            panic!("Invalid driver ordering for programs");
        } else {
            current_driver = program.driver();
        }
        for region in &program.regions {
            if region.enabled() {
                let phys_addr = region.get_phys();
                let virt_addr = region.get_virt();
                println!("phys address: 0x{:x}", phys_addr);
                println!("virt address: 0x{:x}", virt_addr);
                println!("length: 0x{:x}", region.len);
                if let Err(err) = region.get_attr() {
                    panic!("Have invalid region permission bits of 0b{:b}", err);
                }
                if !region.len.is_power_of_two() || region.len < 256 {
                    panic!("Have invalid region size of {}", region.len);
                }
                if !virt_addr.is_multiple_of(region.len) {
                    panic!("Have invalid region alignment with address 0x{:x} for size 0x{:x}", virt_addr, region.len);
                }
                if region.should_zero() {
                    let mut virt_ptr: *mut u32 = ptr::with_exposed_provenance_mut(virt_addr as usize);
                    for _ in 0..region.len / 4 {
                        unsafe {
                            virt_ptr.write_volatile(0);
                            virt_ptr = virt_ptr.add(1);
                        }
                    }
                } else if phys_addr != virt_addr {
                    let mut phys_ptr: *const u32 = ptr::with_exposed_provenance(phys_addr as usize);
                    let mut virt_ptr: *mut u32 = ptr::with_exposed_provenance_mut(virt_addr as usize);
                    for _ in 0..region.len / 4 {
                        unsafe {
                            virt_ptr.write_volatile(phys_ptr.read_volatile());
                            phys_ptr = phys_ptr.add(1);
                            virt_ptr = virt_ptr.add(1);
                        }
                    }
                }
            }
        }
        println!("Entry: 0x{:x}", program.entry);
        println!("Stack: 0x{:x}", program.sp);
        let driver = program.driver() as usize;
        let r0 = if driver != 0 {
            program.regions[0].virt_addr
        } else {
            0
        };
        let pid = unsafe {
            scheduler.create_proc(current_pid, program, r0).unwrap()
        };
        scheduler.schedule_process(pid).unwrap();
        current_pid += 1;
    }
}
