use core::{cell::UnsafeCell, mem, ptr, slice};

use crc32::{check_crc};
use hamming::{calc_symbol_len, calc_symbols, check_hamming_crc, correct_errors, update_msg};

use crate::{inter::CS, message_queue::{AsyncMessageQueue, SyncMessageQueue}, println, proc::Proc, scheduler::scheduler};

fn read_time() -> u64 {
    let addr = 0x40054000;
    let timer: *const u32 = ptr::with_exposed_provenance(addr);
    unsafe {
        let lower = timer.add(3).read_volatile();
        let upper = timer.add(4).read_volatile();
        ((upper as u64) << 32) | (lower as u64)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegionAttr {
    R = 0b00,
    RW = 0b10,
    RX = 0b01
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
    pub len: u32, // 0x8
    pub actual_len: u32,
    pub crc: u32,
    pub codes: u32,
}

impl Region {
    // len mask
    pub const ENABLE_SHIFT: usize = 0;
    pub const VIRTUAL_SHIFT: usize = 1;
    pub const PHYSICAL_SHIFT: usize = 2;
    pub const DEVICE_SHIFT: usize = 3;
    pub const PERM_SHIFT: usize = 4;
    pub const ZERO_SHIFT: usize = 6;
    pub const LEN_SHIFT: usize = 16;

    pub const ENABLE_MASK: u32 = 1 << Self::ENABLE_SHIFT;
    pub const VIRTUAL_MASK: u32 = 1 << Self::VIRTUAL_SHIFT;
    pub const PHYSICAL_MASK: u32 = 1 << Self::PHYSICAL_SHIFT;
    pub const DEVICE_MASK: u32 = 1 << Self::DEVICE_SHIFT;
    pub const PERM_MASK: u32 = 0x3 << Self::PERM_SHIFT;
    pub const ZERO_MASK: u32 = 1 << Self::ZERO_SHIFT;
    pub const LEN_MASK: u32 = 0xffff << Self::LEN_SHIFT;

    pub const fn default() -> Self {
        Self {
            phys_addr: 0,
            virt_addr: 0,
            len: 0,
            actual_len: 0,
            crc: 0,
            codes: 0,
        }
    }

    #[inline(always)]
    pub fn has_phys(&self) -> bool {
        self.len & Self::PHYSICAL_MASK != 0
    }
    
    #[inline(always)]
    pub fn has_virt(&self) -> bool {
        self.len & Self::VIRTUAL_MASK != 0
    }

    #[inline(always)]
    pub fn get_virt(&self) -> Option<u32> {
        if self.has_virt() {
            Some(self.virt_addr)
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_phys(&self) -> Option<u32> {
        if self.has_phys() {
            Some(self.phys_addr)
        } else {
            None
        }
    }

    pub fn get_runtime_addr(&self) -> Option<u32> {
        self.get_virt().or_else(|| self.get_phys())
    }

    #[inline(always)]
    pub fn get_attr(&self) -> Result<RegionAttr, u32> {
        match (self.len & Self::PERM_MASK) >> Self::PERM_SHIFT {
            0b00 => Ok(RegionAttr::R),
            0b10 => Ok(RegionAttr::RW),
            0b01 => Ok(RegionAttr::RX),
            val => Err(val)
        }
    }

    #[inline(always)]
    pub fn is_device(&self) -> bool {
        self.len & Self::DEVICE_MASK != 0
    }

    #[inline(always)]
    pub fn should_zero(&self) -> bool {
        self.len & Self::ZERO_MASK != 0
    }

    #[inline(always)]
    pub fn enabled(&self) -> bool {
        self.len & Self::ENABLE_MASK != 0
    }

    #[inline(always)]
    pub fn has_error_codes(&self) -> bool {
        self.enabled() && !self.is_device()
    }

    pub fn check_crc(&self) -> Option<Result<(), ()>> {
        if self.has_error_codes() && self.has_phys() {
            let data = unsafe {
                slice::from_raw_parts(ptr::with_exposed_provenance(self.phys_addr as usize), (self.actual_len as usize + mem::size_of::<u32>() - 1) / mem::size_of::<u32>())
            };
            if check_crc(data, self.crc) {
                Some(Ok(()))
            } else {
                Some(Err(()))
            }
        } else {
            None
        }
    }

    pub fn error_codes(&self) -> Option<u32> {
        if self.has_error_codes() && self.has_virt() {
            Some(self.codes)
        } else {
            None
        }
    }

    pub fn fix_valid(&mut self, block_len: u32) -> Result<(), ()> {
        if !self.enabled() || self.is_device() {
            return Ok(());
        }
        if let Some(codes) = self.error_codes() {
            let start = read_time();
            let block_len = block_len as usize;
            let blocks = (((self.actual_len as usize + mem::size_of::<u32>() - 1) / mem::size_of::<u32>()) + block_len - 1) / block_len;
            let symbol_len = calc_symbol_len(block_len);
            let code_len = symbol_len + 1; // + 1 for CRC
            let region_mem_len = (self.actual_len as usize + mem::size_of::<u32>() - 1) / mem::size_of::<u32>();
            let mem_ptr: *mut u32 = ptr::with_exposed_provenance_mut(self.get_virt().unwrap() as usize);
            let codes_ptr: *mut u32 = ptr::with_exposed_provenance_mut(codes as usize);
            for i in 0..blocks {
                let mem_offset = block_len * i;
                let region_len = if mem_offset + block_len <= region_mem_len {
                    block_len
                } else {
                    region_mem_len - mem_offset
                };
                let current_symbol_len = calc_symbol_len(region_len);
                let crc: &u32 = unsafe {
                    &*codes_ptr.add(i * code_len + current_symbol_len)
                };
                let region_mem: &mut [u32] = unsafe {
                    slice::from_raw_parts_mut(mem_ptr.add(mem_offset), region_len)
                };
                let symbols: &mut [u32] = unsafe {
                    slice::from_raw_parts_mut(codes_ptr.add(i * code_len), current_symbol_len)
                };
                if !check_hamming_crc(symbols, region_mem, *crc) {
                    println!("Correcting errors");
                    panic!("Panic");
                    if let Err(err) = correct_errors(symbols, region_mem) {
                        return Err(err);
                    }
                    // check CRC says no errors
                    if !check_hamming_crc(symbols, region_mem, *crc) {
                        return Err(())
                    }
                }
            }
        }
        Ok(())
    }
    
    pub fn encode_codes(&mut self, block_len: u32) {
        if let Some(codes) = self.error_codes() {
            let block_len = block_len as usize;
            let blocks = (((self.actual_len as usize + mem::size_of::<u32>() - 1) / mem::size_of::<u32>()) + block_len - 1) / block_len;
            let symbol_len = calc_symbol_len(block_len);
            let region_mem_len = (self.actual_len as usize + mem::size_of::<u32>() - 1) / mem::size_of::<u32>();
            let code_len = symbol_len + 1; // + 1 for CRC
            let mem_ptr: *mut u32 = ptr::with_exposed_provenance_mut(self.get_virt().unwrap() as usize);
            let codes_ptr: *mut u32 = ptr::with_exposed_provenance_mut(codes as usize);
            for i in 0..blocks {
                let mem_offset = block_len * i;
                let region_len = if mem_offset + block_len <= region_mem_len {
                    block_len
                } else {
                    region_mem_len - mem_offset
                };
                let current_symbol_len = calc_symbol_len(region_len);
                let crc: &mut u32 = unsafe {
                    &mut *codes_ptr.add(i * code_len + current_symbol_len)
                };
                let region_mem: &mut [u32] = unsafe {
                    slice::from_raw_parts_mut(mem_ptr.add(mem_offset), region_len)
                };
                let symbols: &mut [u32] = unsafe {
                    slice::from_raw_parts_mut(codes_ptr.add(i * code_len), current_symbol_len)
                };
                *crc = calc_symbols(symbols, region_mem);
            }
        }
    }

    /// SAFETY
    /// offset must be within memory bounds and codes must be the region's codes offset
    unsafe fn write_word_internal(&mut self, codes: u32, addr: u32, offset: usize, word: u32, block_len: u32) {
        let block_len = block_len as usize;
        let block = offset / block_len;
        let current_block_len = if (block + 1) * block_len * mem::size_of::<u32>() > self.actual_len as usize {
            ((self.actual_len as usize + mem::size_of::<u32>() - 1) / mem::size_of::<u32>()) - block * block_len
        } else {
            block_len
        };
        let block_offset = offset - block * block_len;
        let symbol_len = calc_symbol_len(block_len);
        let code_len = symbol_len + 1; // + 1 for CRC
        let current_symbol_len = calc_symbol_len(current_block_len);
        let mem_ptr: *mut u32 = ptr::with_exposed_provenance_mut(addr as usize);
        let codes_ptr: *mut u32 = ptr::with_exposed_provenance_mut(codes as usize);
        let crc: &mut u32 = unsafe {
            &mut *codes_ptr.add(block * code_len + current_symbol_len)
        };
        let region_mem: &mut [u32] = unsafe {
            slice::from_raw_parts_mut(mem_ptr.add(block * block_len), current_block_len)
        };
        let symbols: &mut [u32] = unsafe {
            slice::from_raw_parts_mut(codes_ptr.add(block * code_len), current_symbol_len)
        };
        *crc = update_msg(symbols, region_mem, *crc, block_offset, word);
    }

    /// SAFETY
    /// offset must be within memory bounds
    #[inline(always)]
    pub unsafe fn write_word_raw(&mut self, offset: usize, word: u32, block_len: u32) -> Result<(), ()> {
        unsafe {
            self.write_word_internal(self.error_codes().ok_or(())?, self.get_virt().ok_or(())?, offset, word, block_len);
        }
        Ok(())
    }

    #[inline(always)]
    pub fn write_word(&mut self, offset: usize, word: u32, block_len: u32) -> Result<(), ()> {
        assert!(offset * mem::size_of::<u32>() < (1 << (self.get_len() + 1)));
        unsafe {
            self.write_word_raw(offset, word, block_len)
        }
    }
    
    pub unsafe fn write_block_raw(&mut self, offset: usize, block: &[u32], block_len: u32) -> Result<(), ()> {
        let codes = self.error_codes().ok_or(())?;
        let virt = self.get_virt().ok_or(())?;
        for (i, word) in block.iter().enumerate() {
            unsafe {
                self.write_word_internal(codes, virt, offset + i, *word, block_len);
            }
        }
        Ok(())
    }

    #[inline(always)]
    pub fn write_block(&mut self, offset: usize, block: &[u32], block_len: u32) -> Result<(), ()> {
        assert!((offset + block.len()) * 4 < (1 << (self.get_len() + 1)));
        unsafe {
            self.write_block_raw(offset, block, block_len)
        }
    }
    
    pub unsafe fn write_bytes_raw(&mut self, offset: usize, bytes: &[u8], block_len: u32) -> Result<(), ()> {
        let codes = self.error_codes().ok_or(())?;
        let virt = self.get_virt().ok_or(())?;
        const LEN_U32: usize = mem::size_of::<u32>();
        let align_offset = offset & !(LEN_U32 - 1);
        let before_bytes = offset - align_offset;
        let mut current_offset = 0;
        let mut start_bytes = [0; LEN_U32];
        for i in 0..before_bytes {
            start_bytes[i] = unsafe {
                *ptr::with_exposed_provenance(i + align_offset + virt as usize)
            };
        }
        for i in 0..bytes.len().min(start_bytes.len() - before_bytes) {
            start_bytes[i + before_bytes] = bytes[i];
            current_offset += 1;
        }
        for i in before_bytes + bytes.len()..start_bytes.len() {
            start_bytes[i] = unsafe {
                *ptr::with_exposed_provenance(i + align_offset + virt as usize)
            };
        }
        unsafe {
            self.write_word_internal(codes, virt, align_offset / LEN_U32, u32::from_ne_bytes(start_bytes), block_len);
        }

        let block_offset = (offset + current_offset) / LEN_U32;
        let blocks_end = (offset + bytes.len()) / LEN_U32;
        for i in block_offset..blocks_end {
            unsafe {
                self.write_word_internal(codes, virt, i, u32::from_ne_bytes(bytes[current_offset..current_offset + LEN_U32].try_into().unwrap()), block_len);
            }
            current_offset += LEN_U32;
        }
        if current_offset < bytes.len() {
            let mut end_bytes = [0; LEN_U32];
            let remainder = bytes.len() - current_offset;
            for i in current_offset..bytes.len() {
                end_bytes[i - current_offset] = bytes[i];
            }
            let end_start = offset + bytes.len();
            let end_addr = (end_start + LEN_U32 - 1) & !(LEN_U32 - 1);
            for i in end_start..end_addr {
                end_bytes[i - end_start + remainder] = unsafe {
                    *ptr::with_exposed_provenance(i + virt as usize)
                };
            }
            unsafe {
                self.write_word_internal(codes, virt, end_start / LEN_U32, u32::from_ne_bytes(end_bytes), block_len);
            }
        }
        Ok(())
    }

    #[inline(always)]
    pub fn write_bytes(&mut self, offset: usize, bytes: &[u8], block_len: u32) -> Result<(), ()> {
        assert!(offset + bytes.len() < (1 << (self.get_len() + 1)));
        unsafe {
            self.write_bytes_raw(offset, bytes, block_len)
        }
    }

    pub fn get_len(&self) -> u16 {
        (self.len >> Self::LEN_SHIFT) as u16
    }

    pub fn size(&self) -> u32 {
        1 << (self.get_len() + 1)
    }
}

#[repr(C)]
pub struct Program {
    pub flags: u32,
    pub inter: [u8; 4],
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
    pub async_endpoints: *const *mut AsyncMessageQueue,
    pub block_len: u32
}

impl Program {
    const PRIORITY_SHIFT: usize = 0;
    const DRIVER_SHIFT: usize = 16;

    const PRIORITY_MASK: u32 = 0xff << Self::PRIORITY_SHIFT;
    const DRIVER_MASK: u32 = 0xffff << Self::DRIVER_SHIFT;
    const INTERRUPT_NONE: u8 = 0xff;

    pub fn priority(&self) -> u8 {
        ((self.flags & Self::PRIORITY_MASK) >> Self::PRIORITY_SHIFT) as u8
    }
    
    pub fn driver(&self) -> u16 {
        ((self.flags & Self::DRIVER_MASK) >> Self::DRIVER_SHIFT) as u16
    }

    pub fn interrupt(&self, inter: usize) -> Option<u8> {
        let inter = self.inter[inter];
        if inter >= 32 {
            None
        } else {
            Some(inter)
        }
    }
    
    pub fn interrupts(&self) -> &[u8] {
        &self.inter
    }
    
    pub fn interrupts_mut(&mut self) -> &mut [u8] {
        &mut self.inter
    }

    pub fn has_interrupt(&self) -> bool {
        for inter in self.inter {
            if inter < 32 {
                return true;
            }
        }
        false
    }

    /// Returns index of irq
    pub fn index_irq(&self, irq: u8) -> Option<usize> {
        for (i, inter) in self.inter.iter().enumerate() {
            if *inter == irq {
                return Some(i);
            }
        }
        None
    }

    pub fn write_word(&mut self, addr: u32, word: u32) -> Result<(), ()> {
        for region in &mut self.regions {
            if region.enabled() {
                let start = region.get_runtime_addr().unwrap();
                let end = start + region.size();
                if start <= addr && addr + mem::size_of::<u32>() as u32 <= end && region.get_attr().unwrap().access_valid(AccessAttr::new(false, true, false)) {
                    region.fix_valid(self.block_len).unwrap();
                    region.write_word((addr - start) as usize / mem::size_of::<u32>(), word, self.block_len).unwrap();
                    return Ok(());
                }
            }
        }
        Err(())
    }
    
    pub fn read_word(&mut self, addr: u32) -> Result<u32, ()> {
        for region in &mut self.regions {
            if region.enabled() {
                let start = region.get_runtime_addr().unwrap();
                let end = start + region.size();
                if start <= addr && addr + mem::size_of::<u32>() as u32 <= end && region.get_attr().unwrap().access_valid(AccessAttr::new(true, false, false)) {
                    region.fix_valid(self.block_len).unwrap();
                    return unsafe {
                        Ok(*ptr::with_exposed_provenance(addr as usize))
                    };
                }
            }
        }
        Err(())
    }

    pub fn read_bytes(&mut self, addr: u32, len: usize) -> Result<&[u8], ()> {
        if len == 0 {
            return Ok(&[]);
        }
        let mut error = false;
        let mut checked = false;
        let mut current_addr = addr;
        let mut current_len = len;
        while !error && !checked {
            error = true;
            for region in &mut self.regions {
                if region.enabled() {
                    let start = region.get_runtime_addr().unwrap();
                    let end = start + region.size();
                    if start <= current_addr && current_addr < end && region.get_attr().unwrap().access_valid(AccessAttr::new(true, false, false)) {
                        region.fix_valid(self.block_len).unwrap();
                        if current_addr + current_len as u32 <= end {
                            checked = true;
                            error = false;
                            break;
                        } else {
                            current_len -= end as usize - current_len;
                            current_addr = end;
                            error = false;
                            break;
                        }
                    }
                }
            }
        }
        if error {
            Err(())
        } else {
            unsafe {
                Ok(slice::from_raw_parts(ptr::with_exposed_provenance(addr as usize), len))
            }
        }
    }
    
    pub fn write_bytes(&mut self, mut addr: u32, mut data: &[u8]) -> Result<(), ()> {
        if data.len() == 0 {
            return Ok(());
        }
        let mut error = false;
        while !error {
            error = true;
            for region in &mut self.regions {
                if region.enabled() {
                    let start = region.get_runtime_addr().unwrap();
                    let end = start + region.size();
                    if start <= addr && addr < end && region.get_attr().unwrap().access_valid(AccessAttr::new(false, true, false)) {
                        region.fix_valid(self.block_len).unwrap();
                        if addr + data.len() as u32 <= end {
                            region.write_bytes((addr - start) as usize, data, self.block_len).unwrap();
                            return Ok(());
                        } else {
                            region.write_bytes((addr - start) as usize, &data[..(end - addr) as usize], self.block_len).unwrap();
                            addr = end;
                            data = &data[(end - addr) as usize..];
                            error = false;
                            break;
                        }
                    }
                }
            }
        }
        return Err(())
    }

    pub fn update_codes(&mut self) {
        for region in &mut self.regions {
            if region.enabled() {
                region.encode_codes(self.block_len);
            }
        }
    }

    pub fn correct_errors(&mut self) -> Result<(), ()> {
        for region in &mut self.regions {
            if region.enabled() {
                region.check_crc().unwrap_or(Ok(())).expect("Have uncorrectable flash error");
            }
        }
        for region in &mut self.regions {
            if region.enabled() {
                region.fix_valid(self.block_len)?;
            }
        }
        Ok(())
    }

    pub fn check_access(&self, addr: u32, len: u32, attr: AccessAttr) -> Result<(), ()> {
        if len == 0 {
            return Ok(());
        }
        let mut error = false;
        let mut checked = false;
        let mut current_addr = addr;
        let mut current_len = len;
        while !error && !checked {
            error = true;
            for region in &self.regions {
                if region.enabled() {
                    let start = region.get_runtime_addr().unwrap();
                    let end = start + region.size();
                    if start <= current_addr && current_addr < end && region.get_attr().unwrap().access_valid(attr) {
                        if current_addr + current_len <= end {
                            checked = true;
                            error = false;
                            break;
                        } else {
                            current_len -= end - current_len;
                            current_addr = end;
                            error = false;
                            break;
                        }
                    }
                }
            }
        }
        if error {
            Err(())
        } else {
            Ok(())
        }

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
        for region in &mut program.regions {
            if region.enabled() {
                let phys_addr = region.get_phys();
                let virt_addr = region.get_virt();
                if let Some(phys_addr) = phys_addr {
                    println!("phys address: 0x{:x}", phys_addr);
                }
                if let Some(virt_addr) = virt_addr {
                    println!("virt address: 0x{:x}", virt_addr);
                }
                println!("length: 0x{:x}", region.size());
                if let Err(err) = region.get_attr() {
                    panic!("Have invalid region permission bits of 0b{:b}", err);
                }
                let len = region.get_len();
                if len < 7 || len > 0x1f {
                    panic!("Have invalid region len of {}", len);
                }
                let runtime_addr = region.get_runtime_addr().unwrap();
                let region_len = 1 << (len + 1);
                if !runtime_addr.is_multiple_of(region_len) {
                    panic!("Have invalid region alignment with address 0x{:x} for len 0x{:x}", runtime_addr, region_len);
                }
                if region.should_zero() {
                    let mut virt_ptr: *mut u32 = ptr::with_exposed_provenance_mut(runtime_addr as usize);
                    for _ in 0..region.size() / 4 {
                        unsafe {
                            virt_ptr.write_volatile(0);
                            virt_ptr = virt_ptr.add(1);
                        }
                    }
                } else if let Some(phys_addr) = phys_addr && let Some(virt_addr) = virt_addr {
                    let mut phys_ptr: *const u32 = ptr::with_exposed_provenance(phys_addr as usize);
                    let mut virt_ptr: *mut u32 = ptr::with_exposed_provenance_mut(virt_addr as usize);
                    for _ in 0..region.size() / 4 {
                        unsafe {
                            virt_ptr.write_volatile(phys_ptr.read_volatile());
                            phys_ptr = phys_ptr.add(1);
                            virt_ptr = virt_ptr.add(1);
                        }
                    }
                }
                region.encode_codes(program.block_len);
            }
        }
        println!("Entry: 0x{:x}", program.entry);
        println!("Stack Region: {}", program.sp);
        let driver = program.driver() as usize;
        let r0 = if driver != 0 {
            program.regions[0].get_runtime_addr().unwrap()
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
