/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS kernel.
 *
 * The SmallOS kernel is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS kernel is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS kernel. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

use core::{cell::UnsafeCell, mem, ptr, slice};

use crc32::{check_crc};
use hamming::{calc_symbol_len, calc_symbols, check_hamming_crc, correct_errors, update_msg};

use crate::{inter::CS, message_queue::{AsyncMessageQueue, SyncMessageQueue}, println, proc::Proc, scheduler::scheduler};

/// Region Attributes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegionAttr {
    /// Read Only
    R = 0b00,
    /// Read Write
    RW = 0b10,
    /// Read and Execute
    RX = 0b01
}

unsafe extern "C" {
    /// Program table address
    static mut __program_table: u32;
    /// Process table in memory address
    static __procs_virt_start: u32;
    /// End of process table in memory address
    static __procs_virt_end: u32;
    /// Address of kernel args
    pub static __args: DriverArgs;
}

/// Kernel driver arguments
#[repr(C)]
pub struct DriverArgs {
    /// GPIO pin functions
    pub pin_func: [u32; 4],
    /// GPIO pads setup
    pub pads: [u32; 2],
    /// Devices that can be reset
    pub resets: u32
}

/// Access attributes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AccessAttr {
    attr: u8
}

impl AccessAttr {
    /// Shift for read access
    pub const READ_SHIFT: usize = 2;
    /// Shift for write access
    pub const WRITE_SHIFT: usize = 1;
    /// Shift for execute access
    pub const EXEC_SHIFT: usize = 0;
    /// Mask for read access
    pub const READ_MASK: u8 = 1 << Self::READ_SHIFT;
    /// Mask for write access
    pub const WRITE_MASK: u8 = 1 << Self::WRITE_SHIFT;
    /// Mask for execute access
    pub const EXEC_MASK: u8 = 1 << Self::EXEC_SHIFT;

    /// Creates a new `AccessAttr`  
    /// `read` means this access includes reading  
    /// `write` means this access includes writing  
    /// `exec` means this access includes executing code  
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

    /// Whether the access includes reading
    pub fn read(&self) -> bool {
        self.attr & Self::READ_MASK != 0
    }

    /// Whether the access includes writing
    pub fn write(&self) -> bool {
        self.attr & Self::WRITE_MASK != 0
    }
    
    /// Whether the access includes executing code
    pub fn exec(&self) -> bool {
        self.attr & Self::EXEC_MASK != 0
    }

    /// Modify whether access includes reading  
    /// `read` means whether this access involves reading or not
    pub fn set_read(&mut self, read: bool) {
        if read {
            self.attr |= Self::READ_MASK;
        } else {
            self.attr &= !Self::READ_MASK;
        }
    }
    
    /// Modify whether access includes writing   
    /// `write` means whether this access involves writing or not
    pub fn set_write(&mut self, write: bool) {
        if write {
            self.attr |= Self::WRITE_MASK;
        } else {
            self.attr &= !Self::WRITE_MASK;
        }
    }
    
    /// Modify whether access includes executing code   
    /// `exec` means whether this access involves executing code or not
    pub fn set_exec(&mut self, exec: bool) {
        if exec {
            self.attr |= Self::EXEC_MASK;
        } else {
            self.attr &= !Self::EXEC_MASK;
        }
    }
}

impl RegionAttr {
    /// Checks an access is valid  
    /// `access` is how this region is accessed
    /// Returns `true` if valid or else `false`
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

    /// Whether this region attribute supports reading
    /// Always returns true
    #[inline(always)]
    pub fn read(&self) -> bool {
        true
    }

    /// Whether this region attribute supports writing
    #[inline(always)]
    pub fn write(&self) -> bool {
        match self {
            Self::RW => true,
            _ => false,
        }
    }

    /// Whether this region attribute supports executing code
    #[inline(always)]
    pub fn exec(&self) -> bool {
        match self {
            Self::RX => true,
            _ => false,
        }
    }
}

/// Number of errors corrected
#[cfg(feature = "radiation")]
static mut ERRORS_CORRECTED: usize = 0;

/// Gets the number of error corrections
#[cfg(feature = "radiation")]
pub unsafe fn get_num_corrections() -> usize {
    unsafe {
        (&raw const ERRORS_CORRECTED).read_volatile()
    }
}

/// A memory region
#[repr(C)]
#[derive(Clone)]
pub struct Region {
    /// The flash address the region exists at (0x00)
    pub phys_addr: u32, // 0x0
    /// The memory address the region runs from (0 if run from flash) (0x04)
    pub virt_addr: u32, // 0x4
    /// The region length and flag masks (0x08)
    pub len: u32, // 0x8
    /// The actual region's length (not the length padded to the right amount for the MPU) (0x0c)
    pub actual_len: u32,
    /// The address of the region's CRC codes (protects the flash memory) (0x10)
    pub crc: u32,
    /// The address of the region's hamming codes (protects the SRAM memory) (0x14)
    pub codes: u32,
}

impl Region {
    // len mask
    /// Shift for whether the region is enabled
    pub const ENABLE_SHIFT: usize = 0;
    /// Shift to specify if the region is loaded into SRAM
    pub const VIRTUAL_SHIFT: usize = 1;
    /// Shift to specify the region exists in flash memory
    pub const PHYSICAL_SHIFT: usize = 2;
    /// Shift to specify this region refers to memory mapped IO
    pub const DEVICE_SHIFT: usize = 3;
    /// Shift to specify the region's access permissions
    pub const PERM_SHIFT: usize = 4;
    /// Shift to specify the region should be zeroed
    pub const ZERO_SHIFT: usize = 6;
    /// Region length shift (which is padded to the length required by the MPU)
    pub const LEN_SHIFT: usize = 16;

    /// Mask for whether the region is enabled
    pub const ENABLE_MASK: u32 = 1 << Self::ENABLE_SHIFT;
    /// Mask to specify if the region is loaded into SRAM
    pub const VIRTUAL_MASK: u32 = 1 << Self::VIRTUAL_SHIFT;
    /// Mask to specify the region exists in flash memory
    pub const PHYSICAL_MASK: u32 = 1 << Self::PHYSICAL_SHIFT;
    /// Mask to specify this region refers to memory mapped IO
    pub const DEVICE_MASK: u32 = 1 << Self::DEVICE_SHIFT;
    /// Mask to specify the region's access permissions
    pub const PERM_MASK: u32 = 0x3 << Self::PERM_SHIFT;
    /// Mask to specify the region should be zeroed
    pub const ZERO_MASK: u32 = 1 << Self::ZERO_SHIFT;
    /// Region length mask (which is padded to the length required by the MPU)
    pub const LEN_MASK: u32 = 0xffff << Self::LEN_SHIFT;

    /// Empty region for testing
    #[cfg(test)]
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

    /// Whether this region exists in flash
    #[inline(always)]
    pub fn has_phys(&self) -> bool {
        self.len & Self::PHYSICAL_MASK != 0
    }
    
    /// Whether this region is loaded into SRAM
    #[inline(always)]
    pub fn has_virt(&self) -> bool {
        self.len & Self::VIRTUAL_MASK != 0
    }

    /// Gets the SRAM address this region is loaded to if it exists
    #[inline(always)]
    pub fn get_virt(&self) -> Option<u32> {
        if self.has_virt() {
            Some(self.virt_addr)
        } else {
            None
        }
    }

    /// Gets the address this region exists at in flash if it exists
    #[inline(always)]
    pub fn get_phys(&self) -> Option<u32> {
        if self.has_phys() {
            Some(self.phys_addr)
        } else {
            None
        }
    }

    /// Gets the address the region runs at  
    /// This is the virtual address if its loaded into SRAM or the physical address if it runs from
    /// flash
    pub fn get_runtime_addr(&self) -> Option<u32> {
        self.get_virt().or_else(|| self.get_phys())
    }

    /// Gets the region's access permissions
    /// Returns the permissions on success or an error with the permission bits if the attributes
    /// have been corrupted
    #[inline(always)]
    pub fn get_attr(&self) -> Result<RegionAttr, u32> {
        match (self.len & Self::PERM_MASK) >> Self::PERM_SHIFT {
            0b00 => Ok(RegionAttr::R),
            0b10 => Ok(RegionAttr::RW),
            0b01 => Ok(RegionAttr::RX),
            val => Err(val)
        }
    }

    /// Whether the region refers to memory mapped IO
    #[inline(always)]
    pub fn is_device(&self) -> bool {
        self.len & Self::DEVICE_MASK != 0
    }

    /// Whether the region should be zeroed on initialisation
    #[inline(always)]
    pub fn should_zero(&self) -> bool {
        self.len & Self::ZERO_MASK != 0
    }

    /// Whether the region is enabled
    #[inline(always)]
    pub fn enabled(&self) -> bool {
        self.len & Self::ENABLE_MASK != 0
    }

    /// Whether the region contains error codes
    #[inline(always)]
    pub fn has_error_codes(&self) -> bool {
        self.enabled() && !self.is_device()
    }

    /// Calculates the CRC checksums for the region's flash memory if it has them  
    /// Returns `None` if it doesn't have them  
    /// Returns whether the check was successful or not if it does have them
    pub fn check_crc(&self) -> Option<Result<(), ()>> {
        if cfg!(feature = "error_codes") {
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
        } else {
            None
        }
    }

    /// Returns the address of the regions hamming error correction codes if it has them
    pub fn error_codes(&self) -> Option<u32> {
        if self.has_error_codes() && self.has_virt() {
            Some(self.codes)
        } else {
            None
        }
    }

    /// Checks the region's SRAM for errors and attempts to correct them if present  
    /// Returns whether the region now has no errors or if there were uncorrectable errors
    pub fn fix_valid(&mut self, block_len: u32) -> Result<(), ()> {
        if !self.enabled() || self.is_device() {
            return Ok(());
        }
        if cfg!(feature = "error_codes") {
            if let Some(codes) = self.error_codes() {
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
                        if let Err(err) = correct_errors(symbols, region_mem) {
                            return Err(err);
                        }
                        // check CRC says no errors
                        if !check_hamming_crc(symbols, region_mem, *crc) {
                            return Err(())
                        }
                        #[cfg(feature = "radiation")]
                        {
                            let mut corrected = unsafe {
                                (&raw const ERRORS_CORRECTED).read_volatile()
                            };
                            corrected += 1;
                            unsafe {
                                (&raw mut ERRORS_CORRECTED).write_volatile(corrected);
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }
    
    /// Updates the region's hamming error codes if present  
    /// `block_len` is the hamming codes block length
    pub fn update_codes(&mut self, block_len: u32) {
        if cfg!(feature = "error_codes") {
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
                    // Check if CRC is different before updating to save time
                    if !check_hamming_crc(symbols, region_mem, *crc) {
                        *crc = calc_symbols(symbols, region_mem);
                    }
                }
            }
        }
    }
    
    /// Initialises the region's hamming error codes if present  
    /// `block_len` is the hamming codes block length
    pub fn encode_codes(&mut self, block_len: u32) {
        if cfg!(feature = "error_codes") {
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
                    // always update symbols on initialisation
                    *crc = calc_symbols(symbols, region_mem);
                }
            }
        }
    }

    /// Writes a 32 bit integer to a memory location in this region
    /// `codes` is the address of the hamming error correction codes
    /// `addr` is the address to write to
    /// `offset` is the offset from the address `addr` to write to
    /// `word` is the integer to write
    /// `block_len` is the hamming block length
    /// # Safety
    /// `addr` + `offset` * 4 must be within the region's memory bounds  
    /// `codes` must be the address of the region's codes offset  
    /// `block_len` must be the region's hamming code block length
    unsafe fn write_word_internal(&mut self, codes: u32, addr: u32, offset: usize, word: u32, block_len: u32) {
        if cfg!(feature = "error_codes") {
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
        } else {
            let ptr: *mut u32 = ptr::with_exposed_provenance_mut(addr as usize + offset * 4);
            unsafe {
                ptr.write(word);
            }
        }
    }

    /// Writes a 32 bit integer to a memory location in this region
    /// `offset` is how far from the start of the memory region to write to  
    /// `word` is the integer to write
    /// `block_len` is the region's hamming code block length
    /// # Safety
    /// `offset` must be less than a quarter of the region's length
    /// `block_len` must be the region's hamming code block length
    #[inline(always)]
    pub unsafe fn write_word_raw(&mut self, offset: usize, word: u32, block_len: u32) -> Result<(), ()> {
        unsafe {
            self.write_word_internal(self.error_codes().ok_or(())?, self.get_virt().ok_or(())?, offset, word, block_len);
        }
        Ok(())
    }

    /// Writes a 32 bit integer to a memory location in this region  
    /// `offset` is how far from the start of the memory region to write to in words   
    /// `word` is the integer to write  
    /// `block_len` is the region's hamming code block length  
    /// Panics if `offset` goes beyond the region's length
    /// # Safety
    /// `block_len` must be the region's hamming code block length
    #[inline(always)]
    pub unsafe fn write_word(&mut self, offset: usize, word: u32, block_len: u32) -> Result<(), ()> {
        assert!(offset * mem::size_of::<u32>() + 4 <= (1 << (self.get_len() + 1)));
        unsafe {
            self.write_word_raw(offset, word, block_len)
        }
    }
    
    /// Writes a block of 32 bit integers to a memory location in this region  
    /// `offset` is how far from the start of the memory region to write to in words   
    /// `block` is the block of integers to write  
    /// `block_len` is the region's hamming code block length
    /// # Safety
    /// `offset` must be less than the region's length  
    /// `block_len` must be the region's hamming code block length
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

    /// Writes a block of 32 bit integers to a memory location in this region  
    /// `offset` is how far from the start of the memory region to write to in words   
    /// `block` is the block of integers to write  
    /// `block_len` is the region's hamming code block length  
    /// Panics if `offset` goes beyond the region's length
    /// # Safety
    /// `block_len` must be the region's hamming code block length
    #[inline(always)]
    pub unsafe fn write_block(&mut self, offset: usize, block: &[u32], block_len: u32) -> Result<(), ()> {
        assert!((offset + block.len()) * 4 <= (1 << (self.get_len() + 1)));
        unsafe {
            self.write_block_raw(offset, block, block_len)
        }
    }
    
    /// Writes a series of bytes to a memory location in this region
    /// `offset` is how far from the start of the memory region to write to in bytes   
    /// `bytes` is the series of bytes to write  
    /// `block_len` is the region's hamming code block length  
    /// # Safety
    /// `offset` must be less than the region's length  
    /// `block_len` must be the region's hamming code block length
    pub unsafe fn write_bytes_raw(&mut self, offset: usize, bytes: &[u8], block_len: u32) -> Result<(), ()> {
        /* 
         * Because updating the error codes requires updates to the underlying memory to happen in 4
         * byte blocks and we're writing a series of bytes to memory, we need to first align offset
         * to be on a 4 byte offset, write the byte series in chunks of 4 bytes and write the
         * remaining bytes to memory reading in the original memory to fill the missing bytes in the
         * chunks. As a demonstration of how the algorithm works, a demo has been set up of writing
         * a byte series of w0 to w9 at offset 1 to byte series r0 to r11
         *
         *  __ w0 w1 w2 w3 w4 w5 w6 w7 w8 w9  ___
         *  r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 r10 r11
         *
         * The first step is aligning the writes to 4 byte boundaries
         * This means reading back r0 and adding w0 to w2 to create a 4 byte integer r0 w0 w1 w2
         * which can then be written to give
         *  __ __ __ __ w3 w4 w5 w6 w7 w8 w9  ___
         *  r0 w0 w1 w2 r4 r5 r6 r7 r8 r9 r10 r11
         *
         * Then the data can be written in 4 byte blocks until there's less than 4 bytes left to be
         * written giving
         *  __ __ __ __ __ __ __ __ w7 w8 w9  ___
         *  r0 w0 w1 w2 w3 w4 w5 w6 r8 r9 r10 r11
         *
         *  Then the final word is constructed out of the remaining bytes to be written concatenated
         *  to the original data up to a 4 byte aligned offset which would be w7 w8 w9 r11
         *  This then gives
         *  r0 w0 w1 w2 w3 w4 w5 w6 w7 w8 w9 r11
         *  which is the data after the write
         */
        let codes = self.error_codes().ok_or(())?;
        let virt = self.get_virt().ok_or(())?;
        const LEN_U32: usize = mem::size_of::<u32>();
        // First align the write to a word boundary
        // calculate the largest aligned address smaller than offset
        let align_offset = offset & !(LEN_U32 - 1);
        // number of bytes before aligning is the offset minus the aligned address
        let before_bytes = offset - align_offset;
        let mut current_offset = 0;
        let mut start_bytes = [0; LEN_U32];
        // Read the bytes from the aligned address up to the write start so they can be written back
        for i in 0..before_bytes {
            start_bytes[i] = unsafe {
                *ptr::with_exposed_provenance(i + align_offset + virt as usize)
            };
        }
        // Write the remaining bytes of the first word into start bytes
        for i in 0..bytes.len().min(start_bytes.len() - before_bytes) {
            start_bytes[i + before_bytes] = bytes[i];
            current_offset += 1;
        }
        // if bytes is less than a word in length and less than 4 bytes has been written into start
        // bytes, need to fill start bytes with the previous data so it can be written back (edge
        // case)
        for i in before_bytes + bytes.len()..start_bytes.len() {
            start_bytes[i] = unsafe {
                *ptr::with_exposed_provenance(i + align_offset + virt as usize)
            };
        }
        // write the first word back to memory
        unsafe {
            self.write_word_internal(codes, virt, align_offset / LEN_U32, u32::from_ne_bytes(start_bytes), block_len);
        }

        // write `bytes` in a series of 4 bytes to the memory region
        let block_offset = (offset + current_offset) / LEN_U32;
        let blocks_end = (offset + bytes.len()) / LEN_U32;
        for i in block_offset..blocks_end {
            unsafe {
                self.write_word_internal(codes, virt, i, u32::from_ne_bytes(bytes[current_offset..current_offset + LEN_U32].try_into().unwrap()), block_len);
            }
            current_offset += LEN_U32;
        }
        // write the final bytes to the memory region
        if current_offset < bytes.len() {
            let mut end_bytes = [0; LEN_U32];
            let remainder = bytes.len() - current_offset;
            // get the final set of bytes which weren't enough to be 4 bytes in size
            for i in current_offset..bytes.len() {
                end_bytes[i - current_offset] = bytes[i];
            }
            let end_start = offset + bytes.len();
            let end_addr = (end_start + LEN_U32 - 1) & !(LEN_U32 - 1);
            // read missing chunk bytes from memory
            for i in end_start..end_addr {
                end_bytes[i - end_start + remainder] = unsafe {
                    *ptr::with_exposed_provenance(i + virt as usize)
                };
            }
            // write final word
            unsafe {
                self.write_word_internal(codes, virt, end_start / LEN_U32, u32::from_ne_bytes(end_bytes), block_len);
            }
        }
        Ok(())
    }

    /// Writes a series of bytes to a memory location in this region
    /// `offset` is how far from the start of the memory region to write to in bytes   
    /// `bytes` is the series of bytes to write  
    /// `block_len` is the region's hamming code block length  
    /// Panics if `offset` goes beyond the region's length
    /// # Safety
    /// `block_len` must be the region's hamming code block length
    #[inline(always)]
    pub unsafe fn write_bytes(&mut self, offset: usize, bytes: &[u8], block_len: u32) -> Result<(), ()> {
        assert!(offset + bytes.len() < (1 << (self.get_len() + 1)));
        unsafe {
            self.write_bytes_raw(offset, bytes, block_len)
        }
    }

    /// Gets the regions length 
    /// In this case this means the length written directly into the MPU
    pub fn get_len(&self) -> u16 {
        (self.len >> Self::LEN_SHIFT) as u16
    }

    /// Gets the region length in bytes  
    /// This is not the actual length and includes the padding bytes to align this region to the
    /// correct size to be used by the MPU
    pub fn size(&self) -> u32 {
        1 << (self.get_len() + 1)
    }
}

/// A Program image
#[repr(C)]
pub struct Program {
    /// Program's PID
    pub pid: u32,
    /// Program flags
    pub flags: u32,
    /// Program interrupts  
    /// An interrupt of 32 or over means no interrupt
    pub inter: [u8; 4],
    /// Which region is the stack
    pub sp: u32,
    /// The program's entry address
    pub entry: u32,
    /// The program's regions
    pub regions: [Region; 8],
    /// The number of synchronous and asynchronous program queues
    pub num_queues: u32,
    /// The number of synchronous endpoints
    pub num_sync_endpoints: u32,
    /// Pointer to the program's array of queues
    pub sync_queues: *mut SyncMessageQueue,
    /// Pointer to the program's array of endpoints
    pub sync_endpoints: *const *mut SyncMessageQueue,
    /// The number of asynchronous endpoints
    pub num_async_endpoints: u32,
    /// Pointer to the program's array of asynchronous queues
    pub async_queues: *mut AsyncMessageQueue,
    /// Pointer to the program's array of asynchronous endpoints
    pub async_endpoints: *const *mut AsyncMessageQueue,
    /// Pointer to the program's array of notifier queues
    pub notifiers: *mut SyncMessageQueue,
    /// The program's hamming block length
    pub block_len: u32,
    /// Bitmask of the GPIO pins the program has been allocated
    pub pin_mask: u32
}

impl Program {
    /// Shift for program priority
    const PRIORITY_SHIFT: usize = 0;
    /// Shift for program driver
    const DRIVER_SHIFT: usize = 16;

    /// Mask for program priority
    const PRIORITY_MASK: u32 = 0xff << Self::PRIORITY_SHIFT;
    /// Mask for program driver
    const DRIVER_MASK: u32 = 0xffff << Self::DRIVER_SHIFT;
    /// No Interrupt (a more lenient definition is an interrupt of 32 or more
    const INTERRUPT_NONE: u8 = 0xff;

    /// Shift for the number of synchronous queues
    const SYNC_QUEUES_SHIFT: usize = 0;
    /// Shift for the number of asynchronous queues
    const ASYNC_QUEUES_SHIFT: usize = 8;
    /// Shift for the number of notifier queues
    const NOTIFIER_QUEUES_SHIFT: usize = 16;

    /// Mask for the number of synchronous queues
    const SYNC_QUEUES_MASK: u32 = 0xff << Self::SYNC_QUEUES_SHIFT;
    /// Mask for the number of asynchronous queues
    const ASYNC_QUEUES_MASK: u32 = 0xff << Self::ASYNC_QUEUES_SHIFT;
    /// Mask for the number of notifier queues
    const NOTIFIER_QUEUES_MASK: u32 = 0xff << Self::NOTIFIER_QUEUES_SHIFT;

    /// Creates an empty program for test purposes
    #[cfg(test)]
    pub const fn default() -> Self {
        Self {
            pid: 0,
            flags: 0,
            inter: [0xff; 4],
            sp: 0,
            entry: 0,
            regions: [const { Region::default() }; 8],
            num_queues: 0,
            num_sync_endpoints: 0,
            sync_queues: ptr::null_mut(),
            sync_endpoints: ptr::null_mut(),
            num_async_endpoints: 0,
            async_queues: ptr::null_mut(),
            async_endpoints: ptr::null_mut(),
            notifiers: ptr::null_mut(),
            block_len: 0,
            pin_mask: 0
        }
    }

    /// Gets the program's priority
    pub fn priority(&self) -> u8 {
        ((self.flags & Self::PRIORITY_MASK) >> Self::PRIORITY_SHIFT) as u8
    }
    
    /// Gets the program's driver
    pub fn driver(&self) -> u16 {
        ((self.flags & Self::DRIVER_MASK) >> Self::DRIVER_SHIFT) as u16
    }

    /// Gets the number of synchronous queues this program has
    pub fn num_sync_queues(&self) -> u32 {
        (self.num_queues & Self::SYNC_QUEUES_MASK) >> Self::SYNC_QUEUES_SHIFT
    }
    
    /// Gets the number of asynchronous queues this program has
    pub fn num_async_queues(&self) -> u32 {
        (self.num_queues & Self::ASYNC_QUEUES_MASK) >> Self::ASYNC_QUEUES_SHIFT
    }
    
    /// Gets the number of notifier queues this program has
    pub fn num_notifier_queues(&self) -> u32 {
        (self.num_queues & Self::NOTIFIER_QUEUES_MASK) >> Self::NOTIFIER_QUEUES_SHIFT
    }

    /// Returns the interrupt at the given index if present or `None` otherwise  
    /// `inter` is the interrupt index  
    /// Panics if `inter` is 4 or more
    pub fn interrupt(&self, inter: usize) -> Option<u8> {
        let inter = self.inter[inter];
        if inter >= 32 {
            None
        } else {
            Some(inter)
        }
    }
    
    /// Returns the array of program interrupts  
    /// Interrupts of 32 or more are null interrupts
    pub fn interrupts(&self) -> &[u8] {
        &self.inter
    }
    
    /// Returns if the program has any interrupts
    pub fn has_interrupt(&self) -> bool {
        if self.driver() == 0 {
            return false;
        }
        for inter in self.inter {
            if inter < 32 {
                return true;
            }
        }
        false
    }

    /// Returns index of the given IRQ  
    /// `irq` is the IRQ to index  
    /// Returns `None` if not present
    pub fn index_irq(&self, irq: u8) -> Option<usize> {
        for (i, inter) in self.inter.iter().enumerate() {
            if *inter == irq {
                return Some(i);
            }
        }
        None
    }

    /// Writes a 32 bit integer at address `addr`  
    /// `addr` is the address to write at  
    /// `word` is the integer to write  
    /// Returns if the write was successful or not
    /// Will partially write the data if fails
    pub fn write_word(&mut self, addr: u32, word: u32) -> Result<(), ()> {
        for region in &mut self.regions {
            if region.enabled() {
                let start = region.get_runtime_addr().unwrap();
                let end = start + region.size();
                if start <= addr && addr + mem::size_of::<u32>() as u32 <= end && region.get_attr().unwrap().access_valid(AccessAttr::new(false, true, false)) {
                    region.fix_valid(self.block_len).unwrap();
                    unsafe {
                        region.write_word((addr - start) as usize / mem::size_of::<u32>(), word, self.block_len).unwrap();
                    }
                    return Ok(());
                }
            }
        }
        Err(())
    }
    
    /// Reads a 32 bit integer at address `addr`  
    /// `addr` is the address to read from  
    /// Will fail if `addr` is not aligned ona 4 byte boundary  
    /// Returns if the read was successful or not
    pub fn read_word(&mut self, addr: u32) -> Result<u32, ()> {
        if !addr.is_multiple_of(4) {
            return Err(())
        }
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

    /// Reads `len` bytes from address `addr`  
    /// `addr` is the address to read from  
    /// `len` is the number of bytes to read
    /// Returns if the read was successful or not
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
    
    /// Writes `data` to address `addr`  
    /// `addr` is the address to write to   
    /// `data` is the bytes to write  
    /// Returns if the write was successful or not  
    /// Will partially write the data if fails
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
                            unsafe {
                                region.write_bytes((addr - start) as usize, data, self.block_len).unwrap();
                            }
                            return Ok(());
                        } else {
                            unsafe {
                                region.write_bytes((addr - start) as usize, &data[..(end - addr) as usize], self.block_len).unwrap();
                            }
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

    /// Updates the program's hamming codes
    pub fn update_codes(&mut self) {
        if cfg!(feature = "error_codes") {
            for region in &mut self.regions {
                if region.enabled() {
                    region.update_codes(self.block_len);
                }
            }
        }
    }

    /// Attempts to fix any errors in the program's memory regions  
    /// Returns if the corrections were successful or not
    pub fn correct_errors(&mut self) -> Result<(), ()> {
        if cfg!(feature = "error_codes") {
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
        }
        Ok(())
    }

    /// Checks whether an access is valid  
    /// `addr` is the address the access happens at  
    /// `len` is the number of bytes from `addr` being accessed  
    /// `attr` is the access attributes being used  
    /// Returns if the access is valid or not
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
                            current_len -= end - current_addr;
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
    
    /// Gets a test program from 10 statically allocated test programs  
    /// `prog` is the program number to use  
    /// # Safety
    /// When using the test programs, `prog` must be different for each program allocated  
    /// At any instance, only one thread can being using the test programs
    #[cfg(test)]
    pub unsafe fn get_test_prog(prog: usize) -> &'static mut Program {
        const TEST_PROGRAMS: usize = 10;
        static mut PROGS: [Program; TEST_PROGRAMS] = [const { Program::default() }; 10];
        let res = unsafe {
            assert!(prog < TEST_PROGRAMS);
            &mut *(&raw mut PROGS as *mut Program).add(prog)
        };
        *res = Program::default();
        res
    }
}

/// Table of all the programs
pub struct ProgramTable {
    table: &'static [Program]
}

impl ProgramTable {
    /// Creates the `ProgramTable`
    /// `base` is the base address of where the program table is located at
    /// # Safety
    /// `base` must point to a `ProgramTable` and not used by any other part of the program
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

    /// Gets the slice to all the programs
    #[inline(always)]
    pub fn table(&self) -> &'static [Program] {
        self.table
    }
}

/// Creates all the system processes from their corresponding programs  
/// `cs` is a token to specify interrupts are non-active
/// # Safety
/// this function is non-reentrant
pub unsafe fn init_processes(cs: &CS) {
    // first item at the start of the program table is the number of programs
    let num_programs: u32 = unsafe {
        *&raw const __program_table
    };

    // Get the list of all programs
    let programs: &mut[Program] = unsafe {
        slice::from_raw_parts_mut((&raw mut __program_table).add(1) as *mut Program, num_programs as usize)
    };
    let mut scheduler = scheduler(&cs);
    // locate the process table
    let procs_start = &raw const __procs_virt_start as usize;
    let procs_end = &raw const __procs_virt_end as usize;
    let len = (procs_end - procs_start) / mem::size_of::<Proc>();
    let processes: &mut [UnsafeCell<Proc>] = unsafe {
        slice::from_raw_parts_mut(ptr::with_exposed_provenance_mut(procs_start), len)
    };
    // initialise the scheduler with the process table
    scheduler.init(processes);
    let mut current_driver = 0;
    // allocate each process from its corresponding program
    for program in programs {
        println!("Driver {}", program.driver());
        // drivers stored in increasing order with no driver (0) allowed to be present multiple
        // times while other drivers only allowed to be present a maximum of one time (due to there
        // being a single device)
        // In this context driver refers more to device rather than driver
        if program.driver() <= current_driver && current_driver != 0 {
            panic!("Invalid driver ordering for programs");
        } else {
            current_driver = program.driver();
        }
        // maximum number of queues is 32 so a single 32 bit number can be used as a queue mask
        if program.num_sync_queues() > 32 {
            panic!("Have invalid number of sync queues of {}", program.num_sync_queues());
        }
        if program.num_async_queues() > 32 {
            panic!("Have invalid number of async queues of {}", program.num_async_queues());
        }
        if program.num_notifier_queues() > 32 {
            panic!("Have invalid number of notifier queues of {}", program.num_notifier_queues());
        }
        // initialise region data
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
                    // zero region
                    let mut virt_ptr: *mut u32 = ptr::with_exposed_provenance_mut(runtime_addr as usize);
                    for _ in 0..region.size() / 4 {
                        unsafe {
                            virt_ptr.write_volatile(0);
                            virt_ptr = virt_ptr.add(1);
                        }
                    }
                } else if let Some(phys_addr) = phys_addr && let Some(virt_addr) = virt_addr {
                    // load region data
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
                // calculate region hamming codes
                region.encode_codes(program.block_len);
            }
        }
        println!("Entry: 0x{:x}", program.entry);
        println!("Stack Region: {}", program.sp);
        let driver = program.driver() as usize;
        // program arguments
        let mut args = [0; 7];
        let kernel_args = unsafe {
            &__args
        };
        let mut arg_len = 0;
        if driver != 0 {
            // driver base address argument
            args[arg_len] = program.regions[0].get_runtime_addr().unwrap();
            arg_len += 1;
            if driver == 6 {
                // IO Bank 0 - needs all the GPIO functions
                args[arg_len] = kernel_args.pin_func[0];
                args[arg_len + 1] = kernel_args.pin_func[1];
                args[arg_len + 2] = kernel_args.pin_func[2];
                args[arg_len + 3] = kernel_args.pin_func[3];
                arg_len += 4;
            } else if driver == 8 {
                // Pads bank 0 - needs all the GPIO pads information
                args[arg_len] = kernel_args.pads[0];
                args[arg_len + 1] = kernel_args.pads[1];
                arg_len += 2;
            } else if driver == 27 {
                // Resets - needs to know what devices to reset
                args[arg_len] = kernel_args.resets;
                arg_len += 1;
            }
        }
        if program.pin_mask != 0 {
            // pass in program's pins
            args[arg_len] = program.pin_mask;
            arg_len += 1;
        }
        // pass in the number of arguments so programs can check they're getting the right number of
        // arguments
        args[arg_len] = arg_len as u32;
        let pid = unsafe {
            scheduler.create_proc(program, &args[..arg_len + 1]).unwrap()
        };
        scheduler.schedule_process(pid).unwrap();
    }
}
