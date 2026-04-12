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

use core::ptr;

use crate::{println, proc::Proc};

/// Previous time from in kernel
static mut PREVIOUS_TIME: u64 = 0;
/// Maximum time spent in kernel
static mut MAX_TIME: u64 = 0;

/// Number of times errors have been created
#[cfg(feature = "radiation")]
static mut ITERATION: u64 = 0;

/// Reads the time from the timer device
pub fn read_time() -> u64 {
    let ptr: *const u32 = ptr::with_exposed_provenance(0x40054000);
    loop {
        let upper = unsafe {
            ptr.add(9).read_volatile()
        };
        let lower = unsafe {
            ptr.add(10).read_volatile()
        };
        let upper2 = unsafe {
            ptr.add(9).read_volatile()
        };
        if upper == upper2 {
            return ((upper as u64) << 32) | (lower as u64);
        }
    }
}

/// Function called entering the kernel  
/// Sets up `PREVIOUS_TIME`  
/// `proc` is the current process pointer which is returned (but not accessed)
/// # Safety
/// This must only be called on kernel entry
#[unsafe(no_mangle)]
pub unsafe fn kernel_entry(proc: *mut Proc) -> *mut Proc {
    unsafe {
        (&raw mut PREVIOUS_TIME).write_volatile(read_time());
    }
    proc
}

/// Modifies a random bit in memory  
/// This simulates the effects of radiation in space  
/// `proc` is the current process pointer which is returned (but not accessed)
/// # Safety
/// This function must be called just before correcting errors
#[cfg(feature = "radiation")]
#[unsafe(no_mangle)]
pub unsafe fn do_radiation(proc: *mut Proc) -> *mut Proc {
    use crate::rosc::ROSC;

    const RAM_ORIGIN: usize = 0x20000000;
    const RAM_LEN: usize = 264 * 1024;
    let cs = unsafe {
        use crate::inter::CS;

        CS::new()
    };
    let mut rosc = ROSC.lock(&cs);
    let mut addr: usize = 0;
    for _ in 0..100 {
        let iter_addr: usize = 0;
        let mut iter_addr = iter_addr.to_ne_bytes();
        let addr_len = iter_addr.len();
        rosc.get_random(&mut iter_addr, addr_len * 8);
        let iter_addr = usize::from_ne_bytes(iter_addr);
        addr ^= iter_addr; 
    }
    addr %= RAM_LEN * 8;
    println!("Have addr 0x{:x} (0x{:x})", addr / 8 + RAM_ORIGIN, RAM_ORIGIN + RAM_LEN);
    let byte: *mut u8 = ptr::with_exposed_provenance_mut((addr / 8) + RAM_ORIGIN);
    unsafe {
        let mut current = byte.read_volatile();
        current ^= 1 << (addr % 8);
        byte.write_volatile(current);
        let mut iter = (&raw const ITERATION).read_volatile();
        iter += 1;
        (&raw mut ITERATION).write_volatile(iter);
    }
    proc
}

/// Gets the radiation iteration
#[cfg(feature = "radiation")]
pub fn get_radiation_iteration() -> u64 {
    unsafe {
        (&raw const ITERATION).read_volatile()
    }
}

/// Function called exiting the kernel  
/// Checks if the time spent is larger than that in `MAX_TIME` and if so, overwrites `MAX_TIME`
/// `proc` is the current process pointer which is returned (but not accessed)
/// # Safety
/// This must only be called on kernel exit
#[unsafe(no_mangle)]
pub unsafe fn kernel_exit(proc: *mut Proc) -> *mut Proc {
    let current = read_time();
    let prev = unsafe {
        (&raw const PREVIOUS_TIME).read_volatile()
    };
    let mut max_time = unsafe {
        (&raw const MAX_TIME).read_volatile()
    };
    let diff = current.wrapping_sub(prev);
    if diff > max_time {
        unsafe {
            (&raw mut MAX_TIME).write_volatile(diff);
        }
        max_time = diff;
    }
    println!("Max time in kernel: {}us", max_time);
    proc
}
