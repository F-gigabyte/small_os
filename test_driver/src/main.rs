// use core intrinsics 
#![feature(core_intrinsics)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::{arch::asm, intrinsics::abort, panic::PanicInfo};

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

unsafe fn do_syscall(num: u32, mut arg0: u32, mut arg1: u32, mut arg2: u32, mut arg3: u32) -> Result<(u32, u32, u32, u32), u32>{
    let mut res: u32;
    unsafe {
        asm!(
            "mov r12, {num}",
            "svc 0",
            "mov {res}, r12",
            num = in(reg) num,
            inout("r0") arg0 => arg0,
            inout("r1") arg1 => arg1,
            inout("r2") arg2 => arg2,
            inout("r3") arg3 => arg3,
            res = out(reg) res
        )
    }
    match res {
        0 => {
            Ok((arg0, arg1, arg2, arg3))
        },
        _ => {
            Err(num)
        }
    }
}

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let text = "This is test proc!".as_bytes();
    loop {
        unsafe {
            _ = do_syscall(0, text.as_ptr() as u32, text.len() as u32, 0, 0).unwrap();
        }
    }
}
