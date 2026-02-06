use core::arch::asm;

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

pub fn test_func() {
    let text = "This is test proc!".as_bytes();
    let msg = "hello".as_bytes();
    unsafe {
        _ = do_syscall(0, text.as_ptr() as u32, text.len() as u32, 0, 0).unwrap();
        _ = do_syscall(3, 0, 0, msg.len() as u32, msg.as_ptr() as u32).unwrap();
        _ = do_syscall(1, 0, 0, 0, 0).unwrap();
    }
}

pub fn test_func2() {
    let text = "This is test proc2!".as_bytes();
    unsafe {
        _ = do_syscall(0, text.as_ptr() as u32, text.len() as u32, 0, 0).unwrap();
        _ = do_syscall(5, 0, 0, 0, 0).unwrap();
        let mut data = [0; 5];
        _ = do_syscall(9, 0, 5, data.as_mut_ptr() as u32, 0).unwrap();
        _ = do_syscall(11, 0, 4, 0, 0).unwrap();
        _ = do_syscall(0, data.as_ptr() as u32, 5, 0, 0).unwrap();
        _ = do_syscall(1, 0, 0, 0, 0).unwrap();
    }
}

pub fn test_func3() {
    let text = "This is test proc3!".as_bytes();
    unsafe {
        _ = do_syscall(0, text.as_ptr() as u32, text.len() as u32, 0, 0).unwrap();
        _ = do_syscall(1, 0, 0, 0, 0).unwrap();
    }
}
