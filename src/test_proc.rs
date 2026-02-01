use core::arch::asm;

pub fn test_func() {
    let text = "This is test proc!".as_bytes();
    let start = text.as_ptr();
    let r8 = 0;
    let r1 = text.len();
    loop {
        unsafe {
            asm!(
                "mov r8, {r8}",
                "svc 0", 
                r8 = in(reg) r8,
                inout("r0") start => _,
                in("r1") r1,
                out("r8") _
            );
        }
    }
}
