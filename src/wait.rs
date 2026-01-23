use core::arch::asm;

pub fn wait_cycles(cycles: usize) {
    for _ in 0..cycles {
        unsafe {
            asm!("nop");
        }
    }
}
