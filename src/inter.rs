use crate::println;

#[repr(C)]
pub struct StackTrace {
    xpsr: u32,
    ret_addr: u32,
    lr: u32,
    r12: u32,
    r3: u32,
    r2: u32,
    r1: u32,
    r0: u32
}

#[unsafe(no_mangle)]
pub extern "C" fn nmi() -> ! {
    println!("NMI");
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn hard_fault(trace: *const StackTrace) -> ! {
    let trace = unsafe {
        &*trace
    };
    println!("Hard Fault!");
    println!("Registers");
    println!("\tr0: {:x}", trace.r0);
    println!("\tr1: {:x}", trace.r1);
    println!("\tr2: {:x}", trace.r2);
    println!("\tr3: {:x}", trace.r3);
    println!("\tr12: {:x}", trace.r12);
    println!("\tlr: {:x}", trace.lr);
    println!("\tret_addr: {:x}", trace.ret_addr);
    println!("\txpsr: {:x}", trace.xpsr);
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn unimplemented_irq(trace: *const StackTrace) -> ! {
    let trace = unsafe {
        &*trace
    };
    println!("Unimplemented IRQ!");
    println!("Registers");
    println!("\tr0: {:x}", trace.r0);
    println!("\tr1: {:x}", trace.r1);
    println!("\tr2: {:x}", trace.r2);
    println!("\tr3: {:x}", trace.r3);
    println!("\tr12: {:x}", trace.r12);
    println!("\tlr: {:x}", trace.lr);
    println!("\tret_addr: {:x}", trace.ret_addr);
    println!("\txpsr: {:x}", trace.xpsr);
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn sys_call() -> ! {
    println!("Sys Call!");
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn sys_pend() -> ! {
    println!("Sys Pending!");
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn sys_tick() -> ! {
    println!("Sys Tick!");
    loop {}
}
