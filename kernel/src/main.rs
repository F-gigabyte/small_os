// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test_runner)]

#![feature(assert_matches)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::{intrinsics::abort, panic::PanicInfo, ptr};

use crate::{clocks::CLOCKS, inter::{CS, disable_irq, enable_irq}, io_bank0::IOBANK0, message_queue::{SYNC_ENDPOINTS1, SYNC_ENDPOINTS2, SYNC_QUEUES1, SYNC_QUEUES2}, mutex::force_spinlock_unlock, nvic::{NVIC, irqs}, pll::PLL, program::{PROGRAM_TABLE, ProgramTable}, reset::RESET, rosc::ROSC, scheduler::scheduler, system::SYSTEM, test_proc::{test_func, test_func2, test_func3}, timer::{TIMER, TimerIRQ}, uart::UART1, watchdog::WATCHDOG, xosc::XOSC};

pub mod uart;
pub mod reset;
pub mod mutex;
pub mod io_bank0;
pub mod clocks;
pub mod xosc;
pub mod mmio;
pub mod wait;
pub mod rosc;
pub mod print;
pub mod inter;
pub mod timer;
pub mod nvic;
pub mod watchdog;
pub mod pll;
pub mod proc;
pub mod scheduler;
pub mod test_proc;
pub mod sys_tick;
pub mod system;
pub mod messages;
pub mod message_queue;
pub mod program;

unsafe extern "C" {
    static __stack: u8;
    static __program_table: u8;
}

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    abort()
}

/// An example test to check tests are working
#[test_case]
fn passes() {
    print!("Asserting true ");
    assert!(true);
    println!("[ok]");
}

/// Stage 2 bootloader
#[unsafe(link_section = ".bootloader")]
#[used]
pub static BOOTLOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
pub fn test_runner(tests: &[&dyn Fn()]) {
    println!("Running {} tests", tests.len());
    for test in tests {
        test();
    }
}

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    // SAFETY
    // No mutex has been acquired yet so this function is safe to call
    unsafe {
        force_spinlock_unlock();
    }
    {
        // SAFETY
        // no IRQ anyway so IRQ already disabled
        // This will stop IRQ from running when NVIC is set up
        unsafe {
            disable_irq();
        }
        let cs = unsafe {
            CS::new()
        };
        let mut reset = RESET.lock(&cs);
        // Clocks need to be set up before the UART
        // Based on https://github.com/dwelch67/raspberrypi-pico/blob/main/uart01/notmain.c accessed 21/01/2026
        let mut clocks = CLOCKS.lock(&cs);
        let mut xosc = XOSC.lock(&cs);
        clocks.disble_sys_resus();
        xosc.reset();
        clocks.preinit_sys_ref();
        reset.reset_pll_sys();
        let mut pll = PLL.lock(&cs);
        pll.reset_sys();
        clocks.setup_clocks();
        let mut rosc = ROSC.lock(&cs);
        rosc.disable();
        reset.reset_iobank0();
        reset.reset_uart1();
        reset.reset_timer();

        let mut io_bank0 = IOBANK0.lock(&cs);
        io_bank0.set_gpio_uart1();

        {
            let mut uart1 = UART1.lock(&cs);
            uart1.reset();
        }

        let mut timer = TIMER.lock(&cs);
        let mut watchdog = WATCHDOG.lock(&cs);
        watchdog.enable_ticks();
        timer.remove_debug_pause();
        let mut nvic = NVIC.lock(&cs);
        nvic.enable_irq(irqs::TIMER0);
        {
            let mut scheduler = scheduler(&cs);
            let mut program_table = PROGRAM_TABLE.lock();
            if program_table.is_none() {
                unsafe {
                    *program_table = Some(ProgramTable::new((&__program_table) as *const _ as usize));
                }
            }
            let program_table = program_table.as_ref().unwrap();
            let mut current_pid = 0;
            for program in program_table.table() {
                for region in &program.regions {
                    let phys_addr = region.phys_addr & !0xff;
                    let virt_addr = region.virt_addr & !0xff;
                    println!("phys address: 0x{:x}", phys_addr);
                    println!("virt address: 0x{:x}", virt_addr);
                    if region.len > 0 && phys_addr != virt_addr {
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
                println!("Entry: 0x{:x}", program.entry);
                println!("Stack: 0x{:x}", program.sp);
                let pid = unsafe {
                    scheduler.create_proc(current_pid, program.entry, program.sp, program.priority as u8, None, None, None, None).unwrap()
                };
                scheduler.schedule_process(pid).unwrap();
                current_pid += 1;
            }
        }
        let mut sys = SYSTEM.lock(&cs);
        #[cfg(not(test))]
        sys.send_pend();
        timer.clear_irq(TimerIRQ::Timer0);
        timer.clear_irq(TimerIRQ::Timer1);
        timer.clear_irq(TimerIRQ::Timer2);
        timer.clear_irq(TimerIRQ::Timer3);
        #[cfg(not(test))]
        timer.enable_irq(TimerIRQ::Timer0);
    }
    // SAFETY
    // IRQ has been setup and can now run
    unsafe {
        enable_irq();
    }
    #[cfg(test)]
    test_main();
    loop {}
}
