// use core intrinsics 
#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{intrinsics::abort, panic::PanicInfo};

use crate::{clocks::CLOCKS, inter::{CS, disable_irq, enable_irq}, io_bank0::IOBANK0, message_queue::{ASYNC_ENDPOINTS1, ASYNC_ENDPOINTS2, ASYNC_ENDPOINTS3, ASYNC_QUEUES1, ASYNC_QUEUES2, ASYNC_QUEUES3, SYNC_ENDPOINTS1, SYNC_ENDPOINTS2, SYNC_ENDPOINTS3, SYNC_QUEUES1, SYNC_QUEUES2, SYNC_QUEUES3}, mutex::force_spinlock_unlock, nvic::{NVIC, irqs}, pll::PLL, reset::RESET, rosc::ROSC, scheduler::scheduler, system::SYSTEM, test_proc::{test_func, test_func2, test_func3}, timer::{TIMER, TimerIRQ}, uart::UART1, watchdog::WATCHDOG, xosc::XOSC};

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

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    abort()
}

/// Stage 2 bootloader
#[unsafe(link_section = ".bootloader")]
#[used]
pub static BOOTLOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

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
            let pid = unsafe {
                scheduler.create_proc(0, test_func as *const () as u32, 0x20005000, 1, (&raw mut SYNC_QUEUES1).as_mut().unwrap(), &SYNC_ENDPOINTS1, (&raw mut ASYNC_QUEUES1).as_mut().unwrap(), &ASYNC_ENDPOINTS1).unwrap()
            };
            scheduler.schedule_process(pid).unwrap();
            let pid = unsafe {
                scheduler.create_proc(1, test_func2 as *const () as u32, 0x20004000, 0, (&raw mut SYNC_QUEUES2).as_mut().unwrap(), & SYNC_ENDPOINTS2, (&raw mut ASYNC_QUEUES2).as_mut().unwrap(), &ASYNC_ENDPOINTS2).unwrap()
            };
            scheduler.schedule_process(pid).unwrap();
            let pid = unsafe {
                scheduler.create_proc(2, test_func3 as *const () as u32, 0x20003000, 0, (&raw mut SYNC_QUEUES3).as_mut().unwrap(), & SYNC_ENDPOINTS3, (&raw mut ASYNC_QUEUES3).as_mut().unwrap(), &ASYNC_ENDPOINTS3).unwrap()
            };
            scheduler.schedule_process(pid).unwrap();
        }
        let mut sys = SYSTEM.lock(&cs);
        sys.send_pend();
        timer.clear_irq(TimerIRQ::Timer0);
        timer.clear_irq(TimerIRQ::Timer1);
        timer.clear_irq(TimerIRQ::Timer2);
        timer.clear_irq(TimerIRQ::Timer3);
        timer.enable_irq(TimerIRQ::Timer0);
    }
    // SAFETY
    // IRQ has been setup and can now run
    unsafe {
        enable_irq();
    }
    println!("Completed Setup");
    loop {}
}
