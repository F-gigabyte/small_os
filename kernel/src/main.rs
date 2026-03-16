// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test_runner)]

#![feature(assert_matches)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use core::{panic::PanicInfo};

use crate::scheduler::QUANTUM_MICROS;
use crate::sys_tick::SYS_TICK;
use crate::{clocks::CLOCKS, inter::{CS, disable_irq, enable_irq}, io_bank0::IOBANK0, mpu::MPU, mutex::{SpinIRQGuard, force_spinlock_unlock}, nvic::{NVIC, irqs}, pll::{PLL_SYS, PLL_USB}, program::init_processes, reset::RESET, rosc::ROSC, system::SYSTEM, timer::{TIMER, Timer, TimerIRQ}, uart::UART1, watchdog::WATCHDOG, xosc::XOSC};

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
pub mod sys_tick;
pub mod system;
pub mod messages;
pub mod message_queue;
pub mod program;
pub mod mpu;

unsafe extern "C" {
    static __stack: u8;
    fn idle() -> !;
}

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    loop {}
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
        reset.reset_pll_usb();
        let mut pll_sys = PLL_SYS.lock(&cs);
        pll_sys.reset_sys();
        let mut pll_usb = PLL_USB.lock(&cs);
        pll_usb.reset_usb();
        clocks.setup_clocks();
        let mut rosc = ROSC.lock(&cs);
        rosc.disable();
        reset.reset_iobank0();
        reset.reset_uart1();

        let mut io_bank0 = IOBANK0.lock(&cs);
        io_bank0.set_gpio_uart1();

        {
            let mut uart1 = UART1.lock(&cs);
            uart1.reset();
        }

        let mut watchdog = WATCHDOG.lock(&cs);
        watchdog.enable_ticks();
        let mut sys_tick = SYS_TICK.lock(&cs);
        sys_tick.set_timeout(QUANTUM_MICROS);
        sys_tick.init();
        let mut mpu = MPU.lock(&cs);
        mpu.init();
        {
            unsafe {
                init_processes(&cs);
            }
        }
        let mut sys = SYSTEM.lock(&cs);
        sys.send_pend();
    }
    #[cfg(test)]
    test_main();
    // SAFETY
    // IRQ has been setup and can now run
    unsafe {
        enable_irq();
        idle();
    }
}
