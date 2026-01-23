// use core intrinsics 
#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{arch::asm, intrinsics::abort, panic::PanicInfo, ptr};

use crate::{clocks::{CLOCKS, PeriAuxSrc}, io_bank0::IOBANK0, nvic::NVIC, reset::RESET, rosc::ROSC, timer::{TIMER, TimerIRQ}, uart::UART1, watchdog::WATCHDOG, xosc::XOSC};

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

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
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
    let mut reset = RESET.lock();
    // https://github.com/dwelch67/raspberrypi-pico/blob/main/uart01/notmain.c accessed 21/01/2026
    let mut clocks = CLOCKS.lock();
    let mut xosc = XOSC.lock();
    clocks.disble_sys_resus();
    xosc.reset();
    clocks.setup_clocks();
    let mut rosc = ROSC.lock();
    rosc.disable();
    reset.reset_iobank0();
    reset.reset_pll_sys();
    reset.reset_uart1();
    reset.reset_timer();

    let mut IO_bank0 = IOBANK0.lock();
    IO_bank0.set_gpio_uart1();

    {
        let mut uart1 = UART1.lock();
        uart1.reset();
    }

    let mut timer = TIMER.lock();
    let mut nvic = NVIC.lock();
    let mut watchdog = WATCHDOG.lock();
    watchdog.enable_ticks();
    nvic.enable_timer_irq();
    timer.remove_debug_pause();
    timer.set_count0(1000);
    timer.set_count1(1000);
    timer.set_count2(1000);
    timer.set_count3(1000);
    timer.enable_irq(TimerIRQ::Timer0);
    timer.enable_irq(TimerIRQ::Timer1);
    timer.enable_irq(TimerIRQ::Timer2);
    timer.enable_irq(TimerIRQ::Timer3);

    loop {
        println!("Interrupts pending: 0x{:x}, timer: 0x{:x}, time: {}, watchdog: {}", nvic.get_pending(), timer.get_irq(), timer.read_time(), watchdog.get_counter());
    }
}
