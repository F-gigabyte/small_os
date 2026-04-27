/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Kernel.
 *
 * The SmallOS Kernel is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Kernel is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS Kernel. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

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

use crate::pads_bank0::PADS_BANK0;
use crate::scheduler::QUANTUM_MICROS;
use crate::sys_tick::SYS_TICK;
use crate::{clocks::CLOCKS, inter::{CS, disable_irq, enable_irq}, io_bank0::IOBANK0, mpu::MPU, mutex::{SpinIRQGuard, force_spinlock_unlock}, nvic::{NVIC, irqs}, pll::{PLL_SYS, PLL_USB}, program::init_processes, reset::RESET, rosc::ROSC, system::SYSTEM, uart::UART1, watchdog::WATCHDOG, xosc::XOSC};

/// Kernel UART driver
pub mod uart;
/// Handles resetting devices and bringing them out of reset
pub mod reset;
/// Different mutexes based on interrupts and spin locks
pub mod mutex;
/// Handles setting up the kernel UART driver to use GPIO 4 to 7
pub mod io_bank0;
/// Handles configuration of the system clocks
pub mod clocks;
/// Handles configuration of the crystal oscillator
pub mod xosc;
/// Memory-mapped IO offsets
pub mod mmio;
/// Waiting a given number of cycles
pub mod wait;
/// Handles configuration of the ring oscillator
pub mod rosc;
/// Kernel printing
pub mod print;
/// Manages interrupts
pub mod inter;
/// Handles configuration of the Nested Vectored Interrupt Controller
pub mod nvic;
/// Handles the watchdog timer (enabling its clock for use in the timer device)
pub mod watchdog;
/// Handles the PLLs
pub mod pll;
/// Handles a process' state
pub mod proc;
/// Handles the scheduling of processes and carrying out system calls for processes
pub mod scheduler;
/// Handles the configuration of the sys tick interrupt
pub mod sys_tick;
/// System configuration information
pub mod system;
/// Message header
pub mod messages;
/// Message queues
pub mod message_queue;
/// Handles management of different programs
pub mod program;
/// Memory Protection Unit
pub mod mpu;
/// Handles the initialisation of GPIO pads
pub mod pads_bank0;
/// Gets timing information for the kernel
#[cfg(feature = "benchmark")]
pub mod timer;

unsafe extern "C" {
    /// Kernel Stack
    static __stack: u8;
    /// Idle Process
    fn idle() -> !;
}

/// Panic Handler  
/// This function is called when a panic happens
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    #[cfg(feature = "radiation")]
    {
        // print test information
        use crate::timer::get_radiation_iteration;
        use crate::program::get_num_corrections;
        unsafe {
            println!("Iteration number is {} and corrected errors are {}", get_radiation_iteration(), get_num_corrections());
        }
    }
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
/// `tests` are the tests passed in by the compiler
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
    // Safety
    // No mutex has been acquired yet so this function is safe to call
    unsafe {
        force_spinlock_unlock();
    }
    {
        // Safety
        // no IRQ anyway so IRQ already disabled
        // This will stop IRQ from running when NVIC is set up
        unsafe {
            disable_irq();
        }
        {
            let cs = unsafe {
                CS::new()
            };
            // Clocks need to be set up before the UART
            // Based on https://github.com/dwelch67/raspberrypi-pico/blob/main/uart01/notmain.c accessed 21/01/2026 under the license
            //-------------------------------------------------------------------------
            //
            // Copyright (c) 2021 David Welch dwelch@dwelch.com
            //
            // Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
            //
            // The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
            //
            // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
            //
            //-------------------------------------------------------------------------
            // Although the processor is supposed to start running from the ring oscillator (ROSC),
            // from the contents of reset it appears it is running from PLL Sys (perhaps it wasn't
            // reset?) By setting up the clocks first and then resetting, we make sure the core is
            // running from the crystal oscillator (XOSC) so we can reset PLL Sys
            let mut clocks = CLOCKS.lock(&cs);
            let mut xosc = XOSC.lock(&cs);
            clocks.disable_sys_resus();
            xosc.reset();
            clocks.preinit_sys_ref();
            let mut reset = RESET.lock(&cs);
            reset.reset_all();
            reset.unreset_pll_sys();
            reset.unreset_pll_usb();
            let mut pll_sys = PLL_SYS.lock(&cs);
            pll_sys.reset_sys();
            let mut pll_usb = PLL_USB.lock(&cs);
            pll_usb.reset_usb();
            unsafe {
                clocks.setup_clocks();
            }
            let mut rosc = ROSC.lock(&cs);
            rosc.disable();
            #[cfg(feature = "random")]
            rosc.enable();
            reset.unreset_iobank0();
            reset.unreset_pads_bank0();
            reset.unreset_uart1();

            let mut pads_bank0 = PADS_BANK0.lock(&cs);
            pads_bank0.set_pads_uart1();

            let mut io_bank0 = IOBANK0.lock(&cs);
            io_bank0.set_gpio_uart1();

            {
                let mut uart1 = UART1.lock(&cs);
                uart1.reset();

            }
            let mut watchdog = WATCHDOG.lock(&cs);
            watchdog.enable_ticks();
            let mut sys_tick = SYS_TICK.lock(&cs);
            sys_tick.init();
            let mut nvic = NVIC.lock(&cs);
            nvic.disable_all_irq();
            let mut mpu = MPU.lock(&cs);
            mpu.init();
            {
                unsafe {
                    init_processes(&cs);
                }
            }
        }
        #[cfg(test)]
        test_main();
        {
            let cs = unsafe {
                CS::new()
            };
            let mut sys_tick = SYS_TICK.lock(&cs);
            sys_tick.set_timeout(QUANTUM_MICROS);
            let mut sys = SYSTEM.lock(&cs);
            sys.send_pend();
        }
    }
    // Safety
    // IRQ has been setup and can now run
    unsafe {
        enable_irq();
        idle();
    }
}
