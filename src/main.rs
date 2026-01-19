// use core intrinsics 
#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{arch::asm, intrinsics::abort, panic::PanicInfo, ptr};

use tock_registers::{interfaces::{Readable, Writeable}, register_bitfields, register_bitmasks, register_structs, registers::{ReadOnly, ReadWrite, WriteOnly}};

// offset for clearing registers as specified in SDK https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21
// lines 18 to 21 (accessed 19/01/2026)
const REG_ALIAS_CLR_BITS: usize = 0x3 << 12;

register_bitfields![u32,
    Reset [
        // Take IO Bank 0 into or out of reset
        IO_BANK0 5
    ],
    ResetDone [
        // Reset for IO Bank 0 done
        IO_BANK0 5
    ],
];

register_structs! {
    pub ResetRegisters {
        (0x0 => reset: WriteOnly<u32, Reset::Register>),
        (0x4 => _reserved1),
        (0x8 => reset_done: ReadOnly<u32, ResetDone::Register>),
        (0xc => @END),
    }
}

register_bitfields![u32,
    GPIOCtrl [
        // Set function select for given GPIO
        FUNCSEL OFFSET(0) NUMBITS(5) [
            F1 = 1,
            F2 = 2,
            F3 = 3,
            F4 = 4,
            F5 = 5, // SIO
            F6 = 6,
            F7 = 7,
            F8 = 8,
            F9 = 9
        ],
        OUTOVER OFFSET(8) NUMBITS(2) [
            NORMAL = 0x0, // drive output from peripheral signal selected by FUNCSEL
            INVERT = 0x1, // drive output from inverse of peripheral signal selected by FUNCSEL
            LOW = 0x2, // drive output low
            HIGH = 0x3, // drive output high
        ],
        OEOVER OFFSET(12) NUMBITS(2) [
            NORMAL = 0x0, // drive output enable from peripheral signal selected by FUNCSEL
            INVERT = 0x1, // drive output enable from inverse of peripheral signal selected by FUNCSEL
            DISABLE = 0x2, // disable output
            ENABLE = 0x3, // enable output
        ],
        INOVER OFFSET(16) NUMBITS(2) [
            NORMAL = 0x0, // don't invert perl input
            INVERT = 0x1, // invert perl input
            LOW = 0x2, // drive perl input low
            HIGH = 0x3, // drive perl input high
        ],
        IRQOVER OFFSET(28) NUMBITS(2) [
            NORMAL = 0x0, // don't invert the interrupt
            INVERT = 0x1, // invert the interrupt
            LOW = 0x2, // drive interrupt low
            HIGH = 0x3, // drive interrupt high
        ],
    ],
];

register_structs! {
    pub IOBank0Registers {
        (0x0 => _reserved0),
        (0xb4 => GPIO22_ctrl: ReadWrite<u32, GPIOCtrl::Register>),
        (0xb8 => @END),
    }
}

register_bitfields![u32,
    GPIO [
        GP0 0,
        GP1 1,
        GP2 2,
        GP3 3,    
        GP4 4,    
        GP5 5,    
        GP6 6,    
        GP7 7,    
        GP8 8,    
        GP9 9,    
        GP10 10,    
        GP11 11,    
        GP12 12,    
        GP13 13,    
        GP14 14,    
        GP15 15,    
        GP16 16,    
        GP17 17,    
        GP18 18,    
        GP19 19,    
        GP20 20,    
        GP21 21,    
        GP22 22,    
        GP23 23,    
        GP24 24,    
        GP25 25,    
        GP26 26,    
        GP27 27,    
        GP28 28,    
        GP29 29,    
    ],
];

register_structs! {
    pub SIORegisters {
        (0x0 => _reserved0),
        (0x14 => GPIO_out: ReadWrite<u32, GPIO::Register>),
        (0x18 => GPIO_out_set: WriteOnly<u32, GPIO::Register>),
        (0x1c => GPIO_out_clr: WriteOnly<u32, GPIO::Register>),
        (0x20 => GPIO_OE: ReadWrite<u32, GPIO::Register>),
        (0x24 => GPIO_OE_set: WriteOnly<u32, GPIO::Register>),
        (0x28 => GPIO_OE_clr: WriteOnly<u32, GPIO::Register>),
        (0x2c => @END),
    }
}


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
    // SAFETY: this address is the reset controller
    let reset: &ResetRegisters = unsafe {
        & *ptr::with_exposed_provenance(0x4000c000)
    };
    // SAFETY: this address is the clear alias for the reset controller
    let reset_clear: &ResetRegisters = unsafe {
        & *ptr::with_exposed_provenance(0x4000c000 + REG_ALIAS_CLR_BITS)
    };
    reset_clear.reset.write(Reset::IO_BANK0::SET);
    while reset.reset_done.read(ResetDone::IO_BANK0) != 1 {}
    // SAFETY: this address is for IO bank 0
    let IO_bank0: &IOBank0Registers = unsafe {
        & *ptr::with_exposed_provenance(0x40014000)
    };
    // set mode to FUNCSEL 5 so it is now controlled by SIO
    IO_bank0.GPIO22_ctrl.write(GPIOCtrl::INOVER::NORMAL + GPIOCtrl::OEOVER::NORMAL + GPIOCtrl::IRQOVER::NORMAL + GPIOCtrl::OUTOVER::NORMAL + GPIOCtrl::FUNCSEL::F5);
    let SIO: &SIORegisters = unsafe {
        & *ptr::with_exposed_provenance(0xd0000000)
    };
    SIO.GPIO_OE_set.write(GPIO::GP22::SET);
    loop {
        SIO.GPIO_out_set.write(GPIO::GP22::SET);
        for _ in 0..5000 {
            unsafe {
                asm!("nop");
            }
        }
        SIO.GPIO_out_clr.write(GPIO::GP22::SET);
        for _ in 0..5000 {
            unsafe {
                asm!("nop");
            }
        }
    }
}
