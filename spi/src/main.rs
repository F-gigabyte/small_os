#![feature(core_intrinsics)]

#![no_std]
#![no_main]

use core::{intrinsics::abort, panic::PanicInfo, ptr::{self, NonNull}};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite, ReadWrite, WriteOnly}};
use small_os_lib::{REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, do_yield, send_empty, wait_irq};

mod ctrl0_register {
    pub const DATA_SIZE_SHIFT: usize = 0;
    pub const FRAME_FORMAT_SHIFT: usize = 4;
    pub const POLARITY_SHIFT: usize = 6;
    pub const PHASE_SHIFT: usize = 7;
    pub const SERIAL_CLOCK_SHIFT: usize = 8;

    pub const DATA_SIZE_MASK: u32 = 0xf << DATA_SIZE_SHIFT;
    pub const FRAME_FORMAT_MASK: u32 = 0x3 << FRAME_FORMAT_SHIFT;
    pub const POLARITY_MASK: u32 = 1 << POLARITY_SHIFT;
    pub const PHASE_MASK: u32 = 1 << PHASE_SHIFT;
    pub const SERIAL_CLOCK_MASK: u32 = 0xf << SERIAL_CLOCK_SHIFT;

    pub const DATA_SIZE_4BIT: u32 = 0x3 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_5BIT: u32 = 0x4 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_6BIT: u32 = 0x5 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_7BIT: u32 = 0x6 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_8BIT: u32 = 0x7 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_9BIT: u32 = 0x8 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_10BIT: u32 = 0x9 << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_11BIT: u32 = 0xa << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_12BIT: u32 = 0xb << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_13BIT: u32 = 0xc << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_14BIT: u32 = 0xd << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_15BIT: u32 = 0xe << DATA_SIZE_SHIFT;
    pub const DATA_SIZE_16BIT: u32 = 0xf << DATA_SIZE_SHIFT;

    pub const FRAME_FORMAT_NORMAL: u32 = 0 << FRAME_FORMAT_SHIFT;
    pub const FRAME_FORMAT_TI: u32 = 1 << FRAME_FORMAT_SHIFT;
    pub const FRAME_FORMAT_NATIONAL_MICROWIRE: u32 = 2 << FRAME_FORMAT_SHIFT;
}

mod ctrl1_register {
    pub const LOOP_BACK_SHIFT: usize = 0;
    pub const ENABLE_SHIFT: usize = 1;
    pub const SLAVE_SHIFT: usize = 2;
    pub const SLAVE_OUTPUT_SHIFT: usize = 3;

    pub const LOOP_BACK_MASK: u32 = 1 << LOOP_BACK_SHIFT;
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const SLAVE_MASK: u32 = 1 << SLAVE_SHIFT;
    pub const SLAVE_OUTPUT_MASK: u32 = 1 << SLAVE_OUTPUT_SHIFT;
}

mod data_register {
    pub const DATA_SHIFT: usize = 0;

    pub const DATA_MASK: u32 = 0xffff << DATA_SHIFT;
}

mod status_register {
    pub const TX_EMPTY_SHIFT: usize = 0;
    pub const TX_FULL_SHIFT: usize = 1;
    pub const RX_NOT_EMPTY_SHIFT: usize = 2;
    pub const RX_FULL_SHIFT: usize = 3;
    pub const BUSY_SHIFT: usize = 4;

    pub const TX_EMPTY_MASK: u32 = 1 << TX_EMPTY_SHIFT;
    pub const TX_FULL_MASK: u32 = 1 << TX_FULL_SHIFT;
    pub const RX_NOT_EMPTY_MASK: u32 = 1 << RX_NOT_EMPTY_SHIFT;
    pub const RX_FULL_MASK: u32 = 1 << RX_FULL_SHIFT;
    pub const BUSY_MASK: u32 = 1 << BUSY_SHIFT;
}

mod div_register {
    pub const DIV_SHIFT: usize = 0;
    
    pub const DIV_MASK: u32 = 0xff << DIV_SHIFT;
}

mod inter_register {
    pub const RX_OVERRUN_SHIFT: usize = 0;
    pub const RX_TIMEOUT_SHIFT: usize = 1;
    pub const RX_FIFO_SHIFT: usize = 2;
    pub const TX_FIFO_SHIFT: usize = 3;

    pub const RX_OVERRUN_MASK: u32 = 1 << RX_OVERRUN_SHIFT;
    pub const RX_TIMEOUT_MASK: u32 = 1 << RX_TIMEOUT_SHIFT;
    pub const RX_FIFO_MASK: u32 = 1 << RX_FIFO_SHIFT;
    pub const TX_FIFO_MASK: u32 = 1 << TX_FIFO_SHIFT;

    pub const ALL_MASK: u32 = RX_OVERRUN_MASK | RX_TIMEOUT_MASK | RX_FIFO_MASK | TX_FIFO_MASK;
}

mod inter_clear_register {
    pub const RX_OVERRUN_SHIFT: usize = 0;
    pub const RX_TIMEOUT_SHIFT: usize = 1;
    
    pub const RX_OVERRUN_MASK: u32 = 1 << RX_OVERRUN_SHIFT;
    pub const RX_TIMEOUT_MASK: u32 = 1 << RX_TIMEOUT_SHIFT;

    pub const ALL_MASK: u32 = RX_OVERRUN_MASK | RX_TIMEOUT_MASK;
}

#[repr(C)]
struct SPIRegisters {
    ctrl0: ReadPureWrite<u32>, // 0x0
    ctrl1: ReadPureWrite<u32>, // 0x4
    data: ReadWrite<u32>, // 0x8
    status: ReadPure<u32>, // 0xc
    div: ReadPureWrite<u32>, // 0x10
    inter_mask_set_clear: ReadPureWrite<u32>, // 0x14
    raw_inter: ReadPure<u32>, // 0x18
    mask_inter: ReadPure<u32>, // 0x1c
    inter_clear: WriteOnly<u32>, // 0x20
    dma: ReadPureWrite<u32>, // 0x24
}

pub struct SPI {
    registers: UniqueMmioPointer<'static, SPIRegisters>,
    set_reg: UniqueMmioPointer<'static, SPIRegisters>,
    clear_reg: UniqueMmioPointer<'static, SPIRegisters>,
}

impl SPI {
    pub unsafe fn new(spi_base: usize) -> Self {
        let mut res = unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(spi_base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(spi_base + REG_ALIAS_SET_BITS)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(spi_base + REG_ALIAS_CLR_BITS)).unwrap())
            }
        };
        while field!(res.registers, status).read() & status_register::BUSY_MASK != 0 {
            do_yield().unwrap();
        }
        field!(res.registers, ctrl1).write(0);
        field!(res.registers, ctrl0).write(0);
        field!(res.registers, ctrl0).write(
            ctrl0_register::DATA_SIZE_8BIT |
            ctrl0_register::FRAME_FORMAT_NORMAL
        );
        field!(res.registers, div).write(16);
        field!(res.registers, inter_mask_set_clear).write(inter_clear_register::ALL_MASK);
        field!(res.registers, inter_clear).write(inter_clear_register::ALL_MASK);
        field!(res.registers, dma).write(0);
        res
    }

    fn wait_busy(&mut self) {
        while field!(self.registers, status).read() & status_register::BUSY_MASK != 0 {
            do_yield().unwrap();
        }
    }

    pub fn send_cmd(&mut self, cmd: u8, tx: &[u8], mut delay: usize, rx: &mut [u8]) {
        self.wait_busy();
        field!(self.clear_reg, ctrl1).write(ctrl1_register::ENABLE_MASK);
        field!(self.registers, data).write(cmd as u32);
        let tx_len = tx.len() + 1;
        let rx_len = delay + rx.len();
        let tx_padding = if rx_len > tx_len {
            rx_len - tx_len
        } else {
            0
        };
        let mut tx_index = 1;
        let mut rx_index = 0;
        while field!(self.registers, status).read() & status_register::TX_FULL_MASK == 0 && tx_index < tx_len {
            if tx_index < 1 + tx.len() {
                field!(self.registers, data).write(tx[tx_index - 1] as u32);
            } else {
                field!(self.registers, data).write(0);
            }
            tx_index += 1;
        }
        let mut tx_inter = 0;
        if tx_index < tx.len() {
            tx_inter = inter_register::TX_FIFO_MASK;
        }
        field!(self.registers, inter_mask_set_clear).write(inter_register::RX_OVERRUN_MASK | inter_register::RX_TIMEOUT_MASK | tx_inter);
        field!(self.set_reg, ctrl1).write(ctrl1_register::ENABLE_MASK);
        loop {
            wait_irq().unwrap();
            if field!(self.registers, mask_inter).read() & inter_register::TX_FIFO_MASK != 0 {
                while field!(self.registers, status).read() & status_register::TX_FULL_MASK == 0 && tx_index < tx_len {
                    if tx_index < 1 + tx.len() {
                        field!(self.registers, data).write(tx[tx_index - 1] as u32);
                    } else {
                        field!(self.registers, data).write(0);
                    }
                    tx_index += 1;
                }
            }
            if field!(self.registers, mask_inter).read() & inter_register::RX_FIFO_MASK != 0 {
                while field!(self.registers, status).read() & status_register::RX_NOT_EMPTY_MASK != 0 {
                    if delay > 0 {
                        _ = field!(self.registers, data).read();
                        delay -= 1;
                    } else if rx_index < rx.len() {
                        rx[rx_index] = (field!(self.registers, data).read() & 0xff) as u8;
                        rx_index += 1;
                    } else {
                        _ = field!(self.registers, data).read();
                    }
                }
            }
        }
    }

    pub fn get_version(&mut self) -> u32 {
        field!(self.registers, data).write(0x40);
        field!(self.registers, data).write(0x00);
        field!(self.set_reg, ctrl1).write(ctrl1_register::ENABLE_MASK);
        let ctrl0 = field!(self.registers, ctrl0).read();
        let ctrl1 = field!(self.registers, ctrl1).read();
        let div = field!(self.registers, div).read();
        let status = field!(self.registers, status).read();
        while field!(self.registers, status).read() & status_register::BUSY_MASK != 0{
            do_yield().unwrap();
        }
        let mut word = 0;
        loop {
            if field!(self.registers, status).read() & status_register::RX_NOT_EMPTY_MASK != 0 {
                word = field!(self.registers, data).read();
            } else {
                break;
            }
        }
        word
    }
}

const CLOCK_FREQ: u32 = 12000000;
const SPI_FREQ: u32 = 6000000;

const IO_BANK0_QUEUE: u32 = 0;

/// panic handler
/// this function is called when a panic happens
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(num_args: usize, spi_base: usize) {
    assert!(num_args == 2);
    send_empty(IO_BANK0_QUEUE, 0, &[]).unwrap();
    let mut spi = unsafe {
        SPI::new(spi_base)
    };
    let version = spi.get_version();
    loop {}
}
