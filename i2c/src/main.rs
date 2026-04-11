// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

/// Based on the I2C implementation in the raspberry pi pico SDK at https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/i2c.c (accessed 25/03/2026)

use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPure, ReadPureWrite, ReadWrite}};
use small_os_lib::{QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, args, check_critical, do_yield, read_header, receive, reply, reply_empty, send_empty, wait_irq};

mod ctrl_register {
    pub const MASTER_MODE_SHIFT: usize = 0;
    pub const SPEED_SHIFT: usize = 1;
    pub const ADDR_SLAVE_SHIFT: usize = 3;
    pub const ADDR_MASTER_SHIFT: usize = 4;
    pub const RESTART_SHIFT: usize = 5;
    pub const SLAVE_DISABLE_SHIFT: usize = 6;
    pub const STOP_ADDR_SHIFT: usize = 7;
    pub const TX_EMPTY_SHIFT: usize = 8;
    pub const RX_FULL_HOLD_SHIFT: usize = 9;
    pub const STOP_MASTER_SHIFT: usize = 10;

    pub const MASTER_MODE_MASK: u32 = 1 << MASTER_MODE_SHIFT;
    pub const SPEED_MASK: u32 = 0x3 << SPEED_SHIFT;
    pub const ADDR10_SLAVE_MASK: u32 = 1 << ADDR_SLAVE_SHIFT;
    pub const ADDR10_MASTER_MASK: u32 = 1 << ADDR_MASTER_SHIFT;
    pub const RESTART_MASK: u32 = 1 << RESTART_SHIFT;
    pub const SLAVE_DISABLE_MASK: u32 = 1 << SLAVE_DISABLE_SHIFT;
    pub const STOP_ADDR_MASK: u32 = 1 << STOP_ADDR_SHIFT;
    pub const TX_EMPTY_MASK: u32 = 1 << TX_EMPTY_SHIFT;
    pub const RX_FULL_HOLD_MASK: u32 = 1 << RX_FULL_HOLD_SHIFT;
    pub const STOP_MASTER_MASK: u32 = 1 << STOP_MASTER_SHIFT;

    pub const SPEED_STANDARD: u32 = 0x1 << SPEED_SHIFT;
    pub const SPEED_FAST: u32 = 0x2 << SPEED_SHIFT;
    pub const SPEED_HIGH: u32 = 0x3 << SPEED_SHIFT;

    pub const ADDR_SLAVE7: u32 = 0 << ADDR_SLAVE_SHIFT;
    pub const ADDR_SLAVE10: u32 = 1 << ADDR_SLAVE_SHIFT;

    pub const ADDR_MASTER7: u32 = 0 << ADDR_MASTER_SHIFT;
    pub const ADDR_MASTER10: u32 = 1 << ADDR_MASTER_SHIFT;

    pub const VALID_MASK: u32 = MASTER_MODE_MASK |
        SPEED_MASK |
        ADDR10_SLAVE_MASK |
        ADDR10_MASTER_MASK |
        RESTART_MASK |
        SLAVE_DISABLE_MASK |
        STOP_ADDR_MASK |
        TX_EMPTY_MASK |
        RX_FULL_HOLD_MASK |
        STOP_MASTER_MASK;
}

mod target_addr_register {
    pub const TARGET_SHIFT: usize = 0;
    pub const GC_OR_START_SHIFT: usize = 10;
    pub const SPECIAL_SHIFT: usize = 11;

    pub const TARGET_MASK: u32 = 0x3ff << TARGET_SHIFT;
    pub const GC_OR_START_MASK: u32 = 1 << GC_OR_START_SHIFT;
    pub const SPECIAL_MASK: u32 = 1 << SPECIAL_SHIFT;

    pub const VALID_MASK: u32 = TARGET_MASK |
        GC_OR_START_MASK |
        SPECIAL_MASK;
}

mod slave_addr_register {
    pub const SLAVE_SHIFT: usize = 0;
    
    pub const SLAVE_MASK: u32 = 0x3ff << SLAVE_SHIFT;

    pub const VALID_MASK: u32 = SLAVE_MASK;
}

mod data_cmd_register {
    pub const DATA_SHIFT: usize = 0;
    pub const CMD_SHIFT: usize = 8;
    pub const STOP_SHIFT: usize = 9;
    pub const RESTART_SHIFT: usize = 10;
    pub const FIRST_DATA_BYTE_SHIFT: usize = 11;

    pub const DATA_MASK: u32 = 0xff << DATA_SHIFT;
    pub const CMD_MASK: u32 = 1 << CMD_SHIFT;
    pub const STOP_MASK: u32 = 1 << STOP_SHIFT;
    pub const RESTART_MASK: u32 = 1 << RESTART_SHIFT;
    pub const FIRST_DATA_BYTE_MASK: u32 = 1 << FIRST_DATA_BYTE_SHIFT;

    pub const VALID_MASK: u32 = DATA_MASK |
        CMD_MASK |
        STOP_MASK |
        RESTART_MASK |
        FIRST_DATA_BYTE_MASK;
}

mod fast_clock_high_register {
    pub const HIGH_COUNT_SHIFT: usize = 0;

    pub const HIGH_COUNT_MASK: u32 = 0xffff << HIGH_COUNT_SHIFT;

    pub const VALID_MASK: u32 = HIGH_COUNT_MASK;
}

mod fast_clock_low_register {
    pub const LOW_COUNT_SHIFT: usize = 0;

    pub const LOW_COUNT_MASK: u32 = 0xffff << LOW_COUNT_SHIFT;

    pub const VALID_MASK: u32 = LOW_COUNT_MASK;
}

mod inter_register {
    pub const RX_UNDER_SHIFT: usize = 0;
    pub const RX_OVER_SHIFT: usize = 1;
    pub const RX_FULL_SHIFT: usize = 2;
    pub const TX_OVER_SHIFT: usize = 3;
    pub const TX_EMPTY_SHIFT: usize = 4;
    pub const READ_REQUEST_SHIFT: usize = 5;
    pub const TX_ABORT_SHIFT: usize = 6;
    pub const RX_DONE_SHIFT: usize = 7;
    pub const ACTIVITY_SHIFT: usize = 8;
    pub const STOP_SHIFT: usize = 9;
    pub const START_SHIFT: usize = 10;
    pub const GEN_CALL_SHIFT: usize = 11;
    pub const RESTART_SHIFT: usize = 12;

    pub const RX_UNDER_MASK: u32 = 1 << RX_UNDER_SHIFT;
    pub const RX_OVER_MASK: u32 = 1 << RX_OVER_SHIFT;
    pub const RX_FULL_MASK: u32 = 1 << RX_FULL_SHIFT;
    pub const TX_OVER_MASK: u32 = 1 << TX_OVER_SHIFT;
    pub const TX_EMPTY_MASK: u32 = 1 << TX_EMPTY_SHIFT;
    pub const READ_REQUEST_MASK: u32 = 1 << READ_REQUEST_SHIFT;
    pub const TX_ABORT_MASK: u32 = 1 << TX_ABORT_SHIFT;
    pub const RX_DONE_MASK: u32 = 1 << RX_DONE_SHIFT;
    pub const ACTIVITY_MASK: u32 = 1 << ACTIVITY_SHIFT;
    pub const STOP_MASK: u32 = 1 << STOP_SHIFT;
    pub const START_MASK: u32 = 1 << START_SHIFT;
    pub const GEN_CALL_MASK: u32 = 1 << GEN_CALL_SHIFT;
    pub const RESTART_MASK: u32 = 1 << RESTART_SHIFT;

    pub const VALID_MASK: u32 = RX_UNDER_MASK |
        RX_OVER_MASK |
        RX_FULL_MASK |
        TX_OVER_MASK |
        TX_EMPTY_MASK |
        READ_REQUEST_MASK |
        TX_ABORT_MASK |
        RX_DONE_MASK |
        ACTIVITY_MASK |
        STOP_MASK |
        START_MASK |
        GEN_CALL_MASK |
        RESTART_MASK;
}

mod threshold_register {
    pub const THRESHOLD_SHIFT: usize = 0;

    pub const THRESHOLD_MASK: u32 = 0xff << THRESHOLD_SHIFT;

    pub const VALID_MASK: u32 = THRESHOLD_MASK;
}

mod clear_inter_register {
    pub const CLEAR_SHIFT: usize = 0;

    pub const CLEAR_MASK: u32 = 1 << CLEAR_SHIFT;

    pub const VALID_MASK: u32 = CLEAR_MASK;
}

mod enable_register {
    pub const ENABLE_SHIFT: usize = 0;
    pub const ABORT_SHIFT: usize = 1;
    pub const TX_CMD_BLOCK_SHIFT: usize = 2;

    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const ABORT_MASK: u32 = 1 << ABORT_SHIFT;
    pub const TX_CMD_BLOCK_MASK: u32 = 1 << TX_CMD_BLOCK_SHIFT;

    pub const VALID_MASK: u32 = ENABLE_MASK |
        ABORT_MASK |
        TX_CMD_BLOCK_MASK;
}

mod status_register {
    pub const ACTIVITY_SHIFT: usize = 0;
    pub const TX_NOT_FULL_SHIFT: usize = 1;
    pub const TX_EMPTY_SHIFT: usize = 2;
    pub const RX_NOT_EMPTY_SHIFT: usize = 3;
    pub const RX_FULL_SHIFT: usize = 4;
    pub const MASTER_ACTIVITY_SHIFT: usize = 5;
    pub const SLAVE_ACTIVITY_SHIFT: usize = 6;

    pub const ACTIVITY_MASK: u32 = 1 << ACTIVITY_SHIFT;
    pub const TX_NOT_FULL_MASK: u32 = 1 << TX_NOT_FULL_SHIFT;
    pub const TX_EMPTY_MASK: u32 = 1 << TX_EMPTY_SHIFT;
    pub const RX_NOT_EMPTY_MASK: u32 = 1 << RX_NOT_EMPTY_SHIFT;
    pub const RX_FULL_MASK: u32 = 1 << RX_FULL_SHIFT;
    pub const MASTER_ACTIVITY_MASK: u32 = 1 << MASTER_ACTIVITY_SHIFT;
    pub const SLAVE_ACTIVITY_MASK: u32 = 1 << SLAVE_ACTIVITY_SHIFT;

    pub const VALID_MASK: u32 = ACTIVITY_MASK |
        TX_NOT_FULL_MASK |
        TX_EMPTY_MASK |
        RX_NOT_EMPTY_MASK |
        RX_FULL_MASK |
        MASTER_ACTIVITY_MASK |
        SLAVE_ACTIVITY_MASK;
}

mod level_register {
    pub const LEVEL_SHIFT: usize = 0;

    pub const LEVEL_MASK: u32 = 0x1f << LEVEL_SHIFT;

    pub const VALID_MASK: u32 = LEVEL_MASK;
}

mod sda_hold_register {
    pub const TX_HOLD_SHIFT: usize = 0;
    pub const RX_HOLD_SHIFT: usize = 16;

    pub const TX_HOLD_MASK: u32 = 0xffff << TX_HOLD_SHIFT;
    pub const RX_HOLD_MASK: u32 = 0xff << RX_HOLD_SHIFT;

    pub const VALID_MASK: u32 = TX_HOLD_MASK |
        RX_HOLD_MASK;
}

mod tx_abort_src_register {
    pub const ADDR7_NO_ACK_SHIFT: usize = 0;
    pub const ADDR10_NO_ACK1_SHIFT: usize = 1;
    pub const ADDR10_NO_ACK2_SHIFT: usize = 2;
    pub const TX_DATA_NO_ACK_SHIFT: usize = 3;
    pub const GCALL_NO_ACK_SHIFT: usize = 4;
    pub const GCALL_READ_SHIFT: usize = 5;
    pub const HIGH_SPEED_ACK_SHIFT: usize = 6;
    pub const START_ACK_SHIFT: usize = 7;
    pub const HIGH_SPEED_NO_RESTART_SHIFT: usize = 8;
    pub const START_NO_RESTART_SHIFT: usize = 9;
    pub const ADDR10_READ_NO_RESTART_SHIFT: usize = 10;
    pub const MASTER_DISABLE_SHIFT: usize = 11;
    pub const ARBITRATION_LOST_SHIFT: usize = 12;
    pub const TX_FLUSH_SHIFT: usize = 13;
    pub const SLAVE_ARBITRATION_LOST_SHIFT: usize = 14;
    pub const SLAVE_READ_SHIFT: usize = 15;
    pub const USER_ABORT_SHIFT: usize = 16;
    pub const TX_FLUSH_COUNT_SHIFT: usize = 23;

    pub const ADDR7_NO_ACK_MASK: u32 = 1 << ADDR7_NO_ACK_SHIFT;
    pub const ADDR10_NO_ACK1_MASK: u32 = 1 << ADDR10_NO_ACK1_SHIFT;
    pub const ADDR10_NO_ACK2_MASK: u32 = 1 << ADDR10_NO_ACK2_SHIFT;
    pub const TX_DATA_NO_ACK_MASK: u32 = 1 << TX_DATA_NO_ACK_SHIFT;
    pub const GCALL_NO_ACK_MASK: u32 = 1 << GCALL_NO_ACK_SHIFT;
    pub const GCALL_READ_MASK: u32 = 1 << GCALL_READ_SHIFT;
    pub const HIGH_SPEED_ACK_MASK: u32 = 1 << HIGH_SPEED_ACK_SHIFT;
    pub const START_ACK_MASK: u32 = 1 << START_ACK_SHIFT;
    pub const HIGH_SPEED_NO_RESTART_MASK: u32 = 1 << HIGH_SPEED_NO_RESTART_SHIFT;
    pub const START_NO_RESTART_MASK: u32 = 1 << START_NO_RESTART_SHIFT;
    pub const ADDR10_READ_NO_RESTART_MASK: u32 = 1 << ADDR10_READ_NO_RESTART_SHIFT;
    pub const MASTER_DISABLE_MASK: u32 = 1 << MASTER_DISABLE_SHIFT;
    pub const ARBITRATION_LOST_MASK: u32 = 1 << ARBITRATION_LOST_SHIFT;
    pub const TX_FLUSH_MASK: u32 = 1 << TX_FLUSH_SHIFT;
    pub const SLAVE_ARBITRATION_LOST_MASK: u32 = 1 << SLAVE_ARBITRATION_LOST_SHIFT;
    pub const SLAVE_READ_MASK: u32 = 1 << SLAVE_READ_SHIFT;
    pub const USER_ABORT_MASK: u32 = 1 << USER_ABORT_SHIFT;
    pub const TX_FLUSH_COUNT_MASK: u32 = 0x1ff << TX_FLUSH_COUNT_SHIFT;

    pub const VALID_MASK: u32 = ADDR7_NO_ACK_MASK |
        ADDR10_NO_ACK1_MASK |
        ADDR10_NO_ACK2_MASK |
        TX_DATA_NO_ACK_MASK |
        GCALL_NO_ACK_MASK |
        GCALL_READ_MASK |
        HIGH_SPEED_ACK_MASK |
        START_ACK_MASK |
        HIGH_SPEED_NO_RESTART_MASK |
        START_NO_RESTART_MASK |
        ADDR10_READ_NO_RESTART_MASK |
        MASTER_DISABLE_MASK |
        ARBITRATION_LOST_MASK |
        TX_FLUSH_MASK |
        SLAVE_ARBITRATION_LOST_MASK |
        SLAVE_READ_MASK |
        USER_ABORT_MASK |
        TX_FLUSH_COUNT_MASK;
}

mod nack_register {
    pub const NACK_SHIFT: usize = 0;

    pub const NACK_MASK: u32 = 1 << NACK_SHIFT;

    pub const VALID_MASK: u32 = NACK_MASK;
}

mod sda_setup_register {
    pub const SDA_SETUP_SHIFT: usize = 0;

    pub const SDA_SETUP_MASK: u32 = 0xff << SDA_SETUP_SHIFT;

    pub const VALID_MASK: u32 = SDA_SETUP_MASK;
}

mod ack_general_call_register {
    pub const ACK_GENERAL_CALL_SHIFT: usize = 0;

    pub const ACK_GENERAL_CALL_MASK: u32 = 1 << ACK_GENERAL_CALL_SHIFT;

    pub const ACK_GENERAL_CALL_ACK: u32 = 1 << ACK_GENERAL_CALL_SHIFT;
    pub const ACK_GENERAL_CALL_NACK: u32 = 0 << ACK_GENERAL_CALL_SHIFT;

    pub const VALID_MASK: u32 = ACK_GENERAL_CALL_MASK;
}

mod enable_status_register {
    pub const ENABLED_SHIFT: usize = 0;
    pub const SLAVE_DISABLE_BUSY_SHIFT: usize = 1;
    pub const SLAVE_RX_DATA_LOST_SHIFT: usize = 2;

    pub const ENABLED_MASK: u32 = 1 << ENABLED_SHIFT;
    pub const SLAVE_DISABLE_BUSY_MASK: u32 = 1 << SLAVE_DISABLE_BUSY_SHIFT;
    pub const SLAVE_RX_DATA_LOST_MASK: u32 = 1 << SLAVE_RX_DATA_LOST_SHIFT;

    pub const VALID_MASK: u32 = ENABLED_MASK |
        SLAVE_DISABLE_BUSY_MASK |
        SLAVE_RX_DATA_LOST_MASK;
}

mod spike_len_register {
    pub const SPIKE_LEN_SHIFT: usize = 0;

    pub const SPIKE_LEN_MASK: u32 = 0xff << SPIKE_LEN_SHIFT;

    pub const VALID_MASK: u32 = SPIKE_LEN_MASK;
}

#[repr(C)]
struct I2CRegisters {
    ctrl: ReadPureWrite<u32>, // 0x0
    target_addr: ReadPureWrite<u32>, // 0x4
    slave_addr: ReadPureWrite<u32>, // 0x8
    _reserved0: u32, // 0xc
    data_cmd: ReadWrite<u32>, // 0x10
    standard_clock_high: ReadPureWrite<u32>, // 0x14
    standard_clock_low: ReadPureWrite<u32>, // 0x18
    fast_clock_high: ReadPureWrite<u32>, // 0x1c
    fast_clock_low: ReadPureWrite<u32>, // 0x20
    _reserved1: u32, // 0x24
    _reserved2: u32, // 0x28
    inter_status: ReadPure<u32>, // 0x2c
    inter_mask: ReadPureWrite<u32>, // 0x30
    raw_inter_status: ReadPure<u32>, // 0x34
    rx_threshold: ReadPureWrite<u32>, // 0x38
    tx_threshold: ReadPureWrite<u32>, // 0x3c
    clear_inter: ReadOnly<u32>, // 0x40
    clear_rx_under: ReadOnly<u32>, // 0x44
    clear_rx_over: ReadOnly<u32>, // 0x48
    clear_tx_over: ReadOnly<u32>, // 0x4c
    clear_read_request: ReadOnly<u32>, // 0x50 
    clear_tx_abort: ReadOnly<u32>, // 0x54 
    clear_rx_done: ReadOnly<u32>, // 0x58 
    clear_activity: ReadOnly<u32>, // 0x5c 
    clear_stop: ReadOnly<u32>, // 0x60 
    clear_start: ReadOnly<u32>, // 0x64
    clear_gen_call: ReadOnly<u32>, // 0x68
    enable: ReadPureWrite<u32>, // 0x6c
    status: ReadPure<u32>, // 0x70
    tx_level: ReadPure<u32>, // 0x74
    rx_level: ReadPure<u32>, // 0x78
    sda_hold: ReadPureWrite<u32>, // 0x7c
    tx_abort_src: ReadPure<u32>, // 0x80
    nack: ReadPureWrite<u32>, // 0x84
    dma_ctrl: ReadPureWrite<u32>, // 0x88
    dma_tx_level: ReadPureWrite<u32>, // 0x8c
    dma_rx_level: ReadPureWrite<u32>, // 0x90
    sda_setup: ReadPureWrite<u32>, // 0x94
    ack_general_call: ReadPureWrite<u32>, // 0x98
    enable_status: ReadPure<u32>, // 0x9c
    spike_len: ReadPureWrite<u32>, // 0xa0
    _reserved3: u32, // 0xa4
    clear_restart: ReadPure<u32> // 0xa8
}

pub struct I2C {
    registers: UniqueMmioPointer<'static, I2CRegisters>,
    set_reg: UniqueMmioPointer<'static, I2CRegisters>,
    clear_reg: UniqueMmioPointer<'static, I2CRegisters>
}

impl I2C {
    pub unsafe fn new(i2c_base: usize) -> Self {
        let mut  res = unsafe {
            Self { 
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(i2c_base)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(i2c_base + REG_ALIAS_SET_BITS)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(i2c_base + REG_ALIAS_CLR_BITS)).unwrap())
            }
        };
        field!(res.registers, enable).modify(|enable| enable & !enable_register::VALID_MASK);
        field!(res.registers, ctrl).modify(|ctrl|
            (ctrl & !ctrl_register::VALID_MASK) |
            ctrl_register::MASTER_MODE_MASK |
            ctrl_register::SPEED_FAST | 
            ctrl_register::ADDR_MASTER7 |
            ctrl_register::SLAVE_DISABLE_MASK |
            ctrl_register::RESTART_MASK |
            ctrl_register::TX_EMPTY_MASK
        );
        // clock high 48
        // clock low 72
        field!(res.registers, fast_clock_low).modify(|fast_clock_low|
            (fast_clock_low & !fast_clock_low_register::VALID_MASK) |
            (288 << fast_clock_low_register::LOW_COUNT_SHIFT)
        );
        field!(res.registers, fast_clock_high).modify(|fast_clock_high|
            (fast_clock_high & !fast_clock_high_register::VALID_MASK) |
            (192 << fast_clock_high_register::HIGH_COUNT_SHIFT)
        );
        // not entirely sure if this register value is correct
        // from what I understand, fast mode spike supression must be 50ns
        // This means the spike len register should have 50 ns x clock frequency
        // which is 50 ns x 48 MHz = 2.4 cycles (or 3 cycles when rounded up)
        field!(res.registers, spike_len).modify(|spike_len|
            (spike_len & !spike_len_register::VALID_MASK) |
            (18 << spike_len_register::SPIKE_LEN_SHIFT)
        );
        field!(res.registers, sda_hold).modify(|sda_hold|
            (sda_hold & !sda_hold_register::VALID_MASK) |
            (15 << sda_hold_register::TX_HOLD_SHIFT) |
            (0 << sda_hold_register::RX_HOLD_SHIFT)
        );
        field!(res.registers, tx_threshold).modify(|tx_threshold| tx_threshold & !threshold_register::VALID_MASK);
        field!(res.registers, rx_threshold).modify(|rx_threshold| rx_threshold & !threshold_register::VALID_MASK);
        res
    }

    

    fn wait_tx_done(&mut self) -> Result<(), I2CReplyError> {
        field!(self.registers, tx_threshold).modify(|tx_threshold| tx_threshold & !threshold_register::VALID_MASK);
        loop {
            wait_irq().unwrap();
            let inter = field!(self.registers, inter_status).read();
            self.check_abort(inter)?;
            if inter & inter_register::TX_EMPTY_MASK != 0 {
                break;
            }
        }
        Ok(())
    }

    fn fill_tx_no_rx(&mut self, buffer: &[u8], index: &mut usize) {
        while field!(self.registers, status).read() & status_register::TX_NOT_FULL_MASK != 0 && *index < buffer.len() {
            let stop_mask = if *index == buffer.len() - 1 {
                data_cmd_register::STOP_MASK
            } else {
                0
            };
            field!(self.registers, data_cmd).write(
                (((buffer[*index] as u32) << data_cmd_register::DATA_SHIFT) & data_cmd_register::DATA_MASK) |
                stop_mask
            );
            *index += 1;
        }
    }

    fn clear_irq(&mut self) {
        _ = field!(self.registers, clear_inter).read();
    }

    fn fill_rx_tx(&mut self, len: usize, tx_index: &mut usize) {
        while field!(self.registers, status).read() & status_register::RX_FULL_MASK == 0 && *tx_index < len {
            let restart_mask = if *tx_index == 0 {
                data_cmd_register::RESTART_MASK
            } else {
                0
            };
            let stop_mask = if *tx_index == len - 1 {
                data_cmd_register::STOP_MASK
            } else {
                0
            };
            field!(self.registers, data_cmd).write(
                data_cmd_register::CMD_MASK | 
                restart_mask |
                stop_mask
            );
            *tx_index += 1;
        }
    }

    fn remove_rx_buffer(&mut self, buffer: &mut [u8], rx_index: &mut usize) {
        while field!(self.registers, status).read() & status_register::RX_NOT_EMPTY_MASK != 0 && *rx_index < buffer.len() {
            let data = ((field!(self.registers, data_cmd).read() & data_cmd_register::DATA_MASK) >> data_cmd_register::DATA_SHIFT) as u8;
            buffer[*rx_index] = data;
            *rx_index += 1;
        }
    }
    
    fn fill_tx_before_rx(&mut self, buffer: &[u8], index: &mut usize) {
        while field!(self.registers, status).read() & status_register::TX_NOT_FULL_MASK != 0 && *index < buffer.len() {
            field!(self.registers, data_cmd).write(
                ((buffer[*index] as u32) << data_cmd_register::DATA_SHIFT) & data_cmd_register::DATA_MASK
            );
            *index += 1;
        }
    }

    fn check_abort(&mut self, inter: u32) -> Result<(), I2CReplyError> {
        if inter & inter_register::TX_ABORT_MASK != 0 {
            field!(self.registers, clear_tx_abort).read();
            Err(I2CReplyError::Abort)
        } else {
            Ok(())
        }
    }

    fn do_tx_only(&mut self, buffer: &[u8]) -> Result<(), I2CReplyError> {
        if buffer.len() == 0 {
            return Ok(());
        }
        let mut i = 0;
        self.fill_tx_no_rx(buffer, &mut i);
        field!(self.registers, inter_mask).write(inter_register::TX_EMPTY_MASK | inter_register::TX_ABORT_MASK);
        while i < buffer.len() {
            let slots = 8.min((buffer.len() - i) as u32);
            field!(self.registers, tx_threshold).modify(|tx_threshold| 
                (tx_threshold & !threshold_register::VALID_MASK) |
                ((16 - slots) << threshold_register::THRESHOLD_SHIFT)
            );
            wait_irq().unwrap();
            let inter = field!(self.registers, inter_status).read();
            self.check_abort(inter)?;
            if inter & inter_register::TX_EMPTY_MASK != 0 {
                self.fill_tx_no_rx(buffer, &mut i);
            }
        }
        self.wait_tx_done()?;
        Ok(())
    }

    fn do_tx_before_rx(&mut self, buffer: &[u8]) -> Result<(), I2CReplyError> {
        if buffer.len() == 0 {
            return Ok(());
        }
        let mut i = 0;
        self.fill_tx_before_rx(buffer, &mut i);
        field!(self.registers, inter_mask).modify(|inter_mask|
            (inter_mask & !inter_register::VALID_MASK) |
            inter_register::TX_EMPTY_MASK | 
            inter_register::TX_ABORT_MASK
        );
        self.clear_irq();
        while i < buffer.len() {
            let slots = 8.min((buffer.len() - i) as u32);
            field!(self.registers, tx_threshold).modify(|tx_threshold|
                (tx_threshold & !threshold_register::VALID_MASK) |
                ((16 - slots) << threshold_register::THRESHOLD_SHIFT)
            );
            wait_irq().unwrap();
            let inter = field!(self.registers, inter_status).read();
            self.clear_irq();
            self.check_abort(inter)?;
            if inter & inter_register::TX_EMPTY_MASK != 0 {
                self.fill_tx_before_rx(buffer, &mut i);
            }
        }
        self.wait_tx_done()?;
        Ok(())
    }

    fn do_rx(&mut self, buffer: &mut [u8]) -> Result<(), I2CReplyError> {
        assert!(buffer.len() != 0);
        let mut tx = 0;
        let mut rx = 0;
        self.fill_rx_tx(buffer.len(), &mut tx);
        field!(self.registers, inter_mask).modify(|inter_mask|
            (inter_mask & !inter_register::VALID_MASK) |
            inter_register::RX_FULL_MASK | 
            inter_register::TX_ABORT_MASK
        );
        self.clear_irq();
        if tx < buffer.len() {
            let slots = 8.min((buffer.len() - tx) as u32);
            field!(self.registers, tx_threshold).modify(|tx_threshold|
                (tx_threshold & !threshold_register::VALID_MASK) |
                ((16 - slots) << threshold_register::THRESHOLD_SHIFT)
            );
            field!(self.set_reg, inter_mask).write(inter_register::TX_EMPTY_MASK);
        }
        while rx < buffer.len() {
            let slots = 16.min((buffer.len() - rx - 1) as u32);
            field!(self.registers, rx_threshold).modify(|rx_threshold|
                (rx_threshold & !threshold_register::VALID_MASK) |
                (slots << threshold_register::THRESHOLD_SHIFT)
            );
            wait_irq().unwrap();
            let inter = field!(self.registers, inter_status).read();
            self.clear_irq();
            self.check_abort(inter)?;
            if inter & inter_register::RX_FULL_MASK != 0 {
                self.remove_rx_buffer(buffer, &mut rx);
            }
            if inter & inter_register::TX_EMPTY_MASK != 0 {
                self.fill_rx_tx(buffer.len(), &mut tx);
                if tx < buffer.len() {
                    let slots = 8.min((buffer.len() - tx) as u32);
                    field!(self.registers, tx_threshold).modify(|tx_threshold|
                        (tx_threshold & !threshold_register::VALID_MASK) |
                        ((16 - slots) << threshold_register::THRESHOLD_SHIFT)
                    );
                } else {
                    field!(self.clear_reg, inter_mask).write(inter_register::TX_EMPTY_MASK);
                }
            }
        }
        while field!(self.registers, status).read() & status_register::RX_NOT_EMPTY_MASK != 0 {
            // flush rx buffer
            _ = field!(self.registers, data_cmd).read();
        }
        Ok(())
    }

    fn do_tx_blocking(&mut self, addr: u8, buffer: &[u8], stop: bool) {
        field!(self.registers, enable).write(0);
        field!(self.registers, target_addr).write(addr as u32);
        field!(self.registers, enable).write(enable_register::ENABLE_MASK);
        for (i, byte) in buffer.iter().enumerate() {
            let stop = if i == buffer.len() - 1 && stop {
                data_cmd_register::STOP_MASK
            } else {
                0
            };
            let word = *byte as u32 | stop;
            field!(self.registers, data_cmd).write(word);
        }
        loop {
            if field!(self.registers, raw_inter_status).read() & inter_register::TX_EMPTY_MASK != 0 {
                break;
            }
        }
    }

    fn do_rx_blocking(&mut self, addr: u8, buffer: &mut [u8]) {
        field!(self.registers, enable).write(0);
        field!(self.registers, target_addr).write(addr as u32);
        field!(self.registers, enable).write(enable_register::ENABLE_MASK);
        for i in 0..buffer.len() {
            let restart = if i == 0 {
                data_cmd_register::RESTART_MASK
            } else {
                0
            };
            let stop = if i == buffer.len() - 1 {
                data_cmd_register::STOP_MASK
            } else {
                0
            };
            field!(self.registers, data_cmd).write(restart | stop | data_cmd_register::CMD_MASK);
            while field!(self.registers, rx_level).read() == 0 {
                do_yield().unwrap();
            }
            buffer[i] = (field!(self.registers, data_cmd).read() & 0xff) as u8;
        }
    }

    pub fn send_cmd(&mut self, addr: u8, buffer: &mut [u8], tx_len: usize, rx_len: usize) -> Result<(), I2CReplyError> {
        assert!(addr < 0x80);
        assert!(tx_len <= buffer.len());
        assert!(rx_len <= buffer.len());
        field!(self.registers, target_addr).write(
            ((addr as u32) << target_addr_register::TARGET_SHIFT) & target_addr_register::TARGET_MASK
        );
        field!(self.registers, enable).write(enable_register::ENABLE_MASK);
        while field!(self.registers, enable_status).read() & enable_status_register::ENABLED_MASK == 0 {
            do_yield().unwrap();
        }
        if rx_len == 0 {
            self.do_tx_only(&buffer[..tx_len])?;
        } else {
            self.do_tx_before_rx(&buffer[..tx_len])?;
            self.do_rx(&mut buffer[..rx_len])?;
        }
        field!(self.registers, enable).write(0);
        Ok(())
    }
}

pub enum I2CReplyError {
    SendError,
    InvalidRequest,
    InvalidAddress,
    Abort,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<I2CReplyError> for u32 {
    fn from(value: I2CReplyError) -> Self {
        match value {
            I2CReplyError::SendError => 1,
            I2CReplyError::InvalidRequest => 2,
            I2CReplyError::InvalidAddress => 3,
            I2CReplyError::Abort => 4,
            I2CReplyError::InvalidSendBuffer => 5,
            I2CReplyError::InvalidReplyBuffer => 6
        }
    }
}

pub enum I2CError {
    ReplyError(I2CReplyError),
    QueueError(QueueError)
}

impl From<I2CReplyError> for I2CError {
    fn from(value: I2CReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for I2CError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

pub struct Request {
    addr: u8,
    tx_len: usize,
    rx_len: usize,
    buffer: [u8; 32],
}

impl Request {
    pub fn parse() -> Result<Self, I2CError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // I2C operation
                let mut req_buffer = [0; 33];
                if header.send_len as usize > req_buffer.len() {
                    return Err(I2CError::ReplyError(I2CReplyError::InvalidSendBuffer));
                }
                if header.reply_len as usize > req_buffer.len() - 1 {
                    return Err(I2CError::ReplyError(I2CReplyError::InvalidReplyBuffer));
                }
                _ = receive(0, &mut req_buffer)?;
                let addr = req_buffer[0];
                if addr >= 0x80 {
                    return Err(I2CError::ReplyError(I2CReplyError::InvalidAddress));
                }
                let mut buffer = [0; 32];
                buffer[..header.send_len as usize - 1].copy_from_slice(&req_buffer[1..header.send_len as usize]);
                Ok(Request { 
                    addr, 
                    tx_len: header.send_len as usize - 1, 
                    rx_len: header.reply_len as usize, 
                    buffer 
                })
            },
            _ => {
                Err(I2CError::ReplyError(I2CReplyError::InvalidRequest))
            }
        }
    }
}

const IO_BANK0_QUEUE: u32 = 0;

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 2);
    let i2c_base = args[0] as usize;
    send_empty(IO_BANK0_QUEUE, 0, &[]).unwrap();
    let mut i2c = unsafe {
        I2C::new(i2c_base)
    };
    #[cfg(test)]
    test_main();
    loop {
        match Request::parse() {
            Ok(mut request) => {
                match i2c.send_cmd(request.addr, &mut request.buffer, request.tx_len, request.rx_len) {
                    Ok(()) => {
                        check_critical(reply(0, 0, &request.buffer[..request.rx_len])).unwrap_or(Ok(())).unwrap();
                    },
                    Err(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    }
                }
            },
            Err(err) => {
                match err {
                    I2CError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    I2CError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(I2CReplyError::SendError))).unwrap_or(Ok(())).unwrap();
                            },
                            _ => {
                                panic!("{:?}", err);
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::{kprint, kprintln};

    use super::*;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for I2C", tests.len());
        for test in tests {
            test();
        }
    }

    #[test_case]
    fn test_setup() {
        let args = args();
        let i2c_base = args[0] as usize;
        let mut i2c = unsafe {
            I2C::new(i2c_base)
        };
        kprintln!("Testing I2C setup");
        kprint!("Testing ctrl register ");
        let ctrl = field!(i2c.registers, ctrl).read();
        assert_eq!(ctrl & ctrl_register::VALID_MASK, 0x165);
        kprintln!("[ok]");
        kprint!("Testing fast clock low register ");
        let clock_low = field!(i2c.registers, fast_clock_low).read();
        assert_eq!(clock_low & fast_clock_low_register::VALID_MASK, 288);
        kprintln!("[ok]");
        kprint!("Testing fast clock high register ");
        let clock_high = field!(i2c.registers, fast_clock_high).read();
        assert_eq!(clock_high & fast_clock_high_register::VALID_MASK, 192);
        kprintln!("[ok]");
        kprint!("Testing spike len register ");
        let spike_len = field!(i2c.registers, spike_len).read();
        assert_eq!(spike_len & spike_len_register::VALID_MASK, 18);
        kprintln!("[ok]");
        kprint!("Testing sda hold register ");
        let sda_hold = field!(i2c.registers, sda_hold).read();
        assert_eq!(sda_hold & sda_hold_register::VALID_MASK, 15);
        kprintln!("[ok]");
        kprint!("Testing tx threshold register ");
        let tx_threshold = field!(i2c.registers, tx_threshold).read();
        assert_eq!(tx_threshold & threshold_register::VALID_MASK, 0);
        kprintln!("[ok]");
        kprint!("Testing rx threshold register ");
        let rx_threshold = field!(i2c.registers, rx_threshold).read();
        assert_eq!(rx_threshold & threshold_register::VALID_MASK, 0);
        kprintln!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        kprintln!("Testing I2C register mask values");
        kprint!("Testing ctrl register ");
        assert_eq!(ctrl_register::VALID_MASK, 0x7ff);
        kprintln!("[ok]");
        kprint!("Testing target addr register ");
        assert_eq!(target_addr_register::VALID_MASK, 0xfff);
        kprintln!("[ok]");
        kprint!("Testing slave addr register ");
        assert_eq!(slave_addr_register::VALID_MASK, 0x3ff);
        kprintln!("[ok]");
        kprint!("Testing data cmd register ");
        assert_eq!(data_cmd_register::VALID_MASK, 0xfff);
        kprintln!("[ok]");
        kprint!("Testing fast clock high register ");
        assert_eq!(fast_clock_high_register::VALID_MASK, 0xffff);
        kprintln!("[ok]");
        kprint!("Testing fast clock low register ");
        assert_eq!(fast_clock_low_register::VALID_MASK, 0xffff);
        kprintln!("[ok]");
        kprint!("Testing inter register ");
        assert_eq!(inter_register::VALID_MASK, 0x1fff);
        kprintln!("[ok]");
        kprint!("Testing threshold register ");
        assert_eq!(threshold_register::VALID_MASK, 0xff);
        kprintln!("[ok]");
        kprint!("Testing clear inter register ");
        assert_eq!(clear_inter_register::VALID_MASK, 0x1);
        kprintln!("[ok]");
        kprint!("Testing enable register ");
        assert_eq!(enable_register::VALID_MASK, 0x7);
        kprintln!("[ok]");
        kprint!("Testing status register ");
        assert_eq!(status_register::VALID_MASK, 0x7f);
        kprintln!("[ok]");
        kprint!("Testing level register ");
        assert_eq!(level_register::VALID_MASK, 0x1f);
        kprintln!("[ok]");
        kprint!("Testing sda hold register ");
        assert_eq!(sda_hold_register::VALID_MASK, 0xffffff);
        kprintln!("[ok]");
        kprint!("Testing tx abort src register ");
        assert_eq!(tx_abort_src_register::VALID_MASK, 0xff81ffff);
        kprintln!("[ok]");
        kprint!("Testing nack register ");
        assert_eq!(nack_register::VALID_MASK, 0x1);
        kprintln!("[ok]");
        kprint!("Testing sda setup register ");
        assert_eq!(sda_setup_register::VALID_MASK, 0xff);
        kprintln!("[ok]");
        kprint!("Testing ack general call register ");
        assert_eq!(ack_general_call_register::VALID_MASK, 0x1);
        kprintln!("[ok]");
        kprint!("Testing enable status register ");
        assert_eq!(enable_status_register::VALID_MASK, 0x7);
        kprintln!("[ok]");
        kprint!("Testing spike len register ");
        assert_eq!(spike_len_register::VALID_MASK, 0xff);
        kprintln!("[ok]");
    }
}
