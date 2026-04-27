/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS I2C Driver.
 *
 * The SmallOS I2C Driver is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS I2C Driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS I2C Driver. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

/* File based on the I2C implementation in the raspberry pi pico SDK at https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/i2c.c (accessed 25/03/2026)
 * under the license
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]


use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadOnly, ReadPure, ReadPureWrite, ReadWrite}};
use small_os_lib::{QueueError, REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS, args, check_critical, do_yield, read_header, receive, reply, reply_empty, send_empty, wait_irq};

/// Control register masks and shifts
mod ctrl_register {
    /// Shift for master mode enabled
    pub const MASTER_MODE_SHIFT: usize = 0;
    /// Speed shift
    pub const SPEED_SHIFT: usize = 1;
    /// Shift for 10 bit slave addresses
    pub const ADDR_SLAVE_SHIFT: usize = 3;
    /// Shift for 10 bit master addresses
    pub const ADDR_MASTER_SHIFT: usize = 4;
    /// Shift for enabling restarts
    pub const RESTART_SHIFT: usize = 5;
    /// Shift for disabling slave mode
    pub const SLAVE_DISABLE_SHIFT: usize = 6;
    /// Shift for issuing the stop detected interrupt only when addressed in slave mode
    pub const STOP_ADDR_SHIFT: usize = 7;
    /// Shift for enabling TX empty interrupts
    pub const TX_EMPTY_SHIFT: usize = 8;
    /// Shift for enabling RX full interrupts
    pub const RX_FULL_HOLD_SHIFT: usize = 9;
    /// Shift for issuing the stop detected interrupt only when addressed in master mode
    pub const STOP_MASTER_SHIFT: usize = 10;

    /// Mask for master mode enabled
    pub const MASTER_MODE_MASK: u32 = 1 << MASTER_MODE_SHIFT;
    /// Speed mask
    pub const SPEED_MASK: u32 = 0x3 << SPEED_SHIFT;
    /// Mask for 10 bit slave addresses
    pub const ADDR10_SLAVE_MASK: u32 = 1 << ADDR_SLAVE_SHIFT;
    /// Mask for 10 bit master addresses
    pub const ADDR10_MASTER_MASK: u32 = 1 << ADDR_MASTER_SHIFT;
    /// Mask for enabling restarts
    pub const RESTART_MASK: u32 = 1 << RESTART_SHIFT;
    /// Mask for disabling slave mode
    pub const SLAVE_DISABLE_MASK: u32 = 1 << SLAVE_DISABLE_SHIFT;
    /// Mask for issuing the stop detected interrupt only when addressed in slave mode
    pub const STOP_ADDR_MASK: u32 = 1 << STOP_ADDR_SHIFT;
    /// Mask for enabling TX empty interrupts
    pub const TX_EMPTY_MASK: u32 = 1 << TX_EMPTY_SHIFT;
    /// Mask for enabling RX full interrupts
    pub const RX_FULL_HOLD_MASK: u32 = 1 << RX_FULL_HOLD_SHIFT;
    /// Mask for issuing the stop detected interrupt only when addressed in master mode
    pub const STOP_MASTER_MASK: u32 = 1 << STOP_MASTER_SHIFT;

    /// Standard speed mode (100 kbit/s)
    pub const SPEED_STANDARD: u32 = 0x1 << SPEED_SHIFT;
    /// Fast speed mode (<= 400 kbit/s) or fast mode plus (<= 1000 kbit/s)
    pub const SPEED_FAST: u32 = 0x2 << SPEED_SHIFT;
    /// High speed mode (3.4 Mbit/s)
    pub const SPEED_HIGH: u32 = 0x3 << SPEED_SHIFT;

    /// 7 bit slave addressing
    pub const ADDR_SLAVE7: u32 = 0 << ADDR_SLAVE_SHIFT;
    /// 10 bit slave addressing
    pub const ADDR_SLAVE10: u32 = 1 << ADDR_SLAVE_SHIFT;

    /// 7 bit master addressing
    pub const ADDR_MASTER7: u32 = 0 << ADDR_MASTER_SHIFT;
    /// 10 bit master addressing
    pub const ADDR_MASTER10: u32 = 1 << ADDR_MASTER_SHIFT;

    /// Mask of all non-reserved bits
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

/// Target address register masks and shifts
mod target_addr_register {
    /// Target address shift
    pub const TARGET_SHIFT: usize = 0;
    /// Shift for performing a general call or sending a start byte
    pub const GC_OR_START_SHIFT: usize = 10;
    /// Shift for programming of general call or start byte transmission
    pub const SPECIAL_SHIFT: usize = 11;

    /// Target address mask
    pub const TARGET_MASK: u32 = 0x3ff << TARGET_SHIFT;
    /// Mask for performing a general call or sending a start byte
    pub const GC_OR_START_MASK: u32 = 1 << GC_OR_START_SHIFT;
    /// Mask for programming of general call or start byte transmission
    pub const SPECIAL_MASK: u32 = 1 << SPECIAL_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = TARGET_MASK |
        GC_OR_START_MASK |
        SPECIAL_MASK;
}

/// Slave address register masks and shifts
mod slave_addr_register {
    /// Slave address shift
    pub const SLAVE_SHIFT: usize = 0;
    
    /// Slave address mask
    pub const SLAVE_MASK: u32 = 0x3ff << SLAVE_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SLAVE_MASK;
}

/// Data command register masks and shifts
mod data_cmd_register {
    /// Data shift
    pub const DATA_SHIFT: usize = 0;
    /// Command shift
    pub const CMD_SHIFT: usize = 8;
    /// Shift for issuing a stop after byte transmission
    pub const STOP_SHIFT: usize = 9;
    /// Shift for issuing a restart before byte transmission
    pub const RESTART_SHIFT: usize = 10;
    /// Shift for indicating this byte received is the first data byte after the address phase
    pub const FIRST_DATA_BYTE_SHIFT: usize = 11;

    /// Data mask
    pub const DATA_MASK: u32 = 0xff << DATA_SHIFT;
    /// Command mask
    pub const CMD_MASK: u32 = 1 << CMD_SHIFT;
    /// Mask for issuing a stop after byte transmission
    pub const STOP_MASK: u32 = 1 << STOP_SHIFT;
    /// Mask for issuing a restart before byte transmission
    pub const RESTART_MASK: u32 = 1 << RESTART_SHIFT;
    /// Mask for indicating this byte received is the first data byte after the address phase
    pub const FIRST_DATA_BYTE_MASK: u32 = 1 << FIRST_DATA_BYTE_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = DATA_MASK |
        CMD_MASK |
        STOP_MASK |
        RESTART_MASK |
        FIRST_DATA_BYTE_MASK;
}

/// Fast mode or fast mode plus clock high count register masks and shifts
mod fast_clock_high_register {
    /// High count shift
    pub const HIGH_COUNT_SHIFT: usize = 0;

    /// High count mask
    pub const HIGH_COUNT_MASK: u32 = 0xffff << HIGH_COUNT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = HIGH_COUNT_MASK;
}

/// Fast mode or fast mode plus clock low count register masks and shifts
mod fast_clock_low_register {
    /// Low count shift
    pub const LOW_COUNT_SHIFT: usize = 0;

    /// Low count mask
    pub const LOW_COUNT_MASK: u32 = 0xffff << LOW_COUNT_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = LOW_COUNT_MASK;
}

/// Interrupt registers masks and shifts
mod inter_register {
    /// RX underflow interrupt shift
    pub const RX_UNDER_SHIFT: usize = 0;
    /// RX overflow interrupt shift
    pub const RX_OVER_SHIFT: usize = 1;
    /// RX full interrupt shift
    pub const RX_FULL_SHIFT: usize = 2;
    /// TX overflow interrupt shift
    pub const TX_OVER_SHIFT: usize = 3;
    /// TX empty interrupt shift
    pub const TX_EMPTY_SHIFT: usize = 4;
    /// Read request interrupt shift
    pub const READ_REQUEST_SHIFT: usize = 5;
    /// TX abort interrupt shift
    pub const TX_ABORT_SHIFT: usize = 6;
    /// RX done interrupt shift
    pub const RX_DONE_SHIFT: usize = 7;
    /// Activity shift
    pub const ACTIVITY_SHIFT: usize = 8;
    /// Stop detected interrupt shift
    pub const STOP_SHIFT: usize = 9;
    /// Start detected interrupt shift
    pub const START_SHIFT: usize = 10;
    /// General call interrupt shift
    pub const GEN_CALL_SHIFT: usize = 11;
    /// Restart detected shift
    pub const RESTART_SHIFT: usize = 12;

    /// RX underflow interrupt mask
    pub const RX_UNDER_MASK: u32 = 1 << RX_UNDER_SHIFT;
    /// RX overflow interrupt mask
    pub const RX_OVER_MASK: u32 = 1 << RX_OVER_SHIFT;
    /// RX full interrupt mask
    pub const RX_FULL_MASK: u32 = 1 << RX_FULL_SHIFT;
    /// TX overflow interrupt mask
    pub const TX_OVER_MASK: u32 = 1 << TX_OVER_SHIFT;
    /// TX empty interrupt mask
    pub const TX_EMPTY_MASK: u32 = 1 << TX_EMPTY_SHIFT;
    /// Read request interrupt mask
    pub const READ_REQUEST_MASK: u32 = 1 << READ_REQUEST_SHIFT;
    /// TX abort interrupt mask
    pub const TX_ABORT_MASK: u32 = 1 << TX_ABORT_SHIFT;
    /// RX done interrupt mask
    pub const RX_DONE_MASK: u32 = 1 << RX_DONE_SHIFT;
    /// Activity mask
    pub const ACTIVITY_MASK: u32 = 1 << ACTIVITY_SHIFT;
    /// Stop detected interrupt mask
    pub const STOP_MASK: u32 = 1 << STOP_SHIFT;
    /// Start detected interrupt mask
    pub const START_MASK: u32 = 1 << START_SHIFT;
    /// General call interrupt mask
    pub const GEN_CALL_MASK: u32 = 1 << GEN_CALL_SHIFT;
    /// Restart detected mask
    pub const RESTART_MASK: u32 = 1 << RESTART_SHIFT;

    /// Mask of all non-reserved bits
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

/// Fifo threshold register masks and shifts
mod threshold_register {
    /// Threshold shift
    pub const THRESHOLD_SHIFT: usize = 0;

    /// Threshold mask
    pub const THRESHOLD_MASK: u32 = 0xff << THRESHOLD_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = THRESHOLD_MASK;
}

/// Interrupt clear registers masks and shifts
mod clear_inter_register {
    /// Shift to clear register
    pub const CLEAR_SHIFT: usize = 0;

    /// Mask to clear register
    pub const CLEAR_MASK: u32 = 1 << CLEAR_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = CLEAR_MASK;
}

/// Enable register masks and shifts
mod enable_register {
    /// Shift to enable I2C
    pub const ENABLE_SHIFT: usize = 0;
    /// Shift to abort operation
    pub const ABORT_SHIFT: usize = 1;
    /// Shift to block the sending of TX data
    pub const TX_CMD_BLOCK_SHIFT: usize = 2;

    /// Mask to enable I2C
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Mask to abort operation
    pub const ABORT_MASK: u32 = 1 << ABORT_SHIFT;
    /// Mask to block the sending of TX data
    pub const TX_CMD_BLOCK_MASK: u32 = 1 << TX_CMD_BLOCK_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ENABLE_MASK |
        ABORT_MASK |
        TX_CMD_BLOCK_MASK;
}

/// Status register masks and shifts
mod status_register {
    /// Activity status shift 
    pub const ACTIVITY_SHIFT: usize = 0;
    /// TX not full shift
    pub const TX_NOT_FULL_SHIFT: usize = 1;
    /// TX empty shift
    pub const TX_EMPTY_SHIFT: usize = 2;
    /// RX not empty shift
    pub const RX_NOT_EMPTY_SHIFT: usize = 3;
    /// RX full shift
    pub const RX_FULL_SHIFT: usize = 4;
    /// Master activity shift
    pub const MASTER_ACTIVITY_SHIFT: usize = 5;
    /// Slave activity shift
    pub const SLAVE_ACTIVITY_SHIFT: usize = 6;

    /// Activity status mask 
    pub const ACTIVITY_MASK: u32 = 1 << ACTIVITY_SHIFT;
    /// TX not full mask
    pub const TX_NOT_FULL_MASK: u32 = 1 << TX_NOT_FULL_SHIFT;
    /// TX empty mask
    pub const TX_EMPTY_MASK: u32 = 1 << TX_EMPTY_SHIFT;
    /// RX not empty mask
    pub const RX_NOT_EMPTY_MASK: u32 = 1 << RX_NOT_EMPTY_SHIFT;
    /// RX full mask
    pub const RX_FULL_MASK: u32 = 1 << RX_FULL_SHIFT;
    /// Master activity mask
    pub const MASTER_ACTIVITY_MASK: u32 = 1 << MASTER_ACTIVITY_SHIFT;
    /// Slave activity mask
    pub const SLAVE_ACTIVITY_MASK: u32 = 1 << SLAVE_ACTIVITY_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ACTIVITY_MASK |
        TX_NOT_FULL_MASK |
        TX_EMPTY_MASK |
        RX_NOT_EMPTY_MASK |
        RX_FULL_MASK |
        MASTER_ACTIVITY_MASK |
        SLAVE_ACTIVITY_MASK;
}

/// Fifo level registers masks and shifts
mod level_register {
    /// Level shift
    pub const LEVEL_SHIFT: usize = 0;

    /// Level mask
    pub const LEVEL_MASK: u32 = 0x1f << LEVEL_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = LEVEL_MASK;
}

/// SDA hold time length register masks and shifts
mod sda_hold_register {
    /// TX hold shift
    pub const TX_HOLD_SHIFT: usize = 0;
    /// RX hold shift
    pub const RX_HOLD_SHIFT: usize = 16;

    /// TX hold mask
    pub const TX_HOLD_MASK: u32 = 0xffff << TX_HOLD_SHIFT;
    /// RX hold mask
    pub const RX_HOLD_MASK: u32 = 0xff << RX_HOLD_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = TX_HOLD_MASK |
        RX_HOLD_MASK;
}

/// TX abort source register
mod tx_abort_src_register {
    /// 7 bit no acknowledge shift
    pub const ADDR7_NO_ACK_SHIFT: usize = 0;
    /// 10 bit no acknowledge shift
    pub const ADDR10_NO_ACK1_SHIFT: usize = 1;
    /// 2nd byte of 10 bit address not acknowledged shift
    pub const ADDR10_NO_ACK2_SHIFT: usize = 2;
    /// TX data not acknowledged shift
    pub const TX_DATA_NO_ACK_SHIFT: usize = 3;
    /// General call not acknowledged shift
    pub const GCALL_NO_ACK_SHIFT: usize = 4;
    /// Read following general call shift
    pub const GCALL_READ_SHIFT: usize = 5;
    /// High speed master code acknowledged shift
    pub const HIGH_SPEED_ACK_SHIFT: usize = 6;
    /// Start byte acknowledged shift
    pub const START_ACK_SHIFT: usize = 7;
    /// High speed no restart shift
    pub const HIGH_SPEED_NO_RESTART_SHIFT: usize = 8;
    /// Start byte sent when restart disabled shift
    pub const START_NO_RESTART_SHIFT: usize = 9;
    /// 10 bit mode read no restart shift
    pub const ADDR10_READ_NO_RESTART_SHIFT: usize = 10;
    /// Master operation with disabled master shift
    pub const MASTER_DISABLE_SHIFT: usize = 11;
    /// Arbitration lost shift
    pub const ARBITRATION_LOST_SHIFT: usize = 12;
    /// TX abort to flush TX fifo shift
    pub const TX_FLUSH_SHIFT: usize = 13;
    /// Slave arbitration lost shift
    pub const SLAVE_ARBITRATION_LOST_SHIFT: usize = 14;
    /// Slave transmitting to master in read mode shift
    pub const SLAVE_READ_SHIFT: usize = 15;
    /// Transfer abort shift
    pub const USER_ABORT_SHIFT: usize = 16;
    /// TX flush count shift
    pub const TX_FLUSH_COUNT_SHIFT: usize = 23;

    /// 7 bit no acknowledge mask
    pub const ADDR7_NO_ACK_MASK: u32 = 1 << ADDR7_NO_ACK_SHIFT;
    /// 10 bit no acknowledge mask
    pub const ADDR10_NO_ACK1_MASK: u32 = 1 << ADDR10_NO_ACK1_SHIFT;
    /// 2nd byte of 10 bit address not acknowledged mask
    pub const ADDR10_NO_ACK2_MASK: u32 = 1 << ADDR10_NO_ACK2_SHIFT;
    /// TX data not acknowledged mask
    pub const TX_DATA_NO_ACK_MASK: u32 = 1 << TX_DATA_NO_ACK_SHIFT;
    /// General call not acknowledged mask
    pub const GCALL_NO_ACK_MASK: u32 = 1 << GCALL_NO_ACK_SHIFT;
    /// Read following general call mask
    pub const GCALL_READ_MASK: u32 = 1 << GCALL_READ_SHIFT;
    /// High speed master code acknowledged mask
    pub const HIGH_SPEED_ACK_MASK: u32 = 1 << HIGH_SPEED_ACK_SHIFT;
    /// Start byte acknowledged mask
    pub const START_ACK_MASK: u32 = 1 << START_ACK_SHIFT;
    /// High speed no restart mask
    pub const HIGH_SPEED_NO_RESTART_MASK: u32 = 1 << HIGH_SPEED_NO_RESTART_SHIFT;
    /// Start byte sent when restart disabled mask
    pub const START_NO_RESTART_MASK: u32 = 1 << START_NO_RESTART_SHIFT;
    /// 10 bit mode read no restart mask
    pub const ADDR10_READ_NO_RESTART_MASK: u32 = 1 << ADDR10_READ_NO_RESTART_SHIFT;
    /// Master operation with disabled master mask
    pub const MASTER_DISABLE_MASK: u32 = 1 << MASTER_DISABLE_SHIFT;
    /// Arbitration lost mask
    pub const ARBITRATION_LOST_MASK: u32 = 1 << ARBITRATION_LOST_SHIFT;
    /// TX abort to flush TX fifo mask
    pub const TX_FLUSH_MASK: u32 = 1 << TX_FLUSH_SHIFT;
    /// Slave arbitration lost mask
    pub const SLAVE_ARBITRATION_LOST_MASK: u32 = 1 << SLAVE_ARBITRATION_LOST_SHIFT;
    /// Slave transmitting to master in read mode mask
    pub const SLAVE_READ_MASK: u32 = 1 << SLAVE_READ_SHIFT;
    /// Transfer abort mask
    pub const USER_ABORT_MASK: u32 = 1 << USER_ABORT_SHIFT;
    /// TX flush count mask
    pub const TX_FLUSH_COUNT_MASK: u32 = 0x1ff << TX_FLUSH_COUNT_SHIFT;

    /// Mask of all non-reserved bits
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

/// Generate slave data NACK register masks and shifts
mod nack_register {
    /// Shift to generate a NACK
    pub const NACK_SHIFT: usize = 0;

    /// Mask to generate a NACK
    pub const NACK_MASK: u32 = 1 << NACK_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = NACK_MASK;
}

/// SDA setup register masks and shifts
mod sda_setup_register {
    /// SDA setup shift
    pub const SDA_SETUP_SHIFT: usize = 0;

    /// SDA setup mask
    pub const SDA_SETUP_MASK: u32 = 0xff << SDA_SETUP_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SDA_SETUP_MASK;
}

/// ACK general call register masks and shifts
mod ack_general_call_register {
    /// Shift to ACK general call
    pub const ACK_GENERAL_CALL_SHIFT: usize = 0;

    /// Mask to ACK general call
    pub const ACK_GENERAL_CALL_MASK: u32 = 1 << ACK_GENERAL_CALL_SHIFT;

    /// Acknowledge general call
    pub const ACK_GENERAL_CALL_ACK: u32 = 1 << ACK_GENERAL_CALL_SHIFT;
    /// Don't acknowledge general call
    pub const ACK_GENERAL_CALL_NACK: u32 = 0 << ACK_GENERAL_CALL_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ACK_GENERAL_CALL_MASK;
}

/// Enable status register masks and shifts
mod enable_status_register {
    /// Shift for if I2C is enabled
    pub const ENABLED_SHIFT: usize = 0;
    /// Slave disabled while busy shift
    pub const SLAVE_DISABLE_BUSY_SHIFT: usize = 1;
    /// Slave receive data lost shift
    pub const SLAVE_RX_DATA_LOST_SHIFT: usize = 2;

    /// Mask for if I2C is enabled
    pub const ENABLED_MASK: u32 = 1 << ENABLED_SHIFT;
    /// Slave disabled while busy mask
    pub const SLAVE_DISABLE_BUSY_MASK: u32 = 1 << SLAVE_DISABLE_BUSY_SHIFT;
    /// Slave receive data lost mask
    pub const SLAVE_RX_DATA_LOST_MASK: u32 = 1 << SLAVE_RX_DATA_LOST_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ENABLED_MASK |
        SLAVE_DISABLE_BUSY_MASK |
        SLAVE_RX_DATA_LOST_MASK;
}

/// Spike supression limit register masks and shifts
mod spike_len_register {
    /// Spike length shift
    pub const SPIKE_LEN_SHIFT: usize = 0;

    /// Spike length mask
    pub const SPIKE_LEN_MASK: u32 = 0xff << SPIKE_LEN_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SPIKE_LEN_MASK;
}

/// I2C memory mapped registers
#[repr(C)]
struct I2CRegisters {
    /// Control register (0x00)
    ctrl: ReadPureWrite<u32>, // 0x0
    /// Target address register (0x04)
    target_addr: ReadPureWrite<u32>, // 0x4
    /// Slave address register (0x08)
    slave_addr: ReadPureWrite<u32>, // 0x8
    _reserved0: u32, // 0xc
    /// Data command register (0x10)
    data_cmd: ReadWrite<u32>, // 0x10
    /// Standard clock high count register (0x14)
    standard_clock_high: ReadPureWrite<u32>, // 0x14
    /// Standard clock low count register (0x18)
    standard_clock_low: ReadPureWrite<u32>, // 0x18
    /// Fast clock high count register (0x1c)
    fast_clock_high: ReadPureWrite<u32>, // 0x1c
    /// Fast clock low count register (0x20)
    fast_clock_low: ReadPureWrite<u32>, // 0x20
    _reserved1: u32, // 0x24
    _reserved2: u32, // 0x28
    /// Interrupt status register (0x2c)
    inter_status: ReadPure<u32>, // 0x2c
    /// Interrupt mask register (0x30)
    inter_mask: ReadPureWrite<u32>, // 0x30
    /// Raw interrupt status register (0x34)
    raw_inter_status: ReadPure<u32>, // 0x34
    /// RX threshold register (0x38)
    rx_threshold: ReadPureWrite<u32>, // 0x38
    /// TX threshold register (0x3c)
    tx_threshold: ReadPureWrite<u32>, // 0x3c
    /// Interrupt clear register (0x40)
    clear_inter: ReadOnly<u32>, // 0x40
    /// RX underflow interrupt clear register (0x44)
    clear_rx_under: ReadOnly<u32>, // 0x44
    /// RX overflow interrupt clear register (0x48)
    clear_rx_over: ReadOnly<u32>, // 0x48
    /// TX overflow interrupt clear register (0x4c)
    clear_tx_over: ReadOnly<u32>, // 0x4c
    /// Read request interrupt clear register (0x50)
    clear_read_request: ReadOnly<u32>, // 0x50 
    /// TX abort interrupt clear register (0x54)
    clear_tx_abort: ReadOnly<u32>, // 0x54 
    /// TX done interrupt clear register (0x58)
    clear_rx_done: ReadOnly<u32>, // 0x58 
    /// Activity clear register (0x5c)
    clear_activity: ReadOnly<u32>, // 0x5c 
    /// Stop clear register (0x60)
    clear_stop: ReadOnly<u32>, // 0x60 
    /// Start clear register (0x64)
    clear_start: ReadOnly<u32>, // 0x64
    /// General call clear register (0x68)
    clear_gen_call: ReadOnly<u32>, // 0x68
    /// Enable register (0x6c)
    enable: ReadPureWrite<u32>, // 0x6c
    /// Status register (0x70)
    status: ReadPure<u32>, // 0x70
    /// TX level register (0x74)
    tx_level: ReadPure<u32>, // 0x74
    /// RX level register (0x78)
    rx_level: ReadPure<u32>, // 0x78
    /// SDA hold register (0x7c)
    sda_hold: ReadPureWrite<u32>, // 0x7c
    /// TX abort source register (0x80)
    tx_abort_src: ReadPure<u32>, // 0x80
    /// NACK register (0x84)
    nack: ReadPureWrite<u32>, // 0x84
    /// DMA control register (0x88)
    dma_ctrl: ReadPureWrite<u32>, // 0x88
    /// DMA TX level register (0x8c)
    dma_tx_level: ReadPureWrite<u32>, // 0x8c
    /// DMA RX level register (0x90)
    dma_rx_level: ReadPureWrite<u32>, // 0x90
    /// SDA setup register (0x94)
    sda_setup: ReadPureWrite<u32>, // 0x94
    /// ACK general call register (0x98)
    ack_general_call: ReadPureWrite<u32>, // 0x98
    /// Enable status register (0x9c)
    enable_status: ReadPure<u32>, // 0x9c
    /// Spike supression limit register (0xa0)
    spike_len: ReadPureWrite<u32>, // 0xa0
    _reserved3: u32, // 0xa4
    /// Clear restart register (0xa8)
    clear_restart: ReadPure<u32> // 0xa8
}

/// I2C object to manage an I2C device
pub struct I2C {
    /// Memory mapped registers
    registers: UniqueMmioPointer<'static, I2CRegisters>,
    /// Memory mapped registers where writing a bit sets the corresponding bit in `registers`
    set_reg: UniqueMmioPointer<'static, I2CRegisters>,
    /// Memory mapped registers where writing a bit clears the corresponding bit in `registers`
    clear_reg: UniqueMmioPointer<'static, I2CRegisters>
}

impl I2C {
    /// Creates a new `I2C` object  
    /// `i2c_base` is the base address of the I2C memory mapped registers
    /// # Safety
    /// `i2c_base` must be a valid address which points to an I2C memory mapped registers and not
    /// being used by anything else
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

    

    /// Waits until all data has been sent from the TX fifo
    /// Returns if the operation was successful or not
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

    /// Writes `buffer` from `index` into the TX fifo until the TX fifo is full or the entirety of
    /// buffer has been written  
    /// This assumes there will be no data read from RX after TX completes  
    /// `buffer` is the buffer to write from  
    /// `index` is the current index into the buffer which is updated as `buffer` is written
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

    /// Clears I2C IRQ
    fn clear_irq(&mut self) {
        _ = field!(self.registers, clear_inter).read();
    }

    /// Fills TX with NULL bytes so that many bytes is read out into RX  
    /// `len` is the number of bytes to write  
    /// `tx_index` is the current index into the hypothetical TX buffer and is updated as bytes are
    /// written  
    /// Either all remaing bytes will be written into the TX buffer or the TX buffer is filled until
    /// it's full
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

    /// Reads from the RX buffer into `buffer`  
    /// `buffer` is the buffer to read into  
    /// `rx_index` is the current index into the buffer which is updated as bytes are read out  
    /// This function terminates when either `buffer` is full or the RX fifo is empty
    fn remove_rx_buffer(&mut self, buffer: &mut [u8], rx_index: &mut usize) {
        while field!(self.registers, status).read() & status_register::RX_NOT_EMPTY_MASK != 0 && *rx_index < buffer.len() {
            let data = ((field!(self.registers, data_cmd).read() & data_cmd_register::DATA_MASK) >> data_cmd_register::DATA_SHIFT) as u8;
            buffer[*rx_index] = data;
            *rx_index += 1;
        }
    }
    
    /// Writes `buffer` from `index` into the TX fifo until the TX fifo is full or the entirety of
    /// buffer has been written  
    /// This assumes there will be data read from RX after TX completes  
    /// `buffer` is the buffer to write from  
    /// `index` is the current index into the buffer which is updated as `buffer` is written
    fn fill_tx_before_rx(&mut self, buffer: &[u8], index: &mut usize) {
        while field!(self.registers, status).read() & status_register::TX_NOT_FULL_MASK != 0 && *index < buffer.len() {
            field!(self.registers, data_cmd).write(
                ((buffer[*index] as u32) << data_cmd_register::DATA_SHIFT) & data_cmd_register::DATA_MASK
            );
            *index += 1;
        }
    }

    /// Checks if a TX abort has happened  
    /// `inter` is the interrupt register mask to check  
    /// Returns success if no abort happened and an error if it has  
    /// It also clears the TX abort register
    fn check_abort(&mut self, inter: u32) -> Result<(), I2CReplyError> {
        if inter & inter_register::TX_ABORT_MASK != 0 {
            field!(self.registers, clear_tx_abort).read();
            Err(I2CReplyError::Abort)
        } else {
            Ok(())
        }
    }

    /// Sends `buffer` over I2C with no reading afterwards  
    /// `buffer` is the buffer to transmit
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

    /// Sends `buffer` over I2C assuming received data will be read afterwards  
    /// `buffer` is the buffer to transmit
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

    /// Reads from the RX fifo into `buffer`
    /// `buffer` is the buffer to write into
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

    /// Sends to `addr` the contents of buffer before reading the received data back into the buffer  
    /// `addr` is the I2C address to send to  
    /// `buffer` initially contains the data to be sent and later the data received  
    /// `tx_len` is the number of bytes to transmit from the buffer  
    /// `rx_len` is the number of bytes to receive into the buffer
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

/// I2C reply errors
pub enum I2CReplyError {
    /// Queue send error
    SendError,
    /// An invalid request was made
    InvalidRequest,
    /// An invalid address was provided
    InvalidAddress,
    /// The I2C request was aborted
    Abort,
    /// The send buffer didn't have the correct size
    InvalidSendBuffer,
    /// The reply buffer didn't have the correct size
    InvalidReplyBuffer
}

/// Converts from an `I2CReplyError` to a `u32`
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

/// I2C Errors
pub enum I2CError {
    /// Error with the request
    ReplyError(I2CReplyError),
    /// Error with queue operations
    QueueError(QueueError)
}

/// Converts from an `I2CReplyError` to an `I2CError`
impl From<I2CReplyError> for I2CError {
    fn from(value: I2CReplyError) -> Self {
        Self::ReplyError(value)
    }
}

/// Converts from a `QueueError` to an `I2CError`
impl From<QueueError> for I2CError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

/// An I2C Request
pub struct Request {
    /// I2C address
    addr: u8,
    /// Send length
    tx_len: usize,
    /// Receive length
    rx_len: usize,
    /// Data buffer
    buffer: [u8; 32],
}

impl Request {
    /// Parses the next request  
    /// Returns the request on success or an `I2CError` on failure
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

/// IO Bank 0 endpoint
const IO_BANK0_QUEUE: u32 = 0;

/// Driver entry point
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
