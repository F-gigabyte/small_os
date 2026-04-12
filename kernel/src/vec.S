/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS kernel.
 *
 * The SmallOS kernel is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU Lesser General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS kernel is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with the SmallOS kernel. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

.section .text.vectors

// interrupt vector table
_vec_table:
    .word __kernel_stack // main sp (0)
    .word _start // on reset jump to start (reset) (1)
    .word _nmi // nmi interrupt (2)
    .word _hard_fault // Hard fault (3)
    .word 0 // reserved (4)
    .word 0 // reserved (5)
    .word 0 // reserved (6)
    .word 0 // reserved (7)
    .word 0 // reserved (8)
    .word 0 // reserved (9)
    .word 0 // reserved (10)
    .word _sys_call // sys call (11)
    .word 0 // reserved (12)
    .word 0 // reserved (13)
    .word _sys_pend // svc pend (14)
    .word _proc_switch // sys tick (15)
// interrupts
    .word _irq // IRQ 0 -> timer irq 0 (16)
    .word _irq // IRQ 1 -> timer irq 1 (17)
    .word _irq // IRQ 2 -> timer irq 2 (18)
    .word _irq // IRQ 3 -> timer irq 3 (19)
    .word _irq // IRQ 4 -> pwm irq wrap (20)
    .word _irq // IRQ 5 -> usb ctrl irq (21)
    .word _irq // IRQ 6 -> xip irq (22)
    .word _irq // IRQ 7 -> pio0 irq 0 (23)
    .word _irq // IRQ 8 -> pio0 irq 1 (24)
    .word _irq // IRQ 9 -> pio1 irq 0 (25)
    .word _irq // IRQ 10 -> pio1 irq 1 (26)
    .word _irq // IRQ 11 -> dma irq 0 (27)
    .word _irq // IRQ 12 -> dma irq 1 (28)
    .word _irq // IRQ 13 -> io irq bank 0 (29)
    .word _irq // IRQ 14 -> io irq qspi (30)
    .word _irq // IRQ 15 -> sio irq proc0 (31)
    .word _irq // IRQ 16 -> sio irq proc1 (32)
    .word _irq // IRQ 17 -> clocks irq (33)
    .word _irq // IRQ 18 -> spi0 irq (34)
    .word _irq // IRQ 19 -> spi1 irq (35)
    .word _irq // IRQ 20 -> uart0 irq (36)
    .word _irq // IRQ 21 -> uart1 irq (37)
    .word _irq // IRQ 22 -> adc irq fifo (38)
    .word _irq // IRQ 23 -> i2c0 irq (39)
    .word _irq // IRQ 24 -> i2c1 irq (40)
    .word _irq // IRQ 25 -> rtc irq (41)
    .word _irq // set to 0 (42)
    .word _irq // set to 0 (43)
    .word _irq // set to 0 (44)
    .word _irq // set to 0 (45)
    .word _irq // set to 0 (46)
    .word _irq // set to 0 (47)
