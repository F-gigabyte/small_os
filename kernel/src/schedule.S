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

.globl schedule
.type schedule, "function"
.globl switch_regions

// starts running next process
// r0 -> pointer to current Proc struct
// This must be called within handler mode
schedule:
#ifdef RADIATION
    bl do_radiation
#endif
#ifdef ERROR_CODES
    bl correct_errors
#endif
    bl switch_regions
#ifdef BENCHMARK
    bl kernel_exit
#endif
    // start sys tick timer
    bl start_sys_tick
    // set r1 to process stack pointer
    ldr r1, [r0, #0x20]
    // r4
    ldr r4, [r0, #0x0]
    // r5
    ldr r5, [r0, #0x4]
    // r6
    ldr r6, [r0, #0x8]
    // r7
    ldr r7, [r0, #0xc]
    // r8
    ldr r3, [r0, #0x10]
    mov r8, r3
    // r9
    ldr r3, [r0, #0x14]
    mov r9, r3
    // r10
    ldr r3, [r0, #0x18]
    mov r10, r3
    // r11
    ldr r3, [r0, #0x1c]
    mov r11, r3
    // make thread mode unprivileged
    mrs r0, control
    movs r2, #1
    orrs r0, r2
    msr control, r0
    isb
    // set process stack pointer to r1
    msr psp, r1
    ldr r0, =0xfffffffd
    bx r0
