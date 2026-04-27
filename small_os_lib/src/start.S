/* 
 * Copyright 2026 Fraser Griffin
 *
 * This file is part of the SmallOS Library.
 *
 * The SmallOS Library is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * The SmallOS Library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with the SmallOS Library. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

.globl _start
.globl _stack_args
.type _start, "function"

// Program entry point
_start:
    bl main
    b _start

.section ".rodata"
// Address of stack arguments
_stack_args:
    .word 0
