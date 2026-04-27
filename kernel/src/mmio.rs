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

/// MMIO xor alias offset from register base
/// Writing a bit sets the corresponding bit in the base registers if its unset or else clears it  
/// Register aliases as specified in SDK <https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21>
/// lines 18 to 21 (accessed 19/01/2026)
/// under the license
/// Copyright (c) 2024 Raspberry Pi Ltd.
///
/// SPDX-License-Identifier: BSD-3-Clause
pub const REG_ALIAS_XOR_BITS: usize = 0x1 << 12;
/// MMIO set alias offset from register base
/// Writing a bit sets the corresponding bit in the base registers  
/// Register aliases as specified in SDK <https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21>
/// lines 18 to 21 (accessed 19/01/2026)
/// under the license
/// Copyright (c) 2024 Raspberry Pi Ltd.
///
/// SPDX-License-Identifier: BSD-3-Clause
pub const REG_ALIAS_SET_BITS: usize = 0x2 << 12;
/// MMIO clear alias offset from register base  
/// Writing a bit clears the corresponding bit in the base registers  
/// Register aliases as specified in SDK <https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21>
/// lines 18 to 21 (accessed 19/01/2026)
/// under the license
/// Copyright (c) 2024 Raspberry Pi Ltd.
///
/// SPDX-License-Identifier: BSD-3-Clause
pub const REG_ALIAS_CLR_BITS: usize = 0x3 << 12;
/// The total number of aliases, including the original register values at offset 0
pub const REG_ALIAS_LEN: usize = 4;
