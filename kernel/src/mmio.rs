/// MMIO xor alias offset from register base
/// Writing a bit sets the corresponding bit in the base registers if its unset or else clears it  
/// Register aliases as specified in SDK <https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21>
/// lines 18 to 21 (accessed 19/01/2026)
pub const REG_ALIAS_XOR_BITS: usize = 0x1 << 12;
/// MMIO set alias offset from register base
/// Writing a bit sets the corresponding bit in the base registers  
/// Register aliases as specified in SDK <https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21>
/// lines 18 to 21 (accessed 19/01/2026)
pub const REG_ALIAS_SET_BITS: usize = 0x2 << 12;
/// MMIO clear alias offset from register base  
/// Writing a bit clears the corresponding bit in the base registers  
/// Register aliases as specified in SDK <https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21>
/// lines 18 to 21 (accessed 19/01/2026)
pub const REG_ALIAS_CLR_BITS: usize = 0x3 << 12;
/// The total number of aliases, including the original register values at offset 0
pub const REG_ALIAS_LEN: usize = 4;
