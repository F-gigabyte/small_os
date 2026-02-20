// offset for clearing registers as specified in SDK https://github.com/raspberrypi/pico-sdk/blob/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h#L21
// lines 18 to 21 (accessed 19/01/2026)
pub const REG_ALIAS_XOR_BITS: usize = 0x1 << 12;
pub const REG_ALIAS_SET_BITS: usize = 0x2 << 12;
pub const REG_ALIAS_CLR_BITS: usize = 0x3 << 12;
pub const REG_ALIAS_LEN: usize = 4;
