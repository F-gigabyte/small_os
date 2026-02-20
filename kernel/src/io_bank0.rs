use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::ReadPureWrite};

use crate::mutex::SpinIRQ;

mod gpio_ctrl_register {
    pub const FUNCSEL_SHIFT: usize = 0;
    pub const OUTOVER_SHIFT: usize = 8;
    pub const OEOVER_SHIFT: usize = 12;
    pub const INOVER_SHIFT: usize = 16;
    pub const IRQOVER_SHIFT: usize = 28;

    pub const FUNCSEL_MASK: u32 = 0x1f << FUNCSEL_SHIFT;
    pub const OUTOVER_MASK: u32 = 0x2 << OUTOVER_SHIFT;
    pub const OEOVER_MASK: u32 = 0x2 << OEOVER_SHIFT;
    pub const INOVER_MASK: u32 = 0x2 << INOVER_SHIFT;
    pub const IRQOVER_MASK: u32 = 0x2 << IRQOVER_SHIFT;

    pub const FUNCSEL_MIN: u32 = 1;
    pub const FUNCSEL_MAX: u32 = 9;
    pub const FUNCSEL_NULL: u32 = 31;

    pub const OUTOVER_NORMAL: u32 = 0x0 << OUTOVER_SHIFT;
    pub const OUTOVER_INVERT: u32 = 0x1 << OUTOVER_SHIFT;
    pub const OUTOVER_LOW: u32 = 0x2 << OUTOVER_SHIFT;
    pub const OUTOVER_HIGH: u32 = 0x3 << OUTOVER_SHIFT;
    
    pub const OEOVER_NORMAL: u32 = 0x0 << OEOVER_SHIFT;
    pub const OEOVER_INVERT: u32 = 0x1 << OEOVER_SHIFT;
    pub const OEOVER_DISABLE: u32 = 0x2 << OEOVER_SHIFT;
    pub const OEOVER_ENABLE: u32 = 0x3 << OEOVER_SHIFT;
    
    pub const INOVER_NORMAL: u32 = 0x0 << INOVER_SHIFT;
    pub const INOVER_INVERT: u32 = 0x1 << INOVER_SHIFT;
    pub const INOVER_LOW: u32 = 0x2 << INOVER_SHIFT;
    pub const INOVER_HIGH: u32 = 0x3 << INOVER_SHIFT;
    
    pub const IRQOVER_NORMAL: u32 = 0x0 << IRQOVER_SHIFT;
    pub const IRQOVER_INVERT: u32 = 0x1 << IRQOVER_SHIFT;
    pub const IRQOVER_LOW: u32 = 0x2 << IRQOVER_SHIFT;
    pub const IRQOVER_HIGH: u32 = 0x3 << IRQOVER_SHIFT;
}

#[repr(C)]
struct IOBank0Registers {
    // GP0
    _reserved0: u32,
    gp0_ctrl: ReadPureWrite<u32>,
    // GP1
    _reserved2: u32,
    gp1_ctrl: ReadPureWrite<u32>,
    // GP2
    _reserved4: u32,
    gp2_ctrl: ReadPureWrite<u32>,
    // GP3
    _reserved6: u32,
    gp3_ctrl: ReadPureWrite<u32>,
    // GP4
    _reserved8: u32,
    gp4_ctrl: ReadPureWrite<u32>,
    // GP5
    _reserved9: u32,
    gp5_ctrl: ReadPureWrite<u32>,
    // GP6
    _reserved10: u32,
    gp6_ctrl: ReadPureWrite<u32>,
    // GP7
    _reserved11: u32,
    gp7_ctrl: ReadPureWrite<u32>,
}

pub struct IOBank0 {
    registers: UniqueMmioPointer<'static, IOBank0Registers>
}

impl IOBank0 {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                registers: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap())
            }
        }
    }

    pub fn set_gpio_uart1(&mut self) {
        let val = ((2 << gpio_ctrl_register::FUNCSEL_SHIFT) & gpio_ctrl_register::FUNCSEL_MASK) |
            gpio_ctrl_register::OUTOVER_NORMAL |
            gpio_ctrl_register::OEOVER_NORMAL |
            gpio_ctrl_register::INOVER_NORMAL |
            gpio_ctrl_register::IRQOVER_NORMAL;
        field!(self.registers, gp4_ctrl).write(val);
        field!(self.registers, gp5_ctrl).write(val);
        field!(self.registers, gp6_ctrl).write(val);
        field!(self.registers, gp7_ctrl).write(val);
    }
    
    pub fn set_gpio_uart0(&mut self) {
        let val = ((2 << gpio_ctrl_register::FUNCSEL_SHIFT) & gpio_ctrl_register::FUNCSEL_MASK) |
            gpio_ctrl_register::OUTOVER_NORMAL |
            gpio_ctrl_register::OEOVER_NORMAL |
            gpio_ctrl_register::INOVER_NORMAL |
            gpio_ctrl_register::IRQOVER_NORMAL;
        field!(self.registers, gp0_ctrl).write(val);
        field!(self.registers, gp1_ctrl).write(val);
        field!(self.registers, gp2_ctrl).write(val);
        field!(self.registers, gp3_ctrl).write(val);
    }
}

unsafe impl Send for IOBank0 {}
unsafe impl Sync for IOBank0 {}

static IOBANK0_BASE: usize = 0x40014000;

pub static IOBANK0: SpinIRQ<IOBank0> = unsafe {
    SpinIRQ::new(IOBank0::new(IOBANK0_BASE))
};
