use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite}};

use crate::{inter::CS, mmio::{REG_ALIAS_CLR_BITS, REG_ALIAS_SET_BITS}, mutex::IRQMutex, proc::Proc, program::Region};


struct MPURegisters {
    mpu_type: ReadPure<u32>, // 0xed90
    ctrl: ReadPureWrite<u32>, // 0xed94
    region_num: ReadPureWrite<u32>, // 0xed98
    region_base: ReadPureWrite<u32>, // 0xed9c
    attr_size: ReadPureWrite<u32>, // 0xeda0
}

mod mpu_type_register {
    pub const SEPARATE_SHIFT: usize = 0;
    pub const DREGION_SHIFT: usize = 8;
    pub const IREGION_SHIFT: usize = 16;

    pub const SEPARATE_MASK: u32 = 1 << SEPARATE_SHIFT;
    pub const DREGION_MASK: u32 = 0xff << DREGION_SHIFT;
    pub const IREGION_MASK: u32 = 0xff << IREGION_SHIFT;
}

mod ctrl_register {
    pub const ENABLE_SHIFT: usize = 0;
    pub const HF_NMI_ENABLE_SHIFT: usize = 1;
    pub const DEFAULT_MAP_SHIFT: usize = 2;

    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const HF_NMI_ENABLE_MASK: u32 = 1 << HF_NMI_ENABLE_SHIFT;
    pub const DEFAULT_MAP_MASK: u32 = 1 << DEFAULT_MAP_SHIFT;
}

mod region_num_register {
    pub const REGION_SHIFT: usize = 0;

    pub const REGION_MASK: u32 = 0xf << REGION_SHIFT;
    pub const REGION_MAX: u32 = 8 << REGION_SHIFT;
    pub const REGION_MIN: u32 = 0 << REGION_SHIFT;
}

mod region_base_register {
    pub const REGION_SHIFT: usize = 0;
    pub const VALID_SHIFT: usize = 4;
    pub const ADDR_SHIFT: usize = 8;

    pub const REGION_MASK: u32 = 0xf << REGION_SHIFT;
    pub const VALID_MASK: u32 = 1 << VALID_SHIFT;
    pub const ADDR_MASK: u32 = 0xffffff << ADDR_SHIFT;

    pub const REGION_MAX: u32 = 8 << REGION_SHIFT;
    pub const REGION_MIN: u32 = 0 << REGION_SHIFT;
}

mod attr_size_register {
    pub const ENABLE_SHIFT: usize = 0;
    pub const SIZE_SHIFT: usize = 1;
    pub const SUBREGION_DISABLE_SHIFT: usize = 8;
    pub const ATTR_SHIFT: usize = 16;
    
    pub const ATTR_BUFFERABLE_SHIFT: usize = ATTR_SHIFT + 0;
    pub const ATTR_CACHABLE_SHIFT: usize = ATTR_SHIFT + 1;
    pub const ATTR_SHAREABLE_SHIFT: usize = ATTR_SHIFT + 2;
    pub const ATTR_AP_SHIFT: usize = ATTR_SHIFT + 3;
    pub const ATTR_XN_SHIFT: usize = ATTR_SHIFT + 6;

    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    pub const SIZE_MASK: u32 = 0xf << SIZE_SHIFT;
    pub const SUBREGION_DISABLE_MASK: u32 = 0xff << SUBREGION_DISABLE_SHIFT;
    pub const ATTR_MASK: u32 = 0xffff << ATTR_SHIFT;
    
    pub const ATTR_AP_MASK: u32 = 0x3 << ATTR_AP_SHIFT;
    pub const ATTR_BUFFERABLE_MASK: u32 = 1 << ATTR_BUFFERABLE_SHIFT;
    pub const ATTR_CACHABLE_MASK: u32 = 1 << ATTR_CACHABLE_SHIFT;
    pub const ATTR_SHAREABLE_MASK: u32 = 1 << ATTR_SHAREABLE_SHIFT;
    pub const ATTR_XN_MASK: u32 = 1 << ATTR_XN_SHIFT;

    pub const SIZE_MAX: u32 = 0x1f << SIZE_SHIFT;
    pub const SIZE_MIN: u32 = 0x3 << SIZE_SHIFT;
    
    pub const ATTR_AP_R: u32 = 0x2 << ATTR_AP_SHIFT;
    pub const ATTR_AP_RW: u32 = 0x3 << ATTR_AP_SHIFT;
    
}

pub struct MPU {
    reg: UniqueMmioPointer<'static, MPURegisters>,
    clear_reg: UniqueMmioPointer<'static, MPURegisters>,
    set_reg: UniqueMmioPointer<'static, MPURegisters>
}

#[derive(Debug, Clone, Copy)]
pub enum MPURegion {
    Region0 = 0,
    Region1 = 1,
    Region2 = 2,
    Region3 = 3,
    Region4 = 4,
    Region5 = 5,
    Region6 = 6,
    Region7 = 7
}

impl TryFrom<usize> for MPURegion {
    type Error = usize;
    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Region0),
            1 => Ok(Self::Region1),
            2 => Ok(Self::Region2),
            3 => Ok(Self::Region3),
            4 => Ok(Self::Region4),
            5 => Ok(Self::Region5),
            6 => Ok(Self::Region6),
            7 => Ok(Self::Region7),
            _ => Err(value)
        }
    }
}

impl MPU {
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
                clear_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_CLR_BITS)).unwrap()),
                set_reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base + REG_ALIAS_SET_BITS)).unwrap())
            }
        }
    }

    pub fn init(&mut self) {
        self.unmap_regions();
        field!(self.set_reg, ctrl).write(ctrl_register::ENABLE_MASK | ctrl_register::DEFAULT_MAP_MASK | ctrl_register::HF_NMI_ENABLE_MASK);
    }

    fn set_region_base(&mut self, base: u32, region: MPURegion) {
        let base = base & region_base_register::ADDR_MASK;
        field!(self.reg, region_base).write(base | region_base_register::VALID_MASK | ((region as u32) << region_base_register::REGION_SHIFT));
    }

    pub fn unmap_region(&mut self, region: MPURegion) {
        self.set_region_base(0, region);
        field!(self.clear_reg, attr_size).write(
            attr_size_register::ENABLE_MASK |
            attr_size_register::ATTR_MASK |
            attr_size_register::SIZE_MASK |
            attr_size_register::SUBREGION_DISABLE_MASK
        );
    }

    pub fn unmap_regions(&mut self) {
        self.unmap_region(MPURegion::Region0);
        self.unmap_region(MPURegion::Region1);
        self.unmap_region(MPURegion::Region2);
        self.unmap_region(MPURegion::Region3);
        self.unmap_region(MPURegion::Region4);
        self.unmap_region(MPURegion::Region5);
        self.unmap_region(MPURegion::Region6);
        self.unmap_region(MPURegion::Region7);
    }

    pub fn map_region(&mut self, region: &Region, num: MPURegion) -> Result<(), ()> {
        if !region.enabled() {
            self.unmap_region(num);
            return Ok(());
        }
        let addr = region.get_virt() & !0xff;
        let size = region.len;
        if !size.is_power_of_two() {
            return Err(());
        }
        // check have correct alignment and size
        if !addr.is_multiple_of(size) {
            return Err(());
        }
        let size = size.ilog2() << attr_size_register::SIZE_SHIFT;
        if size < attr_size_register::SIZE_MIN || size > attr_size_register::SIZE_MAX {
            return Err(());
        }
        let mut attr = if region.is_device() {
            attr_size_register::ATTR_SHAREABLE_MASK |
            attr_size_register::ATTR_BUFFERABLE_MASK
        } else {
            attr_size_register::ATTR_CACHABLE_MASK | 
            attr_size_register::ATTR_BUFFERABLE_MASK
        };
        let flags = region.get_attr().unwrap();
        if !flags.exec() {
            attr |= attr_size_register::ATTR_XN_MASK;
        }
        if flags.write() {
            attr |= attr_size_register::ATTR_AP_RW;
        } else {
            attr |= attr_size_register::ATTR_AP_R;
        }
        self.set_region_base(addr, num);
        field!(self.clear_reg, attr_size).write(
            attr_size_register::ENABLE_MASK |
            attr_size_register::ATTR_MASK |
            attr_size_register::SIZE_MASK |
            attr_size_register::SUBREGION_DISABLE_MASK
        );
        field!(self.set_reg, attr_size).write(
            attr_size_register::ENABLE_MASK |
            attr |
            size
        );
        Ok(())
    }
}

unsafe impl Send for MPU {}
unsafe impl Sync for MPU {}

static MPU_BASE: usize = 0xe000ed90;
// SAFETY 
// MPU implemented for both processors so there shouldn't be a data race
pub static MPU: IRQMutex<MPU> = unsafe {
    IRQMutex::new(MPU::new(MPU_BASE))
};

#[unsafe(no_mangle)]
pub unsafe fn switch_regions(proc: *mut Proc) -> *mut Proc {
    let proc = unsafe {
        &mut *proc
    };
    let cs = unsafe {
        CS::new()
    };
    let mut mpu = MPU.lock(&cs);
    let program = unsafe {
        & *proc.program
    };
    for (i, region) in program.regions.iter().enumerate() {
        mpu.map_region(region, MPURegion::try_from(i).unwrap()).unwrap();
    }
    proc
}
