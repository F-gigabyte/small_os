use core::ptr::{self, NonNull};

use safe_mmio::{UniqueMmioPointer, field, fields::{ReadPure, ReadPureWrite}};

use crate::{inter::CS, mutex::IRQMutex, proc::Proc, program::Region, println};

/// MPU memory mapped registers
struct MPURegisters {
    /// MPU type register (0xed90)
    mpu_type: ReadPure<u32>, // 0xed90
    /// MPU control register (0xed94)
    ctrl: ReadPureWrite<u32>, // 0xed94
    /// MPU region number register (0xed98)
    region_num: ReadPureWrite<u32>, // 0xed98
    /// MPU region base address register (0xed9c)
    region_base: ReadPureWrite<u32>, // 0xed9c
    /// MPU attribute size register (0xeda0)
    attr_size: ReadPureWrite<u32>, // 0xeda0
}

/// MPU type register masks and shifts
mod mpu_type_register {
    /// Shift for determining the MPU has different maps for instructions and data
    pub const SEPARATE_SHIFT: usize = 0;
    /// Shift for determining the number of regions supported by the MPU
    pub const DREGION_SHIFT: usize = 8;
    /// Shift for determining the number of instruction regions supported by the MPU
    pub const IREGION_SHIFT: usize = 16;

    /// Mask for determining the MPU has different maps for instructions and data
    pub const SEPARATE_MASK: u32 = 1 << SEPARATE_SHIFT;
    /// Mask for determining the number of regions supported by the MPU
    pub const DREGION_MASK: u32 = 0xff << DREGION_SHIFT;
    /// Mask for determining the number of instruction regions supported by the MPU
    pub const IREGION_MASK: u32 = 0xff << IREGION_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = SEPARATE_MASK | 
        DREGION_MASK |
        IREGION_MASK;
}

/// MPU control register masks and shifts
mod ctrl_register {
    /// Shift for enabling the MPU
    pub const ENABLE_SHIFT: usize = 0;
    /// Shift for enabling the MPU for hard faults and non-maskable interrupts
    pub const HF_NMI_ENABLE_SHIFT: usize = 1;
    /// Shift for allowing the default memory map in priveleged mode
    pub const DEFAULT_MAP_SHIFT: usize = 2;

    /// Mask for enabling the MPU
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Mask for enabling the MPU for hard faults and non-maskable interrupts
    pub const HF_NMI_ENABLE_MASK: u32 = 1 << HF_NMI_ENABLE_SHIFT;
    /// Mask for allowing the default memory map in priveleged mode
    pub const DEFAULT_MAP_MASK: u32 = 1 << DEFAULT_MAP_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ENABLE_MASK |
        HF_NMI_ENABLE_MASK |
        DEFAULT_MAP_MASK;
}

/// MPU region number register mask and shifts
mod region_num_register {
    /// Shift for the region number
    pub const REGION_SHIFT: usize = 0;

    /// Mask for the region number
    pub const REGION_MASK: u32 = 0xf << REGION_SHIFT;

    /// Smallest region number
    pub const REGION_MIN: u32 = 0 << REGION_SHIFT;
    /// Largest region number
    pub const REGION_MAX: u32 = 8 << REGION_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = REGION_MASK;
}

/// MPU region base address register mask and shifts
mod region_base_register {
    /// Shift for the region number
    pub const REGION_SHIFT: usize = 0;
    /// Shift for updating the region base address on write
    pub const UPDATE_SHIFT: usize = 4;
    /// Shift for the region's address
    pub const ADDR_SHIFT: usize = 8;

    /// Mask for the region number
    pub const REGION_MASK: u32 = 0xf << REGION_SHIFT;
    /// Mask for updating the region base address on write
    pub const UPDATE_MASK: u32 = 1 << UPDATE_SHIFT;
    /// Mask for the region's address
    pub const ADDR_MASK: u32 = 0xffffff << ADDR_SHIFT;

    /// Smallest region number
    pub const REGION_MIN: u32 = 0 << REGION_SHIFT;
    /// Largest region number
    pub const REGION_MAX: u32 = 8 << REGION_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = REGION_MASK |
        UPDATE_MASK |
        ADDR_MASK;
}

/// MPU attribute size register mask and shifts
mod attr_size_register {
    /// Shift for enabling the region
    pub const ENABLE_SHIFT: usize = 0;
    /// Shift for the region size  
    /// The actual size encoded is 2 ^ (SIZE + 1) where SIZE is what's stored in these register bits  
    /// The minimum value is 7 (256 byte region)
    pub const SIZE_SHIFT: usize = 1;
    /// Shift for subregion disables  
    /// This splits regions above 256 bytes into 8 subregions and disables each subregion where its
    /// corresponding bit is set
    pub const SUBREGION_DISABLE_SHIFT: usize = 8;
    /// Shift for region attributes
    pub const ATTR_SHIFT: usize = 16;
    
    /// Shift for region is bufferable
    pub const ATTR_BUFFERABLE_SHIFT: usize = ATTR_SHIFT + 0;
    /// Shift for region is cachable
    pub const ATTR_CACHABLE_SHIFT: usize = ATTR_SHIFT + 1;
    /// Shift for region is sharable between cores
    pub const ATTR_SHAREABLE_SHIFT: usize = ATTR_SHIFT + 2;
    pub const ATTR_TEX_SHIFT: usize = ATTR_SHIFT + 3;
    /// Shift for region access permissions
    pub const ATTR_AP_SHIFT: usize = ATTR_SHIFT + 8;
    /// Shift for region execute never
    pub const ATTR_XN_SHIFT: usize = ATTR_SHIFT + 12;

    /// Mask for enabling the region
    pub const ENABLE_MASK: u32 = 1 << ENABLE_SHIFT;
    /// Mask for the region size  
    /// The actual size encoded is 2 ^ (SIZE + 1) where SIZE is what's stored in these register bits  
    /// The minimum value is 7 (256 byte region)
    pub const SIZE_MASK: u32 = 0x1f << SIZE_SHIFT;
    /// Mask for subregion disables  
    /// This splits regions above 256 bytes into 8 subregions and disables each subregion where its
    /// corresponding bit is set
    pub const SUBREGION_DISABLE_MASK: u32 = 0xff << SUBREGION_DISABLE_SHIFT;
    /// Mask for region attributes
    pub const ATTR_MASK: u32 = 0x173f << ATTR_SHIFT;
    
    /// Mask for region access permissions
    pub const ATTR_AP_MASK: u32 = 0x3 << ATTR_AP_SHIFT;
    /// Mask for region is bufferable
    pub const ATTR_BUFFERABLE_MASK: u32 = 1 << ATTR_BUFFERABLE_SHIFT;
    /// Mask for region is cachable
    pub const ATTR_CACHABLE_MASK: u32 = 1 << ATTR_CACHABLE_SHIFT;
    /// Mask for region is sharable between cores
    pub const ATTR_SHAREABLE_MASK: u32 = 1 << ATTR_SHAREABLE_SHIFT;
    pub const ATTR_TEX_MASK: u32 = 1 << ATTR_TEX_SHIFT;
    /// Mask for region execute never
    pub const ATTR_XN_MASK: u32 = 1 << ATTR_XN_SHIFT;

    /// Region maximum size
    pub const SIZE_MAX: u32 = 0x1f << SIZE_SHIFT;
    /// Region minimum size
    pub const SIZE_MIN: u32 = 0x7 << SIZE_SHIFT;
    
    /// Region privileged read-write but unprivileged read-only
    pub const ATTR_AP_R: u32 = 0x2 << ATTR_AP_SHIFT;
    /// Region privileged read-write and unprivileged read-write
    pub const ATTR_AP_RW: u32 = 0x3 << ATTR_AP_SHIFT;

    /// Mask of all non-reserved bits
    pub const VALID_MASK: u32 = ENABLE_MASK |
        SIZE_MASK |
        SUBREGION_DISABLE_MASK |
        ATTR_MASK;
}

/// MPU object for managing the processor's MPU
pub struct MPU {
    /// Memory mapped registers
    reg: UniqueMmioPointer<'static, MPURegisters>,
}

/// MPU Regions
#[derive(Debug, Clone, Copy)]
pub enum MPURegion {
    /// MPU region 0
    Region0 = 0,
    /// MPU region 1
    Region1 = 1,
    /// MPU region 2
    Region2 = 2,
    /// MPU region 3
    Region3 = 3,
    /// MPU region 4
    Region4 = 4,
    /// MPU region 5
    Region5 = 5,
    /// MPU region 6
    Region6 = 6,
    /// MPU region 7
    Region7 = 7
}

/// Converts from `usize` to `MPURegion`
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
    /// Creates a new `MPU` object
    /// `base` is the base address of the MPU memory mapped registers
    /// # Safety
    /// `base` must be a valid address which points to the MPU memory mapped registers and not being
    /// used by anything else
    const unsafe fn new(base: usize) -> Self {
        unsafe {
            Self {
                reg: UniqueMmioPointer::new(NonNull::new(ptr::with_exposed_provenance_mut(base)).unwrap()),
            }
        }
    }

    /// Enables the MPU
    pub fn enable(&mut self) {
        field!(self.reg, ctrl).modify(|ctrl| 
            ctrl | 
            ctrl_register::ENABLE_MASK 
        );
    }

    /// Disables the MPU
    pub fn disable(&mut self) {
        field!(self.reg, ctrl).modify(|ctrl| 
            ctrl & 
            !ctrl_register::ENABLE_MASK 
        );
    }

    /// Initialises the MPU
    pub fn init(&mut self) {
        self.unmap_regions();
        field!(self.reg, ctrl).modify(|ctrl| 
            (ctrl & !ctrl_register::VALID_MASK) | 
            ctrl_register::DEFAULT_MAP_MASK | 
            ctrl_register::HF_NMI_ENABLE_MASK
        );
    }

    /// Sets the MPU's region base address register and sets the MPU region number register to the
    /// corresponding region  
    /// `base` is the address to put into the region base address register  
    /// `region` is the MPU region to update
    fn set_region_base(&mut self, base: u32, region: MPURegion) {
        let base = base & region_base_register::ADDR_MASK;
        field!(self.reg, region_base).modify(|region_base| 
            (region_base & !region_base_register::VALID_MASK) | 
            base | 
            region_base_register::UPDATE_MASK | 
            ((region as u32) << region_base_register::REGION_SHIFT)
        );
    }

    /// Unmaps a region and sets the MPU region number to the corresponding region  
    /// `region` is the MPU region to unmap
    pub fn unmap_region(&mut self, region: MPURegion) {
        self.set_region_base(0, region);
        field!(self.reg, attr_size).modify(|attr_size| attr_size & !attr_size_register::VALID_MASK);
    }

    /// Unmaps all MPU regions and sets the MPU region number to the last region
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

    /// Performs the region mapping  
    /// `region` is the region to be mapped  
    /// `num` is the MPU region to map to  
    /// Indicates whether the operation succeeded or not
    fn map_region_impl(&mut self, region: &Region, num: MPURegion) -> Result<(), ()> {
        if !region.enabled() {
            self.unmap_region(num);
            return Ok(());
        }
        let addr = region.get_runtime_addr().unwrap() & !0xff;
        let size = region.get_len() as u32;
        if size < attr_size_register::SIZE_MIN || size > attr_size_register::SIZE_MAX {
            return Err(());
        }
        let actual_size = 1 << (size + 1);
        // check have correct alignment and size
        if !addr.is_multiple_of(actual_size) {
            return Err(());
        }
        let size = (size as u32) << attr_size_register::SIZE_SHIFT;
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
        field!(self.reg, attr_size).modify(|attr_size| 
            (attr_size & !attr_size_register::VALID_MASK) |
            attr_size_register::ENABLE_MASK |
            attr |
            size
        );
        Ok(())

    }

    /// Maps an MPU region
    /// `region` is the region to be mapped  
    /// `num` is the MPU region to map to  
    /// Indicates whether the operation succeeded or not
    pub fn map_region(&mut self, region: &Region, num: MPURegion) -> Result<(), ()> {
        // disable the MPU to avoid issues of partially updated registers
        self.disable();
        self.map_region_impl(region, num)?;
        self.enable();
        Ok(())
    }

    /// Maps 8 MPU regions
    /// `regions` is an array of regions to be mapped  
    /// region at index 0 is mapped to MPU region 0 etc.  
    /// Indicates whether the operation succeeded or not
    pub fn map_regions(&mut self, regions: &[Region; 8]) -> Result<(), ()> {
        // disable the MPU to avoid issues of partially updated registers
        self.disable();
        for (i, region) in regions.iter().enumerate() {
            self.map_region_impl(&region, MPURegion::try_from(i).unwrap())?;
        }
        Ok(())
    }
}

unsafe impl Send for MPU {}
unsafe impl Sync for MPU {}

/// Base address for the MPU memory mapped registers
static MPU_BASE: usize = 0xe000ed90;

// Safety 
// MPU implemented for both processors so there shouldn't be a data race
/// MPU object
pub static MPU: IRQMutex<MPU> = unsafe {
    IRQMutex::new(MPU::new(MPU_BASE))
};

/// Switches the MPU regions for `proc`  
/// `proc` is the process to switch the regions for  
/// # Safety
/// `proc` must be a valid process pointer  
/// Interrupts must not be enabled at the time this function is called
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
    mpu.map_regions(&program.regions).unwrap();
    proc
}

#[cfg(test)]
mod test {
    use crate::{print, println};
    use core::assert_matches;
    use super::*;

    #[test_case]
    fn test_mpu_region() {
        println!("Testing MPU region");
        print!("Testing 0 to MPURegion ");
        assert_matches!(MPURegion::try_from(0), Ok(MPURegion::Region0));
        println!("[ok]");
        print!("Testing 1 to MPURegion ");
        assert_matches!(MPURegion::try_from(1), Ok(MPURegion::Region1));
        println!("[ok]");
        print!("Testing 2 to MPURegion ");
        assert_matches!(MPURegion::try_from(2), Ok(MPURegion::Region2));
        println!("[ok]");
        print!("Testing 3 to MPURegion ");
        assert_matches!(MPURegion::try_from(3), Ok(MPURegion::Region3));
        println!("[ok]");
        print!("Testing 4 to MPURegion ");
        assert_matches!(MPURegion::try_from(4), Ok(MPURegion::Region4));
        println!("[ok]");
        print!("Testing 5 to MPURegion ");
        assert_matches!(MPURegion::try_from(5), Ok(MPURegion::Region5));
        println!("[ok]");
        print!("Testing 6 to MPURegion ");
        assert_matches!(MPURegion::try_from(6), Ok(MPURegion::Region6));
        println!("[ok]");
        print!("Testing 7 to MPURegion ");
        assert_matches!(MPURegion::try_from(7), Ok(MPURegion::Region7));
        println!("[ok]");
        print!("Testing 8 to MPURegion ");
        assert_matches!(MPURegion::try_from(8), Err(8));
        println!("[ok]");
    }

    #[test_case]
    fn test_setup_correct() {
        println!("Testing MPU setup");
        let cs = unsafe {
            CS::new()
        };
        let mut mpu = MPU.lock(&cs);
        print!("Testing MPU ctrl ");
        let ctrl = field!(mpu.reg, ctrl).read();
        assert_eq!(ctrl, 0x7);
        println!("[ok]");
    }

    #[test_case]
    fn test_valid() {
        println!("Testing MPU register mask values");
        print!("Testing mpu type register ");
        assert_eq!(mpu_type_register::VALID_MASK, 0xffff01);
        println!("[ok]");
        print!("Testing ctrl register ");
        assert_eq!(ctrl_register::VALID_MASK, 0x7);
        println!("[ok]");
        print!("Testing region num register ");
        assert_eq!(region_num_register::VALID_MASK, 0xf);
        println!("[ok]");
        print!("Testing region base register ");
        assert_eq!(region_base_register::VALID_MASK, 0xffffff1f);
        println!("[ok]");
        print!("Testing attr size register ");
        assert_eq!(attr_size_register::VALID_MASK, 0x173fff3f);
        println!("[ok]");
    }

    #[test_case]
    fn test_map() {
        println!("Testing MPU map region");
        let cs = unsafe {
            CS::new()
        };
        let mut mpu = MPU.lock(&cs);
        let region = Region { 
            phys_addr: 0x0, 
            virt_addr: 0x2000, 
            len: 0x70023, 
            actual_len: 256, 
            crc: 0, 
            codes: 0 
        };
        mpu.map_region(&region, MPURegion::Region0).unwrap();
        field!(mpu.reg, region_num).modify(|region_num| region_num & !region_num_register::VALID_MASK);
        print!("Testing region base register ");
        let region_base = field!(mpu.reg, region_base).read();
        assert_eq!(region_base & region_base_register::VALID_MASK, 0x2000);
        println!("[ok]");
        print!("Testing attr size register ");
        let attr_size = field!(mpu.reg, attr_size).read();
        assert_eq!(attr_size & attr_size_register::VALID_MASK, 0x1303000f);
        println!("[ok]");
        mpu.unmap_regions();
    }

    #[test_case]
    fn test_map_read_only() {
        println!("Testing MPU map region (read only)");
        let cs = unsafe {
            CS::new()
        };
        let mut mpu = MPU.lock(&cs);
        let region = Region { 
            phys_addr: 0x0, 
            virt_addr: 0x2000, 
            len: 0xb0003, 
            actual_len: 0x1000, 
            crc: 0, 
            codes: 0 
        };
        mpu.map_region(&region, MPURegion::Region0).unwrap();
        field!(mpu.reg, region_num).modify(|region_num| region_num & !region_num_register::VALID_MASK);
        print!("Testing region base register ");
        let region_base = field!(mpu.reg, region_base).read();
        assert_eq!(region_base & region_base_register::VALID_MASK, 0x2000);
        println!("[ok]");
        print!("Testing attr size register ");
        let attr_size = field!(mpu.reg, attr_size).read();
        assert_eq!(attr_size & attr_size_register::VALID_MASK, 0x12030017);
        println!("[ok]");
        mpu.unmap_regions();
    }

    #[test_case]
    fn test_unmaps() {
        println!("Testing MPU unmap regions");
        let cs = unsafe {
            CS::new()
        };
        let mut mpu = MPU.lock(&cs);
        let region = Region { 
            phys_addr: 0x0, 
            virt_addr: 0x2000, 
            len: 0x70023, 
            actual_len: 256, 
            crc: 0, 
            codes: 0 
        };
        let region2 = Region { 
            phys_addr: 0x0, 
            virt_addr: 0x4000, 
            len: 0xb0023, 
            actual_len: 0x1000, 
            crc: 0, 
            codes: 0 
        };
        mpu.map_region(&region, MPURegion::Region0).unwrap();
        mpu.map_region(&region2, MPURegion::Region7).unwrap();
        mpu.unmap_regions();
        for i in 0..8 {
            field!(mpu.reg, region_num).modify(|region_num| (region_num & !region_num_register::VALID_MASK) | (i << region_num_register::REGION_SHIFT));
            print!("Testing attr size register ({}) ", i);
            let attr_size = field!(mpu.reg, attr_size).read();
            assert!(attr_size & attr_size_register::ENABLE_MASK == 0);
            println!("[ok]");
        }
    }

    #[test_case]
    fn test_unmap() {
        println!("Testing MPU unmap region");
        let cs = unsafe {
            CS::new()
        };
        let mut mpu = MPU.lock(&cs);
        let region = Region { 
            phys_addr: 0x0, 
            virt_addr: 0x2000, 
            len: 0x70023, 
            actual_len: 256, 
            crc: 0, 
            codes: 0 
        };
        mpu.map_region(&region, MPURegion::Region0).unwrap();
        mpu.unmap_region(MPURegion::Region0);
        field!(mpu.reg, region_num).modify(|region_num| region_num & !region_num_register::VALID_MASK);
        print!("Testing attr size register ");
        let attr_size = field!(mpu.reg, attr_size).read();
        assert!(attr_size & attr_size_register::ENABLE_MASK == 0);
        println!("[ok]");
    }

}
