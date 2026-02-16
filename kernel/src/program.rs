use core::{mem, ptr, slice};

use crate::{mutex::Spin, println};

#[repr(C)]
pub struct Region {
    pub phys_addr: u32,
    pub virt_addr: u32,
    pub len: u32
}

#[repr(C)]
pub struct Program {
    pub priority: u32,
    pub sp: u32,
    pub entry: u32,
    pub regions: [Region; 8],
}

pub struct ProgramTable {
    table: &'static [Program]
}

impl ProgramTable {
    pub unsafe fn new(base: usize) -> Self {
        let table_len: u32 = unsafe {
            *ptr::with_exposed_provenance(base)
        };
        println!("Base is 0x{:x}, need alignment of 0x{:x} and have len 0x{:x}", base + 4, mem::align_of::<Program>(), table_len);
        let table = unsafe {
            slice::from_raw_parts(ptr::with_exposed_provenance(base + 4), table_len as usize)
        };
        Self { 
            table 
        }
    }

    #[inline(always)]
    pub fn table(&self) -> &'static [Program] {
        self.table
    }
}

pub static PROGRAM_TABLE: Spin<Option<ProgramTable>> = Spin::new(None);
