use crate::println;

#[repr(C)]
pub struct TarHeader {
    filename: [u8; 100],
    mode: [u8; 8],
    uid: [u8; 8],
    gid: [u8; 8],
    size: [u8; 12],
    mtime: [u8; 12],
    checksum: [u8; 8],
    type_flag: u8,
    _padding: [u8; 355]
}

impl TarHeader {
    pub fn get_size(&self) -> usize {
        let mut size = 0;
        let mut count = 1;
        for byte in self.size[..11].iter().rev() {
            size += ((byte - b'0') as usize) * count;
            count *= 8;
        }
        size
    }
}

pub unsafe fn parse_headers(mut addr: *const TarHeader) {
    loop {
        let header = unsafe {
            &*addr
        };
        if header.filename[0] == 0 {
            break;
        }
        if let Ok(name) = str::from_utf8(&header.filename) {
            println!("Have file {}", name);
        }
        let size = header.get_size();
        unsafe {
            addr = addr.add(((size + 511) / 512) + 1);
        }
    }
}
