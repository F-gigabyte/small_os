use std::{path::Path, fs};


/// Recursively searches each directory in `path` to find assembly code files (end in .s or .S) and
/// marks them to be compiled along with telling cargo to recompile the codebase if they change
/// It is assumed `path` points to a directory
fn read_dir(path: &Path, builder: &mut cc::Build) {
    for file in fs::read_dir(path).unwrap() {
        let file = file.unwrap();
        let file_type = file.file_type().unwrap();
        if file_type.is_dir() {
            // directory so need to recursively check it
            read_dir(&file.path(), builder);
        } else if file_type.is_file() {
            // is a file so need to check if its assembly (has a .s or .S extension)
            let file_name = file.file_name().into_string().unwrap();
            if file_name.ends_with(".S") || file_name.ends_with(".s") {
                let file_path = file.path();
                println!("cargo::rerun-if-changed={}", file_path.to_str().unwrap());
                builder.file(file_path.to_str().unwrap());
            }
        }
    }
}

fn main() {
    // rerun if linker script changes
    println!("cargo::rerun-if-changed=link.ld");
    // rerun if this build script changes
    println!("cargo::rerun-if-changed=build.rs");
    // use link.ld linker script
    println!("cargo::rustc-link-arg=-Tlink.ld");
    let start_dir = Path::new("src");
    let mut builder = cc::Build::new();
    // set default compiler to clang as its by default a cross compiler while gcc would need to be
    // recompiled for armv6m
    builder.compiler("clang");
    read_dir(start_dir, &mut builder);
    // compiles to a binary called 'small_os'
    builder.compile("small_os");
}
