fn main() {
    // rerun if linker script changes
    println!("cargo::rerun-if-changed=driver.ld");
    // rerun if this build script changes
    println!("cargo::rerun-if-changed=test_driver/build.rs");
    // use link.ld linker script
    println!("cargo::rustc-link-arg=-Tdriver2.ld");
    // build relocatable code
    println!("cargo::rustc-link-arg=-r");
}
