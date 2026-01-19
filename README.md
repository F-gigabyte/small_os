# SmallOS
An embedded OS developed towards being used in satellites

## Installing depenedencies
It is recommended to use a Linux based system.
1. Install rust as described at [https://rust-lang.org/tools/install/](https://rust-lang.org/tools/install/)
2. Install rust nightly with `rustup toolchain install nightly`
3. Set this projects default channel to nightly with `rustup override set nightly`
4. Add the thumbv6m-none-eabi target with `rustup target add thumbv6m-none-eabi`
4. Install clang as described at [https://clang.llvm.org/get_started.html](https://clang.llvm.org/get_started.html). On Linux or MacOS, this could be done from a package manager

## Dependencies
- [rp2040-boot2](https://docs.rs/rp2040-boot2/0.3.0/rp2040_boot2/) - a stage 2 bootloader for the raspberry pi pico
- [tock-registers](https://docs.rs/tock-registers/0.10.1/tock_registers/) - for safely accessing memory mapped registers

## Build Dependencies
- [cc](https://docs.rs/cc/latest/cc/) - this allows linking in a c compiler to compile c and assembly programs so they can be linked in with the rust code. It is used to assemble the ARM assembly using clang.
