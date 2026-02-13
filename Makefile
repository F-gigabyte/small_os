DEBUG_PATH = target/thumbv6m-none-eabi/debug
RELEASE_PATH = target/thumbv6m-none-eabi/release
DEBUG_KERNEL = $(DEBUG_PATH)/kernel
RELEASE_KERNEL = $(RELEASE_PATH)/kernel
DEBUG_DRIVERS = $(shell find $(DEBUG_PATH) -maxdepth 1 -type f \! -name "kernel" \! -name "*.d" \! -name "\.*" \! -name "small_os")
RELEASE_DRIVERS = $(shell find $(RELEASE_PATH) -maxdepth 1 -type f \! -name "kernel" \! -name "*.d" \! -name "\.*" \! -name "small_os")

build_debug:
	openocd&
	cargo build
	@echo $(DRIVERS)
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg $(DEBUG_KERNEL) $(DEBUG_DRIVERS)
	gdb -x openocd.gdb $(DEBUG_KERNEL)

build_release:
	openocd&
	cargo build -r
	@echo $(DRIVERS)
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg $(RELEASE_KERNEL) $(RELEASE_DRIVERS)
	gdb -x openocd.gdb $(DEBUG_KERNEL)
