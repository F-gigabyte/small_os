DEBUG_DIR = target/thumbv6m-none-eabi/debug
RELEASE_DIR = target/thumbv6m-none-eabi/release

EXE_NAME = small_os

build_debug:
	openocd&
	cargo build
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg small_os.toml -o $(DEBUG_DIR)/$(EXE_NAME)
	gdb -x openocd.gdb $(DEBUG_DIR)/$(EXE_NAME)

build_release:
	openocd&
	cargo build -r
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg -r small_os.toml -o $(RELEASE_DIR)/$(EXE_NAME)
	gdb -x openocd.gdb $(RELEASE_DIR)/$(EXE_NAME)
