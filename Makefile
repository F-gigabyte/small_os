DEBUG_DIR = target/thumbv6m-none-eabi/debug
RELEASE_DIR = target/thumbv6m-none-eabi/release

EXE_NAME = small_os

CONFIG = small_os.toml
TEST_CONFIG = small_os_test.toml

build_debug:
	openocd&
	cargo build
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg $(CONFIG) -o $(DEBUG_DIR)/$(EXE_NAME)
	gdb -x openocd.gdb $(DEBUG_DIR)/$(EXE_NAME)

build_release:
	openocd&
	cargo build -r
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg -r $(CONFIG) -o $(RELEASE_DIR)/$(EXE_NAME)
	gdb -x openocd.gdb $(RELEASE_DIR)/$(EXE_NAME)

build_test:
	openocd&
	cargo test --no-run
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg -t $(TEST_CONFIG) -o $(DEBUG_DIR)/$(EXE_NAME)
	gdb -x openocd.gdb $(DEBUG_DIR)/$(EXE_NAME)

build_test_release:
	openocd&
	cargo test -r --no-run
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg -r -t $(TEST_CONFIG) -o $(RELEASE_DIR)/$(EXE_NAME)
	gdb -x openocd.gdb $(RELEASE_DIR)/$(EXE_NAME)

deploy_debug:
	cargo build
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg $(CONFIG) -o $(DEBUG_DIR)/$(EXE_NAME)
	elf2uf2-rs -d $(DEBUG_DIR)/$(EXE_NAME) $(DEBUG_DIR)/$(EXE_NAME).uf2

deploy_release:
	cargo build -r
	LD=arm-none-eabi-ld OBJCOPY=arm-none-eabi-objcopy pkg $(CONFIG) -r -o $(RELEASE_DIR)/$(EXE_NAME)
	elf2uf2-rs -d $(RELEASE_DIR)/$(EXE_NAME) $(RELEASE_DIR)/$(EXE_NAME).uf2

# Based on https://os.phil-opp.com/freestanding-rust-binary/ accessed 30/04/2026
init_setup:
	rustup override set nightly
	rustup target add thumbv6m-none-eabi
