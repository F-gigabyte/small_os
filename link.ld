MEMORY {
    FLASH (RX) : ORIGIN = 0x10000000, LENGTH = 2048K
    RAM (RW) : ORIGIN = 0x20000000, LENGTH = 256K
    SRAM4 (RW) : ORIGIN = 0x20040000, LENGTH = 4K
    SRAM5 (RW) : ORIGIN = 0x20041000, LENGTH = 4K
}

PAGE_SIZE = 256;

ENTRY(_start);

SECTIONS {
    . = ORIGIN(FLASH);
    .bootloader : {
        KEEP(*(.bootloader));
        KEEP(*(.bootloader.*));
    } > FLASH

    .text.vectors : {
        KEEP(*(.text.vectors));
    } > SRAM4 AT > FLASH

    .text : {
        *(.text);
        *(.text.*);
    } > FLASH
    
    .rodata : {
        *(.rodata);
        *(.rodata.*);
    } > FLASH
    
    .data : {
        *(.data);
        *(.data.*);
        *(.bss);
        *(.bss.*);
    } > RAM AT > FLASH

    __data_start = LOADADDR(.data);
    __data_end = LOADADDR(.data) + SIZEOF(.data);

    __ram_start = ORIGIN(RAM);
    __ram_end = ORIGIN(RAM) + LENGTH(RAM);

    __sram4_start = ORIGIN(SRAM4);
    __sram4_end = ORIGIN(SRAM4) + LENGTH(SRAM4);

    __flash_start = ORIGIN(FLASH);
    __flash_end = ORIGIN(FLASH) + LENGTH(FLASH);

    __stack = ORIGIN(RAM) + LENGTH(RAM);

    __vec_start = LOADADDR(.text.vectors);
    __vec_end = LOADADDR(.text.vectors) + SIZEOF(.text.vectors);

    /DISCARD/ : {
        *(.ARM.exidx);
        *(.ARM.exidx.*);
    }
}
