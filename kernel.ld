MEMORY {
    FLASH (RX) : ORIGIN = 0x10000000, LENGTH = 2048K
    RAM (RW) : ORIGIN = 0x20000000, LENGTH = 264K
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
    } > FLASH

    .text : {
        *(.text);
        *(.text.*);
    } > FLASH
    
    .rodata : {
        *(.rodata);
        *(.rodata.*);
    } > FLASH
    
    .data ORIGIN(RAM) + 0x2000 : {
        *(.data);
        *(.data.*);
        *(.bss);
        *(.bss.*);
    } > RAM AT > FLASH

    __proc_table = ALIGN(LOADADDR(.data) + SIZEOF(.data), 4);

    __data_start = LOADADDR(.data);
    __data_end = LOADADDR(.data) + SIZEOF(.data);

    __data_load = ADDR(.data);

    __ram_start = ORIGIN(RAM);
    __ram_end = ORIGIN(RAM) + LENGTH(RAM);

    __flash_start = ORIGIN(FLASH);
    __flash_end = ORIGIN(FLASH) + LENGTH(FLASH);

    __stack = ORIGIN(RAM) + 0x2000;

    /DISCARD/ : {
        *(.ARM.exidx);
        *(.ARM.exidx.*);
    }
}
