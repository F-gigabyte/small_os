MEMORY {
    FLASH (RX) : ORIGIN = 0x10000000, LENGTH = 2048K
    RAM (RW) : ORIGIN = 0x20000000, LENGTH = 264K
}

PAGE_SIZE = 256;

ENTRY(_start);

SECTIONS {
    __stack_size = 0x2000;

    .bootloader : {
        KEEP(*(.bootloader));
        KEEP(*(.bootloader.*));
    }

    .text.vectors : {
        KEEP(*(.text.vectors));
    }

    .text : {
        *(.text);
        *(.text.*);
    }
    
    .rodata : {
        *(.rodata);
        *(.rodata.*);
    }
    
    .data : {
        *(.data);
        *(.data.*);
    }

    .bss : {
        *(.bss);
        *(.bss.*);
    }

    __ram_start = ORIGIN(RAM);
    __ram_end = ORIGIN(RAM) + LENGTH(RAM);

    __flash_start = ORIGIN(FLASH);
    __flash_end = ORIGIN(FLASH) + LENGTH(FLASH);

    /DISCARD/ : {
        *(.ARM.exidx);
        *(.ARM.exidx.*);
    }
}
