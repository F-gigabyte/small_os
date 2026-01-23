MEMORY {
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    RAM : ORIGIN = 0x20000000, LENGTH = 264K
}

PAGE_SIZE = 256;

ENTRY(_start);

SECTIONS {
    . = ORIGIN(FLASH);
    .bootloader : {
        KEEP(*(.bootloader));
        KEEP(*(.bootloader.*));
    } > FLASH
    . = ALIGN(PAGE_SIZE);

    .text.vectors : {
        KEEP(*(.text.vectors));
    } > FLASH
    . = ALIGN(PAGE_SIZE);

    .text : {
        *(.text);
        *(.text.*);
    } > FLASH
    . = ALIGN(PAGE_SIZE);
    
    .rodata : {
        *(.rodata);
        *(.rodata.*);
    } > FLASH
    . = ALIGN(PAGE_SIZE);
    
    .data : {
        *(.data);
        *(.data.*);
    } > FLASH
    . = ALIGN(PAGE_SIZE);
    
    .bss : {
        *(.bss);
        *(.bss.*);
    } > FLASH
    . = ALIGN(PAGE_SIZE);

    /DISCARD/ : {
        *(.ARM.exidx);
        *(.ARM.exidx.*);
    }
}
