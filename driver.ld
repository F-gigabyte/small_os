ENTRY(main);

SECTIONS {
    __stack_size = 0x1000;

    .text : {
        *(.text);
        *(.text.*);
    }

    .data : {
        *(.data);
        *(.data.*);
        *(.bss);
        *(.bss.*);
    }

    .rodata : {
        *(.rodata);
        *(.rodata.*);
    }
    
    /DISCARD/ : {
        *(.ARM.exidx);
        *(.ARM.exidx.*);
    }
}
