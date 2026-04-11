ENTRY(_start);

SECTIONS {
    __stack_size = 0x4000;

    .text : {
        *(.text);
        *(.text.*);
    }

    .data : {
        *(.data);
        *(.data.*);
    }

    .bss : {
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
