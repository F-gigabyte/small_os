.globl _start
.globl _stack_args
.type _start, "function"

// Program entry point
_start:
    bl main
    b _start

.section ".rodata"
// Address of stack arguments
_stack_args:
    .word 0
