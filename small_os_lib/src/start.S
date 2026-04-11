.globl _start
.globl _stack_args
.type _start, "function"
_start:
    bl main
    b _start

.section ".rodata"
_stack_args:
    .word 0
