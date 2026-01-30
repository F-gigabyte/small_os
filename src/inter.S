// need .type to specify the labels as functions so bit 0 is set to 1
// Chapter 4 "Calling an assembly function from C" of "Getting Started With ARM Assembly Language"
// demonstrates the syntax for this.
// https://documentation-service.arm.com/static/6813592879c4c17ad8038691 (accessed 22/01/2026)
.globl _nmi
.type _nmi, "function"
.globl _hard_fault
.type _hard_fault, "function"
.globl _sys_call
.type _sys_call, "function"
.globl _sys_pend
.type _sys_pend, "function"
.globl _sys_tick
.type _sys_tick, "function"
.globl _unimplemented_irq
.type _unimplemented_irq, "function"
.globl _timer_irq
.type _timer_irq, "function"

_nmi:
    mov r0, sp
    bl nmi

_hard_fault:
    mov r0, sp
    bl hard_fault

_sys_call:
    mov r0, sp
    bl sys_call

_sys_pend:
    mov r0, sp
    bl sys_pend

_sys_tick:
    mov r0, sp
    bl sys_tick

_unimplemented_irq:
    mov r0, sp
    bl unimplemented_irq

_timer_irq:
    mov r0, sp
    bl timer_tick
    ldr r0, =0xfffffff1
    bx r0
