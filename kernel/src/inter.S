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
.globl _timer_irq
.type _timer_irq, "function"

.globl _irq_entry
.type _irq_entry, "function"

.globl _proc_switch
.type _proc_switch, "function"

.type exit_exception, "function"

.globl _start
.type _start, "function"

.globl _enable_irq
.type _enable_irq, "function"

.globl _disable_irq
.type _disable_irq, "function"

// r0 -> load start, r1 -> load end, r2 -> virt start
_memset:
2:
    cmp r0, r1
    bge 1f
    ldr r3, [r0]
    adds r0, #4
    str r3, [r2]
    adds r2, #4
    b 2b
1:
    mov pc, lr

// r0 -> start, r1 -> end
_zero:
    movs r2, #0
2:
    cmp r0, r1
    bge 1f
    str r2, [r0]
    adds r0, #4
    b 2b
1:
    mov pc, lr

_start:
    // copy data into memory
    ldr r0, =__kernel_data_phys_start // Data physical address
    ldr r1, =__kernel_data_phys_end // Data physical end address
    ldr r2, =__kernel_data_virt_start // Data load address
    bl _memset
    ldr r0, =__async_queues_phys_start // Data physical address
    ldr r1, =__async_queues_phys_end // Data physical address
    ldr r2, =__async_queues_virt_start // Data load address
    bl _memset
    // 0 bss section
    ldr r0, =__kernel_bss_virt_start // bss load address
    ldr r1, =__kernel_bss_virt_end // bss load end address
    bl _zero
    // 0 sync queues
    ldr r0, =__sync_queues_virt_start
    ldr r1, =__sync_queues_virt_end
    bl _zero
    // 0 proc table
    ldr r0, =__procs_virt_start
    ldr r1, =__procs_virt_end
    bl _zero
    // jump to main
    bl main
    // halt if return
    wfi
    b .


_nmi:
    mov r0, sp
    mov r1, lr
    bl nmi
// if return, because was a process that was terminated
2:
    movs r0, r0
    bne 3f
    wfi
    bl do_proc_switch
    b 2b
3:
    bl schedule

_hard_fault:
    mov r0, sp
    mov r1, lr
    bl hard_fault
// if return, because was a process that was terminated
2:
    movs r0, r0
    bne 3f
    wfi
    bl do_proc_switch
    b 2b
3:
    bl schedule


_sys_call:
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 3f // if not running a process, the sys call is invalid
    // store unsaved process registers (r4 to r11)
    stm r0!, {r4-r7}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    stm r0!, {r4-r7}
    adds r0, #4 // increment after pid
    mrs r1, psp
    // save stack pointer
    str r1, [r0]
    // first argument stack
    mov r0, r1
    bl sys_call
    // if got null pointer (no process) wait for an interrupt and try again
2:
    movs r0, r0
    bne 3f
    wfi
    bl do_proc_switch
    b 2b
3:
    bl schedule

_sys_pend:
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 1f
    // store unsaved process registers (r4 to r11)
    stm r0!, {r4-r7}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    stm r0!, {r4-r7}
    adds r0, #4 // increment after pid
    mrs r1, psp
    // save stack pointer
    str r1, [r0]
1:
    bl pend_sv
    // if got null pointer (no process) wait for an interrupt and try again
2:
    movs r0, r0
    bne 3f
    wfi
    bl do_proc_switch
    b 2b
3:
    bl schedule

_sys_tick:
    // get last 4 bits of lr (will be 1, 9 or d and be used to return to correct privileges after)
    movs r1, #0xf
    mov r0, lr
    ands r1, r0
    // get privileged bit and add into r0
    mrs r0, control
    movs r2, #1
    ands r0, r2
    lsls r0, #1
    orrs r1, r0
    push {r1}
    // get interrupt number
    mrs r0, ipsr
    movs r3, #0x1f
    ands r0, r3
    subs r0, #16
    bl sys_tick
    pop {r0}
    b exit_exception

// r0:
// bit pattern of
// - [0]: 1
// - [1]: 0 -> privileged mode, 1 -> unpriveleged mode
// - [2]: 0 -> main stack, 1 -> process stack
// - [3]: 0 -> handle mode, 1 -> thread mode
// the state of the stack must be the same as on exception entry
exit_exception:
    movs r2, #2
    mov r3, r2
    // get privileged bit
    ands r3, r0
    lsrs r3, #1
    mrs r1, control
    movs r4, #1
    // write back priveleged bit
    bics r1, r4
    orrs r1, r3
    msr control, r1
    isb
    // setup pc to return from IRQ and return
    mvns r2, r2
    ands r0, r2
    ldr r1, =0xfffffff0
    orrs r1, r0
    bx r1

_irq_entry:
    // get last 4 bits of lr (will be 1, 9 or d and be used to return to correct privileges after)
    movs r1, #0xf
    mov r0, lr
    ands r1, r0
    // get privileged bit and add into r0
    mrs r0, control
    movs r2, #1
    ands r0, r2
    lsls r0, #1
    orrs r1, r0
    push {r1}
    // get interrupt number
    mrs r0, ipsr
    movs r3, #0x1f
    ands r0, r3
    subs r0, #16
    bl irq_handler
    pop {r0}
    b exit_exception

_proc_switch:
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 1f
    // store unsaved process registers (r4 to r11)
    stm r0!, {r4-r7}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    stm r0!, {r4-r7}
    adds r0, #4 // increment after pid
    mrs r1, psp
    // save stack pointer
    str r1, [r0]
1:
    bl timer0
    // if got null pointer (no process) wait for an interrupt and try again
2:
    movs r0, r0
    bne 3f
    wfi
    bl do_proc_switch
    b 2b
3:
    bl schedule
