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

.globl _irq
.type _irq, "function"

.globl _proc_switch
.type _proc_switch, "function"

.global idle
.type idle, "function"

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
    ldr r0, =__kernel_text_flash_phys_start // Data physical address
    ldr r1, =__kernel_text_flash_phys_end // Data physical address
    ldr r2, =__kernel_text_flash_virt_start // Data load address
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
    // save lr
    push {lr}
    // pause sys tick
    bl pause_sys_tick
    pop {r1}
    mov r0, sp
    bl nmi
// if return, because was a process that was terminated
2:
    movs r0, r0
    bne 3f
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
3:
    bl schedule

_hard_fault:
    // save lr
    push {lr}
    // pause sys tick
    bl pause_sys_tick
    pop {r1}
    mov r0, sp
    bl hard_fault
// if return, because was a process that was terminated
2:
    movs r0, r0
    bne 3f
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
3:
    bl schedule


_sys_call:
    // pause sys tick
    bl pause_sys_tick
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 3f // if not running a process, the sys call is invalid
    // save current process codes
    bl update_codes
    // store unsaved process registers (r4 to r11)
    stm r0!, {r4-r7}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    stm r0!, {r4-r7}
    mrs r1, psp
    // save stack pointer
    str r1, [r0]
    // first argument stack
    mov r0, r1
    bl sys_call
    // if got null pointer (no process) wait for an interrupt and try again
2:
    movs r0, r0
    bne 4f
3:
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
4:
    bl schedule

idle:
1:
    wfi
    b 1b

_sys_pend:
    // pause system tick
    bl pause_sys_tick
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 1f
    // save current process codes
    bl update_codes
    // store unsaved process registers (r4 to r11)
    stm r0!, {r4-r7}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    stm r0!, {r4-r7}
    mrs r1, psp
    // save stack pointer
    str r1, [r0]
1:
    bl pend_sv
    // if got null pointer (no process) wait for an interrupt and try again
    movs r0, r0
    bne 3f
2:
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
3:
    bl schedule

_irq:
    // pause sys tick
    bl pause_sys_tick
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 2f // if not running a process, don't need to save
    // save current process codes
    bl update_codes
    // store unsaved process registers (r4 to r11)
    stm r0!, {r4-r7}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    stm r0!, {r4-r7}
    mrs r1, psp
    // save stack pointer
    str r1, [r0]
2:
    // first argument IRQ number
    mrs r0, ipsr
    movs r1, #0x3f
    ands r0, r1
    subs r0, #16
    bl irq_handler
    // if got null pointer (no process) wait for an interrupt and try again
    movs r0, r0
    bne 4f
3:
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
4:
    bl schedule

_proc_switch:
    // pause system tick
    bl pause_sys_tick
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 1f
    // save current process codes
    bl update_codes
    mov r2, r0
    // store unsaved process registers (r4 to r11)
    stm r2!, {r4-r7}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    stm r2!, {r4-r7}
    mrs r1, psp
    // save stack pointer
    str r1, [r2]
1:
    bl sys_tick
    // if got null pointer (no process) wait for an interrupt and try again
    movs r0, r0
    bne 3f
2:
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
3:
    bl schedule
