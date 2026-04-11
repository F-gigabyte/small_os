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

// copies memory from one buffer (r0) to another (r2)
// r0 -> load start, r1 -> load end, r2 -> virt start
// Safety
// r0 points to a readable buffer of byte length r1 - r0
// r2 points to a writable buffer of byte length r1 - r0
// Both buffers must be a multiple of 4 bytes in size and aligned at a 4 byte boundary
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

// zeros out a buffer (r0)
// r0 -> buffer start, r1 -> buffer end
// Safety
// r0 points to a writable buffer of byte length r1 - r0
// The buffer must be a multiple of 4 bytes in size and aligned at a 4 byte boundary
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

// Address jumped to when the second stage bootloader finishes executing
// This sets up the kernel's memory and jumps to main
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
    // 0 notifiers
    ldr r0, =__notifier_virt_start
    ldr r1, =__notifier_virt_end
    bl _zero
    // jump to main
    bl main
    // halt if return
    wfi
    b .

// Non-maskable interrupt handler
_nmi:
    // save lr
    push {lr}
    // pause sys tick
    bl pause_sys_tick
#ifdef BENCHMARK
    bl kernel_entry
#endif
    pop {r1}
    mov r0, sp
    // jump to main handler
    bl nmi
// if return, because was a process that was terminated
2:
    // check if returned process is null
    movs r0, r0
    bne 3f
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
#ifdef RADIATION
    bl do_radiation
#endif
#ifdef BENCHMARK
    bl kernel_exit
#endif
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
3:
    // schedule returned process
    bl schedule

// hard fault handler
_hard_fault:
    // save lr
    push {lr}
    // pause sys tick
    bl pause_sys_tick
#ifdef BENCHMARK
    bl kernel_entry
#endif
    pop {r1}
    mov r0, sp
    // jump to main handler
    bl hard_fault
// if return, because was a process that was terminated
2:
    // check if returned process is null
    movs r0, r0
    bne 3f
    // process is null
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
#ifdef RADIATION
    bl do_radiation
#endif
#ifdef BENCHMARK
    bl kernel_exit
#endif
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
3:
    // schedule returned process
    bl schedule


_sys_call:
    // pause sys tick
    bl pause_sys_tick
#ifdef BENCHMARK
    bl kernel_entry
#endif
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 3f // if not running a process, the sys call is invalid
#ifdef ERROR_CODES
    // save current process codes
    bl update_codes
#endif
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
    // jump to main handler
    bl sys_call
    // if got null pointer (no process) wait for an interrupt and try again
2:
    // check if returned process is null
    movs r0, r0
    bne 4f
3:
    // process is null
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
#ifdef RADIATION
    bl do_radiation
#endif
#ifdef BENCHMARK
    bl kernel_exit
#endif
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
4:
    // schedule returned process
    bl schedule

// Idle process
// Waits for an interrupt to happen in a loop
idle:
1:
    wfi
    b 1b

// Sys Pend interrupt generated
// This is the interrupt caused by the kernel when scheduling starts
_sys_pend:
    // pause system tick
    bl pause_sys_tick
#ifdef BENCHMARK
    bl kernel_entry
#endif
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 1f
#ifdef ERROR_CODES
    // save current process codes
    bl update_codes
#endif
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
    // check if returned process is null
    movs r0, r0
    bne 3f
2:
    // process is null
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
#ifdef RADIATION
    bl do_radiation
#endif
#ifdef BENCHMARK
    bl kernel_exit
#endif
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
3:
    // schedule returned process
    bl schedule

_irq:
    // pause sys tick
    bl pause_sys_tick
#ifdef BENCHMARK
    bl kernel_entry
#endif
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 2f // if not running a process, don't need to save
#ifdef ERROR_CODES
    // save current process codes
    bl update_codes
    // store unsaved process registers (r4 to r11)
#endif
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
    // jump to main handler
    bl irq_handler
    // check if returned process is null
    movs r0, r0
    bne 4f
3:
    // process is null
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
#ifdef RADIATION
    bl do_radiation
#endif
#ifdef BENCHMARK
    bl kernel_exit
#endif
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
4:
    // schedule returned process
    bl schedule

_proc_switch:
    // pause system tick
    bl pause_sys_tick
#ifdef BENCHMARK
    bl kernel_entry
#endif
    // get current process
    bl get_current_proc
    // check if currently running a process and if so, save the state
    movs r0, r0
    beq 1f
#ifdef ERROR_CODES
    // save current process codes
    bl update_codes
#endif
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
    bl sys_tick
    // check if returned process is null
    movs r0, r0
    bne 3f
2:
    // process is null
    // make thread mode privileged
    mrs r0, control
    movs r2, #1
    bics r0, r2
    msr control, r0
    isb
#ifdef RADIATION
    bl do_radiation
#endif
#ifdef BENCHMARK
    bl kernel_exit
#endif
    // return to thread mode using main stack (idle)
    ldr r0, =0xfffffff9
    bx r0
3:
    // schedule returned process
    bl schedule
