.section .text.vectors
_vec_table:
    .word 0x20042000 // main sp
    .word main // on reset jump to main
    .word _nmi // nmi interrupt
    .word _hard_fault // Hard fault
    .word _sys_call // sys call
    .word 0 // reserved
    .word 0 // reserved
    .word _sys_pend // svc pend
    .word _sys_tick // sys tick
// interrupts
    .word _unimplemented_irq // IRQ 0 -> timer irq 0
    .word _unimplemented_irq // IRQ 1 -> timer irq 1
    .word _unimplemented_irq // IRQ 2 -> timer irq 2
    .word _unimplemented_irq // IRQ 3 -> timer irq 3
    .word _unimplemented_irq // IRQ 4 -> pwm irq wrap
    .word _unimplemented_irq // IRQ 5 -> usb ctrl irq
    .word _unimplemented_irq // IRQ 6 -> xip irq
    .word _unimplemented_irq // IRQ 7 -> pio0 irq 0
    .word _unimplemented_irq // IRQ 8 -> pio0 irq 1
    .word _unimplemented_irq // IRQ 9 -> pio1 irq 0
    .word _unimplemented_irq // IRQ 10 -> pio1 irq 1
    .word _unimplemented_irq // IRQ 11 -> dma irq 0
    .word _unimplemented_irq // IRQ 12 -> dma irq 1
    .word _unimplemented_irq // IRQ 13 -> io irq bank 0
    .word _unimplemented_irq // IRQ 14 -> io irq qspi
    .word _unimplemented_irq // IRQ 15 -> sio irq proc0
    .word _unimplemented_irq // IRQ 16 -> sio irq proc1
    .word _unimplemented_irq // IRQ 17 -> clocks irq
    .word _unimplemented_irq // IRQ 18 -> spi0 irq
    .word _unimplemented_irq // IRQ 19 -> spi1 irq
    .word _unimplemented_irq // IRQ 20 -> uart0 irq
    .word _unimplemented_irq // IRQ 21 -> uart1 irq
    .word _unimplemented_irq // IRQ 22 -> adc irq fifo
    .word _unimplemented_irq // IRQ 23 -> i2c0 irq
    .word _unimplemented_irq // IRQ 24 -> i2c1 irq
    .word _unimplemented_irq // IRQ 25 -> rtc irq
    .word _unimplemented_irq // set to 0
    .word _unimplemented_irq // set to 0
    .word _unimplemented_irq // set to 0
    .word _unimplemented_irq // set to 0
    .word _unimplemented_irq // set to 0
    .word _unimplemented_irq // set to 0
