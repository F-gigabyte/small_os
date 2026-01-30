.section .text.vectors

.globl _irq_0
.type _irq_0, "function"
.globl _irq_1
.type _irq_1, "function"
.globl _irq_2
.type _irq_2, "function"
.globl _irq_3
.type _irq_3, "function"
.globl _irq_4
.type _irq_4, "function"
.globl _irq_5
.type _irq_5, "function"
.globl _irq_6
.type _irq_6, "function"
.globl _irq_7
.type _irq_7, "function"
.globl _irq_8
.type _irq_8, "function"
.globl _irq_9
.type _irq_9, "function"
.globl _irq_10
.type _irq_10, "function"
.globl _irq_11
.type _irq_11, "function"
.globl _irq_12
.type _irq_12, "function"
.globl _irq_13
.type _irq_13, "function"
.globl _irq_14
.type _irq_14, "function"
.globl _irq_15
.type _irq_15, "function"
.globl _irq_16
.type _irq_16, "function"
.globl _irq_17
.type _irq_17, "function"
.globl _irq_18
.type _irq_18, "function"
.globl _irq_19
.type _irq_19, "function"
.globl _irq_20
.type _irq_20, "function"
.globl _irq_21
.type _irq_21, "function"
.globl _irq_22
.type _irq_22, "function"
.globl _irq_23
.type _irq_23, "function"
.globl _irq_24
.type _irq_24, "function"
.globl _irq_25
.type _irq_25, "function"
.globl _irq_26
.type _irq_26, "function"
.globl _irq_27
.type _irq_27, "function"
.globl _irq_28
.type _irq_28, "function"
.globl _irq_29
.type _irq_29, "function"
.globl _irq_30
.type _irq_30, "function"
.globl _irq_31
.type _irq_31, "function"

.globl _start
.type _start, "function"

_vec_table:
    .word 0x20042000 // main sp (0)
    .word _start // on reset jump to start (reset) (1)
    .word _nmi // nmi interrupt (2)
    .word _hard_fault // Hard fault (3)
    .word 0 // reserved (4)
    .word 0 // reserved (5)
    .word 0 // reserved (6)
    .word 0 // reserved (7)
    .word 0 // reserved (8)
    .word 0 // reserved (9)
    .word 0 // reserved (10)
    .word _sys_call // sys call (11)
    .word 0 // reserved (12)
    .word 0 // reserved (13)
    .word _sys_pend // svc pend (14)
    .word _sys_tick // sys tick (15)
// interrupts
    .word _timer_irq // IRQ 0 -> timer irq 0 (16)
    .word _irq_1 // IRQ 1 -> timer irq 1 (17)
    .word _irq_2 // IRQ 2 -> timer irq 2 (18)
    .word _irq_3 // IRQ 3 -> timer irq 3 (19)
    .word _irq_4 // IRQ 4 -> pwm irq wrap (20)
    .word _irq_5 // IRQ 5 -> usb ctrl irq (21)
    .word _irq_6 // IRQ 6 -> xip irq (22)
    .word _irq_7 // IRQ 7 -> pio0 irq 0 (23)
    .word _irq_8 // IRQ 8 -> pio0 irq 1 (24)
    .word _irq_9 // IRQ 9 -> pio1 irq 0 (25)
    .word _irq_10 // IRQ 10 -> pio1 irq 1 (26)
    .word _irq_11 // IRQ 11 -> dma irq 0 (27)
    .word _irq_12 // IRQ 12 -> dma irq 1 (28)
    .word _irq_13 // IRQ 13 -> io irq bank 0 (29)
    .word _irq_14 // IRQ 14 -> io irq qspi (30)
    .word _irq_15 // IRQ 15 -> sio irq proc0 (31)
    .word _irq_16 // IRQ 16 -> sio irq proc1 (32)
    .word _irq_17 // IRQ 17 -> clocks irq (33)
    .word _irq_18 // IRQ 18 -> spi0 irq (34)
    .word _irq_19 // IRQ 19 -> spi1 irq (35)
    .word _irq_20 // IRQ 20 -> uart0 irq (36)
    .word _irq_21 // IRQ 21 -> uart1 irq (37)
    .word _irq_22 // IRQ 22 -> adc irq fifo (38)
    .word _irq_23 // IRQ 23 -> i2c0 irq (39)
    .word _irq_24 // IRQ 24 -> i2c1 irq (40)
    .word _irq_25 // IRQ 25 -> rtc irq (41)
    .word _irq_26 // set to 0 (42)
    .word _irq_27 // set to 0 (43)
    .word _irq_28 // set to 0 (44)
    .word _irq_29 // set to 0 (45)
    .word _irq_30 // set to 0 (46)
    .word _irq_31 // set to 0 (47)

_start:
    // copy data into memory
    ldr r0, =__data_start // Data physical address
    ldr r2, =__data_end // Data physical end address
    ldr r1, =__ram_start // RAM start
2:
    cmp r0, r2
    bge 1f
    ldr r3, [r0]
    adds r0, #4
    str r3, [r1]
    adds r1, #4
    b 2b
1:
    bl main

_irq_0:
	ldr r0, =0
	b _irq_0
_irq_1:
	ldr r0, =1
	b _irq_1
_irq_2:
	ldr r0, =2
	b _irq_2
_irq_3:
	ldr r0, =3
	b _irq_3
_irq_4:
	ldr r0, =4
	b _irq_4
_irq_5:
	ldr r0, =5
	b _irq_5
_irq_6:
	ldr r0, =6
	b _irq_6
_irq_7:
	ldr r0, =7
	b _irq_7
_irq_8:
	ldr r0, =8
	b _irq_8
_irq_9:
	ldr r0, =9
	b _irq_9
_irq_10:
	ldr r0, =10
	b _irq_10
_irq_11:
	ldr r0, =11
	b _irq_11
_irq_12:
	ldr r0, =12
	b _irq_12
_irq_13:
	ldr r0, =13
	b _irq_13
_irq_14:
	ldr r0, =14
	b _irq_14
_irq_15:
	ldr r0, =15
	b _irq_15
_irq_16:
	ldr r0, =16
	b _irq_16
_irq_17:
	ldr r0, =17
	b _irq_17
_irq_18:
	ldr r0, =18
	b _irq_18
_irq_19:
	ldr r0, =19
	b _irq_19
_irq_20:
	ldr r0, =20
	b _irq_20
_irq_21:
	ldr r0, =21
	b _irq_21
_irq_22:
	ldr r0, =22
	b _irq_22
_irq_23:
	ldr r0, =23
	b _irq_23
_irq_24:
	ldr r0, =24
	b _irq_24
_irq_25:
	ldr r0, =25
	b _irq_25
_irq_26:
	ldr r0, =26
	b _irq_26
_irq_27:
	ldr r0, =27
	b _irq_27
_irq_28:
	ldr r0, =28
	b _irq_28
_irq_29:
	ldr r0, =29
	b _irq_29
_irq_30:
	ldr r0, =30
	b _irq_30
_irq_31:
	ldr r0, =31
	b _irq_31
