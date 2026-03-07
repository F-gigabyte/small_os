.section .text.flash
.globl test_flash
.type test_flash, "function"

#define CTRLR0_OFFSET 0x0
#define CTRLR1_OFFSET 0x4
#define SSI_ENABLE_OFFSET 0x8
#define SLAVE_ENABLE_OFFSET 0x10
#define BAUD_OFFSET 0x14
#define STATUS_OFFSET 0x28
#define DATA_OFFSET 0x60
#define SAMPLE_DELAY_OFFSET 0xf0
#define SPI_CTRLR0_OFFSET 0xf4

#define LOOKUP_CODE 0x18
#define FUNC_TABLE_CODE 0x14

#define EX 0x5845

#define CTRLR0_TMOD_SHIFT 8
#define CTRLR0_DFS_32_SHIFT 16
#define CTRLR0_SPI_FRF_SHIFT 21

#define CTRLR0_TMOD_TX_RX (0 << CTRLR0_TMOD_SHIFT)
#define CTRLR0_TMOD_EEPROM_READ (0x3 << CTRLR0_TMOD_SHIFT)
#define CTRLR0_DFS_32_8 (7 << CTRLR0_DFS_32_SHIFT)
#define CTRLR0_DFS_32_32 (31 << CTRLR0_DFS_32_SHIFT)
#define CTRLR0_SPI_FRF_QUAD (0x2 << CTRLR0_SPI_FRF_SHIFT)

#define CTRLR1_NDF_SHIFT 0
#define CTRLR1_NDF_16 (0xf << CTRLR1_NDF_SHIFT)

#define SSI_ENABLE_ENABLE_SHIFT 0
#define SSI_ENABLE_ENABLE (1 << SSI_ENABLE_ENABLE_SHIFT)
#define SSI_ENABLE_DISABLE (0 << SSI_ENABLE_ENABLE_SHIFT)

#define SLAVE_ENABLE_0_SHIFT 0
#define SLAVE_ENABLE_0 (1 << SLAVE_ENABLE_0_SHIFT)

#define BAUD_SCKDV_SHIFT 0
#define BAUD_SCKDV_4 (4 << BAUD_SCKDV_SHIFT)

#define STATUS_BUSY_SHIFT 0
#define STATUS_FIFO_NOT_FULL_SHIFT 1
#define STATUS_FIFO_EMPTY_SHIFT 2
#define STATUS_RECEIVE_NOT_EMPTY_SHIFT 3
#define STATUS_RECEIVE_FULL_SHIFT 4

#define STATUS_BUSY (1 << STATUS_BUSY_SHIFT)
#define STATUS_FIFO_NOT_FULL (1 << STATUS_FIFO_NOT_FULL_SHIFT)
#define STATUS_FIFO_EMPTY (1 << STATUS_FIFO_EMPTY_SHIFT)
#define STATUS_RECEIVE_NOT_EMPTY (1 << STATUS_RECEIVE_NOT_EMPTY_SHIFT)
#define STATUS_RECEIVE_FULL (1 << STATUS_RECEIVE_FULL_SHIFT)

#define SPI_CTRLR0_TRANS_TYPE_SHIFT 0
#define SPI_CTRLR0_ADDR_L_SHIFT 2
#define SPI_CTRLR0_INST_L_SHIFT 8
#define SPI_CTRLR0_WAIT_CYCLES_SHIFT 11

#define SPI_CTRLR0_TRANS_TYPE_1C1A (0 << SPI_CTRLR0_TRANS_TYPE_SHIFT)
#define SPI_CTRLR0_TRANS_TYPE_1C2A (0x1 << SPI_CTRLR0_TRANS_TYPE_SHIFT)
#define SPI_CTRLRO_ADDR_L_24 (6 << SPI_CTRLR0_ADDR_L_SHIFT)
#define SPI_CTRLR0_INST_L_8 (2 << SPI_CTRLR0_INST_L_SHIFT)
#define SPI_CTRLR0_WAIT_CYCLES (6 << SPI_CTRLR0_WAIT_CYCLES_SHIFT)

#define CMD_READ 0xeb
#define CMD_READ_STATUS1 0x05
#define CMD_READ_STATUS2 0x35
#define CMD_WRITE_STATUS 0x01
#define CMD_WRITE_ENABLE 0x06

#define CONTINUOUS_READ 0xa0

#define SREG 0x02

wait_ready:
    movs r2, #STATUS_FIFO_EMPTY
1:
    ldr r1, [r0, #STATUS_OFFSET]
    tst r1, r2
    beq 1b
    movs r2, #STATUS_BUSY
1:
    ldr r1, [r0, #STATUS_OFFSET]
    tst r1, r2
    bne 1b
    mov pc, lr

read_sreg:
    push {lr}
    str r1, [r0, #DATA_OFFSET]
    str r1, [r0, #DATA_OFFSET]
    
    bl wait_ready

    ldr r1, [r0, #DATA_OFFSET]
    ldr r1, [r0, #DATA_OFFSET]
    pop {pc}

boot_func:
    mov r1, r0
    push {lr}
    // get lookup function
    movs r2, #LOOKUP_CODE
    ldrh r3, [r2]
    movs r2, #FUNC_TABLE_CODE
    ldrh r0, [r2]
    blx r3
    pop {pc}

call_boot:
    push {lr}
    blx r0
    pop {pc}

test_flash:
    // find flash exit (code 'E', 'X')
    ldr r0, =EX
    push {lr}
    bl boot_func
    bl call_boot
    // 0x18000000
    movs r0, #0xc0
    lsls r0, 21

    // Disable SSI (enable bit set to 0)
    movs r1, #SSI_ENABLE_DISABLE
    str r1, [r0, #SSI_ENABLE_OFFSET]

    // SSI set to use TX and RX in standard SPI mode and 32 clocks a frame
    ldr r1, =(CTRLR0_DFS_32_8 | CTRLR0_TMOD_TX_RX)
    str r1, [r0, #CTRLR0_OFFSET]
    
    movs r2, #SPI_CTRLR0_OFFSET
    
    // format specified by FRF (Quad mode), 24 bit address, 8 bit instruction, 4 wait cycles
    ldr r1, =(SPI_CTRLR0_INST_L_8 | SPI_CTRLRO_ADDR_L_24 | SPI_CTRLR0_WAIT_CYCLES | SPI_CTRLR0_TRANS_TYPE_1C1A)
    str r1, [r0, r2]

    // Clock divider 4
    movs r1, #BAUD_SCKDV_4
    str r1, [r0, #BAUD_OFFSET]

    // select slave 0
    movs r1, #SLAVE_ENABLE_0
    str r1, [r0, #SLAVE_ENABLE_OFFSET]


    movs r2, #SAMPLE_DELAY_OFFSET
    movs r1, #1
    str r1, [r0, r2]
    
    ldr r1, =(SSI_ENABLE_ENABLE)
    // enable SSI
    str r1, [r0, #SSI_ENABLE_OFFSET]

    // write data register 0 and 1
    movs r1, #CMD_READ_STATUS2

    bl read_sreg

    movs r2, #SREG
    cmp r1, r2
    beq 2f

    movs r1, #CMD_WRITE_ENABLE
    str r1, [r0, #DATA_OFFSET]

    bl wait_ready

    ldr r1, [r0, #DATA_OFFSET]

    movs r1, #CMD_WRITE_STATUS
    str r1, [r0, #DATA_OFFSET]
    movs r1, #0
    str r1, [r0, #DATA_OFFSET]
    str r2, [r0, #DATA_OFFSET]

    bl wait_ready

    ldr r1, [r0, #DATA_OFFSET]
    ldr r1, [r0, #DATA_OFFSET]
    ldr r1, [r0, #DATA_OFFSET]
1:
    movs r1, #CMD_READ_STATUS1
    bl read_sreg
    movs r2, #1
    tst r1, r2
    bne 1b
2:
    movs r1, #SSI_ENABLE_DISABLE
    str r1, [r0, #SSI_ENABLE_OFFSET]

    ldr r1, =(CTRLR0_DFS_32_32 | CTRLR0_TMOD_EEPROM_READ | CTRLR0_SPI_FRF_QUAD)
    str r1, [r0, #CTRLR0_OFFSET]

    ldr r1, =16
    str r1, [r0, #CTRLR1_OFFSET]
    
    movs r2, #SPI_CTRLR0_OFFSET
    
    // format specified by FRF (Quad mode), 24 bit address, 8 bit instruction, 4 wait cycles
    ldr r1, =(SPI_CTRLR0_INST_L_8 | SPI_CTRLRO_ADDR_L_24 | SPI_CTRLR0_WAIT_CYCLES | SPI_CTRLR0_TRANS_TYPE_1C2A)
    str r1, [r0, r2]

    movs r1, #SSI_ENABLE_ENABLE
    str r1, [r0, #SSI_ENABLE_OFFSET]

    movs r1, #CMD_READ
    str r1, [r0, #DATA_OFFSET]
    movs r1, #0
    str r1, [r0, #DATA_OFFSET]

    bl wait_ready

1:
    ldr r1, [r0, #DATA_OFFSET]
    ldr r1, [r0, #STATUS_OFFSET]
    b 1b
