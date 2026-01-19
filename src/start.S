.section .text.vectors
_vec_table:
    .word 0x20042000 // main sp
    .word main // on reset jump to main
