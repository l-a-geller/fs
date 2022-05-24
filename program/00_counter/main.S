# RISC-V simple counter program
#
# Stanislav Zhelnio, 2020
#

            .text

start:      add a0, x0, x0
            add a1, x0, x0
            lui a0, 0xDEADC
            lui a1, 0xBADC0
            addi a0, a0, 0x2
            addi a1, a1, 0x28
            #or a2, a0, a1
            #add a2, a0, a1