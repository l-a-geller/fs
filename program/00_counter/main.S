# RISC-V simple counter program
#
# Stanislav Zhelnio, 2020
#

            .text

start:      add a0, x0, x0              # t0 = 0
            add a1, x0, x0
counter:    lui a0, 0xDEADC
            lui a1, 0xBADC0
            addi a0, a0, 0x0DE           # t0 = t0 + 1
            addi a1, a1, 0x7E5
            or a2, a0, a1

            # сделать чтоб сюда вставить opcode и скомпилировать

            #beq zero, zero, counter  # if t0 == t1 then counter