# RISC-V simple counter program
#
# Stanislav Zhelnio, 2020
#

            .text

start:      mv a0, zero               # t0 = 0
            mv a1, zero
            addi a1, a1, 5
counter:    addi a0, a0, 1            # t0 = t0 + 1
            bltu a0, a1, counter      # if t0 < t1 then target
counter1:   addi a1, a1, 1            # t0 = t0 + 1
            beq zero, zero, counter1  # if t0 == t1 then counter
