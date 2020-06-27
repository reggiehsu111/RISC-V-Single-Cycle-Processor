.data
    n: .word 10 # You can change this number
    
.text
.globl __start

FUNCTION:
    # Todo: define your own function in HW1
    addi t1, x0, 1 # const t1 = 1
    addi t2, x0, 2 # const t2 = 2
    addi t3, x0, 4 # const t3 = 4
    addi sp, sp, -4 # Save return addr. and n in stack 
    sw x1, 0(sp)
    jal x1, T # call T
    lw x1, 0(sp)
    jalr x0, 0(x1)

T:
    addi sp, sp, -8 # Save return addr. and n in stack 
    sw x1, 4(sp)
    sw a0, 0(sp)
    beq a0, t1, T_end # If n == 1, goto T_end
    div a0, a0, t2 # n = n / 2
    jal x1, T # call T
    addi t4, a0, 0 # t4 = T(n / 2)
    lw a0, 0(sp) # Restore caller’s n and return addr.
    lw x1, 4(sp)
    addi sp, sp, 8
    mul t5, t4, t3 # t5 = T(n / 2) * 4
    mul t6, a0, t2 # t6 = n * 2
    add a0, t5, t6
    jalr x0, 0(x1) # return t5 + t6

T_end:
    addi sp, sp, 8 # Pop stack
    jalr x0, 0(x1) # return n = 1

# Do not modify this part!!! #
__start:                     #
    la   t0, n               #
    lw   x10, 0(t0)          #
    jal  x1,FUNCTION         #
    la   t0, n               #
    sw   x10, 4(t0)          #
    addi a0,x0,10            #
    ecall                    #
##############################