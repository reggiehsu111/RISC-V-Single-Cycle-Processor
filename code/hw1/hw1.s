.data
    n: .word 10 # You can change this number
    
.text
.globl __start

FUNCTION:
    # Todo: define your own function in HW1
    

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