.data
   n: .word 7
.text
.globl __start
 
# recursive function for T(n)
FUNCTION:
    addi sp, sp, -16
    sw   x1, 8(sp)
    sw   x10, 0(sp)
    
  # check if n >= 2
    slti x5, x10, 2
    beq x5, x0, L1
  #  addi t1, a0, -2
  #  bge  t1, x0, L1
    
  # set ans to 7 and return
    addi x10, x0, 7
    addi sp, sp, 16
    jalr x0, 0(x1)

L1:
  # recursive call T(n/2)
    srai x10, x10, 1
    jal  x1, FUNCTION
    
  # calculate the ans
    slli x5, x10, 3
    lw   x10, 0(sp)
    lw   x1, 8(sp)
    addi sp, sp, 16
    slli x10, x10, 2
  
  # save ans to t0 and return
    add  x10, x5, x10
  # add  x18, x10, x0 # output ans to x18
    jalr x0, 0(x1)

__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall