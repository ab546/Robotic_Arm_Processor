# 350assembler

**@author: pf51**
**@date: Nov 5 2017**

### About

1. **What is:** this is a simple parser that converts MIPS code into machine code using the ISA provided in our ECE350 handout. The code isn't super elegant and won't check for many syntax errors in your MIPS so ideally make sure there aren't any mistakes when you write it. If you find any bugs let me know, but it seems like it's working correctly.
2. **How to:** (1) clone or download the repo, (2) navigate to your folder in terminal OR open in an IDE, (3) write your mips code in the file "mips.txt" included, (4) compile Assembulator.java and run it, (5) copy and paste the printed output into your .mif file that you're using for your project
3. Things you **CAN** do: (1) write comments using "#" to start a line, (2) use nop's by typing nop (3) use $rstatus as syntax, (4) use $ra as syntax (5) use all other instructions in the correct syntax, found in our handout and also included below
4. Things you **CAN NOT** do: (1) put semi-colons at the end of your lines (2) write code that isn't in MIPS provided in handout (3) write same-line comments (i.e. `add $r1, $r5, $r6 # pls dont comment on same line like this`)

### Instructions

    nop
    add   $rd,   $rs,   $rt
    sub   $rd,   $rs,   $rt
    or   $rd,   $rs,   $rt
    sll   $rd,   $rs,   shamt
    sra   $rd,   $rs,   shamt
    mul   $rd,   $rd,   $rt
    div   $rd,   $rs,   $rt
    sw   $rd,   N($rs)
    lw   $rd,   N($rs)
    j   T
    bne   $rd,   $rs,   N
    jal   T
    jr   $rd
    blt   $rd,   $rs,   N
    bex   T
    setx   T