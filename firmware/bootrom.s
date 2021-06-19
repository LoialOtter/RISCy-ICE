#
# This file is part of the RISCy-ICE project.
#
# The MIT License (MIT)
#
# Copyright (c) 2021 Matthew Peters - matthew@ottersoft.ca
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#======================================
# Initial Boot Memory
#======================================
#
# This routine is an initial bootloader used by
# the CPU to fetch the BIOS and runtime code from
# QSPI flash. If no valid image is found, then
# it will jump to die() and flash the display
# red to indicate failure.
#
.section .text

.globl coldboot
.func coldboot
coldboot:
    la a0, 0x20000400
    li a1, 0x01234567
    sw a1, 0(a0)
    lw a2, 0(a0)
    bne a1, a2, die

    j success
    
    #------------- disabled -----------------
    la s0, (_stack_start - 64)  # SRAM location of memcpy.
    la s1, 0x02100000           # Flash location of the boot header.

    # Check if the BIOS boot header is valid.
    lw a1, 0(s1)
    la a0, 0xdeadbeef
    bne a0, a1, die

    # Relocate the memcpy function into SRAM
    addi a0, s0, 0      # SRAM location for the copy.
    la   a1, memcpy     # .text location of memcpy.
    li   a2, 64         # rough size of the memcpy routine.
    jalr a1             # Call memcpy to relocate itself.

    # Prepare arguments to memcpy to relocate the .text segment.
    addi a0, zero, 0    # copy into start of ROM (address zero)
    lw   a1, 16(s1)     # get the offset of the .text segment to relocate.
    add  a1, a1, s1     # get the address of the .text segment to relocate.
    lw   a2, 12(s1)     # get the length of data to relocate.

    # Call the SRAM version of memcpy, and return into warm boot entry point. 
    lw   ra, 8(s1)      # Set return address to warm boot entry.
    jr   s0             # call the SRAM location of memcpy.
.endfunc

# We could not load the BIOS image. Time to die.
.globl success
.func success
success:
    # Find the misc registers.
    la a0, 0x30000000
    li t0, 0x0000
    li t1, 0x0000
    li t2, 0x0200
    li t3, 0x0100

0:  # update the level
    addi t0, t0, 1
    blt t0, t2, 1f      # if less than 512, continue
    li t0, 0x0000       # otherwise reset
1:  # apply the value
    add t1, t0, zero    # copy the value - not sure if this is the best way
    blt t0, t3, 1f      # if the value's 0-255 continue
    sub t1, t2, t0      # otherwise subtract scale from 255-0
1:  # save to the peripheral
    sw t1, 8(a0)
    li s0, 0x1000       # reset the counter
2:  # The delay loop
    beqz s0, 0b         # Redraw if the counter reaches zero.
    addi s0, s0, -1     # Decrement the counter otherwise.
    beqz zero, 2b
.endfunc
    
# We could not load the BIOS image. Time to die.
.globl die
.func die
die:
    # Find the misc registers.
    la a0, 0x30000000
    li a1, 0x000F
    sw a1, 0(a0)

    # Pick a fill color (red, for badness).
    li s1, 0x0F
    li a1, 0
0:  # The display loop
    sw   a1, 0(a0)
    li   s0, 0x200000   # Reset the delay counter.
    xor  a1, a1, s1     # Toggle the color.
1:  # The delay loop
    beqz s0, 0b         # Redraw if the counter reaches zero.
    addi s0, s0, -1     # Decrement the counter otherwise.
    beqz zero, 1b
.endfunc

# A minimal memcpy - only accepts aligned pointers.
.globl memcpy
.func
memcpy:
    beqz a2, 1f     # exit immediately if the count is zero.
    add  a3, a1, a2 # a3 holds the source end pointer.
0:                  # Start of the memcpy loop.
    lw   a4, 0(a1)  # load a word from source.
    addi a1, a1, 4
    sw   a4, 0(a0)  # store a word to dest.
    addi a0, a0, 4
    blt  a1, a3, 0b # continue as long as source < end.
    sub  a0, a0, a2 # restore the dest pointer before returning.
1:
    ret
.endfunc
