# crt0.S
# Startup file for riscv_3d_kinematics_accel firmware

.section .init
.global _start

_start:
    # 1. Clear all registers except sp (x2)
    li x1,  0
    li x3,  0
    li x4,  0
    li x5,  0
    li x6,  0
    li x7,  0
    li x8,  0
    li x9,  0
    li x10, 0
    li x11, 0
    li x12, 0
    li x13, 0
    li x14, 0
    li x15, 0
    li x16, 0
    li x17, 0
    li x18, 0
    li x19, 0
    li x20, 0
    li x21, 0
    li x22, 0
    li x23, 0
    li x24, 0
    li x25, 0
    li x26, 0
    li x27, 0
    li x28, 0
    li x29, 0
    li x30, 0
    li x31, 0

    # 2. Set Stack Pointer (sp) to the top of Data Memory
    # Our data memory is 16KB (starts at 0x4000, ends at 0x8000)
    li sp, 0x00008000

    # 3. Clear BSS section
    la t0, __bss_start
    la t1, __bss_end
bss_clear_loop:
    bge t0, t1, bss_clear_done
    sw zero, 0(t0)
    addi t0, t0, 4
    j bss_clear_loop
bss_clear_done:

    # 4. Call main
    jal ra, main

    # 5. If main returns, loop indefinitely
loop:
    j loop
