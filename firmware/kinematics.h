// kinematics.h
// Custom compiler inline assembly macro definitions for the 3D Kinematics custom instruction extension.

#ifndef KINEMATICS_H
#define KINEMATICS_H

// Packing macros to pack two 16-bit Q8.8 fixed-point numbers into a single 32-bit register.
#define PACK_Q8_8(a, b)  ((((int32_t)(a) & 0xFFFF) << 16) | ((int32_t)(b) & 0xFFFF))

// MATLOAD Instruction macros
// opcode = 0x0B, funct7 = 0x00, funct3 = 0, 1, 2, rd = x0 (ignored)
// rs1: [A, B] (packed 16-bit Q8.8 elements)
// rs2: [C, D] (packed 16-bit Q8.8 elements)
#define MATLOAD_ROW0(rs1, rs2) \
    __asm__ volatile (".insn r 0x0B, 0, 0, x0, %0, %1" :: "r"(rs1), "r"(rs2))

#define MATLOAD_ROW1(rs1, rs2) \
    __asm__ volatile (".insn r 0x0B, 1, 0, x0, %0, %1" :: "r"(rs1), "r"(rs2))

#define MATLOAD_ROW2(rs1, rs2) \
    __asm__ volatile (".insn r 0x0B, 2, 0, x0, %0, %1" :: "r"(rs1), "r"(rs2))

// MATMUL3D Instruction macros
// opcode = 0x0B, funct7 = 0x00, funct3 = 4, 5, 6, rd = destination register
// rs1: [X, Y] (packed 16-bit Q8.8 coordinate inputs)
// rs2: [Z, 1] (packed 16-bit Q8.8 coordinate input Z and translation multiplier 1.0)
#define MATMUL3D_X(rd, rs1, rs2) \
    __asm__ volatile (".insn r 0x0B, 4, 0, %0, %1, %2" : "=r"(rd) : "r"(rs1), "r"(rs2))

#define MATMUL3D_Y(rd, rs1, rs2) \
    __asm__ volatile (".insn r 0x0B, 5, 0, %0, %1, %2" : "=r"(rd) : "r"(rs1), "r"(rs2))

#define MATMUL3D_Z(rd, rs1, rs2) \
    __asm__ volatile (".insn r 0x0B, 6, 0, %0, %1, %2" : "=r"(rd) : "r"(rs1), "r"(rs2))

#endif // KINEMATICS_H
