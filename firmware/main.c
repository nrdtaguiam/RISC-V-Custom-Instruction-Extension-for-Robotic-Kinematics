// main.c
// Test firmware for the riscv_3d_kinematics_accel project.
// Compares hardware custom instructions vs pure software math loop.
// Configured to strictly use 32-bit integers to bypass byte/halfword hardware requirements.

#include <stdint.h>
#include "kinematics.h"

// Memory-Mapped IO Addresses
#define MMIO_CYCLE_COUNT (*(volatile uint32_t*)0xFFFF0000)
#define MMIO_SIM_OUT     (*(volatile uint32_t*)0xFFFF0004)
#define MMIO_SIM_EXIT    (*(volatile uint32_t*)0x80000000)

// 1.0 in Q8.8 fixed-point is 256 (0x0100)
#define Q8_8_ONE 256

// Struct for 3D Vector in Q8.8 fixed-point
// Uses int32_t elements to compile strictly into word LW/SW instructions
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} vec3_t;

// Print a character to simulation output
void print_char(char c) {
    MMIO_SIM_OUT = c;
}

// Read a single byte from any address using strictly 32-bit aligned memory reads
uint8_t read_byte(const void *addr) {
    uint32_t aligned_addr = (uint32_t)addr & ~3;
    uint32_t offset = (uint32_t)addr & 3;
    uint32_t word = *(volatile uint32_t*)aligned_addr;
    return (uint8_t)((word >> (offset * 8)) & 0xFF);
}

// Print a string using 32-bit aligned byte reads
void print_str(const char *str) {
    while (1) {
        uint8_t c = read_byte(str);
        if (c == 0) break;
        print_char(c);
        str++;
    }
}

// Custom 32-bit unsigned division and modulo helper
// Uses a binary restoring division algorithm (no / or % operators)
uint32_t div_mod(uint32_t dividend, uint32_t divisor, uint32_t *remainder) {
    if (divisor == 0) {
        if (remainder) *remainder = 0;
        return 0;
    }
    uint32_t quotient = 0;
    uint32_t accum = 0;
    for (int i = 31; i >= 0; i--) {
        accum = (accum << 1) | ((dividend >> i) & 1);
        if (accum >= divisor) {
            accum -= divisor;
            quotient |= (1U << i);
        }
    }
    if (remainder) {
        *remainder = accum;
    }
    return quotient;
}

// Print a decimal number using a 32-bit word array on the stack
void print_num(int32_t num) {
    if (num < 0) {
        print_char('-');
        num = -num;
    }
    if (num == 0) {
        print_char('0');
        return;
    }
    int32_t buf[12];
    int i = 0;
    uint32_t val = num;
    while (val > 0) {
        uint32_t rem;
        val = div_mod(val, 10, &rem);
        buf[i++] = '0' + rem;
    }
    for (int j = i - 1; j >= 0; j--) {
        print_char(buf[j]);
    }
}

int main() {
    print_str("=========================================================\n");
    print_str("Starting 3D Robotic Kinematics Custom Extension Test\n");
    print_str("=========================================================\n");

    // --- 3x4 Transformation Matrix in Q8.8 Fixed-Point ---
    // Represents rotation, scaling, and translation:
    // Row 0: [ 1.2, -0.8,  0.5,  10.0 ] -> [ 307, -205, 128, 2560 ]
    // Row 1: [ 0.5,  1.5, -0.3, -15.5 ] -> [ 128,  384, -77, -3968 ]
    // Row 2: [ -0.2, 0.4,  1.0,   5.2 ] -> [ -51,  102, 256, 1331 ]
    int32_t mat[3][4] = {
        { 307, -205,  128,  2560 },
        { 128,  384,  -77, -3968 },
        { -51,  102,  256,  1331 }
    };

    // --- Batch of 3D Input Points ---
    vec3_t points[5] = {
        { 256,  512,  -128 },  // [1.0, 2.0, -0.5]
        { -512, 1024,   768 },  // [-2.0, 4.0, 3.0]
        { 1280, -256,   512 },  // [5.0, -1.0, 2.0]
        { 0,    128,   1024 },  // [0.0, 0.5, 4.0]
        { 2560, 2560,  2560 }   // [10.0, 10.0, 10.0]
    };

    vec3_t results_sw[5];
    vec3_t results_hw[5];

    uint32_t sw_start, sw_end, sw_cycles;
    uint32_t hw_start, hw_end, hw_cycles;

    // ========================================================================
    // A. PURE SOFTWARE FIXED-POINT TRANSFORMATION
    // ========================================================================
    sw_start = MMIO_CYCLE_COUNT;

    for (int i = 0; i < 5; i++) {
        // Compute Row 0 (X Component)
        int32_t rx = (mat[0][0] * points[i].x) +
                     (mat[0][1] * points[i].y) +
                     (mat[0][2] * points[i].z) +
                     (mat[0][3] * Q8_8_ONE);
        results_sw[i].x = (int32_t)(rx >> 8);

        // Compute Row 1 (Y Component)
        int32_t ry = (mat[1][0] * points[i].x) +
                     (mat[1][1] * points[i].y) +
                     (mat[1][2] * points[i].z) +
                     (mat[1][3] * Q8_8_ONE);
        results_sw[i].y = (int32_t)(ry >> 8);

        // Compute Row 2 (Z Component)
        int32_t rz = (mat[2][0] * points[i].x) +
                     (mat[2][1] * points[i].y) +
                     (mat[2][2] * points[i].z) +
                     (mat[2][3] * Q8_8_ONE);
        results_sw[i].z = (int32_t)(rz >> 8);
    }

    sw_end = MMIO_CYCLE_COUNT;
    sw_cycles = sw_end - sw_start;

    // ========================================================================
    // B. HARDWARE ACCELERATED TRANSFORMATION
    // ========================================================================
    hw_start = MMIO_CYCLE_COUNT;

    // 1. Load the 3x4 Homogenous Transformation Matrix coefficients into ALU
    uint32_t row0_ab = PACK_Q8_8(mat[0][0], mat[0][1]);
    uint32_t row0_cd = PACK_Q8_8(mat[0][2], mat[0][3]);
    MATLOAD_ROW0(row0_ab, row0_cd);

    uint32_t row1_ab = PACK_Q8_8(mat[1][0], mat[1][1]);
    uint32_t row1_cd = PACK_Q8_8(mat[1][2], mat[1][3]);
    MATLOAD_ROW1(row1_ab, row1_cd);

    // Load Row 2 using macro
    uint32_t row2_ab = PACK_Q8_8(mat[2][0], mat[2][1]);
    uint32_t row2_cd = PACK_Q8_8(mat[2][2], mat[2][3]);
    MATLOAD_ROW2(row2_ab, row2_cd);

    // 2. Perform Batch transformations
    for (int i = 0; i < 5; i++) {
        uint32_t inputs_xy = PACK_Q8_8(points[i].x, points[i].y);
        uint32_t inputs_z1 = PACK_Q8_8(points[i].z, Q8_8_ONE);

        int32_t rx, ry, rz;
        MATMUL3D_X(rx, inputs_xy, inputs_z1);
        MATMUL3D_Y(ry, inputs_xy, inputs_z1);
        MATMUL3D_Z(rz, inputs_xy, inputs_z1);

        results_hw[i].x = (int32_t)rx;
        results_hw[i].y = (int32_t)ry;
        results_hw[i].z = (int32_t)rz;
    }

    hw_end = MMIO_CYCLE_COUNT;
    hw_cycles = hw_end - hw_start;

    // ========================================================================
    // C. VERIFICATION & REPORTING
    // ========================================================================
    print_str("--- Verification Output (SW vs HW) ---\n");
    int error_detected = 0;
    for (int i = 0; i < 5; i++) {
        print_str("Point "); print_num(i); print_str(":\n");
        print_str("  SW Output: ["); print_num(results_sw[i].x); print_str(", "); print_num(results_sw[i].y); print_str(", "); print_num(results_sw[i].z); print_str("]\n");
        print_str("  HW Output: ["); print_num(results_hw[i].x); print_str(", "); print_num(results_hw[i].y); print_str(", "); print_num(results_hw[i].z); print_str("]\n");
        
        if (results_sw[i].x != results_hw[i].x || 
            results_sw[i].y != results_hw[i].y || 
            results_sw[i].z != results_hw[i].z) {
            print_str("  ERROR: SW/HW mismatch!\n");
            error_detected = 1;
        } else {
            print_str("  Match: SUCCESS\n");
        }
    }

    print_str("\n--- Profiling & Performance Comparison ---\n");
    print_str("Software Loop Cycles: "); print_num(sw_cycles); print_str(" cycles\n");
    print_str("Hardware Accel Cycles: "); print_num(hw_cycles); print_str(" cycles (including matrix load)\n");
    
    uint32_t speedup_pct_rem;
    uint32_t speedup_pct = div_mod(sw_cycles * 100, hw_cycles, &speedup_pct_rem);
    uint32_t whole_rem;
    uint32_t whole = div_mod(speedup_pct, 100, &whole_rem);
    
    print_str("Speedup Factor: ");
    print_num(whole);
    print_char('.');
    if (whole_rem < 10) {
        print_char('0');
    }
    print_num(whole_rem);
    print_str("x\n");

    print_str("=========================================================\n");
    if (error_detected == 0) {
        print_str("TEST PASSED SUCCESSFULLY!\n");
        MMIO_SIM_EXIT = 1; // Success code
    } else {
        print_str("TEST FAILED!\n");
        MMIO_SIM_EXIT = 2; // Failure code
    }

    // Intentional infinite jump trap loop (j .)
    __asm__ volatile("1: j 1b");
    return 0;
}
