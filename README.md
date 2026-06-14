# RISC-V 3D Robotic Kinematics Accelerator (`riscv_3d_kinematics_accel`)

This repository contains a **5-stage pipelined RV32I RISC-V processor core** extended with a custom hardware instruction set (`MATLOAD` and `MATMUL3D`) optimized for high-speed 3D robotic coordinate transformations, joint kinematics, and homogeneous vector operations.

The design targets a **standard 180nm CMOS library** and is optimized for synthesis on the open-source **SkyWater 130nm PDK** using Yosys.

---

## 1. Custom Instruction Extension (Robotic Kinematics)

Robotic kinematic operations require repeating matrix-vector multiplications of the form:
$$\vec{V}_{out} = \mathbf{M}_{3 \times 4} \cdot \vec{V}_{in, homogeneous}$$

To accelerate this, we define a set of custom RISC-V instructions mapped to the **Custom-0 Opcode space (0x0B)** in an R-type instruction format:

### A. Matrix Loading (`MATLOAD`)
Loads the $3 \times 4$ transformation matrix into internal ALU buffers row by row. Each element is represented as a **16-bit signed Q8.8 fixed-point number**.
*   **MATLOAD_ROW0 rs1, rs2**: Loads row 0 coefficients: $rs1 = [A_0, B_0]$, $rs2 = [C_0, D_0]$.
*   **MATLOAD_ROW1 rs1, rs2**: Loads row 1 coefficients: $rs1 = [A_1, B_1]$, $rs2 = [C_1, D_1]$.
*   **MATLOAD_ROW2 rs1, rs2**: Loads row 2 coefficients: $rs1 = [A_2, B_2]$, $rs2 = [C_2, D_2]$.
*   **Format**: Custom-0 Opcode (`0x0B`), `funct3` selects row (`000` for Row 0, `001` for Row 1, `010` for Row 2).

### B. Matrix-Vector Transform (`MATMUL3D`)
Performs a parallel dot product for a single row:
$$Result = (A \cdot X) + (B \cdot Y) + (C \cdot Z) + (D \cdot 1.0)$$
*   **MATMUL3D_X rd, rs1, rs2**: Computes X coordinate: $rs1 = [X, Y]$, $rs2 = [Z, 1.0]$. Writes result component to $rd$.
*   **MATMUL3D_Y rd, rs1, rs2**: Computes Y coordinate: $rs1 = [X, Y]$, $rs2 = [Z, 1.0]$. Writes result component to $rd$.
*   **MATMUL3D_Z rd, rs1, rs2**: Computes Z coordinate: $rs1 = [X, Y]$, $rs2 = [Z, 1.0]$. Writes result component to $rd$.
*   **Format**: Custom-0 Opcode (`0x0B`), `funct3` selects row calculation (`100` for Row 0, `101` for Row 1, `110` for Row 2).

---

## 2. Hardware Pipeline Architecture

The custom arithmetic block is integrated directly into the **Execution (EX) stage** of the CPU:

1.  **Parallel Multiplier Array**: Four 16x16-bit signed multipliers compute the terms $A \cdot X$, $B \cdot Y$, $C \cdot Z$, and $D \cdot 1.0$ in parallel in Cycle 1 (EX1 stage).
2.  **Synchronous Adder Tree**: Adds the four products in Cycle 2 (EX2 stage) and arithmetically shifts the 32-bit result by 8 bits (`>>> 8`) to scale back from Q16.16 to Q8.8.
3.  **Deterministic Hazard Stall**: Since the execution latency is 2 cycles, the **Hazard Detection Unit** halts the IF and ID stages for exactly 1 cycle when a `MATMUL3D` instruction is decoded. The instruction stays in the EX stage for 2 cycles, after which its output is written back to the register file normally, preventing any data corruption without needing complex out-of-order execution logic.

---

## 3. Directory Layout

```
/riscv_3d_kinematics_accel
├── /rtl                         # synthesizable behavioral Verilog HDL
│   ├── riscv_defs.v             # Opcode definitions and ALU control mapping
│   ├── kinematics_accel.v       # Parallel multipliers & adder-tree pipeline
│   ├── riscv_alu.v              # Standard RV32I arithmetic-logic unit
│   ├── riscv_regfile.v          # Dual-port general purpose register file
│   ├── riscv_hazard.v           # Forwarding & pipeline stall hazard detector
│   └── riscv_core.v             # Top-level 5-stage CPU Core
├── /firmware                    # Verification payload & compiler wrappers
│   ├── kinematics.h             # Custom instruction GCC inline assembly macros
│   ├── main.c                   # Profiling payload (SW vs HW comparisons)
│   ├── crt0.S                   # CPU boot startup code (stack, register reset)
│   ├── link.ld                  # ROM/RAM memory map linker script
│   └── bin2hex.py               # Post-compile binary-to-hex converter utility
└── /tb                          # Icarus Verilog simulation files
    ├── tb_riscv_core.v          # Simulation testbench with MMIO UART printer
    └── Makefile                 # Compilation & simulation make target wrapper
```

---

## 4. Compile & Verify Simulation

### Prerequisites
*   **WSL (Windows Subsystem for Linux)** environment.
*   WSL packages: `iverilog`, `vvp`, `gcc-riscv64-unknown-elf` or `gcc-riscv32-unknown-elf`.
*   Host Python 3 installed and on PATH.

### Steps to Run
Navigate to the `/tb` directory and run the simulation using the host Makefile (configured to automatically invoke the WSL compiler and simulation tools):

```bash
cd tb
make
```

### Clean generated files:
```bash
make clean
```

---

## 5. SkyWater 130nm PDK Synthesis
The RTL code is fully synthesizable and conforms to standard ASIC design rules:
*   No latch structures or asynchronous resets.
*   Single clock edge execution (`posedge clk`).
*   Optimized for Yosys mapping to standard cell cells like `sky130_fd_sc_hd__dfxtp_1` for registers and `sky130_fd_sc_hd__mux2_1` for multiplexers.
