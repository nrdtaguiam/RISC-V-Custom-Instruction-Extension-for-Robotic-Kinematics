// riscv_defs.v
// Defines for the RV32I Core with Custom 3D Kinematics Extension

`ifndef RISCV_DEFS_V
`define RISCV_DEFS_V

// Standard Opcodes
`define OP_LUI      7'b0110111
`define OP_AUIPC    7'b0010111
`define OP_JAL      7'b1101111
`define OP_JALR     7'b1100111
`define OP_BRANCH   7'b1100011
`define OP_LOAD     7'b0000011
`define OP_STORE    7'b0100011
`define OP_OP_IMM   7'b0010011
`define OP_OP       7'b0110011
`define OP_CUSTOM0  7'b0001011   // Custom-0 Opcode space (0x0B)

// funct3 codes for custom-0 kinematics extension
`define F3_MATLOAD_R0  3'b000   // Load Row 0 of 3x4 Matrix: rs1=[A,B], rs2=[C,D]
`define F3_MATLOAD_R1  3'b001   // Load Row 1 of 3x4 Matrix: rs1=[A,B], rs2=[C,D]
`define F3_MATLOAD_R2  3'b010   // Load Row 2 of 3x4 Matrix: rs1=[A,B], rs2=[C,D]

`define F3_MATMUL3D_R0 3'b100   // Compute Row 0: X_out = (A0*X) + (B0*Y) + (C0*Z) + D0
`define F3_MATMUL3D_R1 3'b101   // Compute Row 1: Y_out = (A1*X) + (B1*Y) + (C1*Z) + D1
`define F3_MATMUL3D_R2 3'b110   // Compute Row 2: Z_out = (A2*X) + (B2*Y) + (C2*Z) + D2

// Standard funct3 codes
`define F3_ADD_SUB  3'b000
`define F3_SLL      3'b001
`define F3_SLT      3'b010
`define F3_SLTU     3'b011
`define F3_XOR      3'b100
`define F3_SRL_SRA  3'b101
`define F3_OR       3'b110
`define F3_AND      3'b111

`define F3_BEQ      3'b000
`define F3_BNE      3'b001
`define F3_BLT      3'b100
`define F3_BGE      3'b101
`define F3_BLTU     3'b110
`define F3_BGEU     3'b111

`define F3_LB       3'b000
`define F3_LH       3'b001
`define F3_LW       3'b010
`define F3_LBU      3'b100
`define F3_LHU      3'b101

`define F3_SB       3'b000
`define F3_SH       3'b001
`define F3_SW       3'b010

// ALU Control Signals
`define ALU_ADD     4'b0000
`define ALU_SUB     4'b0001
`define ALU_SLL     4'b0010
`define ALU_SLT     4'b0011
`define ALU_SLTU    4'b0100
`define ALU_XOR     4'b0101
`define ALU_SRL     4'b0110
`define ALU_SRA     4'b0111
`define ALU_OR      4'b1000
`define ALU_AND     4'b1001
`define ALU_COPY2   4'b1010  // copy operand 2 (for LUI)
`define ALU_CUSTOM  4'b1111  // custom instruction mode

`endif // RISCV_DEFS_V
