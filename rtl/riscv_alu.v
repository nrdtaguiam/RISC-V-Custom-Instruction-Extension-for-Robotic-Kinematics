// riscv_alu.v
// Standard 32-bit ALU for the RV32I instruction set.

`include "riscv_defs.v"

module riscv_alu (
    input  wire [3:0]  alu_control, // Control signal
    input  wire [31:0] op1,          // First operand
    input  wire [31:0] op2,          // Second operand
    output reg  [31:0] alu_result,   // Computation result
    output wire        zero          // High if result is zero
);

    wire signed [31:0] s_op1 = op1;
    wire signed [31:0] s_op2 = op2;

    always @(*) begin
        case (alu_control)
            `ALU_ADD:    alu_result = op1 + op2;
            `ALU_SUB:    alu_result = op1 - op2;
            `ALU_SLL:    alu_result = op1 << op2[4:0];
            `ALU_SLT:    alu_result = (s_op1 < s_op2) ? 32'd1 : 32'd0;
            `ALU_SLTU:   alu_result = (op1 < op2) ? 32'd1 : 32'd0;
            `ALU_XOR:    alu_result = op1 ^ op2;
            `ALU_SRL:    alu_result = op1 >> op2[4:0];
            `ALU_SRA:    alu_result = s_op1 >>> op2[4:0];
            `ALU_OR:     alu_result = op1 | op2;
            `ALU_AND:    alu_result = op1 & op2;
            `ALU_COPY2:  alu_result = op2;
            default:     alu_result = 32'd0;
        endcase
    end

    assign zero = (alu_result == 32'd0);

endmodule
