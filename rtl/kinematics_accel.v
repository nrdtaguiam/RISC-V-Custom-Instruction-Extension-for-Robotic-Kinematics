// kinematics_accel.v
// Custom hardware accelerator for 3D Robotic Kinematics coordinate transformations.
// Implements Custom RISC-V MATMUL3D and MATLOAD operations using 4 parallel 
// pipelined 16-bit signed fixed-point multipliers and a synchronous adder tree.

`include "riscv_defs.v"

module kinematics_accel (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid,        // High when a custom command is in EX stage
    input  wire [2:0]  op_type,      // Matches funct3 of custom instruction
    input  wire [31:0] rs1_val,      // For MATMUL3D: [X (31:16), Y (15:0)], For MATLOAD: [A, B]
    input  wire [31:0] rs2_val,      // For MATMUL3D: [Z (31:16), 1 (15:0)], For MATLOAD: [C, D]
    output reg  [31:0] result,       // Computed 32-bit vector component (Q8.8 format)
    output reg         ready         // Asserted when computation completes (after 2 cycles)
);

    // ------------------------------------------------------------------------
    // 1. Matrix Storage Buffer (3x4 Matrix, each element 16-bit Q8.8 format)
    // ------------------------------------------------------------------------
    // Row 0 coefficients (for X transformation)
    reg signed [15:0] m00, m01, m02, m03;
    // Row 1 coefficients (for Y transformation)
    reg signed [15:0] m10, m11, m12, m13;
    // Row 2 coefficients (for Z transformation)
    reg signed [15:0] m20, m21, m22, m23;

    // ------------------------------------------------------------------------
    // 2. Control State Machine & Row Loading
    // ------------------------------------------------------------------------
    reg [1:0] cycle_cnt;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m00 <= 16'd0; m01 <= 16'd0; m02 <= 16'd0; m03 <= 16'd0;
            m10 <= 16'd0; m11 <= 16'd0; m12 <= 16'd0; m13 <= 16'd0;
            m20 <= 16'd0; m21 <= 16'd0; m22 <= 16'd0; m23 <= 16'd0;
            cycle_cnt <= 2'd0;
            ready     <= 1'b0;
        end else begin
            if (valid) begin
                case (op_type)
                    // Write / Load Matrix Rows (1-cycle operation)
                    `F3_MATLOAD_R0: begin
                        m00   <= rs1_val[31:16];
                        m01   <= rs1_val[15:0];
                        m02   <= rs2_val[31:16];
                        m03   <= rs2_val[15:0];
                        ready <= 1'b1;
                    end
                    `F3_MATLOAD_R1: begin
                        m10   <= rs1_val[31:16];
                        m11   <= rs1_val[15:0];
                        m12   <= rs2_val[31:16];
                        m13   <= rs2_val[15:0];
                        ready <= 1'b1;
                    end
                    `F3_MATLOAD_R2: begin
                        m20   <= rs1_val[31:16];
                        m21   <= rs1_val[15:0];
                        m22   <= rs2_val[31:16];
                        m23   <= rs2_val[15:0];
                        ready <= 1'b1;
                    end
                    
                    // Multiply-Accumulate Rows (2-cycle pipelined operation)
                    `F3_MATMUL3D_R0, `F3_MATMUL3D_R1, `F3_MATMUL3D_R2: begin
                        if (cycle_cnt == 2'd0) begin
                            cycle_cnt <= 2'd1;
                            ready     <= 1'b1;
                        end else begin
                            cycle_cnt <= 2'd0;
                            ready     <= 1'b0;
                        end
                    end
                    default: begin
                        cycle_cnt <= 2'd0;
                        ready     <= 1'b0;
                    end
                endcase
            end else begin
                cycle_cnt <= 2'd0;
                ready     <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 3. Pipelined Multiplier Stage (EX1)
    // ------------------------------------------------------------------------
    // Operands selection
    reg signed [15:0] coeff0, coeff1, coeff2, coeff3;
    
    always @(*) begin
        case (op_type)
            `F3_MATMUL3D_R0: begin
                coeff0 = m00; coeff1 = m01; coeff2 = m02; coeff3 = m03;
            end
            `F3_MATMUL3D_R1: begin
                coeff0 = m10; coeff1 = m11; coeff2 = m12; coeff3 = m13;
            end
            `F3_MATMUL3D_R2: begin
                coeff0 = m20; coeff1 = m21; coeff2 = m22; coeff3 = m23;
            end
            default: begin
                coeff0 = 16'sd0; coeff1 = 16'sd0; coeff2 = 16'sd0; coeff3 = 16'sd0;
            end
        endcase
    end

    // Multiplier inputs
    wire signed [15:0] in_x = rs1_val[31:16];
    wire signed [15:0] in_y = rs1_val[15:0];
    wire signed [15:0] in_z = rs2_val[31:16];
    wire signed [15:0] in_w = rs2_val[15:0]; // Homogeneous coordinate (normally 1.0 = 0x0100)

    // Pipeline registers for products
    reg signed [31:0] prod0, prod1, prod2, prod3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod0 <= 32'sd0;
            prod1 <= 32'sd0;
            prod2 <= 32'sd0;
            prod3 <= 32'sd0;
        end else if (valid && (op_type[2] == 1'b1) && (cycle_cnt == 2'd0)) begin // active during MATMUL3D first cycle
            prod0 <= coeff0 * in_x;
            prod1 <= coeff1 * in_y;
            prod2 <= coeff2 * in_z;
            prod3 <= coeff3 * in_w;
        end
    end

    // ------------------------------------------------------------------------
    // 4. Combinational Adder Tree Stage
    // ------------------------------------------------------------------------
    wire signed [33:0] comb_sum = $signed(prod0) + $signed(prod1) + $signed(prod2) + $signed(prod3);

    always @(*) begin
        // Q16.16 format summation. Scale back by shifting right by 8 to yield Q8.8
        // We use arithmetic shift (>>>) to preserve the sign.
        result = $signed(comb_sum) >>> 8;
    end

endmodule
