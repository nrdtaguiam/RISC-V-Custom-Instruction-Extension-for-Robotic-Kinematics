// riscv_regfile.v
// Dual-port register file for RV32I. Register x0 is hardwired to 0.

module riscv_regfile (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        we,           // Write enable
    input  wire [4:0]  rs1,          // Read register 1
    input  wire [4:0]  rs2,          // Read register 2
    input  wire [4:0]  rd,           // Write register
    input  wire [31:0] wd,           // Write data
    output wire [31:0] rd1,          // Read data 1
    output wire [31:0] rd2           // Read data 2
);

    reg [31:0] regs [0:31];
    integer i;

    // Dual asynchronous read with write-through (bypass)
    assign rd1 = (rs1 == 5'd0) ? 32'd0 : ((rs1 == rd && we) ? wd : regs[rs1]);
    assign rd2 = (rs2 == 5'd0) ? 32'd0 : ((rs2 == rd && we) ? wd : regs[rs2]);

    // Synchronous write (active high)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset register values to 0
            for (i = 0; i < 32; i = i + 1) begin
                regs[i] <= 32'd0;
            end
        end else begin
            if (we && (rd != 5'd0)) begin
                regs[rd] <= wd;
            end
        end
    end

endmodule
