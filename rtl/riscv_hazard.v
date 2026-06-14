// riscv_hazard.v
// Hazard Detection and Forwarding Unit for 5-stage Pipelined RISC-V Core.

module riscv_hazard (
    // Forwarding inputs from ID/EX
    input  wire [4:0]  id_ex_rs1,
    input  wire [4:0]  id_ex_rs2,
    
    // Forwarding inputs from EX/MEM and MEM/WB
    input  wire [4:0]  ex_mem_rd,
    input  wire        ex_mem_regwrite,
    input  wire [4:0]  mem_wb_rd,
    input  wire        mem_wb_regwrite,
    
    // Stall detection inputs
    input  wire [4:0]  if_id_rs1,
    input  wire [4:0]  if_id_rs2,
    input  wire [4:0]  id_ex_rd,
    input  wire        id_ex_memread,  // Asserted if instruction in EX is a Load
    input  wire        stall_from_ex,  // Stall request from EX stage (for multi-cycle custom instruction)

    // Branch hazard inputs
    input  wire        id_ex_regwrite,  // Asserted if instruction in EX writes to regfile
    input  wire        ex_mem_memread,   // Asserted if instruction in MEM is a Load
    input  wire        is_branch_or_jalr,// Asserted if instruction in ID is branch/jalr

    // Forwarding outputs to EX stage
    output reg  [1:0]  forward_a,
    output reg  [1:0]  forward_b,

    // Stall outputs to IF, ID, and Pipeline Registers
    output wire        stall_if,
    output wire        stall_id,
    output wire        flush_ex
);

    // ------------------------------------------------------------------------
    // 1. Forwarding Logic
    // ------------------------------------------------------------------------
    always @(*) begin
        // Operand A Forwarding
        if (ex_mem_regwrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs1)) begin
            forward_a = 2'b10; // Forward from MEM stage (ALU result)
        end else if (mem_wb_regwrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs1)) begin
            forward_a = 2'b01; // Forward from WB stage (Register writeback data)
        end else begin
            forward_a = 2'b00; // No forwarding (use register output)
        end

        // Operand B Forwarding
        if (ex_mem_regwrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs2)) begin
            forward_b = 2'b10; // Forward from MEM stage (ALU result)
        end else if (mem_wb_regwrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs2)) begin
            forward_b = 2'b01; // Forward from WB stage (Register writeback data)
        end else begin
            forward_b = 2'b00; // No forwarding (use register output)
        end
    end

    // ------------------------------------------------------------------------
    // 2. Stall & Flush Detection
    // ------------------------------------------------------------------------
    // Load-Use Hazard:
    // If the instruction in EX is a load (e.g. LW) and its target rd matches
    // rs1 or rs2 of the instruction in ID stage, we must stall for 1 cycle.
    wire load_use_stall = id_ex_memread && (id_ex_rd != 5'd0) && ((id_ex_rd == if_id_rs1) || (id_ex_rd == if_id_rs2));

    // Stall for branch hazards:
    // If a branch/jump instruction is in the ID stage, and:
    // a) The instruction in EX stage writes to a branch source register
    // b) The instruction in MEM stage is a load and writes to a branch source register
    wire branch_stall = is_branch_or_jalr && (
        (id_ex_regwrite && (id_ex_rd != 5'd0) && ((id_ex_rd == if_id_rs1) || (id_ex_rd == if_id_rs2))) ||
        (ex_mem_memread && (ex_mem_rd != 5'd0) && ((ex_mem_rd == if_id_rs1) || (ex_mem_rd == if_id_rs2)))
    );

    // Combine standard stalls with custom multi-cycle execution stalls
    assign stall_if = load_use_stall || branch_stall || stall_from_ex;
    assign stall_id = load_use_stall || branch_stall || stall_from_ex;
    
    // flush_ex (flushing EX to NOP) should only be active for standard stalls
    // (load-use and branch stalls). Stalls from EX itself should not flush EX.
    assign flush_ex = load_use_stall || branch_stall;

endmodule
