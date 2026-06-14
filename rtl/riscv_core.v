// riscv_core.v
// Main 5-stage Pipelined RISC-V Core with Custom Kinematics Instruction Extension.

`include "riscv_defs.v"

module riscv_core (
    input  wire        clk,
    input  wire        rst_n,
    
    // Instruction Memory Interface
    output wire [31:0] imem_addr,
    input  wire [31:0] imem_data,
    
    // Data Memory Interface
    output wire [31:0] dmem_addr,
    output wire [31:0] dmem_write_data,
    input  wire [31:0] dmem_read_data,
    output wire        dmem_we,
    output wire        dmem_re
);

    // ========================================================================
    // Pipeline Registers Definitions
    // ========================================================================
    
    // --- IF/ID Pipeline Register ---
    reg [31:0] if_id_pc;
    reg [31:0] if_id_instr;

    // --- ID/EX Pipeline Register ---
    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_rd1;
    reg [31:0] id_ex_rd2;
    reg [31:0] id_ex_imm;
    reg [4:0]  id_ex_rs1;
    reg [4:0]  id_ex_rs2;
    reg [4:0]  id_ex_rd;
    // Control signals
    reg        id_ex_regwrite;
    reg        id_ex_memread;
    reg        id_ex_memwrite;
    reg        id_ex_memtoreg;
    reg        id_ex_alusrc;
    reg [3:0]  id_ex_alu_control;
    reg        id_ex_is_custom0;
    reg [2:0]  id_ex_funct3;
    reg        id_ex_is_matmul;
    reg        id_ex_is_matload;

    // --- EX/MEM Pipeline Register ---
    reg [31:0] ex_mem_alu_result;
    reg [31:0] ex_mem_write_data;
    reg [4:0]  ex_mem_rd;
    // Control signals
    reg        ex_mem_regwrite;
    reg        ex_mem_memread;
    reg        ex_mem_memwrite;
    reg        ex_mem_memtoreg;

    // --- MEM/WB Pipeline Register ---
    reg [31:0] mem_wb_read_data;
    reg [31:0] mem_wb_alu_result;
    reg [4:0]  mem_wb_rd;
    // Control signals
    reg        mem_wb_regwrite;
    reg        mem_wb_memtoreg;

    // ========================================================================
    // 1. INSTRUCTION FETCH (IF) STAGE
    // ========================================================================
    reg  [31:0] pc;
    wire [31:0] next_pc;
    wire [31:0] pc_plus_4 = pc + 32'd4;
    
    // Jump / Branch resolution signals from EX
    wire        take_branch;
    wire [31:0] branch_target;

    assign next_pc = take_branch ? branch_target : pc_plus_4;
    assign imem_addr = pc;

    // IF Hazard controls
    wire stall_if;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 32'd0;
        end else if (!stall_if) begin
            pc <= next_pc;
        end
    end

    // IF/ID Register Write
    wire stall_id;
    wire flush_id = take_branch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            if_id_pc    <= 32'd0;
            if_id_instr <= 32'h00000013; // NOP (addi x0, x0, 0)
        end else if (flush_id) begin
            if_id_pc    <= 32'd0;
            if_id_instr <= 32'h00000013; // NOP
        end else if (!stall_id) begin
            if_id_pc    <= pc;
            if_id_instr <= imem_data;
        end
    end

    // ========================================================================
    // 2. INSTRUCTION DECODE (ID) STAGE
    // ========================================================================
    wire [31:0] instr = if_id_instr;
    wire [4:0]  rs1 = instr[19:15];
    wire [4:0]  rs2 = instr[24:20];
    wire [4:0]  rd  = instr[11:7];
    wire [6:0]  opcode = instr[6:0];
    wire [2:0]  funct3 = instr[14:12];
    wire [6:0]  funct7 = instr[31:25];
    wire        is_branch_or_jalr = (opcode == `OP_BRANCH) || (opcode == `OP_JALR);

    // Register File Instantiation
    wire [31:0] reg_rd1, reg_rd2;
    wire        mem_wb_regwrite_signal;
    wire [4:0]  mem_wb_rd_signal;
    wire [31:0] writeback_data;

    riscv_regfile rf (
        .clk   (clk),
        .rst_n (rst_n),
        .we    (mem_wb_regwrite_signal),
        .rs1   (rs1),
        .rs2   (rs2),
        .rd    (mem_wb_rd_signal),
        .wd    (writeback_data),
        .rd1   (reg_rd1),
        .rd2   (reg_rd2)
    );

    // Sign extension for immediate values
    reg [31:0] imm;
    always @(*) begin
        case (opcode)
            `OP_LUI, `OP_AUIPC: imm = {instr[31:12], 12'd0}; // U-type
            `OP_JAL:            imm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; // J-type
            `OP_JALR, `OP_LOAD, `OP_OP_IMM: imm = {{20{instr[31]}}, instr[31:20]}; // I-type
            `OP_STORE:          imm = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // S-type
            `OP_BRANCH:         imm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type
            default:            imm = 32'd0;
        endcase
    end

    // Control Path Decoding
    reg       ctrl_regwrite;
    reg       ctrl_memread;
    reg       ctrl_memwrite;
    reg       ctrl_memtoreg;
    reg       ctrl_alusrc;
    reg [3:0] ctrl_alu_control;
    reg       ctrl_is_custom0;
    reg       ctrl_is_matmul;
    reg       ctrl_is_matload;
    reg       ctrl_is_link;
    reg       ctrl_use_pc;

    always @(*) begin
        // Defaults
        ctrl_regwrite    = 1'b0;
        ctrl_memread     = 1'b0;
        ctrl_memwrite    = 1'b0;
        ctrl_memtoreg    = 1'b0;
        ctrl_alusrc      = 1'b0;
        ctrl_alu_control = `ALU_ADD;
        ctrl_is_custom0  = 1'b0;
        ctrl_is_matmul   = 1'b0;
        ctrl_is_matload  = 1'b0;
        ctrl_is_link     = 1'b0;
        ctrl_use_pc      = 1'b0;

        case (opcode)
            `OP_LUI: begin
                ctrl_regwrite    = 1'b1;
                ctrl_alusrc      = 1'b1;
                ctrl_alu_control = `ALU_COPY2;
            end
            `OP_AUIPC: begin
                ctrl_regwrite    = 1'b1;
                ctrl_alusrc      = 1'b1;
                ctrl_alu_control = `ALU_ADD;
                ctrl_use_pc      = 1'b1;
            end
            `OP_JAL: begin
                ctrl_regwrite    = 1'b1;
                ctrl_is_link     = 1'b1;
            end
            `OP_JALR: begin
                ctrl_regwrite    = 1'b1;
                ctrl_alusrc      = 1'b1;
                ctrl_is_link     = 1'b1;
            end
            `OP_BRANCH: begin
                ctrl_alu_control = `ALU_SUB;
            end
            `OP_LOAD: begin
                ctrl_regwrite    = 1'b1;
                ctrl_memread     = 1'b1;
                ctrl_memtoreg    = 1'b1;
                ctrl_alusrc      = 1'b1;
            end
            `OP_STORE: begin
                ctrl_memwrite    = 1'b1;
                ctrl_alusrc      = 1'b1;
            end
            `OP_OP_IMM: begin
                ctrl_regwrite    = 1'b1;
                ctrl_alusrc      = 1'b1;
                case (funct3)
                    `F3_ADD_SUB: ctrl_alu_control = `ALU_ADD;
                    `F3_SLL:     ctrl_alu_control = `ALU_SLL;
                    `F3_SLT:     ctrl_alu_control = `ALU_SLT;
                    `F3_SLTU:    ctrl_alu_control = `ALU_SLTU;
                    `F3_XOR:     ctrl_alu_control = `ALU_XOR;
                    `F3_SRL_SRA: ctrl_alu_control = (funct7[5]) ? `ALU_SRA : `ALU_SRL;
                    `F3_OR:      ctrl_alu_control = `ALU_OR;
                    `F3_AND:     ctrl_alu_control = `ALU_AND;
                    default:     ctrl_alu_control = `ALU_ADD;
                endcase
            end
            `OP_OP: begin
                ctrl_regwrite    = 1'b1;
                case (funct3)
                    `F3_ADD_SUB: ctrl_alu_control = (funct7[5]) ? `ALU_SUB : `ALU_ADD;
                    `F3_SLL:     ctrl_alu_control = `ALU_SLL;
                    `F3_SLT:     ctrl_alu_control = `ALU_SLT;
                    `F3_SLTU:    ctrl_alu_control = `ALU_SLTU;
                    `F3_XOR:     ctrl_alu_control = `ALU_XOR;
                    `F3_SRL_SRA: ctrl_alu_control = (funct7[5]) ? `ALU_SRA : `ALU_SRL;
                    `F3_OR:      ctrl_alu_control = `ALU_OR;
                    `F3_AND:     ctrl_alu_control = `ALU_AND;
                    default:     ctrl_alu_control = `ALU_ADD;
                endcase
            end
            `OP_CUSTOM0: begin
                ctrl_is_custom0  = 1'b1;
                ctrl_alu_control = `ALU_CUSTOM;
                if (funct3[2] == 1'b0) begin
                    // Load rows of 3x4 Matrix (no register writeback)
                    ctrl_regwrite   = 1'b0;
                    ctrl_is_matload = 1'b1;
                end else begin
                    // 3D vector transformation row calculation (writes to rd)
                    ctrl_regwrite   = 1'b1;
                    ctrl_is_matmul  = 1'b1;
                end
            end
            default: ;
        endcase
    end

    reg        id_ex_is_link;
    reg        id_ex_use_pc;

    // ID/EX Pipeline Register Write
    wire flush_ex;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_ex_pc          <= 32'd0;
            id_ex_rd1         <= 32'd0;
            id_ex_rd2         <= 32'd0;
            id_ex_imm         <= 32'd0;
            id_ex_rs1         <= 5'd0;
            id_ex_rs2         <= 5'd0;
            id_ex_rd          <= 5'd0;
            id_ex_regwrite    <= 1'b0;
            id_ex_memread     <= 1'b0;
            id_ex_memwrite    <= 1'b0;
            id_ex_memtoreg    <= 1'b0;
            id_ex_alusrc      <= 1'b0;
            id_ex_alu_control <= `ALU_ADD;
            id_ex_is_custom0  <= 1'b0;
            id_ex_funct3      <= 3'd0;
            id_ex_is_matmul   <= 1'b0;
            id_ex_is_matload  <= 1'b0;
            id_ex_is_link     <= 1'b0;
            id_ex_use_pc      <= 1'b0;
        end else if (flush_ex) begin
            // Insert NOP
            id_ex_pc          <= 32'd0;
            id_ex_rd1         <= 32'd0;
            id_ex_rd2         <= 32'd0;
            id_ex_imm         <= 32'd0;
            id_ex_rs1         <= 5'd0;
            id_ex_rs2         <= 5'd0;
            id_ex_rd          <= 5'd0;
            id_ex_regwrite    <= 1'b0;
            id_ex_memread     <= 1'b0;
            id_ex_memwrite    <= 1'b0;
            id_ex_memtoreg    <= 1'b0;
            id_ex_alusrc      <= 1'b0;
            id_ex_alu_control <= `ALU_ADD;
            id_ex_is_custom0  <= 1'b0;
            id_ex_funct3      <= 3'd0;
            id_ex_is_matmul   <= 1'b0;
            id_ex_is_matload  <= 1'b0;
            id_ex_is_link     <= 1'b0;
            id_ex_use_pc      <= 1'b0;
        end else if (!stall_from_ex) begin
            id_ex_pc          <= if_id_pc;
            id_ex_rd1         <= reg_rd1;
            id_ex_rd2         <= reg_rd2;
            id_ex_imm         <= imm;
            id_ex_rs1         <= rs1;
            id_ex_rs2         <= rs2;
            id_ex_rd          <= rd;
            id_ex_regwrite    <= ctrl_regwrite;
            id_ex_memread     <= ctrl_memread;
            id_ex_memwrite    <= ctrl_memwrite;
            id_ex_memtoreg    <= ctrl_memtoreg;
            id_ex_alusrc      <= ctrl_alusrc;
            id_ex_alu_control <= ctrl_alu_control;
            id_ex_is_custom0  <= ctrl_is_custom0;
            id_ex_funct3      <= funct3;
            id_ex_is_matmul   <= ctrl_is_matmul;
            id_ex_is_matload  <= ctrl_is_matload;
            id_ex_is_link     <= ctrl_is_link;
            id_ex_use_pc      <= ctrl_use_pc;
        end
    end

    // ========================================================================
    // 3. EXECUTION (EX) STAGE & ACCELERATOR
    // ========================================================================
    wire [1:0] forward_a, forward_b;
    reg  [31:0] mux_a_out, mux_b_out;

    // Operand A Forwarding Mux
    always @(*) begin
        case (forward_a)
            2'b10:   mux_a_out = ex_mem_alu_result;
            2'b01:   mux_a_out = writeback_data;
            default: mux_a_out = id_ex_rd1;
        endcase
    end

    // Operand B Forwarding Mux
    always @(*) begin
        case (forward_b)
            2'b10:   mux_b_out = ex_mem_alu_result;
            2'b01:   mux_b_out = writeback_data;
            default: mux_b_out = id_ex_rd2;
        endcase
    end

    // ALU input B selection (forwarded rs2 value vs immediate)
    wire [31:0] alu_op2 = id_ex_alusrc ? id_ex_imm : mux_b_out;

    // Standard ALU Instance
    wire [31:0] alu_result_val;
    wire        alu_zero;
    
    riscv_alu alu (
        .alu_control (id_ex_alu_control),
        .op1         (id_ex_use_pc ? id_ex_pc : mux_a_out),
        .op2         (alu_op2),
        .alu_result  (alu_result_val),
        .zero        (alu_zero)
    );

    // Forwarding to Branch Comparator in ID Stage
    wire [31:0] branch_val1;
    wire [31:0] branch_val2;
    
    assign branch_val1 = (ex_mem_regwrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == rs1)) ? ex_mem_alu_result :
                         (mem_wb_regwrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == rs1)) ? writeback_data :
                         reg_rd1;

    assign branch_val2 = (ex_mem_regwrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == rs2)) ? ex_mem_alu_result :
                         (mem_wb_regwrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == rs2)) ? writeback_data :
                         reg_rd2;

    // Branch Condition Check
    assign take_branch = (if_id_instr[6:0] == `OP_BRANCH && 
                           ((if_id_instr[14:12] == `F3_BEQ  && branch_val1 == branch_val2)  ||
                            (if_id_instr[14:12] == `F3_BNE  && branch_val1 != branch_val2)  ||
                            (if_id_instr[14:12] == `F3_BLT  && $signed(branch_val1) <  $signed(branch_val2)) ||
                            (if_id_instr[14:12] == `F3_BGE  && $signed(branch_val1) >= $signed(branch_val2)) ||
                            (if_id_instr[14:12] == `F3_BLTU && branch_val1 <  branch_val2)  ||
                            (if_id_instr[14:12] == `F3_BGEU && branch_val1 >= branch_val2))) ||
                         (if_id_instr[6:0] == `OP_JAL) ||
                         (if_id_instr[6:0] == `OP_JALR);

    assign branch_target = (if_id_instr[6:0] == `OP_JALR) ? (branch_val1 + imm) : (if_id_pc + imm);

    // --- Kinematics Accelerator Integration ---
    // Deterministic 2-cycle latency pipeline stall logic
    reg  matmul_state;
    wire stall_from_ex;
    
    // Generates a 1-cycle stall during the first cycle of MATMUL3D.
    // MATLOAD does not require a stall as it is a 1-cycle write.
    assign stall_from_ex = id_ex_is_matmul && (matmul_state == 1'b0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            matmul_state <= 1'b0;
        end else begin
            if (id_ex_is_matmul) begin
                matmul_state <= ~matmul_state; // toggle state: 0 -> 1 -> 0
            end else begin
                matmul_state <= 1'b0;
            end
        end
    end

    // Kinematics block instantiation
    wire [31:0] kinematics_result;
    wire        kinematics_ready;
    
    kinematics_accel accel (
        .clk      (clk),
        .rst_n    (rst_n),
        .valid    (id_ex_is_custom0),
        .op_type  (id_ex_funct3),
        .rs1_val  (mux_a_out),
        .rs2_val  (mux_b_out),
        .result   (kinematics_result),
        .ready    (kinematics_ready)
    );

    // Select Final Execution Result (ALU vs custom kinematics block vs Link PC+4)
    wire [31:0] link_pc_plus_4 = id_ex_pc + 32'd4;
    wire [31:0] final_ex_result = id_ex_is_link    ? link_pc_plus_4 :
                                  id_ex_is_custom0 ? kinematics_result : alu_result_val;

    // EX/MEM Pipeline Register Write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_mem_alu_result <= 32'd0;
            ex_mem_write_data <= 32'd0;
            ex_mem_rd         <= 5'd0;
            ex_mem_regwrite   <= 1'b0;
            ex_mem_memread    <= 1'b0;
            ex_mem_memwrite   <= 1'b0;
            ex_mem_memtoreg   <= 1'b0;
        end else if (stall_from_ex) begin
            ex_mem_alu_result <= 32'd0;
            ex_mem_write_data <= 32'd0;
            ex_mem_rd         <= 5'd0;
            ex_mem_regwrite   <= 1'b0;
            ex_mem_memread    <= 1'b0;
            ex_mem_memwrite   <= 1'b0;
            ex_mem_memtoreg   <= 1'b0;
        end else begin
            ex_mem_alu_result <= final_ex_result;
            ex_mem_write_data <= mux_b_out; // write data for stores
            ex_mem_rd         <= id_ex_rd;
            ex_mem_regwrite   <= id_ex_regwrite;
            ex_mem_memread    <= id_ex_memread;
            ex_mem_memwrite   <= id_ex_memwrite;
            ex_mem_memtoreg   <= id_ex_memtoreg;
        end
    end

    // ========================================================================
    // 4. MEMORY (MEM) STAGE
    // ========================================================================
    assign dmem_addr       = ex_mem_alu_result;
    assign dmem_write_data = ex_mem_write_data;
    assign dmem_we         = ex_mem_memwrite;
    assign dmem_re         = ex_mem_memread;

    // MEM/WB Pipeline Register Write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_wb_read_data  <= 32'd0;
            mem_wb_alu_result <= 32'd0;
            mem_wb_rd         <= 5'd0;
            mem_wb_regwrite   <= 1'b0;
            mem_wb_memtoreg   <= 1'b0;
        end else begin
            mem_wb_read_data  <= dmem_read_data;
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_rd         <= ex_mem_rd;
            mem_wb_regwrite   <= ex_mem_regwrite;
            mem_wb_memtoreg   <= ex_mem_memtoreg;
        end
    end

    // ========================================================================
    // 5. WRITEBACK (WB) STAGE
    // ========================================================================
    assign writeback_data         = mem_wb_memtoreg ? mem_wb_read_data : mem_wb_alu_result;
    assign mem_wb_regwrite_signal = mem_wb_regwrite;
    assign mem_wb_rd_signal       = mem_wb_rd;

    // ========================================================================
    // 6. HAZARD DETECTION AND FORWARDING UNIT
    // ========================================================================
    riscv_hazard hu (
        .id_ex_rs1       (id_ex_rs1),
        .id_ex_rs2       (id_ex_rs2),
        .ex_mem_rd       (ex_mem_rd),
        .ex_mem_regwrite (ex_mem_regwrite),
        .mem_wb_rd       (mem_wb_rd),
        .mem_wb_regwrite (mem_wb_regwrite),
        .if_id_rs1       (rs1),
        .if_id_rs2       (rs2),
        .id_ex_rd        (id_ex_rd),
        .id_ex_memread   (id_ex_memread),
        .stall_from_ex   (stall_from_ex),
        .id_ex_regwrite  (id_ex_regwrite),
        .ex_mem_memread  (ex_mem_memread),
        .is_branch_or_jalr (is_branch_or_jalr),
        .forward_a       (forward_a),
        .forward_b       (forward_b),
        .stall_if        (stall_if),
        .stall_id        (stall_id),
        .flush_ex        (flush_ex)
    );

endmodule
