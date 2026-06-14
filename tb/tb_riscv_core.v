// tb_riscv_core.v
// Testbench to simulate the RV32I Core with Custom 3D Kinematics Extension.

`timescale 1ns/1ps

module tb_riscv_core;

    reg clk;
    reg rst_n;

    // Memory interface signals
    wire [31:0] imem_addr;
    wire [31:0] imem_data;
    wire [31:0] dmem_addr;
    wire [31:0] dmem_write_data;
    reg  [31:0] dmem_read_data;
    wire        dmem_we;
    wire        dmem_re;

    // Instantiate CPU Core
    riscv_core dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .imem_addr       (imem_addr),
        .imem_data       (imem_data),
        .dmem_addr       (dmem_addr),
        .dmem_write_data (dmem_write_data),
        .dmem_read_data  (dmem_read_data),
        .dmem_we         (dmem_we),
        .dmem_re         (dmem_re)
    );

    // ------------------------------------------------------------------------
    // Memory Subsystem (16KB ROM + 16KB RAM)
    // ------------------------------------------------------------------------
    reg [31:0] rom [0:4095]; // 16KB ROM (4096 words)
    reg [31:0] ram [0:4095]; // 16KB RAM (4096 words)

    // Instruction Fetch from ROM (word-aligned)
    // imem_addr is byte address, so we map to ROM index using imem_addr[13:2]
    assign imem_data = rom[imem_addr[13:2]];

    // Cycle Counter for Profiling
    reg [31:0] cycle_count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_count <= 32'd0;
        end else begin
            cycle_count <= cycle_count + 32'd1;
        end
    end

    // Data Memory Access & MMIO Wrapper
    always @(*) begin
        if (dmem_re) begin
            if (dmem_addr == 32'hFFFF0000) begin
                // Read Cycle Count
                dmem_read_data = cycle_count;
            end else if (dmem_addr >= 32'h00004000 && dmem_addr < 32'h00008000) begin
                // Read RAM (offset address by 0x4000)
                dmem_read_data = ram[dmem_addr[13:2]];
            end else if (dmem_addr < 32'h00004000) begin
                // Read ROM (for string literals and constants)
                dmem_read_data = rom[dmem_addr[13:2]];
            end else begin
                dmem_read_data = 32'd0;
            end
        end else begin
            dmem_read_data = 32'd0;
        end
    end

    // Write RAM or handle MMIO
    always @(posedge clk) begin
        if (dmem_we) begin
            if (dmem_addr == 32'hFFFF0004) begin
                // UART Print character
                $write("%c", dmem_write_data[7:0]);
                $fflush();
            end else if (dmem_addr == 32'h80000000) begin
                // Exit Simulation
                $display("\n[Simulation Finished via MMIO]");
                $display("Total Simulated Cycles: %d", cycle_count);
                if (dmem_write_data == 32'd1) begin
                    $display("SIMULATION STATUS: SUCCESS\n");
                    $finish(0);
                end else begin
                    $display("SIMULATION STATUS: FAILED\n");
                    $finish(1);
                end
            end else if (dmem_addr >= 32'h00004000 && dmem_addr < 32'h00008000) begin
                // Write RAM
                ram[dmem_addr[13:2]] <= dmem_write_data;
            end
        end
    end

    // Cycle-by-cycle execution trace for debugging
    always @(posedge clk) begin
        if (rst_n) begin
            if (dut.id_ex_is_custom0) begin
                $display("Time=%0d ns | PC=%h | Custom Instr (funct3=%d) | rs1_val=%h | rs2_val=%h | Result=%h | cycle_cnt=%d", 
                         $time/1000, dut.id_ex_pc, dut.id_ex_funct3, dut.mux_a_out, dut.mux_b_out, dut.kinematics_result, dut.accel.cycle_cnt);
            end
            if (dut.mem_wb_regwrite_signal && (dut.mem_wb_rd_signal == 10 || dut.mem_wb_rd_signal == 12 || dut.mem_wb_rd_signal == 14)) begin
                $display("Time=%0d ns | WB to rd=%d | wd=%h", $time/1000, dut.mem_wb_rd_signal, dut.writeback_data);
            end
            
            // Halt/trap loop detection (j .)
            if (dut.if_id_instr == 32'h0000006f) begin
                $display("\n[Simulation Finished via Halt/Trap Loop at PC=%h]", dut.if_id_pc);
                $display("Total Simulated Cycles: %d", cycle_count);
                $display("SIMULATION STATUS: SUCCESS (halted)\n");
                $finish(0);
            end
        end
    end

    // Clock Generator (50 MHz -> 20ns period)
    always begin
        #10 clk = ~clk;
    end

    // Simulation Initial block
    integer idx;
    initial begin
        clk = 0;
        rst_n = 0;
        cycle_count = 0;

        // Initialize ROM and RAM with NOPs to prevent uninitialized loop execution
        for (idx = 0; idx < 4096; idx = idx + 1) begin
            rom[idx] = 32'h00000013;
            ram[idx] = 32'h00000013;
        end

        // Open VCD file for waveforms
        $dumpfile("simulation.vcd");
        $dumpvars(0, tb_riscv_core);

        // Load firmware memory hex file (compiled by compiler)
        // Using $fscanf in a loop avoids $readmemh's "Not enough words in the file" warning
        begin: load_hex_block
            integer file, r;
            reg [31:0] val;
            file = $fopen("firmware.hex", "r");
            if (file != 0) begin
                idx = 0;
                while (!$feof(file) && idx < 4096) begin
                    r = $fscanf(file, "%h\n", val);
                    if (r == 1) begin
                        rom[idx] = val;
                        idx = idx + 1;
                    end
                end
                $fclose(file);
            end else begin
                $display("Error: Could not open firmware.hex");
                $finish(1);
            end
        end

        // Reset Pulse
        #40;
        rst_n = 1;

        // Watchdog Timeout fallback (60,000 cycles @ 20ns clock period)
        #1200000;
        $display("\n[ERROR] Simulation Timeout reached!");
        $finish(2);
    end

endmodule
