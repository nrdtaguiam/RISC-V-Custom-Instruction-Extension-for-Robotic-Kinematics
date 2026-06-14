# Makefile for riscv_3d_kinematics_accel project.
# Runs natively inside the WSL Linux environment.

PREFIX = riscv64-unknown-elf-

CC       = $(PREFIX)gcc
OBJCOPY  = $(PREFIX)objcopy
IVERILOG = iverilog
VVP      = vvp
PYTHON   = python3

# Target RV32I compilation flags (static, no standard library, optimized)
CFLAGS = -march=rv32i -mabi=ilp32 -static -nostdlib -O2

# Paths
RTL_DIR      = ../rtl
FIRMWARE_DIR = ../firmware
TB_DIR       = .

RTL_SRCS = $(RTL_DIR)/riscv_core.v $(RTL_DIR)/riscv_regfile.v $(RTL_DIR)/riscv_alu.v $(RTL_DIR)/kinematics_accel.v $(RTL_DIR)/riscv_hazard.v

all: compile run

compile:
	@echo "========================================================="
	@echo " 1. Compiling Firmware inside WSL..."
	@echo "========================================================="
	$(CC) $(CFLAGS) -T $(FIRMWARE_DIR)/link.ld $(FIRMWARE_DIR)/crt0.S $(FIRMWARE_DIR)/main.c -o firmware.elf
	$(OBJCOPY) -O binary firmware.elf firmware.bin
	@echo "========================================================="
	@echo " 2. Converting Binary to Verilog Hex..."
	@echo "========================================================="
	$(PYTHON) $(FIRMWARE_DIR)/bin2hex.py firmware.bin firmware.hex

run: compile
	@echo "========================================================="
	@echo " 3. Running Icarus Verilog Simulation inside WSL..."
	@echo "========================================================="
	$(IVERILOG) -o sim_test -I$(RTL_DIR) $(RTL_SRCS) $(TB_DIR)/tb_riscv_core.v
	$(VVP) sim_test

clean:
	rm -f *.elf *.bin *.hex *.vcd kinematics_sim sim_test
