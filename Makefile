#==============================================================================
# Makefile — SM-5800 6-DOF IMU (FPGA AXI4-Lite IP + ARM PS Bare-Metal Driver)
#==============================================================================

BUILD_DIR   := build
RTL_SRCS    := rtl/sm5800_crc16_ccitt.v \
               rtl/sm5800_clk_ctrl.v \
               rtl/sm5800_sdlc_rx.v \
               rtl/sm5800_msg_parser.v \
               rtl/sm5800_axi4_lite_if.v \
               rtl/sm5800_imu_axi_top.v
TB_SRCS     := tb/sm5800_sensor_model.v \
               tb/tb_sm5800_imu_axi.v

HOST_CC     ?= gcc
ARM_CC      ?= arm-none-eabi-gcc
ARM_OBJDUMP ?= arm-none-eabi-objdump
IVERILOG    ?= iverilog
VVP         ?= vvp
VERILATOR   ?= verilator

# Strict DO-178C / MISRA C:2012 / BARR-C:2018 compiler profile (C Rules Sec. 12)
STRICT_CFLAGS := -std=c99 -pedantic-errors -O1 -fno-inline \
                 -Wall -Wextra -Werror -Wconversion -Wsign-conversion \
                 -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef \
                 -Wcast-qual -Wcast-align=strict -Wvla -Wstack-usage=256 \
                 -fstack-usage -fno-common -Idriver

TEST_CFLAGS   := -std=c99 -pedantic-errors -O1 -fno-inline \
                 -Wall -Wextra -Werror -Wconversion -Wsign-conversion \
                 -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef \
                 -Wcast-qual -Wvla -fno-common -Idriver -Itests

.PHONY: all sim xsim lint-rtl test-c build-arm verify-static clean

all: verify-static sim test-c

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Gate 1: Offline static verification (RTL lint + Bit-exact SDLC + C Rules + Traceability)
verify-static:
	powershell -ExecutionPolicy Bypass -File scripts/verify_project.ps1

# Gate 2a: RTL simulation via Icarus Verilog ([T1]-[T13])
sim: $(BUILD_DIR)
	$(IVERILOG) -g2001 -Wall -o $(BUILD_DIR)/sim_sm5800.vvp $(RTL_SRCS) $(TB_SRCS)
	$(VVP) $(BUILD_DIR)/sim_sm5800.vvp

# Gate 2b: RTL simulation via Xilinx Vivado xsim ([T1]-[T13])
xsim:
	xvlog $(RTL_SRCS) $(TB_SRCS)
	xelab -debug typical tb_sm5800_imu_axi -s sim_sm5800
	xsim sim_sm5800 -runall

# Gate 2c: RTL static lint via Verilator
lint-rtl:
	$(VERILATOR) --lint-only -Wall --top-module sm5800_imu_axi_top $(RTL_SRCS)

# Gate 3: Host C Unit Tests ([TC-DRV-01]-[TC-DRV-16]) with Mock HAL
test-c: $(BUILD_DIR)
	$(HOST_CC) $(STRICT_CFLAGS) -c driver/sm5800_drv.c -o $(BUILD_DIR)/sm5800_drv.o
	$(HOST_CC) $(TEST_CFLAGS) -c tests/mock_sm5800_hal.c -o $(BUILD_DIR)/mock_sm5800_hal.o
	$(HOST_CC) $(TEST_CFLAGS) -c tests/test_sm5800_drv.c -o $(BUILD_DIR)/test_sm5800_drv.o
	$(HOST_CC) $(BUILD_DIR)/sm5800_drv.o $(BUILD_DIR)/mock_sm5800_hal.o $(BUILD_DIR)/test_sm5800_drv.o -o $(BUILD_DIR)/test_sm5800_drv
	./$(BUILD_DIR)/test_sm5800_drv

# Gate 4: Cross-compile ARM Cortex-A9 PS Driver + emit .su stack usage + check ISR asm
build-arm: $(BUILD_DIR)
	$(ARM_CC) -mcpu=cortex-a9 $(STRICT_CFLAGS) -c driver/sm5800_hal.c -o $(BUILD_DIR)/sm5800_hal.o
	$(ARM_CC) -mcpu=cortex-a9 $(STRICT_CFLAGS) -c driver/sm5800_drv.c -o $(BUILD_DIR)/sm5800_drv.o
	$(ARM_CC) -mcpu=cortex-a9 $(STRICT_CFLAGS) -c driver/sm5800_ps_example.c -o $(BUILD_DIR)/sm5800_ps_example.o
	$(ARM_OBJDUMP) -d $(BUILD_DIR)/sm5800_drv.o > $(BUILD_DIR)/sm5800_drv.dis
	@echo "Stack usage files (.su) generated in $(BUILD_DIR)/"

clean:
	rm -rf $(BUILD_DIR) xsim.dir obj_dir *.jou *.log *.pb *.wdb *.vvp *.vcd
