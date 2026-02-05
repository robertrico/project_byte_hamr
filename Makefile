# =============================================================================
# Byte Hamr - Apple II FPGA Card
# =============================================================================
# Build system for ECP5 gateware using open-source toolchain
# =============================================================================

# Tools (use oss-cad-suite if available)
OSS_CAD_SUITE := $(HOME)/oss-cad-suite/bin
ifneq ($(wildcard $(OSS_CAD_SUITE)/yosys),)
    YOSYS    := $(OSS_CAD_SUITE)/yosys
    NEXTPNR  := $(OSS_CAD_SUITE)/nextpnr-ecp5
    ECPPACK  := $(OSS_CAD_SUITE)/ecppack
    LOADER   := $(OSS_CAD_SUITE)/openFPGALoader
else
    YOSYS    := yosys
    NEXTPNR  := nextpnr-ecp5
    ECPPACK  := ecppack
    LOADER   := openFPGALoader
endif

# FPGA settings (LFE5U-85F-8BG381I)
DEVICE   := 85k
PACKAGE  := CABGA381
SPEED    := 8

# Directories
BUILD_DIR     := build
GATEWARE_DIR  := gateware
CONSTRAINT_DIR := gateware/constraints
REPORT_DIR    := reporting

# Constraints: use design-specific LPF if it exists, otherwise use base
LPF_BASE    := $(CONSTRAINT_DIR)/byte_hamr.lpf
LPF_DESIGN  := $(GATEWARE_DIR)/$(DESIGN)/$(DESIGN).lpf
LPF         := $(if $(wildcard $(LPF_DESIGN)),$(LPF_DESIGN),$(LPF_BASE))

# Default design
DESIGN ?= signal_check

.PHONY: all clean clean-reports clean-all help synth pnr bit prog prog-flash prog-detect pinout lpf \
        sim wave unit unit-wave assemble extract-dsk create-dsk list-dsk report \
        esp-build esp-flash esp-monitor esp-all esp-clean esp-menuconfig esp-help

# =============================================================================
# Default target
# =============================================================================

all: bit

help:
	@echo "============================================"
	@echo "Byte Hamr - Apple II FPGA Card"
	@echo "============================================"
	@echo ""
	@echo "Build targets:"
	@echo "  make              - Build bitstream (default: signal_check)"
	@echo "  make DESIGN=xxx   - Build specific design"
	@echo "  make synth        - Synthesize only"
	@echo "  make pnr          - Place and route"
	@echo "  make bit          - Generate bitstream"
	@echo "  make prog         - Program via JTAG (volatile)"
	@echo "  make prog-flash   - Program SPI flash (persistent)"
	@echo ""
	@echo "Reporting:"
	@echo "  make report       - View build summary for DESIGN"
	@echo "  Reports saved to: reporting/"
	@echo "    *_synth.log     - Full Yosys synthesis log"
	@echo "    *_synth_stat.txt- Cell/wire statistics"
	@echo "    *_pnr.log       - Full nextpnr log"
	@echo "    *_pnr_report.json - Detailed PnR report (JSON)"
	@echo "    *_timing.txt    - Timing summary"
	@echo "    *_utilization.txt - Resource usage"
	@echo "    *_summary.txt   - Combined build summary"
	@echo ""
	@echo "Simulation:"
	@echo "  make sim          - Run simulation"
	@echo "  make wave         - Run simulation and open waveform viewer"
	@echo ""
	@echo "Unit Testing:"
	@echo "  make unit MODULE=xxx DESIGN=yyy - Run unit test for a module"
	@echo "  make unit-wave MODULE=xxx       - Run unit test and view waveform"
	@echo ""
	@echo "Utility targets:"
	@echo "  make pinout       - Regenerate FPGA pinout JSON"
	@echo "  make lpf          - Regenerate LPF constraints"
	@echo "  make clean        - Remove build files"
	@echo ""
	@echo "6502 Assembly:"
	@echo "  make assemble ASM_SRC=path  - Assemble with Merlin32"
	@echo ""
	@echo "Apple II Disk Utilities:"
	@echo "  make extract-dsk DSK=x.dsk  - Extract files (FORCE=1 to overwrite)"
	@echo "  make create-dsk DSK=NAME     - Create disk from software/NAME/"
	@echo "  make list-dsk                - List disks in ADTPro folder"
	@echo ""
	@echo "ESP32 Firmware (Yellow Hamr):"
	@echo "  make esp-build              - Build firmware"
	@echo "  make esp-flash              - Flash to device"
	@echo "  make esp-monitor            - Serial monitor"
	@echo "  make esp-all                - Build + flash + monitor"
	@echo "  make esp-help               - Full ESP32 help"
	@echo ""
	@echo "Available designs:"
	@for d in $(GATEWARE_DIR)/*/; do echo "  $$(basename $$d)"; done
	@echo ""

# =============================================================================
# Build directories
# =============================================================================

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(REPORT_DIR):
	@mkdir -p $(REPORT_DIR)

# =============================================================================
# Gateware build
# =============================================================================

# Find all Verilog source files for the design (exclude testbenches)
DESIGN_DIR := $(GATEWARE_DIR)/$(DESIGN)
VERILOG_SRC := $(filter-out %_tb.v,$(wildcard $(DESIGN_DIR)/*.v))
MEM_FILES := $(wildcard $(DESIGN_DIR)/*.mem)

# Output files
JSON := $(BUILD_DIR)/$(DESIGN).json
CFG  := $(BUILD_DIR)/$(DESIGN).config
BIT  := $(BUILD_DIR)/$(DESIGN).bit
SVF  := $(BUILD_DIR)/$(DESIGN).svf

# Report files
SYNTH_LOG    := $(REPORT_DIR)/$(DESIGN)_synth.log
SYNTH_STAT   := $(REPORT_DIR)/$(DESIGN)_synth_stat.txt
PNR_LOG      := $(REPORT_DIR)/$(DESIGN)_pnr.log
PNR_REPORT   := $(REPORT_DIR)/$(DESIGN)_pnr_report.json
TIMING_RPT   := $(REPORT_DIR)/$(DESIGN)_timing.txt
UTIL_RPT     := $(REPORT_DIR)/$(DESIGN)_utilization.txt

# Get top module name (assume it matches directory name + _top)
TOP := $(DESIGN)_top

synth: $(JSON)

$(JSON): $(VERILOG_SRC) $(MEM_FILES) | $(BUILD_DIR) $(REPORT_DIR)
	@echo "=== Synthesizing $(DESIGN) with Yosys ==="
	@echo "Synthesis started at $$(date)" > $(SYNTH_LOG)
	@echo "Design: $(DESIGN)" >> $(SYNTH_LOG)
	@echo "Top module: $(TOP)" >> $(SYNTH_LOG)
	@echo "Source files: $(VERILOG_SRC)" >> $(SYNTH_LOG)
	@echo "" >> $(SYNTH_LOG)
	@# Copy any .mem files to build directory for synthesis
	@for f in $(MEM_FILES); do cp "$$f" $(BUILD_DIR)/; done
	cd $(BUILD_DIR) && $(YOSYS) -p "\
		read_verilog $(addprefix ../,$(VERILOG_SRC)); \
		synth_ecp5 -top $(TOP) -json $(DESIGN).json; \
		stat -top $(TOP)" 2>&1 | tee -a ../$(SYNTH_LOG)
	@echo "" >> $(SYNTH_LOG)
	@echo "Synthesis completed at $$(date)" >> $(SYNTH_LOG)
	@# Extract stats to separate file
	@grep -A 100 "Printing statistics" $(SYNTH_LOG) > $(SYNTH_STAT) 2>/dev/null || true
	@echo ""
	@echo "Synthesis complete: $@"
	@echo "Reports: $(SYNTH_LOG), $(SYNTH_STAT)"

pnr: $(CFG)

$(CFG): $(JSON) $(LPF) | $(REPORT_DIR)
	@echo "=== Place & Route with nextpnr ==="
	@echo "Place & Route started at $$(date)" > $(PNR_LOG)
	@echo "Design: $(DESIGN)" >> $(PNR_LOG)
	@echo "Device: $(DEVICE), Package: $(PACKAGE), Speed: $(SPEED)" >> $(PNR_LOG)
	@echo "Constraints: $(LPF)" >> $(PNR_LOG)
	@echo "" >> $(PNR_LOG)
	$(NEXTPNR) --$(DEVICE) --package $(PACKAGE) --speed $(SPEED) \
		--lpf $(LPF) --json $(JSON) --textcfg $@ \
		--report $(PNR_REPORT) \
		--timing-allow-fail 2>&1 | tee -a $(PNR_LOG)
	@echo "" >> $(PNR_LOG)
	@echo "Place & Route completed at $$(date)" >> $(PNR_LOG)
	@# Extract timing summary
	@echo "=== Timing Summary ===" > $(TIMING_RPT)
	@echo "Generated: $$(date)" >> $(TIMING_RPT)
	@echo "" >> $(TIMING_RPT)
	@grep -E "(Max frequency|Slack|Critical|constraint)" $(PNR_LOG) >> $(TIMING_RPT) 2>/dev/null || echo "No timing info found" >> $(TIMING_RPT)
	@# Extract utilization
	@echo "=== Resource Utilization ===" > $(UTIL_RPT)
	@echo "Generated: $$(date)" >> $(UTIL_RPT)
	@echo "Design: $(DESIGN)" >> $(UTIL_RPT)
	@echo "" >> $(UTIL_RPT)
	@grep -E "(TRELLIS_SLICE|TRELLIS_IO|DCCA|DP16KD|MULT18X18D|ALU54B|EHXPLLL|EXTREFB|DCUA|PCSCLKDIV|BRAM|LUT|FF|IO)" $(PNR_LOG) >> $(UTIL_RPT) 2>/dev/null || echo "No utilization info found" >> $(UTIL_RPT)
	@echo ""
	@echo "Place & route complete: $@"
	@echo "Reports: $(PNR_LOG), $(PNR_REPORT), $(TIMING_RPT), $(UTIL_RPT)"

bit: $(BIT)

$(BIT): $(CFG) | $(REPORT_DIR)
	@echo "=== Generating Bitstream ==="
	$(ECPPACK) --input $< --bit $@ --svf $(SVF) --freq 62.0
	@echo "Bitstream ready: $@"
	@# Generate build summary
	@echo "=== Build Summary for $(DESIGN) ===" > $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "Generated: $$(date)" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "Bitstream: $@" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "Size: $$(ls -lh $@ | awk '{print $$5}')" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "--- Synthesis Stats ---" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@cat $(SYNTH_STAT) >> $(REPORT_DIR)/$(DESIGN)_summary.txt 2>/dev/null || echo "(not available)" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "--- Utilization ---" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@cat $(UTIL_RPT) >> $(REPORT_DIR)/$(DESIGN)_summary.txt 2>/dev/null || echo "(not available)" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo "--- Timing ---" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@cat $(TIMING_RPT) >> $(REPORT_DIR)/$(DESIGN)_summary.txt 2>/dev/null || echo "(not available)" >> $(REPORT_DIR)/$(DESIGN)_summary.txt
	@echo ""
	@echo "Build summary: $(REPORT_DIR)/$(DESIGN)_summary.txt"

report:
	@echo "=== Build Reports for $(DESIGN) ==="
	@echo ""
	@if [ -f $(REPORT_DIR)/$(DESIGN)_summary.txt ]; then \
		cat $(REPORT_DIR)/$(DESIGN)_summary.txt; \
	else \
		echo "No reports found. Run 'make DESIGN=$(DESIGN)' first."; \
	fi

# =============================================================================
# Simulation
# =============================================================================

# Simulation tool (use iverilog from oss-cad-suite if available)
ifneq ($(wildcard $(OSS_CAD_SUITE)/iverilog),)
    IVERILOG := $(OSS_CAD_SUITE)/iverilog
    VVP      := $(OSS_CAD_SUITE)/vvp
else
    IVERILOG := iverilog
    VVP      := vvp
endif

# Simulation files - main design testbench
SIM_MAIN_TB := $(DESIGN_DIR)/$(DESIGN)_tb.v
SIM_OUT := $(BUILD_DIR)/$(DESIGN)_tb.vvp
VCD     := $(BUILD_DIR)/$(DESIGN)_tb.vcd

sim: $(SIM_OUT)
	@echo "=== Running Simulation ==="
	cd $(BUILD_DIR) && $(VVP) $(DESIGN)_tb.vvp
	@if [ -f $(VCD) ]; then echo "VCD written to $(VCD)"; fi

$(SIM_OUT): $(VERILOG_SRC) $(SIM_MAIN_TB) $(MEM_FILES) | $(BUILD_DIR)
	@echo "=== Compiling Testbench ==="
	@# Copy any .mem files to build directory for simulation
	@for f in $(MEM_FILES); do cp "$$f" $(BUILD_DIR)/; done
	$(IVERILOG) -o $@ -s $(DESIGN)_tb $(VERILOG_SRC) $(SIM_MAIN_TB)

wave: sim
	@echo "=== Opening Waveform Viewer ==="
	@if [ -f $(VCD) ]; then \
		gtkwave $(VCD) & \
	else \
		echo "No VCD file found. Run 'make sim' first."; \
	fi

# =============================================================================
# Unit Testing (for individual modules)
# =============================================================================
# Usage: make unit DESIGN=logic_hamr_v1 MODULE=decimate_pack

MODULE ?=
UNIT_TB  := $(DESIGN_DIR)/$(MODULE)_tb.v
UNIT_OUT := $(BUILD_DIR)/$(MODULE)_tb.vvp
UNIT_VCD := $(BUILD_DIR)/$(MODULE)_tb.vcd

unit: $(UNIT_OUT)
	@echo "=== Running Unit Test: $(MODULE) ==="
	cd $(BUILD_DIR) && $(VVP) $(MODULE)_tb.vvp
	@if [ -f $(UNIT_VCD) ]; then echo "VCD written to $(UNIT_VCD)"; fi

$(UNIT_OUT): $(VERILOG_SRC) $(UNIT_TB) | $(BUILD_DIR)
	@echo "=== Compiling Unit Testbench: $(MODULE) ==="
	$(IVERILOG) -o $@ -s $(MODULE)_tb $(VERILOG_SRC) $(UNIT_TB)

unit-wave: unit
	@echo "=== Opening Unit Test Waveform ==="
	@if [ -f $(UNIT_VCD) ]; then \
		gtkwave $(UNIT_VCD) & \
	else \
		echo "No VCD file found."; \
	fi

# =============================================================================
# Programming
# =============================================================================

# JTAG cable: FT231X with bitbang over modem control pins
# Pin mapping from schematic: CTS→TDO, DSR→TCK, DCD→TMS, RI→TDI
# Format: --pins TDI:TDO:TCK:TMS
CABLE    := ft231X
JTAG_PINS := RI:CTS:DSR:DCD
SERIAL   ?= DT03D4KG

prog: $(BIT)
	@echo "=== Programming via JTAG (SRAM) ==="
	$(LOADER) --cable $(CABLE) --ftdi-serial $(SERIAL) --pins $(JTAG_PINS) $(BIT)

prog-flash: $(BIT)
	@echo "=== Programming SPI Flash ==="
	$(LOADER) --cable $(CABLE) --ftdi-serial $(SERIAL) --pins $(JTAG_PINS) -f $(BIT)

prog-detect:
	@echo "=== Detecting FPGA ==="
	$(LOADER) --cable $(CABLE) --ftdi-serial $(SERIAL) --pins $(JTAG_PINS) --detect

# =============================================================================
# Pinout and constraint generation
# =============================================================================

pinout:
	@echo "=== Generating FPGA Pinout ==="
	python3 scripts/extract_fpga_pinout.py
	python3 scripts/augment_fpga_pinout.py

lpf: pinout
	@echo "=== Generating LPF Constraints ==="
	python3 scripts/generate_lpf.py

# =============================================================================
# 6502 Assembly (Merlin32)
# =============================================================================

MERLIN32     := /Users/hambook/Development/Merlin32_v1.2/MacOs/Merlin32
MERLIN_LIB   := /Users/hambook/Development/Merlin32_v1.2/Library
ASM_SRC      ?=

assemble:
ifndef ASM_SRC
	@echo "Usage: make assemble ASM_SRC=software/PROJECT/SOURCE.S"
	@exit 1
endif
	$(MERLIN32) $(MERLIN_LIB) $(ASM_SRC)

# =============================================================================
# Apple II Disk Utilities
# =============================================================================

ADTPRO_DISKS := /Applications/ADTPro-v.r.m/disks
SOFTWARE_DIR := software
DSK ?=
FORCE ?=

extract-dsk:
ifndef DSK
	@echo "Usage: make extract-dsk DSK=diskname.dsk [FORCE=1]"
	@exit 1
endif
	software/utils/extract_dsk.sh $(if $(FORCE),--force,) $(DSK) $(SOFTWARE_DIR)

create-dsk:
ifndef DSK
	@echo "Usage: make create-dsk DSK=DISKNAME"
	@echo "       Sources from software/DISKNAME/"
	@exit 1
endif
	software/utils/create_dsk.sh $(SOFTWARE_DIR)/$(DSK)

list-dsk:
	@for f in $(ADTPRO_DISKS)/*.dsk; do [ -f "$$f" ] && basename "$$f"; done 2>/dev/null || echo "No .dsk files found"

# =============================================================================
# ESP32 Firmware (Yellow Hamr companion)
# =============================================================================
# Requires: source ~/esp/esp-idf/export.sh (once per terminal session)
# =============================================================================

ESP_FW_DIR   := gateware/yellow_hamr/firmware
ESP_PROJECT  ?= phase1_signal_monitor
ESP_PORT     ?= $(shell ls /dev/cu.usbserial-* 2>/dev/null | head -1)
IDF_PATH     ?= $(HOME)/esp/esp-idf

.PHONY: esp-build esp-flash esp-monitor esp-all esp-clean esp-menuconfig esp-help esp-check

# Check if IDF is sourced
esp-check:
	@command -v idf.py >/dev/null 2>&1 || { \
		echo ""; \
		echo "ERROR: ESP-IDF not sourced. Run first:"; \
		echo "  source $(IDF_PATH)/export.sh"; \
		echo ""; \
		exit 1; \
	}

esp-help:
	@echo "=== ESP32 Firmware Targets ==="
	@echo ""
	@echo "First, source ESP-IDF (once per terminal):"
	@echo "  source $(IDF_PATH)/export.sh"
	@echo ""
	@echo "Then:"
	@echo "  make esp-build              - Build firmware"
	@echo "  make esp-flash              - Flash to device"
	@echo "  make esp-monitor            - Serial monitor"
	@echo "  make esp-all                - Build + flash + monitor"
	@echo "  make esp-clean              - Clean build"
	@echo "  make esp-menuconfig         - SDK configuration"
	@echo ""
	@echo "Options:"
	@echo "  ESP_PROJECT=xxx   - Firmware project (default: $(ESP_PROJECT))"
	@echo "  ESP_PORT=xxx      - Serial port (default: auto-detect)"
	@echo ""
	@echo "Available firmware projects:"
	@for d in $(ESP_FW_DIR)/*/; do echo "  $$(basename $$d)"; done
	@echo ""

esp-build: esp-check
	@echo "=== Building ESP32 Firmware: $(ESP_PROJECT) ==="
	cd $(ESP_FW_DIR)/$(ESP_PROJECT) && idf.py build

esp-flash: esp-check
	@echo "=== Flashing ESP32: $(ESP_PROJECT) ==="
	cd $(ESP_FW_DIR)/$(ESP_PROJECT) && idf.py -p $(ESP_PORT) flash

esp-monitor: esp-check
	@echo "=== ESP32 Serial Monitor ==="
	cd $(ESP_FW_DIR)/$(ESP_PROJECT) && idf.py -p $(ESP_PORT) monitor

esp-all: esp-build
	@echo "=== Flash + Monitor ==="
	cd $(ESP_FW_DIR)/$(ESP_PROJECT) && idf.py -p $(ESP_PORT) flash monitor

esp-clean:
	@echo "=== Cleaning ESP32 build: $(ESP_PROJECT) ==="
	cd $(ESP_FW_DIR)/$(ESP_PROJECT) && rm -rf build sdkconfig

esp-menuconfig: esp-check
	cd $(ESP_FW_DIR)/$(ESP_PROJECT) && idf.py menuconfig

# =============================================================================
# Cleanup
# =============================================================================

clean:
	@rm -rf $(BUILD_DIR)
	@echo "Build directory cleaned"

clean-reports:
	@rm -rf $(REPORT_DIR)
	@echo "Reports directory cleaned"

clean-all: clean clean-reports
	@echo "All build artifacts cleaned"
