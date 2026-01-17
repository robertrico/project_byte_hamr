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
LPF           := $(CONSTRAINT_DIR)/byte_hamr.lpf

# Default design
DESIGN ?= signal_check

.PHONY: all clean help synth pnr bit prog prog-flash prog-detect pinout lpf sim wave

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
	@echo "Simulation:"
	@echo "  make sim          - Run simulation"
	@echo "  make wave         - Run simulation and open waveform viewer"
	@echo ""
	@echo "Utility targets:"
	@echo "  make pinout       - Regenerate FPGA pinout JSON"
	@echo "  make lpf          - Regenerate LPF constraints"
	@echo "  make clean        - Remove build files"
	@echo ""
	@echo "Available designs:"
	@for d in $(GATEWARE_DIR)/*/; do echo "  $$(basename $$d)"; done
	@echo ""

# =============================================================================
# Build directories
# =============================================================================

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# =============================================================================
# Gateware build
# =============================================================================

# Find all Verilog source files for the design (exclude testbenches)
DESIGN_DIR := $(GATEWARE_DIR)/$(DESIGN)
VERILOG_SRC := $(filter-out %_tb.v,$(wildcard $(DESIGN_DIR)/*.v))

# Output files
JSON := $(BUILD_DIR)/$(DESIGN).json
CFG  := $(BUILD_DIR)/$(DESIGN).config
BIT  := $(BUILD_DIR)/$(DESIGN).bit
SVF  := $(BUILD_DIR)/$(DESIGN).svf

# Get top module name (assume it matches directory name + _top)
TOP := $(DESIGN)_top

synth: $(JSON)

$(JSON): $(VERILOG_SRC) | $(BUILD_DIR)
	@echo "=== Synthesizing $(DESIGN) with Yosys ==="
	$(YOSYS) -q -p "read_verilog $(VERILOG_SRC); synth_ecp5 -top $(TOP) -json $@"
	@echo "Synthesis complete: $@"

pnr: $(CFG)

$(CFG): $(JSON) $(LPF)
	@echo "=== Place & Route with nextpnr ==="
	$(NEXTPNR) --$(DEVICE) --package $(PACKAGE) --speed $(SPEED) \
		--lpf $(LPF) --json $(JSON) --textcfg $@ \
		--timing-allow-fail
	@echo "Place & route complete: $@"

bit: $(BIT)

$(BIT): $(CFG)
	@echo "=== Generating Bitstream ==="
	$(ECPPACK) --input $< --bit $@ --svf $(SVF)
	@echo "Bitstream ready: $@"

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

# Simulation files
SIM_SRC := $(wildcard $(DESIGN_DIR)/*_tb.v)
SIM_OUT := $(BUILD_DIR)/$(DESIGN)_tb.vvp
VCD     := $(BUILD_DIR)/$(DESIGN)_tb.vcd

sim: $(SIM_OUT)
	@echo "=== Running Simulation ==="
	cd $(BUILD_DIR) && $(VVP) $(DESIGN)_tb.vvp
	@if [ -f $(VCD) ]; then echo "VCD written to $(VCD)"; fi

$(SIM_OUT): $(VERILOG_SRC) $(SIM_SRC) | $(BUILD_DIR)
	@echo "=== Compiling Testbench ==="
	$(IVERILOG) -o $@ -s $(DESIGN)_tb $(VERILOG_SRC) $(SIM_SRC)

wave: sim
	@echo "=== Opening Waveform Viewer ==="
	@if [ -f $(VCD) ]; then \
		gtkwave $(VCD) & \
	else \
		echo "No VCD file found. Run 'make sim' first."; \
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
# Cleanup
# =============================================================================

clean:
	@rm -rf $(BUILD_DIR)
	@echo "Build directory cleaned"
