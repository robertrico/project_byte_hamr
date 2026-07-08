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
PNR_SEED ?= 1

# Directories
BUILD_DIR     := build
REV           ?= rev1
GATEWARE_DIR  := gateware/$(REV)
CONSTRAINT_DIR := $(GATEWARE_DIR)/constraints
REPORT_DIR    := reporting

# Constraints: use design-specific LPF if it exists, otherwise use base
LPF_BASE    := $(CONSTRAINT_DIR)/byte_hamr.lpf
LPF_DESIGN  := $(GATEWARE_DIR)/$(DESIGN)/$(DESIGN).lpf
LPF         := $(if $(wildcard $(LPF_DESIGN)),$(LPF_DESIGN),$(LPF_BASE))

# Default design
DESIGN ?= signal_check

.PHONY: all clean clean-reports clean-all help synth pnr bit prog prog-flash prog-detect pinout lpf \
        sim wave gtk unit unit-wave assemble sdmtest cpreg cprace cprace3 cmpskill life8 life8gr cpdemo cpsdrd cpsave cpboot sdmdisk extract-dsk create-dsk list-dsk report farmtasksim farm farmtest conwaytest \
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
	@echo "  make gtk VCD=file.vcd - Open a VCD file in GTKWave"
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

# For flash_hamr: firmware.mem and hamr_rom.mem are synthesis dependencies
FW_MEM_DEP := $(if $(filter flash_hamr,$(DESIGN)),$(FW_MEM),)

# Auto-regenerate hamr_rom.mem from Merlin32 assembly source
HAMR_ROM_MEM := $(GATEWARE_DIR)/flash_hamr/hamr_rom.mem
HAMR_ROM_SRC := $(GATEWARE_DIR)/flash_hamr/hamr_rom.S

$(HAMR_ROM_MEM): $(HAMR_ROM_SRC)
	@echo "=== Assembling Flash Hamr boot ROM (Merlin32) ==="
	cd $(GATEWARE_DIR)/flash_hamr && $(MERLIN32) $(MERLIN_LIB) hamr_rom.S
	python3 scripts/rom2mem.py $(GATEWARE_DIR)/flash_hamr/hamr_rom.bin $@

# project_obscurus slot ROM (256 bytes at $C400). Merlin32 source -> .bin ->
# .mem. rom2mem.py "base" arg controls placement inside the .mem file: passing
# $C000 with size 256 puts bin at .mem[0], matching Verilog's index by A[7:0].
OBSCURUS_ROM_MEM := $(GATEWARE_DIR)/project_obscurus/slot_rom.mem
OBSCURUS_ROM_SRC := $(GATEWARE_DIR)/project_obscurus/slot_rom.S

$(OBSCURUS_ROM_MEM): $(OBSCURUS_ROM_SRC)
	@echo "=== Assembling project_obscurus slot ROM (Merlin32) ==="
	cd $(GATEWARE_DIR)/project_obscurus && $(MERLIN32) $(MERLIN_LIB) slot_rom.S
	python3 scripts/rom2mem.py $(GATEWARE_DIR)/project_obscurus/slot_rom.bin $@ 0xC000 256

# project_obscurus monitor ROM (2KB at $C800). Merlin32 source -> .bin -> .mem.
# NOTE: rom2mem base MUST be 0xC000 (not 0xC800): rom2mem computes
# offset = base - 0xC000 and writes bin at rom[offset]; base=0xC800 gives
# offset==size -> the guard rejects every byte -> all-$FF brick. base=0xC000
# -> offset 0 -> bin lands at mem[0], indexed by monitor_mem[apple_addr[10:0]].
OBSCURUS_MON_MEM := $(GATEWARE_DIR)/project_obscurus/monitor.mem
OBSCURUS_MON_SRC := $(GATEWARE_DIR)/project_obscurus/monitor.S

# Only auto-assemble the monitor ROM when its Merlin32 source actually exists.
# Until the real monitor.S lands, a zero-filled monitor.mem stub is committed so
# $readmemh works; without this guard make would try (and fail) to derive the
# stub .mem from a non-existent .S.
ifneq ($(wildcard $(OBSCURUS_MON_SRC)),)
$(OBSCURUS_MON_MEM): $(OBSCURUS_MON_SRC)
	@echo "=== Assembling project_obscurus monitor ROM (Merlin32) ==="
	cd $(GATEWARE_DIR)/project_obscurus && $(MERLIN32) $(MERLIN_LIB) monitor.S
	python3 scripts/rom2mem.py $(GATEWARE_DIR)/project_obscurus/monitor.bin $@ 0xC000 2048 0x60
endif

# project_obscurus coprocessor KERNEL image. Merlin32 source ORG $1000 -> .bin ->
# .mem (full 8192-byte BRAM image: zeros $0000-$0FFF, kernel spliced at $1000).
# coproc.v loads it with $readmemh("kernel.mem", bram) at offset 0 -- a single
# init with no procedural for-loop, so Yosys infers DP16KD cleanly.
OBSCURUS_KERNEL_MEM := $(GATEWARE_DIR)/project_obscurus/kernel.mem
OBSCURUS_KERNEL_SRC := $(GATEWARE_DIR)/project_obscurus/kernel.S

ifneq ($(wildcard $(OBSCURUS_KERNEL_SRC)),)
$(OBSCURUS_KERNEL_MEM): $(OBSCURUS_KERNEL_SRC)
	@echo "=== Assembling coproc kernel (Merlin32) ==="
	cd $(GATEWARE_DIR)/project_obscurus && $(MERLIN32) $(MERLIN_LIB) kernel.S
	python3 -c "b=open('$(GATEWARE_DIR)/project_obscurus/kernel.bin','rb').read(); m=bytearray(8192); m[0x1000:0x1000+len(b)]=b; open('$(OBSCURUS_KERNEL_MEM)','w').write('\n'.join('%02x'%x for x in m)+'\n')"
endif

# Force project_obscurus to depend on its slot ROM and monitor ROM
ifeq ($(DESIGN),project_obscurus)
$(JSON): $(OBSCURUS_ROM_MEM) $(OBSCURUS_MON_MEM) $(OBSCURUS_KERNEL_MEM)
endif

# b8008_hamr slot ROM (256 bytes at $C400): PR#4 terminal firmware.
# base 0xC000 -> bin at mem[0] (indexed by A[7:0]); fill 0x00 per project rule.
B8008_SLOT_MEM := $(GATEWARE_DIR)/b8008_hamr/b8008_slot.mem
B8008_SLOT_SRC := $(GATEWARE_DIR)/b8008_hamr/b8fw.S

$(B8008_SLOT_MEM): $(B8008_SLOT_SRC)
	@echo "=== Assembling b8008_hamr slot ROM (Merlin32) ==="
	cd $(GATEWARE_DIR)/b8008_hamr && $(MERLIN32) $(MERLIN_LIB) b8fw.S
	python3 scripts/rom2mem.py $(GATEWARE_DIR)/b8008_hamr/b8fw.bin $@ 0xC000 256 0x00

ifeq ($(DESIGN),b8008_hamr)
$(JSON): $(B8008_SLOT_MEM)
endif

# Flash Hamr menu volume (picker + ProDOS)
FLASH_HAMR_DIR := $(GATEWARE_DIR)/flash_hamr
PICKER_SRC     := $(FLASH_HAMR_DIR)/picker.S
PICKER_BIN     := $(FLASH_HAMR_DIR)/picker.bin
PICKER_SYS     := $(FLASH_HAMR_DIR)/picker.sys
MENU_PO        := $(FLASH_HAMR_DIR)/images/menu.po
PRODOS_SRC     := software/SMARTPORT/images/ProDOS_2_4_1.po
AC_JAR         := /Users/hambook/Development/AppleCommander-ac-13.0.jar
AC             := java -jar $(AC_JAR)

$(PICKER_SYS): $(PICKER_SRC)
	@echo "=== Assembling Flash Hamr picker (Merlin32) ==="
	cd $(FLASH_HAMR_DIR) && $(MERLIN32) $(MERLIN_LIB) picker.S
	cp $(PICKER_BIN) $@

$(MENU_PO): $(PICKER_SYS) $(PRODOS_SRC)
	@echo "=== Building Flash Hamr menu volume ==="
	$(AC) -pro140 $@ HAMRDISK
	dd if=$(PRODOS_SRC) of=$@ bs=512 count=2 conv=notrunc 2>/dev/null
	$(AC) -g $(PRODOS_SRC) PRODOS > /tmp/prodos_sys.bin
	$(AC) -p $@ PRODOS SYS 0x2000 < /tmp/prodos_sys.bin
	$(AC) -p $@ A.PICKER.SYSTEM SYS 0x2000 < $(PICKER_SYS)
	@echo "Menu volume ready: $@"

menu: $(MENU_PO)

$(JSON): $(VERILOG_SRC) $(MEM_FILES) $(FW_MEM_DEP) | $(BUILD_DIR) $(REPORT_DIR)
	@echo "=== Synthesizing $(DESIGN) with Yosys ==="
	@echo "Synthesis started at $$(date)" > $(SYNTH_LOG)
	@echo "Design: $(DESIGN)" >> $(SYNTH_LOG)
	@echo "Top module: $(TOP)" >> $(SYNTH_LOG)
	@echo "Source files: $(VERILOG_SRC)" >> $(SYNTH_LOG)
	@echo "" >> $(SYNTH_LOG)
	@# Copy any .mem files to build directory for synthesis
	@for f in $(MEM_FILES); do cp "$$f" $(BUILD_DIR)/; done
	cd $(BUILD_DIR) && $(YOSYS) -p "\
		read_verilog -DSYNTHESIS $(EXTRA_VFLAGS) $(addprefix ../,$(VERILOG_SRC)); \
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
		--seed $(PNR_SEED) \
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
    GTKWAVE  := $(OSS_CAD_SUITE)/gtkwave
else
    IVERILOG := iverilog
    VVP      := vvp
    GTKWAVE  := gtkwave
endif

# Simulation files - main design testbench + any helper testbench modules
SIM_MAIN_TB := $(DESIGN_DIR)/$(DESIGN)_tb.v
SIM_AUX_TB  := $(filter-out $(SIM_MAIN_TB),$(wildcard $(DESIGN_DIR)/*_tb.v))
SIM_MODELS  := $(wildcard $(DESIGN_DIR)/sim/*.v)
SIM_OUT := $(BUILD_DIR)/$(DESIGN)_tb.vvp
VCD     := $(BUILD_DIR)/$(DESIGN)_tb.vcd

# PLUSARGS: runtime test selection for the tb, e.g.
#   make sim PLUSARGS=+farmonly   farm phases only (fast iteration)
#   make sim PLUSARGS=+vcd        enable waveform dump (~20 GB full suite!)
# Default (no PLUSARGS) = full suite, no VCD = the pre-merge gate.
sim: $(SIM_OUT)
	@echo "=== Running Simulation ==="
	cd $(BUILD_DIR) && $(VVP) $(DESIGN)_tb.vvp $(PLUSARGS)
	@if [ -f $(VCD) ]; then echo "VCD written to $(VCD)"; fi

$(SIM_OUT): $(VERILOG_SRC) $(SIM_MODELS) $(SIM_MAIN_TB) $(SIM_AUX_TB) $(MEM_FILES) | $(BUILD_DIR)
	@echo "=== Compiling Testbench ==="
	@# Copy any .mem files to build directory for simulation
	@for f in $(MEM_FILES); do cp "$$f" $(BUILD_DIR)/; done
	$(IVERILOG) -o $@ -s $(DESIGN)_tb $(VERILOG_SRC) $(SIM_MODELS) $(SIM_MAIN_TB) $(SIM_AUX_TB)

wave: sim
	@echo "=== Opening Waveform Viewer ==="
	@if [ -f $(VCD) ]; then \
		$(GTKWAVE) $(VCD) & \
	else \
		echo "No VCD file found. Run 'make sim' first."; \
	fi

# Open any VCD file in GTKWave
# Usage: make gtk VCD=path/to/file.vcd
gtk:
ifndef VCD
	@echo "Usage: make gtk VCD=path/to/file.vcd"
	@exit 1
endif
	@echo "=== Opening $(VCD) in GTKWave ==="
	$(GTKWAVE) $(VCD) &

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

$(UNIT_OUT): $(VERILOG_SRC) $(SIM_MODELS) $(UNIT_TB) | $(BUILD_DIR)
	@echo "=== Compiling Unit Testbench: $(MODULE) ==="
	$(IVERILOG) -o $@ -s $(MODULE)_tb $(VERILOG_SRC) $(SIM_MODELS) $(UNIT_TB)

unit-wave: unit
	@echo "=== Opening Unit Test Waveform ==="
	@if [ -f $(UNIT_VCD) ]; then \
		$(GTKWAVE) $(UNIT_VCD) & \
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
SERIAL   ?= DP0517RX

prog: $(BIT)
	@echo "=== Programming via JTAG (SRAM) ==="
	$(LOADER) --cable $(CABLE) --ftdi-serial $(SERIAL) --pins $(JTAG_PINS) $(BIT)

prog-flash: $(BIT)
	@echo "=== Programming SPI Flash ==="
	$(LOADER) --cable $(CABLE) --ftdi-serial $(SERIAL) --pins $(JTAG_PINS) -f $(BIT)
	@/Users/hambook/Development/bell/bell

DISK_IMAGE ?= software/SMARTPORT/images/ProDOS_2_4_1.po
FLASH_IMG_OFFSET ?= 0x400000

prog-flash-with-image: $(BIT)
	@echo "=== Programming SPI Flash (bitstream + disk image) ==="
	$(LOADER) --cable $(CABLE) --ftdi-serial $(SERIAL) --pins $(JTAG_PINS) -f $(BIT)
	@echo "=== Writing disk image at offset $(FLASH_IMG_OFFSET) ==="
	$(LOADER) --cable $(CABLE) --ftdi-serial $(SERIAL) --pins $(JTAG_PINS) --offset $(FLASH_IMG_OFFSET) -f $(DISK_IMAGE)
	@echo "=== Done ==="
	@/Users/hambook/Development/bell/bell

DUMP_FILE ?= disk_dump.po
DUMP_SIZE ?= 12582912

read-flash:
	@echo "=== Reading disk image from flash ($(DUMP_SIZE) bytes at offset $(FLASH_IMG_OFFSET)) ==="
	$(LOADER) --cable $(CABLE) --ftdi-serial $(SERIAL) --pins $(JTAG_PINS) \
		--dump-flash --offset $(FLASH_IMG_OFFSET) --file-size $(DUMP_SIZE) $(DUMP_FILE)
	@echo "=== Saved to $(DUMP_FILE) ==="
	@/Users/hambook/Development/bell/bell

write-flash:
	@echo "=== Writing disk image to flash at offset $(FLASH_IMG_OFFSET) ==="
	$(LOADER) --cable $(CABLE) --ftdi-serial $(SERIAL) --pins $(JTAG_PINS) \
		--offset $(FLASH_IMG_OFFSET) -f $(DISK_IMAGE)
	@echo "=== Done ==="
	@/Users/hambook/Development/bell/bell

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

# project_obscurus SDRAM driver test (BRUN BIN at $2000)
SDM_DIR  := software/SDM
sdmtest:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) SDMTEST.S

cpreg:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPREG.S

cprace:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) racetask.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPRACE.S

cprace3:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) racetask3.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPRACE3.S

cmpskill:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) cmpskill.S

sdrtest:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) sdrtest.S

mverse:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) MVERSE.S

# LIFE8GR HW blob (LSIM=0, 48-row) -> DFB include for GRVERSE.
# sed flips the committed LSIM=1 literal and renames the DSK output so it does
# NOT clobber the sim LIFE8GR.bin (LSIM=1) used by the gateware life8gr.mem.
# LIFE8GRB.S is regenerated from the LSIM=0 bin, never hand-transcribed
# (NO internal dot in the name -> Merlin32 PUT-safe).
LIFE8GRHW_BIN := $(SDM_DIR)/LIFE8GRHW.bin
LIFE8GRB_S    := $(SDM_DIR)/LIFE8GRB.S

$(LIFE8GRHW_BIN): $(SDM_DIR)/LIFE8GR.S
	sed -e 's/^LSIM = 1/LSIM = 0/' -e 's/^ DSK LIFE8GR.bin/ DSK LIFE8GRHW.bin/' $< > $(SDM_DIR)/LIFE8GRHW.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) LIFE8GRHW.S

$(LIFE8GRB_S): $(LIFE8GRHW_BIN)
	{ echo 'CSKILL'; od -An -tx1 -v $< | awk '{for(i=1;i<=NF;i++)printf " DFB $$%s\n",toupper($$i)}'; echo 'CSKEND'; echo 'CSKLEN = CSKEND-CSKILL'; } > $@

grverse: $(LIFE8GRB_S)
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) GRVERSE.S

# LIFE8 HW blob (LSIM=0, 192-row) -> DFB include for CONWAYTEST.
# sed flips the committed LSIM=1 literal and renames the DSK output so it
# does NOT clobber the sim LIFE8.bin (LSIM=1) used by gateware life8.mem.
# LIFE8B.S is regenerated from the LSIM=0 bin, never hand-transcribed.
LIFE8HW_BIN := $(SDM_DIR)/LIFE8HW.bin
LIFE8B_S    := $(SDM_DIR)/LIFE8B.S

$(LIFE8HW_BIN): $(SDM_DIR)/LIFE8.S $(SDM_DIR)/LIFEMAP.S
	sed -e 's/^LSIM = 1/LSIM = 0/' -e 's/^ DSK LIFE8.bin/ DSK LIFE8HW.bin/' $< > $(SDM_DIR)/LIFE8HW.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) LIFE8HW.S

$(LIFE8B_S): $(LIFE8HW_BIN)
	{ echo 'LSKILL'; od -An -tx1 -v $< | awk '{for(i=1;i<=NF;i++)printf " DFB $$%s\n",toupper($$i)}'; echo 'LSKEND'; echo 'LSKLEN = LSKEND-LSKILL'; } > $@

life8:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) LIFE8.S
	python3 -c "b=open('$(SDM_DIR)/LIFE8.bin','rb').read(); open('gateware/rev2/project_obscurus/life8.mem','w').write('\n'.join('%02x'%x for x in b)+'\n')"

life8gr:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) LIFE8GR.S
	python3 -c "b=open('$(SDM_DIR)/LIFE8GR.bin','rb').read(); open('gateware/rev2/project_obscurus/life8gr.mem','w').write('\n'.join('%02x'%x for x in b)+'\n')"

# FARMTASK sim blob (FSIM=1 tiny dividers) -> farmtask.mem for the tb.
# Committed FARMTASK.S keeps FSIM=0 (hardware dividers); sim variant is
# generated, never hand-edited. Pattern mirrors LIFE8GRHW (lines 503-516).
FARMTASKSIM_BIN := $(SDM_DIR)/FARMTASKSIM.bin
FARMTASK_MEM := gateware/rev2/project_obscurus/farmtask.mem

# FARMTASK blob bound: ORG $2000, region $2000-$3FFF (itr4.1).
# Scratch now lives at $A000+ (out of the way) so the only bound is the
# 8 KB task region. Hard check after every assemble (stale-blob lesson).
FARMTASK_MAXLEN := 8192

$(FARMTASKSIM_BIN): $(SDM_DIR)/FARMTASK.S $(SDM_DIR)/FARMEQU.S $(SDM_DIR)/EVLIB.S
	sed -e 's/^ DSK FARMTASK.bin/ DSK FARMTASKSIM.bin/' $(SDM_DIR)/FARMTASK.S > $(SDM_DIR)/FARMTASKSIM.S
	sed -e 's/^FSIM = 0/FSIM = 1/' $(SDM_DIR)/FARMEQU.S > $(SDM_DIR)/FARMEQUS.S
	cd $(SDM_DIR) && sed -e 's/ PUT FARMEQU$$/ PUT FARMEQUS/' FARMTASKSIM.S > FARMTASKSIM.tmp && mv FARMTASKSIM.tmp FARMTASKSIM.S && $(MERLIN32) $(MERLIN_LIB) FARMTASKSIM.S
	@sz=$$(wc -c < $(FARMTASKSIM_BIN)); if [ $$sz -gt $(FARMTASK_MAXLEN) ]; then echo "FARMTASKSIM.bin $$sz bytes > $(FARMTASK_MAXLEN) (code exceeds \$$4000)"; rm -f $(FARMTASKSIM_BIN); exit 1; fi

$(FARMTASK_MEM): $(FARMTASKSIM_BIN)
	python3 -c "b=open('$(FARMTASKSIM_BIN)','rb').read(); open('$(FARMTASK_MEM)','w').write('\n'.join('%02x'%x for x in b)+'\n')"

farmtasksim: $(FARMTASK_MEM)

# stale-artifact guard (the hamr_rom.mem lesson): editing FARMTASK.S must
# rebuild farmtask.mem before any project_obscurus sim run.
ifeq ($(DESIGN),project_obscurus)
$(SIM_OUT): $(FARMTASK_MEM)
endif

# FARMTASK HW blob (FSIM=0) -> DFB include for FARM.S
FARMTASK_BIN := $(SDM_DIR)/FARMTASK.bin
FARMTASKB_S := $(SDM_DIR)/FARMTASKB.S

$(FARMTASK_BIN): $(SDM_DIR)/FARMTASK.S $(SDM_DIR)/FARMEQU.S $(SDM_DIR)/EVLIB.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) FARMTASK.S
	@sz=$$(wc -c < $(FARMTASK_BIN)); if [ $$sz -gt $(FARMTASK_MAXLEN) ]; then echo "FARMTASK.bin $$sz bytes > $(FARMTASK_MAXLEN) (code exceeds \$$4000)"; rm -f $(FARMTASK_BIN); exit 1; fi

$(FARMTASKB_S): $(FARMTASK_BIN)
	{ echo 'FSKILL'; od -An -tx1 -v $< | awk '{for(i=1;i<=NF;i++)printf " DFB $$%s\n",toupper($$i)}'; echo 'FSKEND'; echo 'FSKLEN = FSKEND-FSKILL'; } > $@

farm: $(FARMTASKB_S) $(WORKTASKB_S) $(SDM_DIR)/NPCBASE.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) FARM.S

FARMTEST_BIN := $(SDM_DIR)/FARMTEST.bin

$(FARMTEST_BIN): $(FARMTASKB_S) $(WORKTASKB_S) \
    $(SDM_DIR)/FARMTEST.S $(SDM_DIR)/FARMEQU.S \
    $(SDM_DIR)/SDRAMLIB.S $(SDM_DIR)/CPLIB.S \
    $(SDM_DIR)/NPCBASE.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) FARMTEST.S

farmtest: $(FARMTEST_BIN)

CONWAYTEST_BIN := $(SDM_DIR)/CONWAYTEST.bin

$(CONWAYTEST_BIN): $(LIFE8B_S) $(LIFE8GRB_S) \
    $(SDM_DIR)/CONWAYTEST.S $(SDM_DIR)/FARMEQU.S \
    $(SDM_DIR)/SDRAMLIB.S $(SDM_DIR)/CPLIB.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CONWAYTEST.S

conwaytest: $(CONWAYTEST_BIN)

# WORKTASK sim blob (FSIM=1 tiny dividers) -> worktask.mem for the tb.
# Committed WORKTASK.S keeps FSIM=0 (hardware dividers); sim variant is
# generated, never hand-edited. Pattern mirrors FARMTASK rules above.
WORKTASKSIM_BIN := $(SDM_DIR)/WORKTASKSIM.bin
WORKTASK_MEM := gateware/rev2/project_obscurus/worktask.mem

# WORKTASK blob bound: ORG $4000, region $4000-$5FFF (itr4.1).
# cap = $6000 - $4000 = 8192 bytes. Scratch lives at $A100+.
WORKTASK_MAXLEN := 8192

$(WORKTASKSIM_BIN): $(SDM_DIR)/WORKTASK.S $(SDM_DIR)/WORKEQU.S \
    $(SDM_DIR)/PORTLIB.S $(SDM_DIR)/EVLIB.S
	sed -e 's/^ DSK WORKTASK.bin/ DSK WORKTASKSIM.bin/' $(SDM_DIR)/WORKTASK.S > $(SDM_DIR)/WORKTASKSIM.S
	sed -e 's/^FSIM = 0/FSIM = 1/' $(SDM_DIR)/WORKEQU.S > $(SDM_DIR)/WORKEQUS.S
	cd $(SDM_DIR) && sed -e 's/ PUT WORKEQU$$/ PUT WORKEQUS/' WORKTASKSIM.S > WORKTASKSIM.tmp && mv WORKTASKSIM.tmp WORKTASKSIM.S && $(MERLIN32) $(MERLIN_LIB) WORKTASKSIM.S
	@sz=$$(wc -c < $(WORKTASKSIM_BIN)); if [ $$sz -gt $(WORKTASK_MAXLEN) ]; then echo "WORKTASKSIM.bin $$sz bytes > $(WORKTASK_MAXLEN) (code exceeds \$$6000)"; rm -f $(WORKTASKSIM_BIN); exit 1; fi

$(WORKTASK_MEM): $(WORKTASKSIM_BIN)
	python3 -c "b=open('$(WORKTASKSIM_BIN)','rb').read(); open('$(WORKTASK_MEM)','w').write('\n'.join('%02x'%x for x in b)+'\n')"

worktasksim: $(WORKTASK_MEM)

# stale-artifact guard: editing WORKTASK.S must rebuild worktask.mem
# before any project_obscurus sim run.
ifeq ($(DESIGN),project_obscurus)
$(SIM_OUT): $(WORKTASK_MEM)
endif

# WORKTASK HW blob (FSIM=0) -> DFB include for FARM.S
WORKTASK_BIN := $(SDM_DIR)/WORKTASK.bin
WORKTASKB_S := $(SDM_DIR)/WORKTASKB.S

$(WORKTASK_BIN): $(SDM_DIR)/WORKTASK.S $(SDM_DIR)/WORKEQU.S \
    $(SDM_DIR)/PORTLIB.S $(SDM_DIR)/EVLIB.S
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) WORKTASK.S
	@sz=$$(wc -c < $(WORKTASK_BIN)); if [ $$sz -gt $(WORKTASK_MAXLEN) ]; then echo "WORKTASK.bin $$sz bytes > $(WORKTASK_MAXLEN) (code exceeds \$$6000)"; rm -f $(WORKTASK_BIN); exit 1; fi

$(WORKTASKB_S): $(WORKTASK_BIN)
	{ echo 'WSKILL'; od -An -tx1 -v $< | awk '{for(i=1;i<=NF;i++)printf " DFB $$%s\n",toupper($$i)}'; echo 'WSKEND'; echo 'WSKLEN = WSKEND-WSKILL'; } > $@

worktask: $(WORKTASKB_S)

cpdemo: cmpskill
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPDEMO.S

cpsdrd: sdrtest
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPSDRD.S

cmpdelay:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) cmpdelay.S

cpwatch: cmpdelay
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPWATCH.S

cpsave:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPSAVE.S

cpboot:
	cd $(SDM_DIR) && $(MERLIN32) $(MERLIN_LIB) CPBOOT.S

# Build a bootable /SDRAM/ floppy with SDMTEST (BIN) + SDRAMLIB.S (TXT source).
# FRESH volume (NOT cp-base + delete-bulk): deleting big base files then
# re-importing reused freed blocks -> ProDOS read I/O errors / cross-links on the
# Apple (e.g. SDRAMLIB.S's first data block landing on a freed COPYIIPLUS block).
# A fresh -pro140 volume allocates our files contiguously = clean. Boot blocks +
# PRODOS + BASIC.SYSTEM are copied from the base so it still boots to ].
# AC_CLASSIC = the classic AppleCommander interface (-pro140/-p/-g); $(AC) = acx
# (used only for the TXT import --aux 0, which makes a sequential L=0 text file;
# classic ac would stamp L=8192 random-access -> ProDOS copy-util crashes).
SDM_PO     := $(SDM_DIR)/SDMTEST.po
AC_CLASSIC := java -jar /Users/hambook/Downloads/AppleCommander-ac-13.0.jar
sdmdisk: sdmtest cpreg cprace cprace3 cmpskill cpdemo cpsdrd cmpdelay cpwatch cpsave cpboot mverse grverse farm farmtest conwaytest
	rm -f $(SDM_PO)
	$(AC_CLASSIC) -pro140 $(SDM_PO) SDRAM
	dd if=$(PRODOS_SRC) of=$(SDM_PO) bs=512 count=2 conv=notrunc 2>/dev/null
	$(AC_CLASSIC) -g $(PRODOS_SRC) PRODOS > /tmp/sdm_prodos.sys
	$(AC_CLASSIC) -p $(SDM_PO) PRODOS SYS 0x2000 < /tmp/sdm_prodos.sys
	$(AC_CLASSIC) -g $(PRODOS_SRC) BASIC.SYSTEM > /tmp/sdm_basic.sys
	$(AC_CLASSIC) -p $(SDM_PO) BASIC.SYSTEM SYS 0x2000 < /tmp/sdm_basic.sys
	$(AC_CLASSIC) -p $(SDM_PO) SDMTEST BIN 0x2000 < $(SDM_DIR)/SDMTEST
	$(AC_CLASSIC) -p $(SDM_PO) CPREG BIN 0x6000 < $(SDM_DIR)/CPREG
	$(AC_CLASSIC) -p $(SDM_PO) CPRACE BIN 0x6000 < $(SDM_DIR)/CPRACE
	$(AC_CLASSIC) -p $(SDM_PO) CPRACE3 BIN 0x6000 < $(SDM_DIR)/CPRACE3
	$(AC_CLASSIC) -p $(SDM_PO) CPDEMO BIN 0x6000 < $(SDM_DIR)/CPDEMO
	$(AC_CLASSIC) -p $(SDM_PO) CPSDRD BIN 0x6000 < $(SDM_DIR)/CPSDRD
	$(AC_CLASSIC) -p $(SDM_PO) CPWATCH BIN 0x6000 < $(SDM_DIR)/CPWATCH
	$(AC_CLASSIC) -p $(SDM_PO) CPSAVE BIN 0x6000 < $(SDM_DIR)/CPSAVE
	$(AC_CLASSIC) -p $(SDM_PO) CPBOOT BIN 0x6000 < $(SDM_DIR)/CPBOOT
	$(AC_CLASSIC) -p $(SDM_PO) MVERSE BIN 0x6000 < $(SDM_DIR)/MVERSE.bin
	$(AC_CLASSIC) -p $(SDM_PO) GRVERSE BIN 0x6000 < $(SDM_DIR)/GRVERSE.bin
	$(AC_CLASSIC) -p $(SDM_PO) FARM BIN 0x2000 < $(SDM_DIR)/FARM.bin
	$(AC_CLASSIC) -p $(SDM_PO) FARM_TEST BIN 0x2000 < $(SDM_DIR)/FARMTEST.bin
	$(AC_CLASSIC) -p $(SDM_PO) CONWAY_TEST BIN 0x2000 < $(SDM_DIR)/CONWAYTEST.bin
	$(AC) import -d $(SDM_PO) -f --text -t TXT --aux 0 -n SDRAMLIB.S $(SDM_DIR)/SDRAMLIB.S
	$(AC) list -d $(SDM_PO)
	@echo "Disk ready (fresh /SDRAM/ volume): $(SDM_PO) — copy to ADTPro disks and send to floppy."

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

# Copy any disk image (.po/.dsk/.2mg) into the ADTPro disks folder
copy-dsk:
ifndef DSK
	@echo "Usage: make copy-dsk DSK=path/to/image.po"
	@exit 1
endif
	cp $(DSK) $(ADTPRO_DISKS)/
	@echo "Copied $(notdir $(DSK)) -> $(ADTPRO_DISKS)/"

# =============================================================================
# ESP32 Firmware (Yellow Hamr companion)
# =============================================================================
# Requires: source ~/esp/esp-idf/export.sh (once per terminal session)
# =============================================================================

ESP_FW_DIR   := $(GATEWARE_DIR)/smart_hamr/firmware
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
# RISC-V Firmware (PicoRV32 for Flash Hamr)
# =============================================================================
# Requires: riscv32-unknown-elf-gcc (brew install riscv-tools)
# =============================================================================

# Auto-detect RISC-V toolchain prefix (riscv64 can target RV32I with -march=rv32i)
ifneq ($(wildcard $(HOME)/oss-cad-suite/bin/riscv32-unknown-elf-gcc),)
    RISCV_PREFIX := $(HOME)/oss-cad-suite/bin/riscv32-unknown-elf-
else ifneq ($(shell which riscv32-unknown-elf-gcc 2>/dev/null),)
    RISCV_PREFIX := riscv32-unknown-elf-
else
    RISCV_PREFIX := riscv64-unknown-elf-
endif

RISCV_CC      := $(RISCV_PREFIX)gcc
RISCV_OBJCOPY := $(RISCV_PREFIX)objcopy
RISCV_OBJDUMP := $(RISCV_PREFIX)objdump
RISCV_SIZE    := $(RISCV_PREFIX)size

FW_DIR     := $(GATEWARE_DIR)/flash_hamr/firmware
FW_SRCS_C  := $(wildcard $(FW_DIR)/*.c)
FW_SRCS_S  := $(wildcard $(FW_DIR)/*.S)
FW_ELF     := $(BUILD_DIR)/firmware.elf
FW_BIN     := $(BUILD_DIR)/firmware.bin
FW_MEM     := $(GATEWARE_DIR)/flash_hamr/firmware.mem
FW_MAP     := $(BUILD_DIR)/firmware.map
FW_LST     := $(BUILD_DIR)/firmware.lst

RISCV_CFLAGS  := -march=rv32i -mabi=ilp32 -Os -Wall -Wextra -nostdlib -ffreestanding -ffunction-sections -fdata-sections
RISCV_LIBGCC  := $(shell $(RISCV_CC) -march=rv32i -mabi=ilp32 -print-libgcc-file-name)
RISCV_LDFLAGS := -T $(FW_DIR)/linker.ld -nostdlib -Wl,--gc-sections -Wl,-Map,$(FW_MAP)

.PHONY: firmware firmware-clean firmware-size

firmware: $(FW_MEM)

$(FW_ELF): $(FW_SRCS_C) $(FW_SRCS_S) $(FW_DIR)/linker.ld $(FW_DIR)/hal.h | $(BUILD_DIR)
	@echo "=== Compiling PicoRV32 firmware ==="
	$(RISCV_CC) $(RISCV_CFLAGS) -DBUILD_TS='"$(shell date +%Y-%m-%dT%H:%M:%S)"' $(RISCV_LDFLAGS) -o $@ $(FW_SRCS_S) $(FW_SRCS_C) $(RISCV_LIBGCC)
	@$(RISCV_SIZE) $@

$(FW_BIN): $(FW_ELF)
	$(RISCV_OBJCOPY) -O binary $< $@
	$(RISCV_OBJDUMP) -d $< > $(FW_LST)

$(FW_MEM): $(FW_BIN)
	python3 scripts/bin2mem.py $< $@ 4 32768
	@echo "Firmware ready: $@ ($$(wc -c < $(FW_BIN)) bytes code)"

firmware-size: $(FW_ELF)
	$(RISCV_SIZE) $@

firmware-clean:
	@rm -f $(FW_ELF) $(FW_BIN) $(FW_MEM) $(FW_MAP) $(FW_LST)
	@echo "Firmware build cleaned"

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

# ============================================================================
# B8008 — Intel 8008 coprocessor software (gateware/rev2/b8008_hamr)
# ============================================================================
B8008_DIR := software/B8008
B8008_PO  := $(B8008_DIR)/B8008.po

b8test:
	cd $(B8008_DIR) && $(MERLIN32) $(MERLIN_LIB) B8TEST.S

b8term:
	cd $(B8008_DIR) && $(MERLIN32) $(MERLIN_LIB) B8TERM.S

b8run:
	cd $(B8008_DIR) && $(MERLIN32) $(MERLIN_LIB) B8RUN.S

# HELLO8.S is //e-editable (no TYP/DSK) — assemble via a wrapper, then
# verify byte-identical to the ASL golden hex (the pre-merge MAC8008 gate).
hello8:
	cd $(B8008_DIR) && printf ' TYP $$06\n DSK HELLO8\n' | cat - HELLO8.S > .H8W.S \
	    && $(MERLIN32) $(MERLIN_LIB) .H8W.S && rm -f .H8W.S
	scripts/validate_mac8008.sh $(HOME)/Development/intel-8008-vhdl/test_programs/samples/hello_8008_ram.asm

# Regenerate MAC8008.S from the b8008 core's isa.json + revalidate 6 samples
mac8008:
	python3 scripts/gen_mac8008.py \
	    $(HOME)/Development/intel-8008-vhdl/docs/isa.json $(B8008_DIR)/MAC8008.S
	scripts/validate_mac8008.sh \
	    $(HOME)/Development/intel-8008-vhdl/test_programs/samples/hello_8008_ram.asm \
	    $(HOME)/Development/intel-8008-vhdl/test_programs/rotate_carry_test_as.asm \
	    $(HOME)/Development/intel-8008-vhdl/test_programs/conditional_call_test_as.asm \
	    $(HOME)/Development/intel-8008-vhdl/test_programs/alu_test_as.asm \
	    $(HOME)/Development/intel-8008-vhdl/test_programs/rst_test_as.asm \
	    $(HOME)/Development/intel-8008-vhdl/test_programs/mov_rr_test_as.asm

# Bootable workflow disk: tools as BIN, 8008 sources as TXT (edit in Merlin 8).
# HELLO8 BIN aux type 0x2040 = its 8008 ORG — B8RUN reads it for L/G.
b8008disk: b8test b8term b8run hello8
	rm -f $(B8008_PO)
	$(AC_CLASSIC) -pro140 $(B8008_PO) B8008
	dd if=$(PRODOS_SRC) of=$(B8008_PO) bs=512 count=2 conv=notrunc 2>/dev/null
	$(AC_CLASSIC) -g $(PRODOS_SRC) PRODOS > /tmp/b8_prodos.sys
	$(AC_CLASSIC) -p $(B8008_PO) PRODOS SYS 0x2000 < /tmp/b8_prodos.sys
	$(AC_CLASSIC) -g $(PRODOS_SRC) BASIC.SYSTEM > /tmp/b8_basic.sys
	$(AC_CLASSIC) -p $(B8008_PO) BASIC.SYSTEM SYS 0x2000 < /tmp/b8_basic.sys
	$(AC_CLASSIC) -p $(B8008_PO) B8TEST BIN 0x2000 < $(B8008_DIR)/B8TEST
	$(AC_CLASSIC) -p $(B8008_PO) B8TERM BIN 0x2000 < $(B8008_DIR)/B8TERM
	$(AC_CLASSIC) -p $(B8008_PO) B8RUN BIN 0x2000 < $(B8008_DIR)/B8RUN
	$(AC_CLASSIC) -p $(B8008_PO) HELLO8 BIN 0x2040 < $(B8008_DIR)/HELLO8
	$(AC) import -d $(B8008_PO) -f --text -t TXT --aux 0 -n MAC8008.S $(B8008_DIR)/MAC8008.S
	$(AC) import -d $(B8008_PO) -f --text -t TXT --aux 0 -n HELLO8.S $(B8008_DIR)/HELLO8.S
	$(AC) import -d $(B8008_PO) -f --text -t TXT --aux 0 -n B8LIB.S $(B8008_DIR)/B8LIB.S
	$(AC) list -d $(B8008_PO)
	@echo "Disk ready: $(B8008_PO) — BRUN B8TEST first, then B8RUN HELLO8."
	@echo "Send to ADTPro with: make copy-dsk DSK=$(B8008_PO)"

.PHONY: b8test b8term b8run hello8 mac8008 b8008disk
