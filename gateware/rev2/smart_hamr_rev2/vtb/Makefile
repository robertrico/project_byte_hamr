# Verilator tb: iwm_asic read framing vs the A2M2053 SmartPort wire code (software IWM oracle).
# The A2M2053 sources are compiled from their repo by path, never copied here.
#   make            build and run
#   make A2M_DIR=/path/to/a2m2053_controller
A2M_DIR   ?= $(HOME)/Development/a2m2053_controller
A2M_SRC   := $(A2M_DIR)/firmware/src
VERILATOR ?= verilator
CC        ?= cc
BUILD     := build

all: run

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/sp_wire.o: $(A2M_SRC)/sp_wire.c $(A2M_SRC)/sp_wire.h | $(BUILD)
	$(CC) -std=c11 -O2 -Wall -I$(A2M_SRC) -c -o $@ $<

$(BUILD)/Vtb: ../iwm_asic.v tb_iwm_framing.cpp $(BUILD)/sp_wire.o
	$(VERILATOR) --cc --exe --build -O2 -Wno-fatal -Wno-lint --top-module iwm_asic \
		-Mdir $(BUILD)/obj -o ../Vtb \
		-CFLAGS "-std=c++17 -I$(A2M_SRC)" -LDFLAGS "$(abspath $(BUILD)/sp_wire.o)" \
		../iwm_asic.v tb_iwm_framing.cpp

run: $(BUILD)/Vtb
	./$(BUILD)/Vtb

clean:
	rm -rf $(BUILD)

.PHONY: all run clean
