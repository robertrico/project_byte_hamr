TARGET      := picoport
TEST_TARGET := picoport_test
BUILD       := build
ELF         := $(BUILD)/$(TARGET).elf
TEST_ELF    := $(BUILD)/$(TEST_TARGET).elf
OPENOCD     := openocd
OCD_CFG     := -f interface/cmsis-dap.cfg -f target/rp2040.cfg

# Disk image — override with: make DISK=images/Merlin.po flash
DISK        ?= images/ProDOS_2_4_1.po

# Serial port — override with: make monitor PORT=/dev/cu.usbmodemXXXX
PORT     ?= $(shell ls /dev/tty.usbmodem* 2>/dev/null | head -1)
BAUD     := 115200

.PHONY: all clean flash reset monitor gdb openocd test test-flash convert

# Build
all: $(BUILD)/Makefile
	@cmake --build $(BUILD) -j$(shell sysctl -n hw.ncpu)

$(BUILD)/Makefile:
	@mkdir -p $(BUILD)
	@cmake -B $(BUILD) -S . -DDISK_IMAGE=$(DISK)

# Nuke build dir and rebuild
clean:
	@rm -rf $(BUILD)

# Flash via SWD
flash: all
	$(OPENOCD) $(OCD_CFG) -c "program $(ELF) verify reset exit"
	@/Users/hambook/Development/bell/bell

# Reset target without flashing
reset:
	$(OPENOCD) $(OCD_CFG) -c "init; reset run; exit"

# Serial monitor
monitor:
	@echo "Connecting to $(PORT) at $(BAUD) baud..."
	@minicom -D $(PORT) -b $(BAUD)

# OpenOCD server (terminal 1)
openocd:
	$(OPENOCD) $(OCD_CFG)

# GDB session (terminal 2, while openocd is running)
gdb: all
	arm-none-eabi-gdb $(ELF) -ex "target remote :3333" -ex "load" -ex "monitor reset halt"

# Build test firmware
test: $(BUILD)/Makefile
	@cmake --build $(BUILD) --target $(TEST_TARGET) -j$(shell sysctl -n hw.ncpu)

# Flash test firmware via SWD
test-flash: test
	$(OPENOCD) $(OCD_CFG) -c "program $(TEST_ELF) verify reset exit"

# Convert .dsk to .po:  make convert SRC=game.dsk
convert:
	@test -n "$(SRC)" || (echo "Usage: make convert SRC=file.dsk"; exit 1)
	python3 dsk2po.py $(SRC) $(basename $(SRC)).po
