# Software

Apple II and host-side software for the Byte Hamr FPGA expansion card.

## Directories

| Directory | Language | Purpose |
|-----------|----------|---------|
| [ASM](ASM/) | 6502 Assembly | Graphics demos, logic analyzer UI prototypes, and FPGA hardware test programs |
| [LOGICHAMR](LOGICHAMR/) | 6502 Assembly | Logic analyzer display program with full FPGA capture engine integration |
| [SMARTPORT](SMARTPORT/) | C + ARM Assembly + PIO | PicoPort: SmartPort disk emulator on Raspberry Pi Pico W with SD card support |
| [utils](utils/) | Bash | Disk image manipulation scripts (extract/create `.dsk` files via AppleCommander) |

## Workflow

1. Write 6502 assembly source (`.S` files)
2. Assemble with **Merlin32** cross-assembler
3. Package into ProDOS `.dsk` images using `utils/create_dsk.sh`
4. Transfer to Apple II via **ADTPro** or embed in FPGA flash
