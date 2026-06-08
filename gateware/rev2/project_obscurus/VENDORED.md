# Vendored: Arlet verilog-6502

- Source: https://github.com/Arlet/verilog-6502 (branch master)
- Files: cpu.v -> arlet_cpu.v, ALU.v -> arlet_alu.v (renamed flat for the
  Makefile $(DESIGN_DIR)/*.v glob; module names `cpu` / `ALU` unchanged)
- Author: Arlet Ottens. Free to use (see the header in each file).
- Used as the coprocessor CPU in coproc.v. RDY is a global clock-enable
  (RDY=0 freezes all register updates, read AND write); reset is active-high.
