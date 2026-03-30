# CLAUDE.md

## Working Style
- **Never stop prematurely.** Do not suggest "shipping what works" and deferring bugs to later sessions. Keep debugging until the problem is solved or the user explicitly says to stop. This is a productivity blocker.
- When stuck, try a different approach — don't give up.

## Build Commands
- FPGA: `make clean && make DESIGN=block_hamr && make DESIGN=block_hamr prog-flash`
- With disk image: `make DESIGN=block_hamr prog-flash-with-image DISK_IMAGE=path.po`
- Pass extra Verilog defines: `make DESIGN=block_hamr EXTRA_VFLAGS="-DFOO"`
- Push: `gh auth setup-git && git push`
