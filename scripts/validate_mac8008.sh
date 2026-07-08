#!/bin/bash
# Validate MAC8008.S: translate ASL samples -> assemble with Merlin32 ->
# diff byte-for-byte against the ASL golden hex from the b8008 repo.
# Merlin32 is the cross twin of Merlin 8; final acceptance is the same
# source pair on the //e.
set -e
B8008=${B8008:-$HOME/Development/intel-8008-vhdl}
MERLIN32=${MERLIN32:-$HOME/Development/Merlin32_v1.2/MacOs/Merlin32}
MERLIN_LIB=${MERLIN_LIB:-$HOME/Development/Merlin32_v1.2/Library}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

cp "$ROOT/software/B8008/MAC8008.S" "$WORK/"
pass=0; fail=0

for asm in "$@"; do
    name=$(basename "$asm" .asm)
    hex="${asm%.asm}.hex"
    [ -f "$hex" ] || { echo "SKIP $name: no golden hex"; continue; }

    python3 "$ROOT/scripts/asl2merlin.py" "$asm" "$WORK/T.S" > /dev/null
    # Merlin32 needs TYP/DSK; org must come after them
    { echo " TYP \$06"; echo " DSK TOUT"; cat "$WORK/T.S"; } > "$WORK/TEST.S"
    ( cd "$WORK" && "$MERLIN32" "$MERLIN_LIB" TEST.S > merlin.log 2>&1 ) || {
        echo "FAIL $name: Merlin32 error"; tail -5 "$WORK/merlin.log"; fail=$((fail+1)); continue; }
    [ -f "$WORK/TOUT" ] || { echo "FAIL $name: no object produced"; tail -5 "$WORK/merlin.log"; fail=$((fail+1)); continue; }

    python3 - "$hex" "$WORK/TOUT" <<'EOF' && pass=$((pass+1)) || fail=$((fail+1))
import sys
golden = {}
base = None
for line in open(sys.argv[1]):
    line = line.strip()
    if not line.startswith(':'): continue
    n = int(line[1:3],16); addr = int(line[3:7],16); typ = int(line[7:9],16)
    if typ != 0: continue
    if base is None: base = addr
    base = min(base, addr)
    for i in range(n):
        golden[addr+i] = int(line[9+i*2:11+i*2],16)
merlin = open(sys.argv[2],'rb').read()
bad = 0
for addr, gb in sorted(golden.items()):
    off = addr - base
    mb = merlin[off] if 0 <= off < len(merlin) else None
    if mb != gb:
        if bad < 5:
            print(f"    {addr:04X}: golden={gb:02X} merlin={'--' if mb is None else f'{mb:02X}'}")
        bad += 1
name = sys.argv[1].split('/')[-1].replace('.hex','')
if bad: print(f"FAIL {name}: {bad}/{len(golden)} bytes differ"); sys.exit(1)
print(f"PASS {name}: {len(golden)}/{len(golden)} bytes identical")
EOF
done
echo "---"
echo "validate_mac8008: $pass pass, $fail fail"
[ "$fail" -eq 0 ]
