#!/bin/bash
# build_menu.sh — Build Flash Hamr base volume (menu.po)
#
# Creates a 140KB ProDOS volume with:
#   - PRODOS (system file, copied from source image)
#   - A.PICKER.SYSTEM (auto-run, assembled from assemble_picker.py)
#
# Requires: java, AppleCommander

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
AC_JAR="/Users/hambook/Development/AppleCommander-ac-13.0.jar"
AC="java -jar $AC_JAR"

# Source ProDOS image (contains PRODOS system file)
PRODOS_SRC="${SCRIPT_DIR}/../../software/SMARTPORT/ProDOS_2_4_1.po"

OUTPUT="${SCRIPT_DIR}/menu.po"
PICKER="${SCRIPT_DIR}/picker.sys"

echo "=== Building Flash Hamr base volume ==="

# Step 1: Assemble picker
echo "Assembling picker..."
cd "$SCRIPT_DIR"
python3 assemble_picker.py

# Step 2: Create blank 140KB ProDOS volume
echo "Creating blank ProDOS volume..."
$AC -pro140 "$OUTPUT" HAMRDISK

# Step 3: Copy boot blocks from real ProDOS image
# AppleCommander's -pro140 writes a branding boot block, not the real loader.
# Copy blocks 0-1 from the source ProDOS image.
echo "Copying ProDOS boot blocks..."
dd if="$PRODOS_SRC" of="$OUTPUT" bs=512 count=2 conv=notrunc 2>/dev/null

# Step 4: Copy PRODOS system file from source image
echo "Extracting PRODOS from source..."
$AC -g "$PRODOS_SRC" PRODOS > /tmp/prodos_sys.bin

echo "Adding PRODOS to volume..."
$AC -p "$OUTPUT" PRODOS SYS 0x2000 < /tmp/prodos_sys.bin

# Step 5: Add picker as A.PICKER.SYSTEM (auto-run: first .SYSTEM file alphabetically)
echo "Adding A.PICKER.SYSTEM..."
$AC -p "$OUTPUT" A.PICKER.SYSTEM SYS 0x2000 < "$PICKER"

# Step 6: Show contents
echo ""
echo "Volume contents:"
$AC -ll "$OUTPUT"

echo ""
echo "Base volume ready: $OUTPUT"
echo "Flash with: make DESIGN=flash_hamr prog-flash-with-image DISK_IMAGE=$OUTPUT"
