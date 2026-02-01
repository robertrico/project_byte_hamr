#!/bin/bash
# Extract files from an Apple II DSK image using AppleCommander
# Uses raw extraction to preserve exact formatting
# Usage: ./extract_dsk.sh [--force] <dsk_file>

AC_JAR="/Users/hambook/Downloads/AppleCommander-ac-13.0.jar"
ADTPRO_DISKS="/Applications/ADTPro-v.r.m/disks"

FORCE=false
if [[ "$1" == "--force" ]]; then
    FORCE=true
    shift
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 [--force] <dsk_file> [output_dir]"
    exit 1
fi

DSK_FILE="$1"
OUTPUT_BASE="${2:-.}"

# Check local path first, then ADTPro disks directory
if [[ ! -f "$DSK_FILE" ]]; then
    if [[ -f "$ADTPRO_DISKS/$DSK_FILE" ]]; then
        DSK_FILE="$ADTPRO_DISKS/$DSK_FILE"
    else
        echo "Error: File not found: $DSK_FILE"
        echo "       Also checked: $ADTPRO_DISKS/$DSK_FILE"
        exit 1
    fi
fi

if [[ ! -f "$AC_JAR" ]]; then
    echo "Error: AppleCommander not found at $AC_JAR"
    exit 1
fi

# Create output directory named after the disk
BASENAME="${DSK_FILE##*/}"
NAME="${BASENAME%.*}"
OUTPUT_DIR="$OUTPUT_BASE/$NAME"

if [[ -d "$OUTPUT_DIR" && "$FORCE" != "true" ]]; then
    echo "Error: Directory already exists: $OUTPUT_DIR"
    echo "       Use --force to overwrite"
    exit 1
fi

mkdir -p "$OUTPUT_DIR"

echo "Extracting $DSK_FILE to $OUTPUT_DIR/"

# Get list of files (parse the -l output)
java -jar "$AC_JAR" -l "$DSK_FILE" | tail -n +2 | while read -r line; do
    # Skip empty lines and the summary line
    [[ -z "$line" ]] && continue
    [[ "$line" == *"format;"* ]] && continue

    # Extract filename (first field, trimmed)
    FILENAME=$(echo "$line" | awk '{print $1}')
    [[ -z "$FILENAME" ]] && continue

    FILETYPE=$(echo "$line" | awk '{print $2}')

    echo "  $FILENAME ($FILETYPE)"

    if [[ "$FILETYPE" == "TXT" ]]; then
        # Text file: raw get, strip high bit, convert $8D to LF
        java -jar "$AC_JAR" -g "$DSK_FILE" "$FILENAME" 2>/dev/null | \
            LC_ALL=C tr '\215' '\012' | \
            LC_ALL=C tr '\200-\377' '\000-\177' > "$OUTPUT_DIR/$FILENAME"
    else
        # Binary: raw get, no conversion
        java -jar "$AC_JAR" -g "$DSK_FILE" "$FILENAME" > "$OUTPUT_DIR/$FILENAME" 2>/dev/null
    fi
done

echo "Done."
ls -la "$OUTPUT_DIR"
