#!/bin/bash
# Create a ProDOS .dsk image from a folder of files and move to ADTPro
# Uses raw put with proper Apple II text conversion
# Usage: ./create_dsk.sh <folder> [output.dsk]

AC_JAR="/Users/hambook/Downloads/AppleCommander-ac-13.0.jar"
ADTPRO_DISKS="/Applications/ADTPro-v.r.m/disks"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <folder> [output.dsk]"
    exit 1
fi

FOLDER="$1"

if [[ ! -d "$FOLDER" ]]; then
    echo "Error: Folder not found: $FOLDER"
    exit 1
fi

if [[ ! -f "$AC_JAR" ]]; then
    echo "Error: AppleCommander not found at $AC_JAR"
    exit 1
fi

# Get folder basename
BASENAME="${FOLDER%/}"
BASENAME="${BASENAME##*/}"

# Output disk name
if [[ $# -ge 2 ]]; then
    DSK_FILE="$2"
else
    DSK_FILE="${BASENAME}.dsk"
fi

# Volume name (max 15 chars for ProDOS)
VOLNAME="${BASENAME:0:15}"

echo "Creating ProDOS disk: $DSK_FILE (volume: $VOLNAME)"
java -jar "$AC_JAR" -pro140 "$DSK_FILE" "$VOLNAME"

# Add each file
for FILE in "$FOLDER"/*; do
    [[ -f "$FILE" ]] || continue

    FILENAME="${FILE##*/}"
    EXT="${FILENAME##*.}"
    EXT_LOWER=$(echo "$EXT" | tr '[:upper:]' '[:lower:]')

    # Determine file type from extension
    case "$EXT_LOWER" in
        s|txt)
            TYPE="TXT"
            IS_TEXT=true
            ;;
        bas)
            TYPE="BAS"
            IS_TEXT=false
            ;;
        bin|obj)
            TYPE="BIN"
            ADDR="0x2000"
            IS_TEXT=false
            ;;
        *)
            TYPE="TXT"
            IS_TEXT=true
            ;;
    esac

    echo "  Adding: $FILENAME ($TYPE)"
    if [[ "$TYPE" == "BIN" ]]; then
        java -jar "$AC_JAR" -p "$DSK_FILE" "$FILENAME" "$TYPE" "$ADDR" < "$FILE"
    elif [[ "$IS_TEXT" == "true" ]]; then
        # Text file: convert LF to CR, set high bit on all bytes
        cat "$FILE" | \
            LC_ALL=C tr '\012' '\015' | \
            LC_ALL=C tr '\000-\177' '\200-\377' | \
            java -jar "$AC_JAR" -p "$DSK_FILE" "$FILENAME" TXT
    else
        java -jar "$AC_JAR" -p "$DSK_FILE" "$FILENAME" "$TYPE" < "$FILE"
    fi
done

echo ""
java -jar "$AC_JAR" -l "$DSK_FILE"

# Move to ADTPro
mv "$DSK_FILE" "$ADTPRO_DISKS/"
echo ""
echo "Moved to: $ADTPRO_DISKS/$DSK_FILE"
