# Apple II Disk Image Utilities

Shell scripts for working with Apple II `.dsk` disk images using AppleCommander.

## Scripts

### extract_dsk.sh

Extracts all files from a `.dsk` image to a folder.

```bash
./extract_dsk.sh <dsk_file>
```

Creates a folder named after the disk image containing all extracted files.

If the file isn't found locally, it checks the ADTPro disks directory automatically.

### create_dsk.sh

Creates a ProDOS `.dsk` image from a folder of files and moves it to ADTPro.

```bash
./create_dsk.sh <folder> [output.dsk]
```

If no output filename is specified, uses the folder name with `.dsk` extension.

## Configuration

Edit the paths at the top of each script:

```bash
AC_JAR="/path/to/AppleCommander-ac-13.0.jar"
ADTPRO_DISKS="/Applications/ADTPro-v.r.m/disks"
```

## Makefile Targets

From the project root:

```bash
make create-dsk DSK=LOGICHAMR           # Creates from software/LOGICHAMR/
make extract-dsk DSK=LOGICHAMR.dsk     # Fails if dir exists
make extract-dsk DSK=LOGICHAMR.dsk FORCE=1
make list-dsk                           # List disks in ADTPro folder
```

## Why Raw Mode?

These scripts use AppleCommander's raw modes (`-g` and `-p`) instead of the text export modes (`-e`, `-x`, `-pt`, `-ptx`) for an important reason:

**AppleCommander's `-e` and `-x` export modes expand leading spaces for Merlin assembler column alignment.** A single leading space becomes 10 spaces to align with Merlin's column structure.

This breaks round-trip editing:
1. Extract a file with `-e` or `-x`
2. Edit it on your modern computer
3. Put it back on the disk
4. **Result:** Code is shifted right, breaking the layout

Raw extraction with `-g` preserves the exact byte layout, enabling clean round-trip workflows.

## Apple II Text Encoding

Apple II text files have two differences from modern text:

| Property | Apple II | Modern |
|----------|----------|--------|
| Character encoding | High bit set (`$80`-`$FF`) | ASCII (`$00`-`$7F`) |
| Line endings | CR with high bit (`$8D`) | LF (`$0A`) |

### Extraction (Apple II → Modern)

```bash
# Strip high bit, convert CR to LF
tr '\215' '\012' | tr '\200-\377' '\000-\177'
```

### Creation (Modern → Apple II)

```bash
# Convert LF to CR, set high bit
tr '\012' '\015' | tr '\000-\177' '\200-\377'
```

## File Type Mapping

| Extension | ProDOS Type | Notes |
|-----------|-------------|-------|
| `.s`, `.txt` | TXT | Text conversion applied |
| `.bas` | BAS | Applesoft BASIC (binary) |
| `.bin`, `.obj` | BIN | Binary, default load address $2000 |
| Other | TXT | Treated as text |

## Requirements

- Java runtime
- [AppleCommander](https://applecommander.github.io/) (ac JAR)
- [ADTPro](https://adtpro.com/) (optional, for disk transfer)
