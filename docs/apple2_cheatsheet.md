# Apple II Address Cheat Sheet

Quick reference for slot peripheral development.

## Slot Address Calculation

| Region | Address Formula | Range (Slot 4) | Purpose |
|--------|-----------------|----------------|---------|
| Device Select | $C080 + (slot × 16) | $C0C0-$C0CF | 16 I/O registers |
| I/O Select | $C000 + (slot × 256) | $C400-$C4FF | 256 bytes slot ROM |
| I/O Strobe | $C800-$CFFF | $C800-$CFFF | 2K shared expansion ROM |

### Decimal Equivalents (for BASIC)

| Slot | Device Select Base | I/O Select Base |
|------|-------------------|-----------------|
| 1 | 49296 ($C090) | 49408 ($C100) |
| 2 | 49312 ($C0A0) | 49664 ($C200) |
| 3 | 49328 ($C0B0) | 49920 ($C300) |
| 4 | 49344 ($C0C0) | 50176 ($C400) |
| 5 | 49360 ($C0D0) | 50432 ($C500) |
| 6 | 49376 ($C0E0) | 50688 ($C600) |
| 7 | 49392 ($C0F0) | 50944 ($C700) |

**Formula:** `BASE = 49280 + SLOT * 16` (Device Select)

## Common Soft Switches

| Address | Read | Write | Description |
|---------|------|-------|-------------|
| $C000 | Keyboard | 80STOREOFF | Last key pressed (bit 7 = strobe) |
| $C010 | Any | Any | Clear keyboard strobe |
| $C030 | Toggle | Toggle | Speaker click |
| $C050 | TXTCLR | TXTCLR | Graphics mode |
| $C051 | TXTSET | TXTSET | Text mode |
| $C052 | MIXCLR | MIXCLR | Full screen |
| $C053 | MIXSET | MIXSET | Mixed (4 lines text) |
| $C054 | PAGE1 | PAGE1 | Display page 1 |
| $C055 | PAGE2 | PAGE2 | Display page 2 |
| $C056 | LORES | LORES | Lo-res graphics |
| $C057 | HIRES | HIRES | Hi-res graphics |

## HIRES Screen Memory

| Page | Address Range | Size |
|------|---------------|------|
| Page 1 | $2000-$3FFF | 8K |
| Page 2 | $4000-$5FFF | 8K |

**Line address calculation:** Non-linear! Use lookup table.

```
Line 0:   $2000, $2400, $2800, $2C00, $3000, $3400, $3800, $3C00
Line 8:   $2080, $2480, $2880, $2C80, $3080, $3480, $3880, $3C80
Line 64:  $2028, $2428, $2828, $2C28, $3028, $3428, $3828, $3C28
...
```

Each line = 40 bytes. Bits 0-6 = pixels, bit 7 = palette.

## BASIC Examples

### Read from Card
```basic
10 SLOT = 4
20 BASE = 49280 + SLOT * 16
30 STATUS = PEEK(BASE + 4)
40 PRINT "STATUS: "; STATUS
```

### Write to Card
```basic
10 SLOT = 4
20 BASE = 49280 + SLOT * 16
30 POKE BASE + 0, 0: REM Select channel 0
40 POKE BASE + 3, 2: REM Send read command
50 DATA = PEEK(BASE + 2): REM Read result
```

### Enter HIRES Mode
```basic
10 HGR: REM Page 1, mixed mode
20 HGR2: REM Page 2, full screen
30 POKE 49234,0: REM Full screen (MIXCLR)
```

## Assembly Examples

### Slot-Independent Register Access
```asm
* X = slot number * 16 (e.g., $40 for slot 4)
        LDA $C080,X     ; Read register 0
        STA $C081,X     ; Write register 1
```

### Calculate Slot Offset
```asm
* Convert slot number (1-7) to offset
        LDA SLOT        ; Slot number
        ASL             ; ×2
        ASL             ; ×4
        ASL             ; ×8
        ASL             ; ×16
        TAX             ; X = slot × 16
```

### Wait for Ready Bit
```asm
WAIT    LDA $C084,X     ; Read status register
        AND #$02        ; Check ready bit
        BEQ WAIT        ; Loop until ready
```

## Hex/Decimal Conversion

| Hex | Decimal | Hex | Decimal |
|-----|---------|-----|---------|
| $00 | 0 | $80 | 128 |
| $10 | 16 | $90 | 144 |
| $20 | 32 | $A0 | 160 |
| $30 | 48 | $B0 | 176 |
| $40 | 64 | $C0 | 192 |
| $50 | 80 | $D0 | 208 |
| $60 | 96 | $E0 | 224 |
| $70 | 112 | $F0 | 240 |
| $FF | 255 | | |

## Byte Hamr Register Maps

### logic_hamr_v1 (Slot 4: $C0C0-$C0CF)

| Offset | Address | R/W | Name | Description |
|--------|---------|-----|------|-------------|
| $00 | $C0C0 | R/W | CHANNEL | Channel select (0-7) |
| $01 | $C0C1 | R/W | ADDR | Byte index (0-37) |
| $02 | $C0C2 | R | DATA | Read result from SDRAM |
| $03 | $C0C3 | W | CMD | $02=Read, $10=Regenerate |
| $04 | $C0C4 | R | STATUS | [0]=busy [1]=ready |
| $05 | $C0C5 | R/W | STRETCH | Pixels per sample (1-15) |

### signal_check (Slot 4: $C0C0-$C0CF)

| Offset | Address | R/W | Name | Description |
|--------|---------|-----|------|-------------|
| $00-$0F | $C0C0-$C0CF | R/W | REG[n] | General-purpose registers |

Reads return last written value. GPIO directly reflects card status.
