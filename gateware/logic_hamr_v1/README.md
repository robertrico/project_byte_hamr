# Decimation Algorithm Findings

## Overview

This document captures the technical findings for converting raw sample data into Apple II HIRES-compatible byte streams for the Byte Hamr logic analyzer.

## Apple II HIRES Byte Format

### Bit Layout
- Bits 0-6: Pixel data (7 pixels per byte)
- Bit 7: Palette select (unused for waveform rendering)
- Maximum byte value: $7F (all pixels on)

### Pixel Ordering
- Bit 0 = leftmost pixel on screen
- Bit 6 = rightmost pixel on screen
- First sample in time = bit 0 = leftmost on screen

### Example Values
| Hex | Binary | Screen Appearance |
|-----|--------|-------------------|
| $00 | 0000000 | All pixels off |
| $7F | 1111111 | All pixels on |
| $38 | 0111000 | 3 off, 3 on, 1 off |
| $07 | 0000111 | 3 on, 4 off |

## Rendering Model

### Channel Dimensions
- 8 channels total
- 12 horizontal lines per channel
- 38 columns (bytes) across screen width
- 266 usable pixels (38 × 7)

### Waveform Representation
- HIGH signal: Pixels on in top row
- LOW signal: Pixels on in bottom row
- Transition: Vertical line connecting top to bottom
- Sample data bytes represent horizontal cross-section at each row

## Decimation Algorithm

### Core Concept
The algorithm is a FIFO-based pipeline that decouples sample rate from byte boundaries.

### Pipeline Stages
```
[Raw Samples] → [Stretch] → [Pixel FIFO] → [7-bit Pack] → [Byte Stream] → [SDRAM]
```

### State Machine Variables
```
bit_pos      : 0-6, current position within output byte
accum        : byte accumulator, builds LSB-first
sample_val   : current sample (0 or 1)
stretch_cnt  : pixels remaining for current sample
```

### Algorithm Pseudocode
```
bit_pos = 0
accum = 0

for each sample in capture_buffer:
    for i = 0 to (pixels_per_sample - 1):
        if sample == 1:
            accum |= (1 << bit_pos)
        bit_pos++
        if bit_pos == 7:
            emit(accum)
            accum = 0
            bit_pos = 0
```

### Key Properties
1. No bit swapping required - LSB-first packing matches Apple II expectations
2. Sample boundaries do not align with byte boundaries
3. Each byte may contain parts of multiple samples
4. Each sample may span multiple bytes
5. Continuous pixel stream, chunked into 7-bit segments on output

## Stretch Factor (Pixels Per Sample)

### Relationship to Zoom
| Pixels/Sample | Visual Result |
|---------------|---------------|
| 1 | Compressed, transitions blur together |
| 2 | Marginal, transitions visible but tight |
| 3+ | Clean, distinct high/low/transition regions |

### Recommended Minimum
3 pixels per sample for clear waveform visibility.

### Screen Capacity at Different Stretch Factors
| Pixels/Sample | Samples on Screen | Time Window (1 MHz) |
|---------------|-------------------|---------------------|
| 1 | 266 | 266 µs |
| 2 | 133 | 133 µs |
| 3 | 88 | 88 µs |
| 7 | 38 | 38 µs |

## Worked Example

### Input
- Sample sequence: `0 1 0 1`
- Stretch factor: 3 pixels per sample

### Pixel Stream (after stretch)
```
0 0 0 1 1 1 0 0 0 1 1 1
```

### Byte Packing (LSB-first)
```
Byte 0 (pixels 0-6):
  bit 0 = 0, bit 1 = 0, bit 2 = 0
  bit 3 = 1, bit 4 = 1, bit 5 = 1
  bit 6 = 0
  Result: $38

Byte 1 (pixels 7-13):
  bit 0 = 0, bit 1 = 0, bit 2 = 1
  bit 3 = 1, bit 4 = 1, bit 5 = (next data)
  ...
```

### Screen Output for $38
```
○ ○ ○ ● ● ● ○
```
Matches expected: 3 low, 3 high, 1 low (start of next period)

## Test Patterns

### Verified Patterns
| Pattern | Hex Sequence | Display |
|---------|--------------|---------|
| All high | 7F 7F 7F... | Solid horizontal line |
| All low | 00 00 00... | Empty row |
| Alternating columns | 00 7F 00 7F... | Clean square wave, 1 column period |

### Button Debounce Example (Realistic)
```
Samples: HHHH...HHHH L HH LL H LLL H LLLL...LLLL (bounce pattern)
```
Produces non-uniform byte values at transition points, uniform $7F or $00 during stable periods.

## FPGA Implementation Notes

### FIFO Approach
- Input: samples at capture rate
- Output: bytes at display rate
- Decouples sample timing from byte boundaries
- Buffer depth flexible (single register to deep FIFO)

### No Byte-Boundary Alignment Required
The algorithm handles arbitrary sample/byte phase relationships. No special cases for alignment.

### Transition Detection
Not required in the byte-packing stage. Transitions appear naturally when sample value changes. Vertical line rendering is handled by the 6502 display code comparing first/last row data.

## Integration with Apple II Software

### Data Contract
FPGA provides 38 bytes per channel. Apple II software handles all rendering:
1. Reads 38 bytes per channel from SDRAM
2. Writes to HIRES addresses via LUT
3. XOR between first/last row detects transitions
4. Draws vertical lines where transitions detected

### Memory Layout in SDRAM
```
Channel 0: [38 bytes]
Channel 1: [38 bytes]
Channel 2: [38 bytes]
Channel 3: [38 bytes]
Channel 4: [38 bytes]
Channel 5: [38 bytes]
Channel 6: [38 bytes]
Channel 7: [38 bytes]
```

Total per screen: 38 × 8 = 304 bytes

### FPGA Responsibility
Fill these 304 bytes with decimated sample data. The Apple II rendering code already exists and works - verified with test pattern `00 7F 00 7F...` producing clean square waves.

## Path Forward

### Immediate Next Steps
1. Implement stretch + pack algorithm in FPGA gateware
2. Test with known patterns ($00, $7F, alternating)
3. Verify against Apple II rendering

### Validation Targets
1. 50 kHz square wave at 1 MHz sample rate
2. Button debounce capture
3. Scroll through captured buffer

### Success Criteria
- Waveform visually matches expected signal shape
- No byte-boundary artifacts
- Zoom (stretch factor change) produces expected scaling