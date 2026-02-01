# Decimation Module Requirements

## Module: `decimate_pack`

### Inputs
| Signal | Width | Description |
|--------|-------|-------------|
| `clk` | 1 | System clock |
| `rst` | 1 | Reset |
| `sample_in` | 1 | Sample value |
| `sample_valid` | 1 | Pulse high when sample_in valid |
| `stretch_factor` | 4 | Pixels per sample (1-15) |

### Outputs
| Signal | Width | Description |
|--------|-------|-------------|
| `byte_out` | 7 | Packed pixel data |
| `byte_valid` | 1 | Pulse high when byte_out ready |

### Internal State
| Signal | Width | Description |
|--------|-------|-------------|
| `bit_pos` | 3 | Current bit position (0-6) |
| `accum` | 7 | Byte accumulator |
| `stretch_cnt` | 4 | Pixels remaining for current sample |
| `sample_latched` | 1 | Latched sample value |

### Behavior

1. On `sample_valid`:
   - Load `stretch_cnt` with `stretch_factor`
   - Latch `sample_in` to `sample_latched`

2. Each clock while `stretch_cnt > 0`:
   - If `sample_latched == 1`: `accum |= (1 << bit_pos)`
   - Increment `bit_pos`
   - Decrement `stretch_cnt`
   - If `bit_pos == 7`:
     - Assert `byte_valid` for one cycle
     - Output `accum` on `byte_out`
     - Clear `accum` to 0
     - Reset `bit_pos` to 0

3. On `rst`:
   - Clear `accum`, `bit_pos`, `stretch_cnt`, `sample_latched`

### Test Vector

**Input:**
- 88 samples: `01010101...` (alternating)
- `stretch_factor`: 3

**Expected Output:**
- 38 bytes
- Renders as 3-pixel-wide square wave
- First byte: `$38` (000111000 → 3 low, 3 high, 1 low)

### Constraints
- No bit swapping required
- LSB-first packing
- Bit 7 unused (Apple II palette bit)
- `byte_out` max value: `$7F`