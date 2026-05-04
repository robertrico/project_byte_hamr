# Pin mapping — smart_hamr_rev2

LPF: `smart_hamr_rev2.lpf`. Identical layout to `smart_hamr/smart_hamr.lpf`
(daughter-board mapping shared).

## Apple II slot

| Signal | FPGA pin | Direction | Notes |
|--------|---------|-----------|-------|
| sig_7M | A9 | in | 7.16 MHz |
| Q3 | B10 | in | 2 MHz IWM strobe |
| PHI0 / PHI1 | A11 / A10 | in | unused |
| R_nW | D10 | in | |
| nDEVICE_SELECT | D16 | in | $C0Cx assertion |
| nI_O_SELECT | A19 | in | $C400-$C4FF |
| nI_O_STROBE | C11 | in | $C800-$CFFF |
| nRES | A8 | out (open-drain) | always released |
| nRES_READ | A5 | in | Apple II reset monitor |
| A0..A15 | (see LPF) | in | individually constrained |
| D0..D7 | (see LPF) | inout | tri-state by `data_oe` |
| DATA_OE | B20 | out | level-shifter U12 OE (active-low) |

## GPIO header (FujiNet bridge)

Daughter-board ribbon mapping. ESP32 column on right of perfboard.

| GPIO | FPGA | Direction | Signal | ESP32 GPIO |
|------|------|-----------|--------|------------|
| GPIO_1 | P3 | out | nDEVICE_SELECT (LA debug) | — |
| GPIO_2 | P1 | out | tied 0 | — |
| GPIO_3 | P2 | out | nRES_READ (LA debug) | — |
| GPIO_4..GPIO_8 | N1, M1, L2, L1, K3 | out | tied 0 (reserved LA debug) | — |
| GPIO_9 | K1 | out | _enbl2 | IO3/RX |
| GPIO_10 | K2 | out | wrdata (FAST slew) | IO22 |
| GPIO_11 | J1 | out | tied 0 | — |
| GPIO_12 | H2 | in | rddata (idle-LOW, inverted internally) | IO14 |
| GPIO_13 | H1 | in | sense (PULLMODE=UP) | IO27 |
| GPIO_14 | G2 | out | _wrreq | IO26 |
| GPIO_15 | G1 | out | phase[1] | IO33 |
| GPIO_16 | F2 | out | phase[0] (damped pad) | IO32 |
| GPIO_17 | F1 | out | phase[3] | IO35 |
| GPIO_18 | E2 | out | phase[2] | IO34 |
| GPIO_19 | E1 | out | tied 0 | — |
| GPIO_20 | D2 | out | _enbl1 | IO36 |

## Pad properties

- `wrdata` (GPIO_10): SLEWRATE=FAST so bit-cell edges stay clean.
- `rddata` (GPIO_12): PULLMODE=DOWN (matches ESP32 idle-LOW).
- `sense` (GPIO_13): PULLMODE=UP (defined idle when ESP32 not driving).
- All others: SLEWRATE=SLOW, PULLMODE=NONE/UP per slot defaults.
