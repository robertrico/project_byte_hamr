#!/usr/bin/env python3
"""Generate the Byte Hamr Rev 3 FPGA pin map (LFE5U-25F TQFP-144).

Reads Lattice's ECP5U-25 pinout CSV, assigns every Rev 3 signal to a
pin grouped by package side, cross-checks against the Trellis database,
and writes hardware/byte_hamr/rev3_pinmap.csv.

NOTE: hardware/byte_hamr/rev3_pinmap.csv now holds the placement-optimized
map from the Rev 3 pass 1 layout (reason "placement-optimized"). Running this
script overwrites it with the original side-based map.

Sides (pin ranges): 1-36 banks 7/6, 37-72 bank 8 + config + bank 3,
73-108 banks 3/2, 109-144 banks 1/0. Slot bus on sides 1 and 3 plus two
bank 3 pins; PSRAM, oscillator and header on side 4, bank 8, bank 3.
"""
import csv, json, os, re, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PINOUT = os.path.join(ROOT, 'docs', 'FPGA-SC-02033-2-0-ECP5U-25-Pinout.csv')
TRELLIS = os.path.expanduser(
    '~/oss-cad-suite/share/trellis/database/ECP5/LFE5U-25F/iodb.json')
OUT = os.path.join(ROOT, 'hardware', 'byte_hamr', 'rev3_pinmap.csv')

rows = list(csv.reader(open(PINOUT)))
hdr = [i for i, r in enumerate(rows) if r and r[0] == 'PAD'][0]
pins = {}
for r in rows[hdr + 1:]:
    if len(r) > 10 and r[10].strip() not in ('', '-'):
        pins[int(r[10])] = dict(func=r[1], bank=r[2], dual=r[3])
assert len(pins) == 144
isio = lambda p: re.match(r'^P[LRTB]\d+[A-D]$', pins[p]['func'])
io = [p for p in pins if isio(p)]
assert len(io) == 98
tr = json.load(open(TRELLIS))['packages']['TQFP144']
assert set(str(p) for p in io) == set(tr), 'pinout CSV and Trellis disagree'

fixed = {  # signal: pin, reason
    'FLASH_MOSI': (47, 'config SPI MOSI'),
    'FLASH_MISO': (46, 'config SPI MISO'),
    '~{FLASH_CS}': (49, 'config SPI CSSPIN'),
    'CLK_25MHz': (69, 'LRC_GPLL0T_IN, PLL input'),
    '7M_3V3': (90, 'PCLKT2_0 clock input'),
    'PHI0_3V3': (91, 'PCLKT2_1 clock input'),
    'GPIO_CLK1': (128, 'PCLKT0_0, header clock-capable'),
    'GPIO_CLK2': (37, 'LLC_GPLL0T_IN, header clock-capable'),
}
side = lambda a, b: [p for p in range(a, b + 1) if isio(p)]
used = {p for p, _ in fixed.values()}
pool = lambda ps: [p for p in ps if p not in used]

bus3 = (['PHI1_3V3', 'Q3_3V3'] + [f'D{i}_3V3' for i in range(8)] +
        [f'A{i}_3V3' for i in range(16)])
bus1 = (['R{slash}~{W}_3V3', 'R{slash}~{W}_OUT_3V3',
         '~{I{slash}O_SELECT}_3V3', '~{I{slash}O_STROBE}_3V3',
         '~{DEVICE_SELECT}_3V3', 'µSync_3V3', 'SYNC_3V3', 'COLOR_REF_3V3',
         'INT_OUT_3V3', 'DMA_OUT_3V3', 'INT_IN_3V3', 'DMA_IN_3V3',
         'DATA_OE', 'DATA_DIR', 'ADDR_OE', 'ADDR_DIR'] +
        [f'{s}_3V3' for s in ('RDY', '~{DMA}', '~{NMI}', '~{IRQ}',
                              '~{RES}', '~{INH}')] +
        [f'{s}_READ_3V3' for s in ('RDY', '~{DMA}', '~{NMI}', '~{IRQ}',
                                   '~{RES}', '~{INH}')])
psram = (['PSRAM_SCLK'] + [f'PSRAM_SIO{i}' for i in range(4)] +
         [f'~{{PSRAM_CS{i}}}' for i in range(4)])
header = (['SP_PH0', 'SP_PH1', 'SP_PH2', 'SP_PH3', 'SP_WRDATA',
           '~{SP_WRREQ}', '~{SP_ENBL1}', '~{SP_ENBL2}', 'SP_RDDATA',
           'SP_SENSE', '~{SP_OE}'] + [f'GPIO_{i}' for i in range(1, 9)])

assign = {s: (p, why) for s, (p, why) in fixed.items()}
def take(sigs, ps, why):
    ps = pool(ps)
    assert len(ps) >= len(sigs), (why, len(ps), len(sigs))
    for s, p in zip(sigs, ps):
        assign[s] = (p, why); used.add(p)

take(bus3, side(73, 108), 'slot bus, side 3')
take(bus1, side(1, 36) + [67, 71], 'slot bus, side 1 + bank 3')
take(psram, side(109, 144), 'PSRAM, side 4')
take(header, side(109, 144) + side(39, 52) + [68, 72], 'header, side 4 + bank 8')
spare = sorted(set(io) - used)
for i, p in enumerate(spare):
    assign[f'SPARE_{p}'] = (p, 'spare')

with open(OUT, 'w', newline='') as f:
    w = csv.writer(f, lineterminator='\n')
    w.writerow(['signal', 'pin', 'pad', 'bank', 'dual_function', 'reason'])
    for s, (p, why) in sorted(assign.items(), key=lambda x: x[1][0]):
        w.writerow([s, p, pins[p]['func'], pins[p]['bank'],
                    pins[p]['dual'], why])
n = len([s for s in assign if not s.startswith('SPARE')])
print(f'{n} signals assigned, {len(spare)} spare: {spare}')
print('wrote', OUT)
