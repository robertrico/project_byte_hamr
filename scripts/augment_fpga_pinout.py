#!/usr/bin/env python3
"""
Augment FPGA pinout JSON with additional information derived from pin functions.

Based on Lattice ECP5 datasheet FPGA-DS-02012 Chapter 4 - Pinout Information.

Pin function naming conventions:
- P[L/R][Group]_[A/B/C/D] - Left/Right edge I/O (supports true LVDS)
- P[T/B][Group]_[A/B] - Top/Bottom edge I/O (emulated differential only)
- PCLK[T/C][Bank]_[num] - Primary Clock pins (T=True, C=Complement)
- GR_PCLK[Bank]_[num] - General Routing to Primary Clock
- VCC - Core power (1.1V)
- VCCAUX - Auxiliary power for differential/referenced inputs (2.5V)
- VCCio[Bank] - I/O bank power (configurable voltage)
- VCCA[0/1] - PLL analog power
- VREF1_[Bank] - Reference voltage for bank
"""

import json
import re
import sys
from pathlib import Path


def get_pin_info(pin_function: str) -> dict:
    """
    Extract additional information from pin function name.
    Returns dict with: edge, bank, group, pair, capabilities, description
    """
    info = {
        'edge': None,
        'bank': None,
        'group': None,
        'pair': None,
        'differential': None,
        'capabilities': [],
        'description': None,
    }

    func = pin_function

    # Power pins
    if func == 'VCC':
        info['description'] = 'Core power supply (1.1V)'
        info['capabilities'] = ['power']
        return info

    if func == 'GND':
        info['description'] = 'Ground'
        info['capabilities'] = ['ground']
        return info

    if func == 'VCCAUX':
        info['description'] = 'Auxiliary power for differential/referenced inputs (2.5V)'
        info['capabilities'] = ['power']
        return info

    if func.startswith('VCCio'):
        match = re.match(r'VCCio(\d+)', func)
        if match:
            info['bank'] = int(match.group(1))
            info['description'] = f'I/O bank {info["bank"]} power supply'
            info['capabilities'] = ['power']
        return info

    if func.startswith('VCCA'):
        match = re.match(r'VCCA(\d+)', func)
        if match:
            info['description'] = f'PLL {match.group(1)} analog power (1.1V)'
            info['capabilities'] = ['power']
        return info

    if func.startswith('VCCHRX') or func.startswith('VCCHTX') or func.startswith('VCCAUXA'):
        info['description'] = 'SERDES power supply'
        info['capabilities'] = ['power', 'serdes']
        return info

    if func.startswith('VREF1_'):
        match = re.match(r'VREF1_(\d+)', func)
        if match:
            info['bank'] = int(match.group(1))
            info['description'] = f'Reference voltage input for bank {info["bank"]}'
            info['capabilities'] = ['vref', 'gpio']
        return info

    # Left/Right edge I/O: P[L/R][Group]_[A/B/C/D]
    match = re.match(r'P([LR])(\d+)([A-D])', func)
    if match:
        info['edge'] = 'left' if match.group(1) == 'L' else 'right'
        info['group'] = int(match.group(2))
        pio = match.group(3)
        info['pair'] = 'AB' if pio in ('A', 'B') else 'CD'
        info['differential'] = 'true_lvds' if pio in ('A', 'B') else 'lvds_input'
        info['capabilities'] = ['gpio', 'lvds_input']
        if pio in ('A', 'B'):
            info['capabilities'].append('lvds_output')
        info['description'] = f'{info["edge"].title()} edge PIO group {info["group"]}{pio}'

        # Determine bank from group number (approximate - depends on package)
        # Left side: banks 6, 7 (bottom to top)
        # Right side: banks 2, 3 (bottom to top)
        if info['edge'] == 'left':
            info['bank'] = 6 if info['group'] < 50 else 7
        else:
            info['bank'] = 2 if info['group'] < 50 else 3
        return info

    # Top/Bottom edge I/O: P[T/B][Group]_[A/B]
    match = re.match(r'P([TB])(\d+)([AB])', func)
    if match:
        info['edge'] = 'top' if match.group(1) == 'T' else 'bottom'
        info['group'] = int(match.group(2))
        pio = match.group(3)
        info['pair'] = 'AB'
        info['differential'] = 'emulated'  # Top/bottom only support emulated differential
        info['capabilities'] = ['gpio', 'emulated_lvds_output']
        info['description'] = f'{info["edge"].title()} edge PIO group {info["group"]}{pio}'

        # Determine bank from position
        # Top: banks 0, 1 (left to right)
        # Bottom: banks 6, 7, 8 (configuration bank)
        if info['edge'] == 'top':
            info['bank'] = 0 if info['group'] < 60 else 1
        return info

    # Primary Clock pins: PCLK[T/C][Bank]_[num]
    match = re.match(r'PCLK([TC])(\d)_(\d)', func)
    if match:
        polarity = 'true' if match.group(1) == 'T' else 'complement'
        info['bank'] = int(match.group(2))
        clk_num = int(match.group(3))
        info['capabilities'] = ['gpio', 'primary_clock', 'pll_input']
        info['differential'] = 'clock_pair'
        info['description'] = f'Primary clock {clk_num} ({polarity}) for bank {info["bank"]}'
        return info

    # General Routing Primary Clock: GR_PCLK[Bank]_[num]
    match = re.match(r'GR_PCLK(\d)_(\d)', func)
    if match:
        info['bank'] = int(match.group(1))
        clk_num = int(match.group(2))
        info['capabilities'] = ['gpio', 'general_routing_clock']
        info['description'] = f'General routing to primary clock {clk_num} for bank {info["bank"]}'
        return info

    # GPLL input pins
    match = re.match(r'GPLL(\d)([TC])_IN', func)
    if match:
        polarity = 'true' if match.group(2) == 'T' else 'complement'
        info['capabilities'] = ['gpio', 'pll_input']
        info['description'] = f'General purpose PLL input ({polarity})'
        return info

    # Configuration pins
    config_pins = {
        'TMS': ('Test Mode Select - JTAG state machine control', ['jtag']),
        'TCK': ('Test Clock - JTAG clock input', ['jtag']),
        'TDI': ('Test Data In - JTAG data input', ['jtag']),
        'TDO': ('Test Data Out - JTAG data output', ['jtag']),
        'INITN': ('Configuration ready indicator (active low, open drain)', ['config']),
        'PROGRAMN': ('Configuration initiate (active low)', ['config']),
        'DONE': ('Configuration complete indicator (open drain)', ['config']),
        'CCLK': ('Configuration clock', ['config']),
        'CSSPIN': ('SPI flash chip select', ['config', 'spi']),
        'D0/MOSI': ('SPI MOSI / Parallel config D0', ['config', 'spi', 'gpio']),
        'D1/MISO': ('SPI MISO / Parallel config D1', ['config', 'spi', 'gpio']),
        'D2/WPn': ('SPI Write Protect / Parallel config D2', ['config', 'spi', 'gpio']),
        'D3/HOLDn': ('SPI Hold / Parallel config D3', ['config', 'spi', 'gpio']),
        'SN/CSn': ('Chip select for parallel config', ['config', 'gpio']),
        'CS1n': ('Secondary chip select', ['config', 'gpio']),
        'WRITEn': ('Write enable for parallel config', ['config', 'gpio']),
        'DOUT/CSOn': ('Serial data out / SPI chip select out', ['config', 'gpio']),
        'CFG_0': ('Configuration mode bit 0', ['config']),
        'CFG_1': ('Configuration mode bit 1', ['config']),
        'CFG_2': ('Configuration mode bit 2', ['config']),
    }

    if func in config_pins:
        info['description'], info['capabilities'] = config_pins[func]
        return info

    # SERDES pins
    if func.startswith('HD'):
        match = re.match(r'HD(TX|RX)([PN])(\d)_D(\d)CH(\d)', func)
        if match:
            direction = 'transmit' if match.group(1) == 'TX' else 'receive'
            polarity = 'positive' if match.group(2) == 'P' else 'negative'
            info['capabilities'] = ['serdes']
            info['description'] = f'SERDES {direction} differential {polarity}'
        return info

    if func.startswith('REFCLK'):
        match = re.match(r'REFCLK([PN])_D(\d)', func)
        if match:
            polarity = 'positive' if match.group(1) == 'P' else 'negative'
            info['capabilities'] = ['serdes', 'reference_clock']
            info['description'] = f'SERDES reference clock {polarity}'
        return info

    # Default for unrecognized pins
    info['description'] = f'Pin function: {func}'
    info['capabilities'] = ['gpio']
    return info


def augment_pinout(input_path: str, output_path: str):
    """Load pinout JSON, augment with additional info, and save."""

    with open(input_path, 'r') as f:
        data = json.load(f)

    # Process each page
    for page, categories in data.items():
        for category, pins in categories.items():
            for pin in pins:
                pin_info = get_pin_info(pin['pin_function'])

                # Add non-None fields
                for key, value in pin_info.items():
                    if value is not None:
                        pin[key] = value

    # Write augmented JSON
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)

    # Print summary
    print(f"Augmented pinout saved to {output_path}")

    # Count capabilities
    cap_counts = {}
    for page, categories in data.items():
        for category, pins in categories.items():
            for pin in pins:
                for cap in pin.get('capabilities', []):
                    cap_counts[cap] = cap_counts.get(cap, 0) + 1

    print("\nCapabilities summary:")
    for cap, count in sorted(cap_counts.items(), key=lambda x: -x[1]):
        print(f"  {cap}: {count} pins")


def main():
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    input_path = project_root / 'docs/fpga_pinout.json'
    output_path = project_root / 'docs/fpga_pinout.json'  # Overwrite in place

    if len(sys.argv) > 1:
        input_path = Path(sys.argv[1])
    if len(sys.argv) > 2:
        output_path = Path(sys.argv[2])

    augment_pinout(str(input_path), str(output_path))


if __name__ == '__main__':
    main()
