#!/usr/bin/env python3
"""
Generate LPF (Lattice Preference File) from FPGA pinout JSON.

Simple starter LPF generator with sensible defaults for Byte Hamr.
"""

import json
import re
import sys
from pathlib import Path
from datetime import datetime


# =============================================================================
# CONFIGURATION - Edit these defaults as needed
# =============================================================================

CONFIG = {
    # System clock
    'system_clock': 'CLK_25MHz',
    'system_clock_freq_mhz': 25,

    # Default I/O settings
    'default_io_type': 'LVCMOS33',
    'default_drive': 8,
    'default_slewrate': 'SLOW',
    'default_pullmode': 'DOWN',

    # SDRAM settings
    'sdram_io_type': 'LVCMOS33',
    'sdram_drive': 8,
    'sdram_slewrate': 'FAST',

    # Apple II bus settings
    'apple2_io_type': 'LVCMOS33',
    'apple2_drive': 8,
    'apple2_slewrate': 'SLOW',

    # Flash/Config settings
    'flash_io_type': 'LVCMOS33',
    'flash_drive': 8,
    'flash_slewrate': 'FAST',
}


def sanitize_signal_name(net_name: str) -> str:
    """Convert net name to valid LPF/Verilog signal name."""
    # Remove hierarchy prefix
    name = net_name.split('/')[-1]
    # Replace special characters
    name = name.replace('{slash}', '_')
    name = name.replace('~{', 'n')
    name = name.replace('}', '')
    name = name.replace('µ', 'u')
    # Remove _3V3 suffix for cleaner names
    name = re.sub(r'_3V3$', '', name)
    # Rename I_O_PIN to GPIO (these are J1 breakout header, not Apple II)
    name = re.sub(r'^I_O_PIN_(\d+)$', r'GPIO\1', name)
    # Handle names starting with numbers (not valid in Verilog)
    if name and name[0].isdigit():
        name = 'sig_' + name  # e.g., "7M" -> "sig_7M"
    return name


def get_io_settings(pin: dict, category: str) -> dict:
    """Determine I/O settings based on pin category and function."""
    net = pin['net_name']

    # SDRAM signals
    if 'SDRAM' in net:
        return {
            'io_type': CONFIG['sdram_io_type'],
            'drive': CONFIG['sdram_drive'],
            'slewrate': CONFIG['sdram_slewrate'],
            'pullmode': 'NONE',
        }

    # Apple II bus signals
    if category in ('Apple II Address', 'Apple II Data', 'Apple II Control', 'Apple II I/O'):
        return {
            'io_type': CONFIG['apple2_io_type'],
            'drive': CONFIG['apple2_drive'],
            'slewrate': CONFIG['apple2_slewrate'],
            'pullmode': 'NONE',
        }

    # Clock input
    if net == CONFIG['system_clock']:
        return {
            'io_type': 'LVCMOS33',
            'drive': None,  # Input only
            'slewrate': None,
            'pullmode': 'NONE',
        }

    # Flash/Config signals
    if 'FLASH' in net or 'FPGA_' in net:
        return {
            'io_type': CONFIG['flash_io_type'],
            'drive': CONFIG['flash_drive'],
            'slewrate': CONFIG['flash_slewrate'],
            'pullmode': 'UP',  # Flash signals typically need pullups
        }

    # Default
    return {
        'io_type': CONFIG['default_io_type'],
        'drive': CONFIG['default_drive'],
        'slewrate': CONFIG['default_slewrate'],
        'pullmode': 'NONE',
    }


def generate_lpf(pinout_path: str, output_path: str):
    """Generate LPF file from pinout JSON."""

    with open(pinout_path, 'r') as f:
        data = json.load(f)

    lines = []

    # Header
    lines.append(f"// =============================================================================")
    lines.append(f"// Byte Hamr - FPGA Pin Constraints")
    lines.append(f"// Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"// Source: {pinout_path}")
    lines.append(f"// =============================================================================")
    lines.append(f"")

    # Frequency constraint for system clock
    lines.append(f"// =============================================================================")
    lines.append(f"// Clock Constraints")
    lines.append(f"// =============================================================================")
    lines.append(f"FREQUENCY PORT \"{CONFIG['system_clock']}\" {CONFIG['system_clock_freq_mhz']} MHz;")
    lines.append(f"")

    # Process each page (skip power, ground, unconnected)
    skip_pages = {'power', 'unconnected'}

    for page, categories in data.items():
        if page in skip_pages:
            continue

        lines.append(f"// =============================================================================")
        lines.append(f"// {page.upper()} Signals")
        lines.append(f"// =============================================================================")

        for category, pins in categories.items():
            if category in ('Power', 'Ground'):
                continue

            lines.append(f"")
            lines.append(f"// {category}")
            lines.append(f"// " + "-" * 60)

            for pin in pins:
                ball = pin['ball']
                net = pin['net_name']
                signal = sanitize_signal_name(net)
                io = get_io_settings(pin, category)

                # Location constraint
                lines.append(f"LOCATE COMP \"{signal}\" SITE \"{ball}\";")

                # I/O buffer settings
                iobuf_parts = [f"IO_TYPE={io['io_type']}"]
                if io['drive']:
                    iobuf_parts.append(f"DRIVE={io['drive']}")
                if io['slewrate']:
                    iobuf_parts.append(f"SLEWRATE={io['slewrate']}")
                if io['pullmode']:
                    iobuf_parts.append(f"PULLMODE={io['pullmode']}")

                lines.append(f"IOBUF PORT \"{signal}\" {' '.join(iobuf_parts)};")

        lines.append(f"")

    # Default settings for unused pins
    lines.append(f"// =============================================================================")
    lines.append(f"// Default Settings for Unused Pins")
    lines.append(f"// =============================================================================")
    lines.append(f"// Pull unused pins to GND to prevent floating inputs")
    lines.append(f"// IOBUF ALLPORTS PULLMODE={CONFIG['default_pullmode']};")
    lines.append(f"")

    # Write output
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f"Generated LPF: {output_path}")

    # Summary
    pin_count = 0
    for page, categories in data.items():
        if page in skip_pages:
            continue
        for category, pins in categories.items():
            if category not in ('Power', 'Ground'):
                pin_count += len(pins)

    print(f"  Constrained {pin_count} signal pins")
    print(f"  System clock: {CONFIG['system_clock']} @ {CONFIG['system_clock_freq_mhz']} MHz")


def main():
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    pinout_path = project_root / 'docs/fpga_pinout.json'
    output_path = project_root / 'hardware/byte_hamr/constraints/byte_hamr.lpf'

    if len(sys.argv) > 1:
        pinout_path = Path(sys.argv[1])
    if len(sys.argv) > 2:
        output_path = Path(sys.argv[2])

    # Create output directory
    output_path.parent.mkdir(parents=True, exist_ok=True)

    generate_lpf(str(pinout_path), str(output_path))


if __name__ == '__main__':
    main()
