#!/usr/bin/env python3
"""
Extract FPGA pinout from KiCad netlist file.
Generates a pinout JSON for the ECP5 FPGA (U1), organized by schematic page.
"""

import json
import re
import sys
from pathlib import Path


# Map categories to schematic pages
CATEGORY_TO_PAGE = {
    'Power': 'power',
    'Ground': 'power',
    'Flash/Config': 'flash',
    'SDRAM': 'ram',
    'JTAG': 'usb',
    'USB/Serial': 'usb',
    'Apple II Address': 'card',
    'Apple II Data': 'card',
    'Apple II Control': 'card',
    'Apple II I/O': 'card',
    'Clock': 'card',
    'GPIO Breakout': 'gpio',
    'Unconnected': 'unconnected',
    'Other': 'other',
}

# Page order matching schematic hierarchy
PAGE_ORDER = ['power', 'flash', 'ram', 'usb', 'card', 'gpio', 'unconnected', 'other']


def parse_netlist(netlist_path: str) -> list[dict]:
    """Parse KiCad netlist and extract FPGA pin assignments."""

    with open(netlist_path, 'r') as f:
        content = f.read()

    # Find the nets section
    nets_match = re.search(r'\(nets\s*(.*)\)\s*\)$', content, re.DOTALL)
    if not nets_match:
        print("Error: Could not find nets section", file=sys.stderr)
        return []

    nets_content = nets_match.group(1)

    # Pattern to match each net block
    net_pattern = re.compile(
        r'\(net\s+\(code\s+"(\d+)"\)\s+\(name\s+"([^"]+)"\)[^)]*\)'
        r'(.*?)(?=\(net\s+\(code|$)',
        re.DOTALL
    )

    # Pattern to match U1 (FPGA) nodes within a net
    u1_node_pattern = re.compile(
        r'\(node\s+\(ref\s+"U1"\)\s+\(pin\s+"([^"]+)"\)\s+\(pinfunction\s+"([^"]+)"\)',
        re.DOTALL
    )

    pins = []

    for net_match in net_pattern.finditer(nets_content):
        net_code = net_match.group(1)
        net_name = net_match.group(2)
        net_body = net_match.group(3)

        # Find all U1 pins in this net
        for node_match in u1_node_pattern.finditer(net_body):
            ball = node_match.group(1)
            pin_function = node_match.group(2)

            pins.append({
                'ball': ball,
                'pin_function': pin_function,
                'net_name': net_name,
            })

    return pins


def categorize_pin(net_name: str, pin_function: str) -> str:
    """Categorize pin by function."""
    net_lower = net_name.lower()
    func_lower = pin_function.lower()

    # Check for unconnected pins first
    if 'unconnected' in net_lower:
        return 'Unconnected'

    if 'vcc' in func_lower or '+' in net_name:
        return 'Power'
    if 'gnd' in net_lower:
        return 'Ground'
    if 'sdram' in net_lower or net_name.startswith('RAM_'):
        return 'SDRAM'
    if any(x in net_lower for x in ['flash', 'miso', 'mosi', 'sck', 'csspin']):
        return 'Flash/Config'
    if any(x in net_lower for x in ['jtag', 'tdi', 'tdo', 'tck', 'tms']):
        return 'JTAG'
    if any(x in net_lower for x in ['d0_', 'd1_', 'd2_', 'd3_', 'd4_', 'd5_', 'd6_', 'd7_']):
        return 'Apple II Data'
    if re.match(r'a\d+_3v3', net_lower):
        return 'Apple II Address'
    if 'clk' in net_lower or 'phi' in net_lower:
        return 'Clock'
    if any(x in net_lower for x in ['dma', 'irq', 'nmi', 'rdy', 'res', 'inh']):
        return 'Apple II Control'
    # J1 GPIO breakout header (directly to FPGA, no level shifters)
    if re.search(r'i.*o_pin_\d+', net_lower):
        return 'GPIO Breakout'
    # Apple II slot I/O select/strobe signals
    if 'i{slash}o' in net_lower:
        return 'Apple II I/O'
    if any(x in net_lower for x in ['gp', 'gn', 'led', 'sw', 'audio', 'wifi']):
        return 'GPIO Header'
    if 'usb' in net_lower or 'ftdi' in net_lower:
        return 'USB/Serial'
    # Apple II misc signals
    if any(x in net_lower for x in ['7m_', 'q3_', 'sync', 'int_in', 'int_out',
                                     'r{slash}~{w}', 'device_select']):
        return 'Apple II Control'

    return 'Other'


def ball_sort_key(ball: str) -> tuple:
    """Sort ball locations: letter first, then number."""
    match = re.match(r'([A-Z]+)(\d+)', ball)
    if match:
        return (match.group(1), int(match.group(2)))
    return (ball, 0)


def get_signal_sort_key(pin: dict) -> tuple:
    """
    Get sort key based on signal semantics.
    Returns tuple for multi-level sorting.
    """
    net = pin['net_name']
    func = pin['pin_function']

    # Power pins: sort by voltage, then by function type
    if pin.get('category') in ('Power', 'Ground'):
        if 'GND' in net:
            return (0, 0, ball_sort_key(pin['ball']))
        voltage_order = {'+1V1': 1, '+2V5': 2, '+3V3': 3}
        voltage = voltage_order.get(net, 9)
        # Sub-sort: VCC core, VCCA, VCCAUX, VCCio
        if 'VCC' == func:
            func_order = 0
        elif 'VCCA' in func:
            func_order = 1
        elif 'VCCAUX' in func:
            func_order = 2
        elif 'VCCio' in func:
            # Extract bank number
            match = re.search(r'VCCio(\d+)', func)
            func_order = 3 + (int(match.group(1)) if match else 0)
        else:
            func_order = 99
        return (voltage, func_order, ball_sort_key(pin['ball']))

    # Address bus: sort by bit number (A0-A15)
    match = re.match(r'A(\d+)_3V3', net)
    if match:
        return (0, int(match.group(1)))

    # Data bus: sort by bit number (D0-D7)
    match = re.match(r'D(\d+)_3V3', net)
    if match:
        return (0, int(match.group(1)))

    # I/O pins: sort by pin number
    match = re.search(r'I.*O_PIN_(\d+)', net)
    if match:
        return (0, int(match.group(1)))

    # SDRAM: group by type, then sort by bit/number
    if 'SDRAM' in net:
        signal = net.replace('SDRAM_', '')
        # Control signals first
        control_order = {
            'CLK': 0, 'CKE': 1, 'nCS': 2, 'nRAS': 3, 'nCAS': 4, 'nWE': 5,
            'DQM0': 6, 'DQM1': 7, 'BA0': 8, 'BA1': 9
        }
        if signal in control_order:
            return (0, control_order[signal])
        # Address bits
        match = re.match(r'A(\d+)', signal)
        if match:
            return (1, int(match.group(1)))
        # Data bits
        match = re.match(r'D(\d+)', signal)
        if match:
            return (2, int(match.group(1)))
        return (3, 0)

    # Flash/config: logical order
    flash_order = {
        '/flash/FLASH_nCS': 0,
        '/flash/FLASH_SCK': 1,
        '/flash/FLASH_MOSI': 2,
        '/flash/FLASH_MISO': 3,
        '/flash/FLASH_nWP': 4,
        '/flash/FLASH_nHOLD': 5,
        '/flash/FPGA_PROGRAMN': 6,
        '/flash/FPGA_INITN': 7,
        '/flash/FPGA_DONE': 8,
    }
    if net in flash_order:
        return (0, flash_order[net])

    # JTAG: standard order
    jtag_order = {'TCK': 0, 'TMS': 1, 'TDI': 2, 'TDO': 3}
    if func in jtag_order:
        return (0, jtag_order[func])

    # Clock signals
    clock_order = {'CLK_25MHz': 0, 'PHI0_3V3': 1, 'PHI1_3V3': 2}
    if net in clock_order:
        return (0, clock_order[net])

    # Control signals: alphabetical but group related signals
    control_groups = {
        'PHI0': 0, 'PHI1': 1, '7M': 2, 'Q3': 3, 'Sync': 4,
        'R/~W': 10, 'RDY': 11,
        'IRQ': 20, 'NMI': 21, 'RES': 22,
        'DMA': 30, 'INH': 31,
        'DEVICE_SELECT': 40, 'I/O_SELECT': 41, 'I/O_STROBE': 42,
        'INT_IN': 50, 'INT_OUT': 51,
    }
    for prefix, order in control_groups.items():
        if prefix.lower() in net.lower() or prefix in net:
            return (0, order)

    # Default: sort by net name
    return (99, net)


def main():
    # Default paths
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    netlist_path = project_root / 'hardware/byte_hamr/kicad/project_byte_hamr.net'
    output_path = project_root / 'docs/fpga_pinout.json'

    if len(sys.argv) > 1:
        netlist_path = Path(sys.argv[1])
    if len(sys.argv) > 2:
        output_path = Path(sys.argv[2])

    print(f"Parsing netlist: {netlist_path}")
    pins = parse_netlist(str(netlist_path))

    if not pins:
        print("No FPGA pins found!", file=sys.stderr)
        sys.exit(1)

    # Add category and page to each pin
    for pin in pins:
        pin['category'] = categorize_pin(pin['net_name'], pin['pin_function'])
        pin['page'] = CATEGORY_TO_PAGE.get(pin['category'], 'other')

    # Organize by page, then by category, sorted by ball
    output = {}
    for page in PAGE_ORDER:
        page_pins = [p for p in pins if p['page'] == page]
        if not page_pins:
            continue

        # Group by category within page
        categories = {}
        for pin in page_pins:
            cat = pin['category']
            if cat not in categories:
                categories[cat] = []
            categories[cat].append({
                'ball': pin['ball'],
                'pin_function': pin['pin_function'],
                'net_name': pin['net_name'],
            })

        # Sort pins within each category by signal semantics
        for cat in categories:
            # Add category to pins for sorting context
            for p in categories[cat]:
                p['category'] = cat
            categories[cat].sort(key=get_signal_sort_key)
            # Remove category from output
            for p in categories[cat]:
                del p['category']

        output[page] = categories

    # Write JSON
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(output, f, indent=2)

    print(f"Wrote {len(pins)} pins to {output_path}")

    # Print summary by page
    print("\nPinout Summary by Page:")
    print("-" * 40)
    for page in PAGE_ORDER:
        if page in output:
            page_total = sum(len(pins) for pins in output[page].values())
            print(f"\n  {page}:")
            for cat, cat_pins in output[page].items():
                print(f"    {cat}: {len(cat_pins)} pins")


if __name__ == '__main__':
    main()
