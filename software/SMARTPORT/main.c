#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/structs/sio.h"
#include "picoport.h"
#include "sp_proto.h"
#include "fm_rx.pio.h"
#include "fm_tx.pio.h"

// SmartPort bus loop — runs on Core 0, launches Core 1 for serial (sp_cmd.c)
extern void sp_main(PIO pio_rx, uint sm_rx, PIO pio_tx, uint sm_tx);

// PIO RX program offset (set in main, used by pio_rx_restart)
static uint g_rx_offset;
// PIO TX program offset (set in main, used by pio_tx_restart)
static uint g_tx_offset;

// Called from assembly to restart PIO RX state machine after each packet
void pio_rx_restart(void) {
    PIO pio = pio0;
    uint sm = 0;
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(g_rx_offset));
    pio_sm_set_enabled(pio, sm, true);
}

// Reset PIO TX SM before each packet — ensures no leftover OSR bits
void pio_tx_restart(void) {
    PIO pio = pio0;
    uint sm = 1;
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));  // force pin LOW (idle)
    pio_sm_exec(pio, sm, pio_encode_jmp(g_tx_offset));
    pio_sm_set_enabled(pio, sm, true);
}

int main() {
    stdio_init_all();

    // --- Configure ACK pin: open-drain, initially deasserted (input/hi-Z) ---
    gpio_init(SP_ACK);
    gpio_put(SP_ACK, 0);           // output value = LOW when driven
    gpio_set_dir(SP_ACK, GPIO_IN); // start as input = hi-Z = deasserted

    // --- Configure all input pins ---
    // Phase pins (GP3-6)
    for (int i = SP_PHI_BASE; i < SP_PHI_BASE + 4; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
    }
    // Other input pins (NOT wrdata/rddata — those are configured by PIO)
    int input_pins[] = { SP_ENBL1, SP_DEVSEL, SP_WREQ, SP_ENBL2 };
    for (int i = 0; i < 4; i++) {
        gpio_init(input_pins[i]);
        gpio_set_dir(input_pins[i], GPIO_IN);
    }

    // --- PIO RX: FM receiver on WRDATA (GP11) ---
    PIO pio_rx = pio0;
    uint sm_rx = 0;
    g_rx_offset = pio_add_program(pio_rx, &fm_rx_program);
    fm_rx_program_init(pio_rx, sm_rx, g_rx_offset, SP_WRDATA);

    // --- PIO TX: FM transmitter on RDDATA (GP10) ---
    PIO pio_tx = pio0;
    uint sm_tx = 1;
    g_tx_offset = pio_add_program(pio_tx, &fm_tx_program);
    uint tx_offset = g_tx_offset;
    fm_tx_program_init(pio_tx, sm_tx, tx_offset, SP_RDDATA);

    // Wait for USB serial to enumerate
    sleep_ms(2000);
    printf("PicoPort SmartPort Device\n");
    printf("  RX(WRDATA)=GP%d  TX(RDDATA)=GP%d\n", SP_WRDATA, SP_RDDATA);
    printf("  ACK=GP%d  REQ=GP%d\n", SP_ACK, SP_REQ);

    // ============================================================
    // GPIO wiring diagnostic — verify every signal
    // ============================================================
    printf("\n--- GPIO Wiring Diagnostic ---\n");
    printf("  Pico   FPGA    Signal       Value  Expected\n");
    printf("  -----  ------  -----------  -----  --------\n");

    // Read all pins once
    uint32_t gpio = sio_hw->gpio_in;

    // _enbl1: GP2 ← GPIO1 (active LOW, should be HIGH when no drive selected)
    int enbl1 = (gpio >> SP_ENBL1) & 1;
    printf("  GP2    GPIO1   _enbl1       %d      1 (inactive)  %s\n",
           enbl1, enbl1 ? "OK" : "WRONG");

    // phase[2]: GP3 ← GPIO2
    int ph2 = (gpio >> 3) & 1;
    printf("  GP3    GPIO2   phase[2]     %d\n", ph2);

    // phase[3]: GP4 ← GPIO3
    int ph3 = (gpio >> 4) & 1;
    printf("  GP4    GPIO3   phase[3]     %d\n", ph3);

    // phase[0]/REQ: GP5 ← GPIO4
    int ph0 = (gpio >> SP_REQ) & 1;
    printf("  GP5    GPIO4   PH0/REQ      %d\n", ph0);

    // phase[1]: GP6 ← GPIO5
    int ph1 = (gpio >> 6) & 1;
    printf("  GP6    GPIO5   phase[1]     %d\n", ph1);

    // ~DEV_SEL: GP7 ← GPIO6 (active LOW, HIGH when idle)
    int devsel = (gpio >> SP_DEVSEL) & 1;
    printf("  GP7    GPIO6   ~DEV_SEL     %d      1 (inactive)  %s\n",
           devsel, devsel ? "OK" : "(active)");

    // _wreq: GP8 ← GPIO7 (active LOW, should be HIGH idle)
    int wreq = (gpio >> SP_WREQ) & 1;
    printf("  GP8    GPIO7   _wreq        %d      1 (inactive)  %s\n",
           wreq, wreq ? "OK" : "WRONG");

    // ACK/sense: GP9 → GPIO8 (open-drain output)
    // Currently deasserted (input/hi-Z), FPGA pull-up should read HIGH
    int ack_idle = (gpio >> SP_ACK) & 1;
    printf("  GP9    GPIO8   ACK(idle)    %d      1 (hi-Z+PU)   %s\n",
           ack_idle, ack_idle ? "OK" : "WRONG");

    // RDDATA: GP10 → GPIO9 (PIO TX output)
    int rddata = (gpio >> SP_RDDATA) & 1;
    printf("  GP10   GPIO9   rddata       %d      (PIO TX out)\n", rddata);

    // WRDATA: GP11 ← GPIO10 (PIO RX input)
    int wrdata = (gpio >> SP_WRDATA) & 1;
    printf("  GP11   GPIO10  wrdata       %d      (PIO RX in)\n", wrdata);

    // _enbl2: GP12 ← GPIO11 (active LOW, should be HIGH when no drive)
    int enbl2 = (gpio >> SP_ENBL2) & 1;
    printf("  GP12   GPIO11  _enbl2       %d      1 (inactive)  %s\n",
           enbl2, enbl2 ? "OK" : "WRONG");

    // --- Logical phase decode ---
    uint32_t raw_phases = (gpio >> SP_PHI_BASE) & SP_PHI_MASK;
    // Unscramble: raw bit0=PH2, bit1=PH3, bit2=PH0, bit3=PH1
    uint32_t logical = ((raw_phases >> 2) & 1) << 0   // PH0
                     | ((raw_phases >> 3) & 1) << 1    // PH1
                     | ((raw_phases >> 0) & 1) << 2    // PH2
                     | ((raw_phases >> 1) & 1) << 3;   // PH3
    printf("\n  Phases: raw=0x%X -> logical=0x%X (PH3:PH2:PH1:PH0)", raw_phases, logical);
    if (logical == 0x0A) printf(" = IDLE");
    else if (logical == 0x0B) printf(" = COMMAND");
    else if (logical == 0x05) printf(" = RESET");
    printf("\n");

    // --- ACK toggle test ---
    printf("\n  ACK toggle test:\n");
    printf("    Before assert: GP9 = %d\n", (sio_hw->gpio_in >> SP_ACK) & 1);
    gpio_set_dir(SP_ACK, GPIO_OUT);  // assert ACK (drives LOW)
    sleep_ms(1);
    printf("    After assert:  GP9 = %d (expect 0)\n", (sio_hw->gpio_in >> SP_ACK) & 1);
    gpio_set_dir(SP_ACK, GPIO_IN);   // deassert ACK (hi-Z)
    sleep_ms(1);
    printf("    After release: GP9 = %d (expect 1)\n", (sio_hw->gpio_in >> SP_ACK) & 1);

    // --- ~DEV_SEL activity test ---
    // Count low pulses over ~100K samples (IWM register accesses)
    int dev_pulses = 0;
    int dev_last = 1;
    for (int i = 0; i < 100000; i++) {
        int cur = (sio_hw->gpio_in >> SP_DEVSEL) & 1;
        if (cur == 0 && dev_last == 1) dev_pulses++;  // falling edge
        dev_last = cur;
    }
    printf("\n  ~DEV_SEL: %d pulses (IWM accesses) in ~100K samples", dev_pulses);
    if (dev_pulses > 0) printf(" -- ACTIVE\n");
    else printf(" -- idle (no IWM access)\n");

    printf("--- End Diagnostic ---\n\n");

    printf("Entering sp_main...\n");

    // Hand off to assembly — never returns
    sp_main(pio_rx, sm_rx, pio_tx, sm_tx);

    return 0;
}
