// iwm_asic read path vs a software IWM oracle (Verilator).
//
// The DUT receives SmartPort packets on rddata (a 1 = 0.98 us low pulse at the start of a
// 3.911 us cell, as the A2M2053 fm_tx sends them) while a 6502 model polls the data register
// the way liron-if.asm Lc987 does (lda q6l,x / bpl). The oracle decodes the same edge list with
// the spec read windows (a 1 at an edge at least half a cell after the last bit, a 0 at 1.5
// cells) and frames with sp_framer_bit from the A2M2053 firmware: MSB framing, as the IWM spec
// gives for asynchronous mode ("the data register will latch the shift register when a one is
// shifted into the MSB", Spec 1982 p. 9). Packets come from the A2M2053 sp_encode, the encoder
// that passed the bench against a real Liron card.
//
// Both sides then go through the card ROM's acceptance rule: C3 within 30 nibbles, then
// sp_decode (counts, checksum, C8), and the decoded data must match. A case fails the DUT only
// when the oracle accepts it: a stream the real IWM cannot read either is not a DUT fault.
//
// Cases: clean stream at several poll phases, read engine started late (bit k of the stream),
// one stray edge before the preamble, one extra edge inside the preamble.
//
// TB_DUMP=1 prints both nibble streams for every DUT failure.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include "Viwm_asic.h"
#include "verilated.h"
extern "C" {
#include "sp_wire.h"
}

static const double FCLK_NS = 1000.0 / 7.15909;    // 139.68 ns
static const double HALF_NS = FCLK_NS / 2.0;
static const double CELL_NS = 28.0 * FCLK_NS;      // 3.911 us
static const double PULSE_NS = 980.0;              // fm_tx low pulse
static const double CYC_NS = 7.0 * FCLK_NS;        // 6502 cycle, 7M / 7
static double setup_ns = 100.0;                    // 6502 read data setup before PHI0 falls (spec min)
static const double T0_NS = 60000.0;               // first stream edge

// ---------------------------------------------------------------- DUT clocking
static std::unique_ptr<Viwm_asic> m;
static long hcount;
static double t_ns;
static std::vector<double> edges;                  // rddata falling edges, sorted
static size_t ei;

static void eval_now() {
    while (ei < edges.size() && edges[ei] + PULSE_NS <= t_ns) ei++;
    m->rddata = (ei < edges.size() && edges[ei] <= t_ns) ? 0 : 1;
    m->eval();
}

static void advance_to(double t_end) {
    while (t_ns + HALF_NS <= t_end + 1e-6) {
        hcount++;
        t_ns = hcount * HALF_NS;
        m->fclk = !m->fclk;
        eval_now();
    }
}

// One 6502 cycle on the IWM: /DEV low while PHI0 is high (second half), data taken setup_ns
// before PHI0 falls.
static uint8_t bus(uint8_t addr, bool rnw, uint8_t data = 0) {
    double t0 = t_ns;
    advance_to(t0 + CYC_NS / 2);
    m->addr = addr;
    m->R_nW = rnw;
    m->data_in = data;
    m->nDEVICE_SELECT = 0;
    eval_now();
    advance_to(t0 + CYC_NS - setup_ns);
    uint8_t v = m->data_out;
    advance_to(t0 + CYC_NS);
    m->nDEVICE_SELECT = 1;
    eval_now();
    return v;
}

static void idle(int cycles) { advance_to(t_ns + cycles * CYC_NS); }

// ---------------------------------------------------------------- packet -> edges
static std::vector<uint8_t> stream_bytes(bool card_preamble, const uint8_t *data, size_t len) {
    static uint8_t out[SP_TX_MAX];
    size_t n = sp_encode(out, 1, SP_TYPE_STATUS, 0, data, len);
    size_t skip = card_preamble ? 5 : 0;           // card: FF 3F CF F3 FC FF C3
    return std::vector<uint8_t>(out + skip, out + n);
}

static std::vector<double> stream_edges(const std::vector<uint8_t> &b, double t0) {
    std::vector<double> e;
    size_t bit = 0;
    for (uint8_t x : b)
        for (int i = 7; i >= 0; i--, bit++)
            if ((x >> i) & 1u) e.push_back(t0 + bit * CELL_NS);
    return e;
}

// ---------------------------------------------------------------- oracle
// Spec read windows on the edge list from t_start to t_end, MSB framing. Returns the bytes.
static std::vector<uint8_t> oracle_bytes(const std::vector<double> &e, double t_start, double t_end) {
    std::vector<uint8_t> out;
    sp_framer_t fr = {0, 0};
    double ref = t_start;
    auto bit = [&](unsigned b) { int v = sp_framer_bit(&fr, b); if (v >= 0) out.push_back((uint8_t)v); };
    size_t i = 0;
    while (i < e.size() && e[i] < t_start) i++;
    for (;;) {
        double ne = i < e.size() ? e[i] : 1e300;
        double nz = ref + 1.5 * CELL_NS;
        if (ne < nz) {
            if (ne > t_end) break;
            if (ne - ref >= 0.5 * CELL_NS) { bit(1); ref = ne; }
            i++;
        } else {
            if (nz > t_end) break;
            bit(0);
            ref += CELL_NS;
        }
    }
    return out;
}

// ---------------------------------------------------------------- card ROM acceptance
static bool rom_accepts(const std::vector<uint8_t> &nib, const uint8_t *data, size_t len) {
    size_t i = 0;
    for (int y = 30; i < nib.size(); i++) {        // Lc987: C3 within 30 nibbles
        if (--y < 0) return false;
        if (nib[i] == SP_PBEGIN) break;
    }
    if (i >= nib.size()) return false;
    std::vector<uint8_t> wire(nib.begin() + i + 1, nib.end());
    sp_hdr_t h;
    static uint8_t got[SP_DATA_MAX];
    if (sp_decode(wire.data(), wire.size(), &h, got) != SP_OK) return false;
    return h.len == len && memcmp(got, data, len) == 0;
}

static void dump(const char *tag, const std::vector<uint8_t> &v) {
    printf("    %s %3zu:", tag, v.size());
    for (size_t i = 0; i < v.size() && i < 24; i++) printf(" %02X", v[i]);
    printf("%s\n", v.size() > 24 ? " ..." : "");
}

// ---------------------------------------------------------------- one case
struct result { bool dut, orc; };

// late_start < 0: read mode from the beginning. extra: added edges (stray or noise).
static result run_case(bool card_pre, const uint8_t *data, size_t len, double t0, double late_start,
                       const std::vector<double> &extra) {
    m.reset(new Viwm_asic);
    hcount = 0;
    t_ns = 0;
    ei = 0;
    std::vector<uint8_t> sb = stream_bytes(card_pre, data, len);
    edges = stream_edges(sb, t0);
    for (double x : extra) edges.push_back(x);
    std::sort(edges.begin(), edges.end());
    double t_end = t0 + sb.size() * 8 * CELL_NS + 200000.0;

    m->fclk = 0; m->Q3 = 0; m->R_nW = 1; m->nDEVICE_SELECT = 1;
    m->addr = 0; m->data_in = 0; m->sense = 1; m->nRES = 0;
    eval_now();
    advance_to(2000.0);
    m->nRES = 1;
    eval_now();
    idle(4);

    bus(0xD, true);                                // Q6 on
    bus(0xF, false, 0x07);                         // Q7 on + mode $07 (motor off): latch, async, timer off
    bus(0x9, true);                                // motor on
    double t_read = 0.0;
    if (late_start >= 0) {                         // stay in write mode (read engine held) until then
        advance_to(late_start - CYC_NS / 2);
        t_read = late_start;
    }
    bus(0xE, true);                                // Q7 off
    bus(0xD, true);                                // Q6 on: status (Lc97d)

    std::vector<uint8_t> nib;
    while (t_ns < t_end) {                         // Lc987: lda q6l,x / bpl
        idle(3);
        uint8_t v = bus(0xC, true);
        if (v & 0x80) { nib.push_back(v); idle(12); }
        else idle(3);
    }
    m->final();

    result r;
    r.dut = rom_accepts(nib, data, len);
    std::vector<uint8_t> ob = oracle_bytes(edges, t_read, t_end);
    r.orc = rom_accepts(ob, data, len);
    if (r.orc && !r.dut && getenv("TB_DUMP")) { dump("dut", nib); dump("orc", ob); }
    return r;
}

// ---------------------------------------------------------------- main
struct tally {
    std::string name;
    const char *unit;
    int n = 0, orc_ok = 0, dut_ok = 0, regress = 0;
    std::vector<double> at;                        // DUT fails where the oracle passes
};

static void add(tally &t, result r, double x) {
    t.n++;
    t.orc_ok += r.orc;
    t.dut_ok += r.dut;
    if (r.orc && !r.dut) { t.regress++; t.at.push_back(x); }
    if (r.orc && !r.dut && getenv("TB_DUMP")) printf("  ^ %s, %s %.2f\n", t.name.c_str(), t.unit, x);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const uint8_t st4[4] = { 0x00, 0x40, 0x06, 0x00 };
    static uint8_t blk[512];
    for (int i = 0; i < 512; i++) blk[i] = (uint8_t)(i * 7 + 3);
    std::vector<tally> all;

    // x7 race: a real 6502 often latches later than its 100 ns spec setup. Any D7 the CPU can
    // see must also start the x7 clear, or the next poll reads the same nibble again.
    for (double su : { 60.0, 40.0, 20.0 }) {
        tally t; t.name = "clean 512B, 6502 setup " + std::to_string((int)su) + " ns"; t.unit = "phase/8";
        setup_ns = su;
        for (int j = 0; j < 8; j++)
            add(t, run_case(true, blk, 512, T0_NS + j * CYC_NS / 8, -1, {}), j);
        all.push_back(t);
    }
    setup_ns = 100.0;

    for (int cp = 1; cp >= 0; cp--) {
        const char *pre = cp ? "card preamble" : "drive preamble";
        size_t pre_bits = (cp ? 7 : 12) * 8;

        tally clean; clean.name = std::string("clean 512B, poll phase, ") + pre; clean.unit = "phase/8";
        for (int j = 0; j < 8; j++)
            add(clean, run_case(cp, blk, 512, T0_NS + j * CYC_NS / 8, -1, {}), j);
        all.push_back(clean);

        tally late; late.name = std::string("late read start, ") + pre; late.unit = "bit";
        for (size_t k = 0; k < pre_bits; k++)
            add(late, run_case(cp, st4, 4, T0_NS, T0_NS + (k + 0.3) * CELL_NS, {}), (double)k + 1);
        all.push_back(late);

        tally stray; stray.name = std::string("stray edge before, ") + pre; stray.unit = "cells";
        for (int i = 3; i <= 40; i++)
            add(stray, run_case(cp, st4, 4, T0_NS, -1, { T0_NS - i * 0.1 * CELL_NS }), i * 0.1);
        all.push_back(stray);

        tally noise; noise.name = std::string("extra edge in preamble, ") + pre; noise.unit = "bit";
        for (size_t k = 0; k + 8 < pre_bits; k++)
            add(noise, run_case(cp, st4, 4, T0_NS, -1, { T0_NS + (k + 0.6) * CELL_NS }), (double)k);
        all.push_back(noise);
    }

    printf("iwm_asic read path vs MSB-framing IWM oracle (A2M2053 sp_wire.c)\n");
    printf("%-42s %5s %7s %7s %8s\n", "case", "n", "oracle", "dut", "dut-only");
    int harness_bad = 0, regress = 0;
    for (const tally &t : all) {
        printf("%-42s %5d %7d %7d %8d", t.name.c_str(), t.n, t.orc_ok, t.dut_ok, t.regress);
        if (!t.at.empty()) {
            printf("  at %s:", t.unit);
            size_t k = 0;
            for (double x : t.at) { if (k++ == 10) { printf(" ..."); break; } printf(" %g", x); }
        }
        printf("\n");
        if (t.name.rfind("clean", 0) == 0) harness_bad += t.n - t.orc_ok;
        regress += t.regress;
    }
    m.reset();
    if (harness_bad) { printf("HARNESS: oracle rejected %d clean cases\n", harness_bad); return 2; }
    if (regress) printf("FAIL: %d cases the oracle reads and the DUT does not\n", regress);
    else printf("PASS\n");
    return regress ? 1 : 0;
}
