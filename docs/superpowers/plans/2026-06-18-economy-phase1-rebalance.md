# Economy Phase 1 — Recipe Rebalance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Invert the recipe markup curve (rarity+risk) so bread is a thin margin and rare/risky recipes pay fat — by retuning the recipe VALs across all four tables that must agree, plus a CVER bump to re-seed.

**Architecture:** Pure data retune (no logic, no blob change). The recipe VALs live in four DFB tables that must stay in sync: `RECTAB` byte 5 (FARM.S, the cash authority streamed to SDRAM), `RVALUE` (FARM.S display), `RPRICEL`/`RPRICEH` (FARM.S learn-cost = 2× VAL), and a duplicate `RECTAB` in FARMTEST.S (the self-test's SDRAM seed). WORKTASK reads VAL from SDRAM at runtime, so no blob changes. CVER 4→5 forces the game to re-seed the new economy on a warm world.

**Tech Stack:** Merlin32 6502 assembly (single-space source); `make farm` / `make farmtest` / `make sdmdisk`; validated on hardware via `BRUN FARM_TEST`.

**Spec:** `docs/superpowers/specs/2026-06-18-economy-rebalance-vendors-design.md` §1.

**The new VALs** (by recipe 0-11): `18,25,32,43,54,64,64,72,112,119,119,127`
**RPRICE = 2× VAL:** `36,50,64,86,108,128,128,144,224,238,238,254` (all ≤254 → RPRICEH stays all 0)

---

## File Structure

| File | Table | Role |
|---|---|---|
| `software/SDM/FARM.S` | `RECTAB` byte 5 | cash authority → streamed to SDRAM by SEED33 |
| `software/SDM/FARM.S` | `RVALUE` | //e goods-price display (must == RECTAB byte 5) |
| `software/SDM/FARM.S` | `RPRICEL`/`RPRICEH` | recipe learn-cost (= 2× VAL) |
| `software/SDM/FARMTEST.S` | `RECTAB` (dup) | self-test SDRAM seed + its WSELL assertion |
| `software/SDM/FARMEQU.S` | `CVERNUM` | 4→5, forces re-seed on warm world |

WORKTASK.S / FARMTASK.S: **unchanged** (recipe VALs are read from SDRAM at runtime, not baked in the blob).

---

### Task 1: Retune the recipe economy (atomic — all four tables + CVER + test)

The four VAL tables MUST agree; retune them together in one commit so there's no inconsistent intermediate.

**Files:**
- Modify: `software/SDM/FARM.S` (RECTAB, RVALUE, RPRICEL)
- Modify: `software/SDM/FARMTEST.S` (RECTAB dup + the WSELL VAL assertion)
- Modify: `software/SDM/FARMEQU.S` (CVERNUM)

- [ ] **Step 1: Retune FARM.S RECTAB byte 5 (the VAL column)**

Replace the `RECTAB` table in `software/SDM/FARM.S`:
```
RECTAB
 DFB 0,0,$FF,$FF,4,28,0,0
 DFB 0,1,$FF,$FF,2,36,0,0
 DFB 1,1,$FF,$FF,4,48,0,0
 DFB 0,2,$FF,$FF,4,56,1,0
 DFB 0,0,2,$FF,8,76,1,0
 DFB 2,2,$FF,$FF,4,80,1,0
 DFB 0,3,$FF,$FF,8,80,1,0
 DFB 1,3,$FF,$FF,8,92,1,0
 DFB 1,1,3,$FF,12,100,2,0
 DFB 0,2,3,$FF,12,105,2,0
 DFB 2,2,2,$FF,12,110,2,0
 DFB 0,1,2,3,20,127,3,0
```
with (only byte 5 — the VAL — changes; inputs/TIME/RARITY/PAD unchanged):
```
RECTAB
 DFB 0,0,$FF,$FF,4,18,0,0
 DFB 0,1,$FF,$FF,2,25,0,0
 DFB 1,1,$FF,$FF,4,32,0,0
 DFB 0,2,$FF,$FF,4,43,1,0
 DFB 0,0,2,$FF,8,54,1,0
 DFB 2,2,$FF,$FF,4,64,1,0
 DFB 0,3,$FF,$FF,8,64,1,0
 DFB 1,3,$FF,$FF,8,72,1,0
 DFB 1,1,3,$FF,12,112,2,0
 DFB 0,2,3,$FF,12,119,2,0
 DFB 2,2,2,$FF,12,119,2,0
 DFB 0,1,2,3,20,127,3,0
```

- [ ] **Step 2: Retune FARM.S RVALUE (display, == new VALs)**

Replace:
```
RVALUE DFB 28,36,48,56,76,80,80,92
 DFB 100,105,110,127
```
with:
```
RVALUE DFB 18,25,32,43,54,64,64,72
 DFB 112,119,119,127
```

- [ ] **Step 3: Retune FARM.S RPRICEL (learn-cost = 2× VAL); confirm RPRICEH all 0**

Replace:
```
RPRICEL DFB 56,72,96,112,152,160,160,184
 DFB 200,210,220,254
```
with:
```
RPRICEL DFB 36,50,64,86,108,128,128,144
 DFB 224,238,238,254
```
Leave `RPRICEH DFB 0,0,0,0,0,0,0,0 / DFB 0,0,0,0` unchanged (every new RPRICE ≤254 < 256, so the high byte stays 0).

- [ ] **Step 4: Retune FARMTEST.S RECTAB (the duplicate test seed) — same byte-5 change**

In `software/SDM/FARMTEST.S`, replace its `RECTAB` block (identical old table) with the SAME new table from Step 1:
```
RECTAB
 DFB 0,0,$FF,$FF,4,18,0,0
 DFB 0,1,$FF,$FF,2,25,0,0
 DFB 1,1,$FF,$FF,4,32,0,0
 DFB 0,2,$FF,$FF,4,43,1,0
 DFB 0,0,2,$FF,8,54,1,0
 DFB 2,2,$FF,$FF,4,64,1,0
 DFB 0,3,$FF,$FF,8,64,1,0
 DFB 1,3,$FF,$FF,8,72,1,0
 DFB 1,1,3,$FF,12,112,2,0
 DFB 0,2,3,$FF,12,119,2,0
 DFB 2,2,2,$FF,12,119,2,0
 DFB 0,1,2,3,20,127,3,0
```

- [ ] **Step 5: Update the FARMTEST WSELL VAL assertion (recipe 0: 28 → 18)**

In `software/SDM/FARMTEST.S`, the WSELL test asserts the per-unit value of recipe 0. Find:
```
* WRES1V == 28 (recipe0 VAL, no BOOM)
```
and the `CMP #28` a few lines below it. Change the comment to `* WRES1V == 18 (recipe0 new VAL, no BOOM)` and the compare:
```
 CMP #28
```
to:
```
 CMP #18
```

- [ ] **Step 6: Bump CVERNUM 4 → 5 (force re-seed of the new economy)**

In `software/SDM/FARMEQU.S`, change:
```
CVERNUM = 4
```
to:
```
CVERNUM = 5
```
(The farm CVER gate + CHKWK now see a mismatch on a warm world → full cold-seed → SEED33 streams the new RECTAB VALs into SDRAM. FARM_TEST is always-cold and unaffected by CVER — it seeds its own RECTAB regardless.)

- [ ] **Step 7: Verify all four tables agree + RPRICE = 2× VAL (hard gate)**

Run this — it exits nonzero on any mismatch (don't commit unless it prints `PASS 4-table sync`):
```bash
cd /Users/hambook/Development/project_byte_hamr/software/SDM
rectab() { awk '/^RECTAB/{f=1;next} f&&/^ DFB/{split($0,a,",");printf "%d ",a[6]; c++} c==12{exit}' "$1"; }
vlist()  { sed -n "/^$2/,+1p" "$1" | grep -oE "[0-9]+" | tr '\n' ' '; }
EXP_V="18 25 32 43 54 64 64 72 112 119 119 127 "
EXP_P="36 50 64 86 108 128 128 144 224 238 238 254 "
ok=1
[ "$(rectab FARM.S)"      = "$EXP_V" ] || { echo "FAIL FARM RECTAB:    $(rectab FARM.S)"; ok=0; }
[ "$(rectab FARMTEST.S)"  = "$EXP_V" ] || { echo "FAIL FARMTEST RECTAB:$(rectab FARMTEST.S)"; ok=0; }
[ "$(vlist FARM.S RVALUE)"  = "$EXP_V" ] || { echo "FAIL RVALUE:  $(vlist FARM.S RVALUE)"; ok=0; }
[ "$(vlist FARM.S RPRICEL)" = "$EXP_P" ] || { echo "FAIL RPRICEL: $(vlist FARM.S RPRICEL)"; ok=0; }
[ "$ok" = 1 ] && echo "PASS 4-table sync (RECTAB==RVALUE==FARMTEST, RPRICE=2xVAL)" || exit 1
```
Expected: `PASS 4-table sync ...`. Any `FAIL` line names the table that's off — fix it, re-run, before Step 8. (`rectab` reads byte 5 = the VAL column of each DFB row; `vlist` pulls the numbers from the named single-+-continuation-line table.)

- [ ] **Step 8: Build farm + farmtest, confirm clean + embed==bin**

Run:
```bash
cd /Users/hambook/Development/project_byte_hamr
make farm 2>&1 | grep -iE "Creating Object file.*FARM.bin|Error" | tail -2
make farmtest 2>&1 | grep -iE "Creating Object file.*FARMTEST|Error" | tail -2
cd software/SDM
echo "embed==bin: FARMTASK $(grep -c DFB FARMTASKB.S)/$(wc -c <FARMTASK.bin) WORKTASK $(grep -c DFB WORKTASKB.S)/$(wc -c <WORKTASK.bin)"
```
Expected: both assemble clean (FARM.bin + FARMTEST.bin produced); embed counts equal bin sizes (the blobs are unchanged by this data retune, so they stay current).

- [ ] **Step 9: Commit**

```bash
cd /Users/hambook/Development/project_byte_hamr
git add software/SDM/FARM.S software/SDM/FARMTEST.S software/SDM/FARMEQU.S
git commit -m "feat(econ): phase 1 rebalance - invert recipe markup curve (rarity+risk)

Retune recipe VALs across all four sync'd tables: FARM.S RECTAB (cash
authority), RVALUE (display), RPRICEL (learn-cost = 2x VAL), and FARMTEST.S
RECTAB (test seed) + its WSELL assertion (28->18). Bread 28->18 (+2 over 2
wheat, marginal not runaway); R3 stays 127. CVERNUM 4->5 re-seeds the new
economy on a warm world. WORKTASK reads VALs from SDRAM - no blob change.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Bench validation (user-run; hardware)

**Files:** none (verification only). FARM_TEST is the proven on-hardware gate; a blob/economy fix iterates via disk rebuild + BRUN (no re-flash — the gateware/kernel are unchanged).

- [ ] **Step 1: Build the disk**

Run: `cd /Users/hambook/Development/project_byte_hamr && make sdmdisk 2>&1 | tail -5`
Expected: `software/SDM/SDMTEST.po` built (farm + farmtest rebuilt with the new economy; CONWAY/others unchanged). No MAXLEN/guard trips.

- [ ] **Step 2: Burn + run FARM_TEST (user, on the //e)**

Burn `SDMTEST.po`, then `BRUN FARM_TEST` (cold-reset first — FARM_TEST is always-cold).
Expected: full suite green, including **WSELL VAL = 18** (the rebalanced recipe-0 per-unit value). If WSELL VAL is red showing the old 28, the FARMTEST RECTAB seed (Step 4) or the assertion (Step 5) wasn't retuned — the four-table sync (Step 7) catches that pre-burn.

- [ ] **Step 3: Confirm the game economy (optional, user)**

`BRUN FARM` (on a warm world, the CVER 4→5 bump triggers a one-time cold-seed → the rebalanced economy is live). Bread now sells/values at 18; the shop learn-cost for bread is 36 (2×18). Spot-check that baking bread is no longer a runaway and high-rarity recipes pay the fat margins.

---

## Self-Review

**Spec coverage (§1):** the rarity+risk curve (new VALs) → Task 1 Step 1; RVALUE sync → Step 2; RPRICE = 2× VAL (the review's HIGH #1) → Step 3; FARMTEST RECTAB sync (review's HIGH #2) → Steps 4-5; CVER bump → Step 6; the four-table agreement guard → Step 7; "WORKTASK unchanged / VALs from SDRAM" → File Structure note + Step 8 (embed==bin). FAILBASE "unchanged, keep" → no task needed (correct). Crop economy untouched → no task (per spec, out of scope for Phase 1). Full coverage.

**Placeholder scan:** none — every table is literal, every value enumerated, the verify command is concrete.

**Type/value consistency:** the new VAL list `18,25,32,43,54,64,64,72,112,119,119,127` is identical in RECTAB (Step 1), RVALUE (Step 2), and FARMTEST RECTAB (Step 4); RPRICEL (Step 3) is exactly 2× each; the WSELL assertion (Step 5, recipe 0) = 18 = the new RECTAB[0] VAL. Consistent.

**Note for Phase 2:** the purchaser/vendor system (spec §2) is a separate, larger plan — write it after Phase 1 is bench-green. It builds on this rebalanced base and will bump CVER again (5→6) for the NPC SDRAM regions.
