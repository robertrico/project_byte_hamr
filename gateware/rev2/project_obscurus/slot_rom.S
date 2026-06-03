* =============================================================================
* slot_rom.S — project_obscurus $C400 stub (Merlin32)
* =============================================================================
* PR#4 does JSR $C400. Restore the BASIC output vector to COUT1 (so the
* monitor's COUT reaches the screen instead of recursing through $C400), then
* JMP into the monitor at $C800. The JSR return address stays on the stack;
* the monitor's Q command RTSes back to BASIC.
* =============================================================================
            TYP   $06
            DSK   slot_rom.bin
            ORG   $C400

CSWL   =    $36
CSWH   =    $37
COUT1  =    $FDF0
MONITOR =   $C800

ENTRY       LDA   #<COUT1
            STA   CSWL
            LDA   #>COUT1
            STA   CSWH
            JMP   MONITOR
