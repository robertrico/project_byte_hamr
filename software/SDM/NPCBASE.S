* NPC base price tables - SINGLE AUTHORITY
* PUT by FARM.S (game seeder) + FARMTEST.S (test
* seeder). The task blobs read base from SDRAM;
* this is the only place the numbers live.
* crop NPCs: 5 x NCROPS, [npc*NCROPS + crop]
* (wheat carrot berry pumpkin)
NPCBASE
 DFB 8,14,20,32
 DFB 4,8,40,64
 DFB 2,4,7,12
 DFB 1,3,7,12
 DFB 3,6,5,9
* goods NPCs: 2 x NRECIP, [npc*NRECIP + recipe]
NPGBASE
 DFB 22,30,38,50,62,72,72,80,118,124,124,130
 DFB 18,25,32,43,54,64,64,72,135,145,145,160
