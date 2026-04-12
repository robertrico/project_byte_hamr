    .syntax unified
    .cpu cortex-m0plus
    .thumb

@ ============================================================
@ disk_image.S — Disk image embedded in flash
@ ============================================================
@
@ 280 blocks × 512 bytes = 143,360 bytes
@ Sits in .rodata (flash), no RAM cost.
@
@ Access: disk_image + (block_num * 512)
@
@ Override at build time:  cmake -DDISK_IMAGE=images/Merlin.po ..
@ Default: images/ProDOS_2_4_1.po

#ifndef DISK_IMAGE
#define DISK_IMAGE images/ProDOS_2_4_1.po
#endif

#define _STR(x) #x
#define STR(x) _STR(x)

    .section .rodata
    .global disk_image
    .global disk_image_blocks
    .align 4
disk_image:
    .incbin STR(DISK_IMAGE)

    .align 2
disk_image_blocks:
    .word 280                       @ number of 512-byte blocks
