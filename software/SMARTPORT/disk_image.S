    .syntax unified
    .cpu cortex-m0plus
    .thumb

@ ============================================================
@ disk_image.S — ProDOS disk image embedded in flash
@ ============================================================
@
@ 280 blocks × 512 bytes = 143,360 bytes
@ Sits in .rodata (flash), no RAM cost.
@
@ Access: disk_image + (block_num * 512)

    .section .rodata
    .global disk_image
    .global disk_image_blocks
    .align 4
disk_image:
    .incbin "ProDOS_2_4_1.po"

    .align 2
disk_image_blocks:
    .word 280                       @ number of 512-byte blocks
