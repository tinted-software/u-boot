/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * GS201 entry hook, called directly from arch/arm/cpu/armv8/start.S.
 * ABL passes its FDT in x0 and leaves both cluster watchdogs armed.
 * This must not use a stack or touch DRAM.
 */
#include <config.h>

.global save_boot_params
save_boot_params:
	/* Preserve ABL's x0 FDT pointer for get_prev_bl_fdt_addr(). */
	adrp	x8, reg0
	add	x8, x8, :lo12:reg0
	str	x0, [x8]

	/* WTCON bit 5 is enable; zero disarms each always-on watchdog. */
	movz	x8, #0x0000
	movk	x8, #0x1006, lsl #16
	str	wzr, [x8]
	movz	x8, #0x0000
	movk	x8, #0x1007, lsl #16
	str	wzr, [x8]


	/* Start U-Boot from a known translation/cache state (ABL may leave EL2 on). */
	mrs	x8, CurrentEL
	lsr	x8, x8, #2
	cmp	x8, #2
	b.eq	1f
	cmp	x8, #1
	b.ne	3f
	mrs	x8, sctlr_el1
	b	2f
1:	mrs	x8, sctlr_el2
2:	movz	x9, #0x1005		/* M, C and I */
	bic	x8, x8, x9
	mrs	x9, CurrentEL
	lsr	x9, x9, #2
	cmp	x9, #2
	b.eq	4f
	msr	sctlr_el1, x8
	b	5f
4:	msr	sctlr_el2, x8
5:	isb
	ic	iallu
	dsb	sy
	isb
3:
	b	save_boot_params_ret
