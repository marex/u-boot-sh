/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2012  Renesas Solutions Corp.
 */

#include <config.h>
#include <asm/processor.h>
#include <asm/macro.h>

	.global	lowlevel_init

	.text
	.align	2

lowlevel_init:
	write8	SCIF_TDR, STR_S
	write8	SCIF_TDR, STR_H
	write8	SCIF_TDR, STR_4
	write8	SCIF_TDR, STR_nl
	rts
	nop

	.align 2

SCIF_TDR:	.long 0xe6e6000c
STR_S:		.long 'S'
STR_H:		.long 'H'
STR_4:		.long '4'
STR_nl:		.long '\n'
