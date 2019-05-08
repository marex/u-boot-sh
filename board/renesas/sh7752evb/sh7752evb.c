// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012  Renesas Solutions Corp.
 */

#include <common.h>
#include <env.h>
#include <malloc.h>
#include <asm/processor.h>
#include <asm/io.h>
#include <asm/mmc.h>
#include <spi.h>
#include <spi_flash.h>

DECLARE_GLOBAL_DATA_PTR;

int checkboard(void)
{
	puts("BOARD: SH7752 evaluation board (R0P7752C00000RZ)\n");

	return 0;
}

int board_init(void)
{
	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

	return 0;
}

int board_late_init(void)
{
	return 0;
}

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size = CONFIG_SYS_SDRAM_SIZE;
	return 0;
}
