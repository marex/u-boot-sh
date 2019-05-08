// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Vladimir Zapolskiy <vz@mleia.com>
 */

#include <common.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;

	return 0;
}

void relocate_code(ulong start_addr_sp, gd_t *new_gd, ulong relocaddr)
{
	void (*reloc_board_init_r)(gd_t *gd, ulong dest) = board_init_r;

	if (new_gd->reloc_off) {
		memcpy((void *)new_gd->relocaddr,
		       (void *)(new_gd->relocaddr - new_gd->reloc_off),
		       new_gd->mon_len);

		reloc_board_init_r += new_gd->reloc_off;
	}

printf("%s[%i] %p %x | %p %p %i\n", __func__, __LINE__, reloc_board_init_r,
new_gd,
(void *)new_gd->relocaddr,
		       (void *)(new_gd->relocaddr - new_gd->reloc_off),
		       new_gd->mon_len
);

	__asm__ __volatile__("mov.l %0, r15\n" : : "m" (new_gd->start_addr_sp));

	while (1)
		reloc_board_init_r(new_gd, 0x0);
}
