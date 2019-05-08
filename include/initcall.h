/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2011 The Chromium OS Authors.
 */

#ifndef __INITCALL_H
#define __INITCALL_H

typedef int (*init_fnc_t)(void);

/*
 * To enable printfging. add #define DEBUG at the top of the including file.
 *
 * To find a symbol, use grep on u-boot.map
 */
#include <asm/io.h>
static void phex(unsigned int addr)
{
	int i, j;

	writeb('0', 0xe6e6000c);
	writeb('x', 0xe6e6000c);
	for (i = 28; i >= 0; i -= 4) {
		char ch = (addr >> i) & 0xf;
		if (ch >= 10)
			ch = ch - 10 + 'a';
		else
			ch += '0';
		writeb(ch, 0xe6e6000c);
		for (j = 0; j < 10000; j++)
			asm volatile("":::"memory");
	}
	writeb('\r', 0xe6e6000c);
	writeb('\n', 0xe6e6000c);
	for (i = 0; i < 100000; i++)
		asm volatile("":::"memory");
}
static inline int initcall_run_list(const init_fnc_t init_sequence[])
{
	const init_fnc_t *init_fnc_ptr;
extern unsigned int *reloc_dst;
extern unsigned int *reloc_dst_end;
phex(&reloc_dst);
phex(&reloc_dst_end);
	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		unsigned long reloc_ofs = 0;
		int ret;

		/*
		 * Sandbox is relocated by the OS, so symbols always appear at
		 * the relocated address.
		 */
		if (IS_ENABLED(CONFIG_SANDBOX) || (gd->flags & GD_FLG_RELOC))
			reloc_ofs = gd->reloc_off;
#ifdef CONFIG_EFI_APP
		reloc_ofs = (unsigned long)image_base;
#endif
		printf("initcall: %p", (char *)*init_fnc_ptr - reloc_ofs);
		if (reloc_ofs)
			printf(" (relocated to %p)\n", (char *)*init_fnc_ptr);
		else
			printf("\n");
		ret = (*init_fnc_ptr)();
		if (ret) {
			printf("initcall sequence %p failed at call %p (err=%d)\n",
			       init_sequence,
			       (char *)*init_fnc_ptr - reloc_ofs, ret);
			return -1;
		}
	}
	return 0;
}

#endif
