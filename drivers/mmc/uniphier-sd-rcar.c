/*
 * Renesas RCar Gen3 SDHI SDR104/HS200 tuning routine
 *
 * Copyright (C) 2018 Marek Vasut <marek.vasut@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <linux/io.h>
#include <mmc.h>

/*
#include <clk.h>
#include <fdtdec.h>
#include <dm/pinctrl.h>
#include <linux/compat.h>
#include <linux/dma-direction.h>
#include <power/regulator.h>
#include <asm/unaligned.h>
*/
#include "uniphier-sd.h"

/* SCC registers */
#define SH_MOBILE_SDHI_SCC_DTCNTL			0x800
#define   SH_MOBILE_SDHI_SCC_DTCNTL_TAPEN		BIT(0)
#define   SH_MOBILE_SDHI_SCC_DTCNTL_TAPNUM_SHIFT	16
#define   SH_MOBILE_SDHI_SCC_DTCNTL_TAPNUM_MASK		0xff
#define SH_MOBILE_SDHI_SCC_TAPSET			0x804
#define SH_MOBILE_SDHI_SCC_DT2FF			0x808
#define SH_MOBILE_SDHI_SCC_CKSEL			0x80c
#define   SH_MOBILE_SDHI_SCC_CKSEL_DTSEL		BIT(0)
#define SH_MOBILE_SDHI_SCC_RVSCNTL			0x810
#define   SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN		BIT(0)
#define SH_MOBILE_SDHI_SCC_RVSREQ			0x814
#define   SH_MOBILE_SDHI_SCC_RVSREQ_RVSERR		BIT(2)
#define SH_MOBILE_SDHI_SCC_SMPCMP			0x818
#define SH_MOBILE_SDHI_SCC_TMPPORT2			0x81c

#define SH_MOBILE_SDHI_MAX_TAP 3

static unsigned int uniphier_sd_init_tuning(struct uniphier_sd_priv *priv)
{
	u32 reg;

	/* Initialize SCC */
	uniphier_sd_writel(priv, 0, UNIPHIER_SD_INFO1);

	reg = uniphier_sd_readl(priv, UNIPHIER_SD_CLKCTL);
	reg &= ~UNIPHIER_SD_CLKCTL_SCLKEN;
	uniphier_sd_writel(priv, reg, UNIPHIER_SD_CLKCTL);

	/* Set sampling clock selection range */
	uniphier_sd_writel(priv, 0x8 << SH_MOBILE_SDHI_SCC_DTCNTL_TAPNUM_SHIFT,
			   SH_MOBILE_SDHI_SCC_DTCNTL);

	reg = uniphier_sd_readl(priv, SH_MOBILE_SDHI_SCC_DTCNTL);
	reg |= SH_MOBILE_SDHI_SCC_DTCNTL_TAPEN;
	uniphier_sd_writel(priv, reg, SH_MOBILE_SDHI_SCC_DTCNTL);

	reg = uniphier_sd_readl(priv, SH_MOBILE_SDHI_SCC_CKSEL);
	reg |= SH_MOBILE_SDHI_SCC_CKSEL_DTSEL;
	uniphier_sd_writel(priv, reg, SH_MOBILE_SDHI_SCC_CKSEL);

	reg = uniphier_sd_readl(priv, SH_MOBILE_SDHI_SCC_RVSCNTL);
	reg &= ~SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN;
	uniphier_sd_writel(priv, reg, SH_MOBILE_SDHI_SCC_RVSCNTL);

	uniphier_sd_writel(priv, 0x300 /* scc_tappos */,
			   SH_MOBILE_SDHI_SCC_DT2FF);

	reg = uniphier_sd_readl(priv, UNIPHIER_SD_CLKCTL);
	reg |= UNIPHIER_SD_CLKCTL_SCLKEN;
	uniphier_sd_writel(priv, reg, UNIPHIER_SD_CLKCTL);

	/* Read TAPNUM */
	return (uniphier_sd_readl(priv, SH_MOBILE_SDHI_SCC_DTCNTL) >>
		SH_MOBILE_SDHI_SCC_DTCNTL_TAPNUM_SHIFT) &
		SH_MOBILE_SDHI_SCC_DTCNTL_TAPNUM_MASK;
}

void rcar_gen3_sd_reset_tuning(struct uniphier_sd_priv *priv)
{
	u32 reg;

	/* Reset SCC */
	reg = uniphier_sd_readl(priv, UNIPHIER_SD_CLKCTL);
	reg &= ~UNIPHIER_SD_CLKCTL_SCLKEN;
	uniphier_sd_writel(priv, reg, UNIPHIER_SD_CLKCTL);

	reg = uniphier_sd_readl(priv, SH_MOBILE_SDHI_SCC_CKSEL);
	reg &= ~SH_MOBILE_SDHI_SCC_CKSEL_DTSEL;
	uniphier_sd_writel(priv, reg, SH_MOBILE_SDHI_SCC_CKSEL);

	reg = uniphier_sd_readl(priv, UNIPHIER_SD_CLKCTL);
	reg |= UNIPHIER_SD_CLKCTL_SCLKEN;
	uniphier_sd_writel(priv, reg, UNIPHIER_SD_CLKCTL);

	reg = uniphier_sd_readl(priv, SH_MOBILE_SDHI_SCC_RVSCNTL);
	reg &= ~SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN;
	uniphier_sd_writel(priv, reg, SH_MOBILE_SDHI_SCC_RVSCNTL);

	reg = uniphier_sd_readl(priv, SH_MOBILE_SDHI_SCC_RVSCNTL);
	reg &= ~SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN;
	uniphier_sd_writel(priv, reg, SH_MOBILE_SDHI_SCC_RVSCNTL);
}

static void uniphier_sd_prepare_tuning(struct uniphier_sd_priv *priv,
				       unsigned long tap)
{
	/* Set sampling clock position */
	uniphier_sd_writel(priv, tap, SH_MOBILE_SDHI_SCC_TAPSET);
}

static unsigned int sh_mobile_sdhi_compare_scc_data(struct uniphier_sd_priv *priv)
{
	/* Get comparison of sampling data */
	return uniphier_sd_readl(priv, SH_MOBILE_SDHI_SCC_SMPCMP);
}

static int uniphier_sd_select_tuning(struct uniphier_sd_priv *priv,
				     unsigned int tap_num, unsigned int taps,
				     unsigned int smpcmp)
{
	unsigned long tap_cnt;  /* counter of tuning success */
	unsigned long tap_set;  /* tap position */
	unsigned long tap_start;/* start position of tuning success */
	unsigned long tap_end;  /* end position of tuning success */
	unsigned long ntap;     /* temporary counter of tuning success */
	unsigned long match_cnt;/* counter of matching data */
	unsigned long i;
	bool select = false;
	u32 reg;

	/* Clear SCC_RVSREQ */
	uniphier_sd_writel(priv, 0, SH_MOBILE_SDHI_SCC_RVSREQ);

	/* Merge the results */
	for (i = 0; i < tap_num * 2; i++) {
		if (!(taps & BIT(i))) {
			taps &= ~BIT(i % tap_num);
			taps &= ~BIT((i % tap_num) + tap_num);
		}
		if (!(smpcmp & BIT(i))) {
			smpcmp &= ~BIT(i % tap_num);
			smpcmp &= ~BIT((i % tap_num) + tap_num);
		}
	}

	/*
	 * Find the longest consecutive run of successful probes.  If that
	 * is more than SH_MOBILE_SDHI_MAX_TAP probes long then use the
	 * center index as the tap.
	 */
	tap_cnt = 0;
	ntap = 0;
	tap_start = 0;
	tap_end = 0;
	for (i = 0; i < tap_num * 2; i++) {
		if (taps & BIT(i))
			ntap++;
		else {
			if (ntap > tap_cnt) {
				tap_start = i - ntap;
				tap_end = i - 1;
				tap_cnt = ntap;
			}
			ntap = 0;
		}
	}

	if (ntap > tap_cnt) {
		tap_start = i - ntap;
		tap_end = i - 1;
		tap_cnt = ntap;
	}

	/*
	 * If all of the TAP is OK, the sampling clock position is selected by
	 * identifying the change point of data.
	 */
	if (tap_cnt == tap_num * 2) {
		match_cnt = 0;
		ntap = 0;
		tap_start = 0;
		tap_end = 0;
		for (i = 0; i < tap_num * 2; i++) {
			if (smpcmp & BIT(i))
				ntap++;
			else {
				if (ntap > match_cnt) {
					tap_start = i - ntap;
					tap_end = i - 1;
					match_cnt = ntap;
				}
				ntap = 0;
			}
		}
		if (ntap > match_cnt) {
			tap_start = i - ntap;
			tap_end = i - 1;
			match_cnt = ntap;
		}
		if (match_cnt)
			select = true;
	} else if (tap_cnt >= SH_MOBILE_SDHI_MAX_TAP)
		select = true;

	if (select)
		tap_set = ((tap_start + tap_end) / 2) % tap_num;
	else
		return -EIO;

	/* Set SCC */
	uniphier_sd_writel(priv, tap_set, SH_MOBILE_SDHI_SCC_TAPSET);

	/* Enable auto re-tuning */
	reg = uniphier_sd_readl(priv, SH_MOBILE_SDHI_SCC_RVSCNTL);
	reg |= SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN;
	uniphier_sd_writel(priv, reg, SH_MOBILE_SDHI_SCC_RVSCNTL);

	return 0;
}

int rcar_gen3_sd_execute_tuning(struct udevice *dev, uint opcode)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct mmc *mmc = upriv->mmc;
	unsigned int tap_num;
	unsigned int taps = 0, smpcmp = 0;
	int i, ret = 0;
	u32 caps;

	/* Only supported on Renesas RCar */
	if (!(priv->caps & UNIPHIER_SD_CAP_RCAR_UHS))
		return -EINVAL;

	/* clock tuning is not needed for upto 52MHz */
	if (!((mmc->selected_mode == MMC_HS_200) ||
	      (mmc->selected_mode == UHS_SDR104) ||
	      (mmc->selected_mode == UHS_SDR50)))
		return 0;

	tap_num = uniphier_sd_init_tuning(priv);
	if (!tap_num)
		/* Tuning is not supported */
		goto out;

	if (tap_num * 2 >= sizeof(taps) * 8) {
		dev_err(dev,
			"Too many taps, skipping tuning. Please consider updating size of taps field of tmio_mmc_host\n");
		goto out;
	}

	/* Issue CMD19 twice for each tap */
	for (i = 0; i < 2 * tap_num; i++) {
		uniphier_sd_prepare_tuning(priv, i % tap_num);

		/* Force PIO for the tuning */
		caps = priv->caps;
		priv->caps &= ~UNIPHIER_SD_CAP_DMA_INTERNAL;

		ret = mmc_send_tuning(mmc, opcode, NULL);

		priv->caps = caps;

		if (ret == 0)
			taps |= BIT(i);

		ret = sh_mobile_sdhi_compare_scc_data(priv);
		if (ret == 0)
			smpcmp |= BIT(i);

		mdelay(1);
	}

	ret = uniphier_sd_select_tuning(priv, tap_num, taps, smpcmp);

out:
	if (ret < 0) {
		dev_warn(dev, "Tuning procedure failed\n");
		rcar_gen3_sd_reset_tuning(priv);
	}

	return ret;
}
