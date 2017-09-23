/*
 * Copyright (C) 2016 Socionext Inc.
 *   Author: Masahiro Yamada <yamada.masahiro@socionext.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MMC_UNIPHIER_SD_H__
#define __MMC_UNIPHIER_SD_H__

#define UNIPHIER_SD_CMD			0x000	/* command */
#define   UNIPHIER_SD_CMD_NOSTOP	BIT(14)	/* No automatic CMD12 issue */
#define   UNIPHIER_SD_CMD_MULTI		BIT(13)	/* multiple block transfer */
#define   UNIPHIER_SD_CMD_RD		BIT(12)	/* 1: read, 0: write */
#define   UNIPHIER_SD_CMD_DATA		BIT(11)	/* data transfer */
#define   UNIPHIER_SD_CMD_APP		BIT(6)	/* ACMD preceded by CMD55 */
#define   UNIPHIER_SD_CMD_NORMAL	(0 << 8)/* auto-detect of resp-type */
#define   UNIPHIER_SD_CMD_RSP_NONE	(3 << 8)/* response: none */
#define   UNIPHIER_SD_CMD_RSP_R1	(4 << 8)/* response: R1, R5, R6, R7 */
#define   UNIPHIER_SD_CMD_RSP_R1B	(5 << 8)/* response: R1b, R5b */
#define   UNIPHIER_SD_CMD_RSP_R2	(6 << 8)/* response: R2 */
#define   UNIPHIER_SD_CMD_RSP_R3	(7 << 8)/* response: R3, R4 */
#define UNIPHIER_SD_ARG			0x008	/* command argument */
#define UNIPHIER_SD_STOP		0x010	/* stop action control */
#define   UNIPHIER_SD_STOP_SEC		BIT(8)	/* use sector count */
#define   UNIPHIER_SD_STOP_STP		BIT(0)	/* issue CMD12 */
#define UNIPHIER_SD_SECCNT		0x014	/* sector counter */
#define UNIPHIER_SD_RSP10		0x018	/* response[39:8] */
#define UNIPHIER_SD_RSP32		0x020	/* response[71:40] */
#define UNIPHIER_SD_RSP54		0x028	/* response[103:72] */
#define UNIPHIER_SD_RSP76		0x030	/* response[127:104] */
#define UNIPHIER_SD_INFO1		0x038	/* IRQ status 1 */
#define   UNIPHIER_SD_INFO1_CD		BIT(5)	/* state of card detect */
#define   UNIPHIER_SD_INFO1_INSERT	BIT(4)	/* card inserted */
#define   UNIPHIER_SD_INFO1_REMOVE	BIT(3)	/* card removed */
#define   UNIPHIER_SD_INFO1_CMP		BIT(2)	/* data complete */
#define   UNIPHIER_SD_INFO1_RSP		BIT(0)	/* response complete */
#define UNIPHIER_SD_INFO2		0x03c	/* IRQ status 2 */
#define   UNIPHIER_SD_INFO2_ERR_ILA	BIT(15)	/* illegal access err */
#define   UNIPHIER_SD_INFO2_CBSY	BIT(14)	/* command busy */
#define   UNIPHIER_SD_INFO2_BWE		BIT(9)	/* write buffer ready */
#define   UNIPHIER_SD_INFO2_BRE		BIT(8)	/* read buffer ready */
#define   UNIPHIER_SD_INFO2_DAT0	BIT(7)	/* SDDAT0 */
#define   UNIPHIER_SD_INFO2_ERR_RTO	BIT(6)	/* response time out */
#define   UNIPHIER_SD_INFO2_ERR_ILR	BIT(5)	/* illegal read err */
#define   UNIPHIER_SD_INFO2_ERR_ILW	BIT(4)	/* illegal write err */
#define   UNIPHIER_SD_INFO2_ERR_TO	BIT(3)	/* time out error */
#define   UNIPHIER_SD_INFO2_ERR_END	BIT(2)	/* END bit error */
#define   UNIPHIER_SD_INFO2_ERR_CRC	BIT(1)	/* CRC error */
#define   UNIPHIER_SD_INFO2_ERR_IDX	BIT(0)	/* cmd index error */
#define UNIPHIER_SD_INFO1_MASK		0x040
#define UNIPHIER_SD_INFO2_MASK		0x044
#define UNIPHIER_SD_CLKCTL		0x048	/* clock divisor */
#define   UNIPHIER_SD_CLKCTL_DIV_MASK	0x104ff
#define   UNIPHIER_SD_CLKCTL_DIV1024	BIT(16)	/* SDCLK = CLK / 1024 */
#define   UNIPHIER_SD_CLKCTL_DIV512	BIT(7)	/* SDCLK = CLK / 512 */
#define   UNIPHIER_SD_CLKCTL_DIV256	BIT(6)	/* SDCLK = CLK / 256 */
#define   UNIPHIER_SD_CLKCTL_DIV128	BIT(5)	/* SDCLK = CLK / 128 */
#define   UNIPHIER_SD_CLKCTL_DIV64	BIT(4)	/* SDCLK = CLK / 64 */
#define   UNIPHIER_SD_CLKCTL_DIV32	BIT(3)	/* SDCLK = CLK / 32 */
#define   UNIPHIER_SD_CLKCTL_DIV16	BIT(2)	/* SDCLK = CLK / 16 */
#define   UNIPHIER_SD_CLKCTL_DIV8	BIT(1)	/* SDCLK = CLK / 8 */
#define   UNIPHIER_SD_CLKCTL_DIV4	BIT(0)	/* SDCLK = CLK / 4 */
#define   UNIPHIER_SD_CLKCTL_DIV2	0	/* SDCLK = CLK / 2 */
#define   UNIPHIER_SD_CLKCTL_DIV1	BIT(10)	/* SDCLK = CLK */
#define   UNIPHIER_SD_CLKCTL_RCAR_DIV1	0xff	/* SDCLK = CLK (RCar ver.) */
#define   UNIPHIER_SD_CLKCTL_OFFEN	BIT(9)	/* stop SDCLK when unused */
#define   UNIPHIER_SD_CLKCTL_SCLKEN	BIT(8)	/* SDCLK output enable */
#define UNIPHIER_SD_SIZE		0x04c	/* block size */
#define UNIPHIER_SD_OPTION		0x050
#define   UNIPHIER_SD_OPTION_WIDTH_MASK	(5 << 13)
#define   UNIPHIER_SD_OPTION_WIDTH_1	(4 << 13)
#define   UNIPHIER_SD_OPTION_WIDTH_4	(0 << 13)
#define   UNIPHIER_SD_OPTION_WIDTH_8	(1 << 13)
#define UNIPHIER_SD_BUF			0x060	/* read/write buffer */
#define UNIPHIER_SD_EXTMODE		0x1b0
#define   UNIPHIER_SD_EXTMODE_DMA_EN	BIT(1)	/* transfer 1: DMA, 0: pio */
#define UNIPHIER_SD_SOFT_RST		0x1c0
#define UNIPHIER_SD_SOFT_RST_RSTX	BIT(0)	/* reset deassert */
#define UNIPHIER_SD_VERSION		0x1c4	/* version register */
#define UNIPHIER_SD_VERSION_IP		0xff	/* IP version */
#define UNIPHIER_SD_HOST_MODE		0x1c8
#define UNIPHIER_SD_IF_MODE		0x1cc
#define   UNIPHIER_SD_IF_MODE_DDR	BIT(0)	/* DDR mode */
#define UNIPHIER_SD_VOLT		0x1e4	/* voltage switch */
#define   UNIPHIER_SD_VOLT_MASK		(3 << 0)
#define   UNIPHIER_SD_VOLT_OFF		(0 << 0)
#define   UNIPHIER_SD_VOLT_330		(1 << 0)/* 3.3V signal */
#define   UNIPHIER_SD_VOLT_180		(2 << 0)/* 1.8V signal */
#define UNIPHIER_SD_DMA_MODE		0x410
#define   UNIPHIER_SD_DMA_MODE_DIR_RD	BIT(16)	/* 1: from device, 0: to dev */
#define   UNIPHIER_SD_DMA_MODE_ADDR_INC	BIT(0)	/* 1: address inc, 0: fixed */
#define UNIPHIER_SD_DMA_CTL		0x414
#define   UNIPHIER_SD_DMA_CTL_START	BIT(0)	/* start DMA (auto cleared) */
#define UNIPHIER_SD_DMA_RST		0x418
#define   UNIPHIER_SD_DMA_RST_RD	BIT(9)
#define   UNIPHIER_SD_DMA_RST_WR	BIT(8)
#define UNIPHIER_SD_DMA_INFO1		0x420
#define   UNIPHIER_SD_DMA_INFO1_END_RD2	BIT(20)	/* DMA from device is complete (uniphier) */
#define   UNIPHIER_SD_DMA_INFO1_END_RD	BIT(17)	/* DMA from device is complete (renesas) */
#define   UNIPHIER_SD_DMA_INFO1_END_WR	BIT(16)	/* DMA to device is complete */
#define UNIPHIER_SD_DMA_INFO1_MASK	0x424
#define UNIPHIER_SD_DMA_INFO2		0x428
#define   UNIPHIER_SD_DMA_INFO2_ERR_RD	BIT(17)
#define   UNIPHIER_SD_DMA_INFO2_ERR_WR	BIT(16)
#define UNIPHIER_SD_DMA_INFO2_MASK	0x42c
#define UNIPHIER_SD_DMA_ADDR_L		0x440
#define UNIPHIER_SD_DMA_ADDR_H		0x444

/* alignment required by the DMA engine of this controller */
#define UNIPHIER_SD_DMA_MINALIGN	0x10

struct uniphier_sd_priv {
	void __iomem *regbase;
	unsigned long mclk;
	unsigned int version;
	u32 caps;
#define UNIPHIER_SD_CAP_NONREMOVABLE	BIT(0)	/* Nonremovable e.g. eMMC */
#define UNIPHIER_SD_CAP_DMA_INTERNAL	BIT(1)	/* have internal DMA engine */
#define UNIPHIER_SD_CAP_DIV1024		BIT(2)	/* divisor 1024 is available */
#define UNIPHIER_SD_CAP_64BIT		BIT(3)	/* Controller is 64bit */
#define UNIPHIER_SD_CAP_RCAR_GEN2	BIT(4)	/* Renesas RCar version of IP */
#define UNIPHIER_SD_CAP_RCAR_GEN3	BIT(5)	/* Renesas RCar version of IP */
#define UNIPHIER_SD_CAP_RCAR_UHS	BIT(6)	/* Renesas RCar UHS/SDR modes */
#define UNIPHIER_SD_CAP_RCAR		\
	(UNIPHIER_SD_CAP_RCAR_GEN2 | UNIPHIER_SD_CAP_RCAR_GEN3)
#ifdef CONFIG_DM_REGULATOR
	struct udevice *vqmmc_dev;
#endif
};

/* IO accessors */
u64 uniphier_sd_readq(struct uniphier_sd_priv *priv, unsigned int reg);
void uniphier_sd_writeq(struct uniphier_sd_priv *priv,
			u64 val, unsigned int reg);
u32 uniphier_sd_readl(struct uniphier_sd_priv *priv, unsigned int reg);
void uniphier_sd_writel(struct uniphier_sd_priv *priv,
			u32 val, unsigned int reg);

#endif /* __MMC_UNIPHIER_SD_H__ */
