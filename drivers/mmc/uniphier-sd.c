/*
 * Copyright (C) 2016 Socionext Inc.
 *   Author: Masahiro Yamada <yamada.masahiro@socionext.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <clk.h>
#include <fdtdec.h>
#include <mmc.h>
#include <dm.h>
#include <dm/pinctrl.h>
#include <linux/compat.h>
#include <linux/dma-direction.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <power/regulator.h>
#include <asm/unaligned.h>

DECLARE_GLOBAL_DATA_PTR;

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

struct uniphier_sd_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct uniphier_sd_priv {
	void __iomem *regbase;
	unsigned long mclk;
	unsigned int version;
	u32 caps;
#define UNIPHIER_SD_CAP_NONREMOVABLE	BIT(0)	/* Nonremovable e.g. eMMC */
#define UNIPHIER_SD_CAP_DMA_INTERNAL	BIT(1)	/* have internal DMA engine */
#define UNIPHIER_SD_CAP_DIV1024		BIT(2)	/* divisor 1024 is available */
#define UNIPHIER_SD_CAP_64BIT		BIT(3)	/* Controller is 64bit */
#define UNIPHIER_SD_CAP_RCAR		BIT(4)	/* Renesas RCar version of IP */
#define UNIPHIER_SD_CAP_RCAR_UHS	BIT(5)	/* Renesas RCar UHS/SDR modes */
#ifdef CONFIG_DM_REGULATOR
	struct udevice *vqmmc_dev;
#endif
};

#if CONFIG_IS_ENABLED(MMC_HS200_SUPPORT)
static void uniphier_sd_reset_tuning(struct uniphier_sd_priv *priv);
#endif

static u64 uniphier_sd_readq(struct uniphier_sd_priv *priv, unsigned int reg)
{
	if (priv->caps & UNIPHIER_SD_CAP_64BIT)
		return readq(priv->regbase + (reg << 1));
	else
		return readq(priv->regbase + reg);
}

static void uniphier_sd_writeq(struct uniphier_sd_priv *priv,
			       u64 val, unsigned int reg)
{
	if (priv->caps & UNIPHIER_SD_CAP_64BIT)
		writeq(val, priv->regbase + (reg << 1));
	else
		writeq(val, priv->regbase + reg);
}

static u32 uniphier_sd_readl(struct uniphier_sd_priv *priv, unsigned int reg)
{
	if (priv->caps & UNIPHIER_SD_CAP_64BIT)
		return readl(priv->regbase + (reg << 1));
	else
		return readl(priv->regbase + reg);
}

static void uniphier_sd_writel(struct uniphier_sd_priv *priv,
			       u32 val, unsigned int reg)
{
	if (priv->caps & UNIPHIER_SD_CAP_64BIT)
		writel(val, priv->regbase + (reg << 1));
	else
		writel(val, priv->regbase + reg);
}

static dma_addr_t __dma_map_single(void *ptr, size_t size,
				   enum dma_data_direction dir)
{
	unsigned long addr = (unsigned long)ptr;

	if (dir == DMA_FROM_DEVICE)
		invalidate_dcache_range(addr, addr + size);
	else
		flush_dcache_range(addr, addr + size);

	return addr;
}

static void __dma_unmap_single(dma_addr_t addr, size_t size,
			       enum dma_data_direction dir)
{
	if (dir != DMA_TO_DEVICE)
		invalidate_dcache_range(addr, addr + size);
}

static int uniphier_sd_check_error(struct udevice *dev)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	u32 info2 = uniphier_sd_readl(priv, UNIPHIER_SD_INFO2);

	if (info2 & UNIPHIER_SD_INFO2_ERR_RTO) {
		/*
		 * TIMEOUT must be returned for unsupported command.  Do not
		 * display error log since this might be a part of sequence to
		 * distinguish between SD and MMC.
		 */
		return -ETIMEDOUT;
	}

	if (info2 & UNIPHIER_SD_INFO2_ERR_TO) {
		dev_err(dev, "timeout error\n");
		return -ETIMEDOUT;
	}

	if (info2 & (UNIPHIER_SD_INFO2_ERR_END | UNIPHIER_SD_INFO2_ERR_CRC |
		     UNIPHIER_SD_INFO2_ERR_IDX)) {
		dev_err(dev, "communication out of sync\n");
		return -EILSEQ;
	}

	if (info2 & (UNIPHIER_SD_INFO2_ERR_ILA | UNIPHIER_SD_INFO2_ERR_ILR |
		     UNIPHIER_SD_INFO2_ERR_ILW)) {
		dev_err(dev, "illegal access\n");
		return -EIO;
	}

	return 0;
}

static int uniphier_sd_wait_for_irq(struct udevice *dev, unsigned int reg,
				    u32 flag)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	long wait = 1000000;
	int ret;

	while (!(uniphier_sd_readl(priv, reg) & flag)) {
		if (wait-- < 0) {
			dev_err(dev, "timeout\n");
			return -ETIMEDOUT;
		}

		ret = uniphier_sd_check_error(dev);
		if (ret)
			return ret;

		udelay(1);
	}

	return 0;
}

static int uniphier_sd_pio_read_one_block(struct udevice *dev, char *pbuf,
					  uint blocksize)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	int i, ret;

	/* wait until the buffer is filled with data */
	ret = uniphier_sd_wait_for_irq(dev, UNIPHIER_SD_INFO2,
				       UNIPHIER_SD_INFO2_BRE);
	if (ret)
		return ret;

	/*
	 * Clear the status flag _before_ read the buffer out because
	 * UNIPHIER_SD_INFO2_BRE is edge-triggered, not level-triggered.
	 */
	uniphier_sd_writel(priv, 0, UNIPHIER_SD_INFO2);

	if (priv->caps & UNIPHIER_SD_CAP_64BIT) {
		u64 *buf = (u64 *)pbuf;
		if (likely(IS_ALIGNED((uintptr_t)buf, 8))) {
			for (i = 0; i < blocksize / 8; i++) {
				*buf++ = uniphier_sd_readq(priv,
							   UNIPHIER_SD_BUF);
			}
		} else {
			for (i = 0; i < blocksize / 8; i++) {
				u64 data;
				data = uniphier_sd_readq(priv,
							 UNIPHIER_SD_BUF);
				put_unaligned(data, buf++);
			}
		}
	} else {
		u32 *buf = (u32 *)pbuf;
		if (likely(IS_ALIGNED((uintptr_t)buf, 4))) {
			for (i = 0; i < blocksize / 4; i++) {
				*buf++ = uniphier_sd_readl(priv,
							   UNIPHIER_SD_BUF);
			}
		} else {
			for (i = 0; i < blocksize / 4; i++) {
				u32 data;
				data = uniphier_sd_readl(priv, UNIPHIER_SD_BUF);
				put_unaligned(data, buf++);
			}
		}
	}

	return 0;
}

static int uniphier_sd_pio_write_one_block(struct udevice *dev,
					   const char *pbuf, uint blocksize)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	int i, ret;

	/* wait until the buffer becomes empty */
	ret = uniphier_sd_wait_for_irq(dev, UNIPHIER_SD_INFO2,
				       UNIPHIER_SD_INFO2_BWE);
	if (ret)
		return ret;

	uniphier_sd_writel(priv, 0, UNIPHIER_SD_INFO2);

	if (priv->caps & UNIPHIER_SD_CAP_64BIT) {
		const u64 *buf = (const u64 *)pbuf;
		if (likely(IS_ALIGNED((uintptr_t)buf, 8))) {
			for (i = 0; i < blocksize / 8; i++) {
				uniphier_sd_writeq(priv, *buf++,
						   UNIPHIER_SD_BUF);
			}
		} else {
			for (i = 0; i < blocksize / 8; i++) {
				u64 data = get_unaligned(buf++);
				uniphier_sd_writeq(priv, data,
						   UNIPHIER_SD_BUF);
			}
		}
	} else {
		const u32 *buf = (const u32 *)pbuf;
		if (likely(IS_ALIGNED((uintptr_t)buf, 4))) {
			for (i = 0; i < blocksize / 4; i++) {
				uniphier_sd_writel(priv, *buf++,
						   UNIPHIER_SD_BUF);
			}
		} else {
			for (i = 0; i < blocksize / 4; i++) {
				u32 data = get_unaligned(buf++);
				uniphier_sd_writel(priv, data,
						   UNIPHIER_SD_BUF);
			}
		}
	}

	return 0;
}

static int uniphier_sd_pio_xfer(struct udevice *dev, struct mmc_data *data)
{
	const char *src = data->src;
	char *dest = data->dest;
	int i, ret;

	for (i = 0; i < data->blocks; i++) {
		if (data->flags & MMC_DATA_READ)
			ret = uniphier_sd_pio_read_one_block(dev, dest,
							     data->blocksize);
		else
			ret = uniphier_sd_pio_write_one_block(dev, src,
							      data->blocksize);
		if (ret)
			return ret;

		if (data->flags & MMC_DATA_READ)
			dest += data->blocksize;
		else
			src += data->blocksize;
	}

	return 0;
}

static void uniphier_sd_dma_start(struct uniphier_sd_priv *priv,
				  dma_addr_t dma_addr)
{
	u32 tmp;

	uniphier_sd_writel(priv, 0, UNIPHIER_SD_DMA_INFO1);
	uniphier_sd_writel(priv, 0, UNIPHIER_SD_DMA_INFO2);

	/* enable DMA */
	tmp = uniphier_sd_readl(priv, UNIPHIER_SD_EXTMODE);
	tmp |= UNIPHIER_SD_EXTMODE_DMA_EN;
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_EXTMODE);

	uniphier_sd_writel(priv, dma_addr & U32_MAX, UNIPHIER_SD_DMA_ADDR_L);

	/* suppress the warning "right shift count >= width of type" */
	dma_addr >>= min_t(int, 32, 8 * sizeof(dma_addr));

	uniphier_sd_writel(priv, dma_addr & U32_MAX, UNIPHIER_SD_DMA_ADDR_H);

	uniphier_sd_writel(priv, UNIPHIER_SD_DMA_CTL_START, UNIPHIER_SD_DMA_CTL);
}

static int uniphier_sd_dma_wait_for_irq(struct udevice *dev, u32 flag,
					unsigned int blocks)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	long wait = 1000000 + 10 * blocks;

	while (!(uniphier_sd_readl(priv, UNIPHIER_SD_DMA_INFO1) & flag)) {
		if (wait-- < 0) {
			dev_err(dev, "timeout during DMA\n");
			return -ETIMEDOUT;
		}

		udelay(10);
	}

	if (uniphier_sd_readl(priv, UNIPHIER_SD_DMA_INFO2)) {
		dev_err(dev, "error during DMA\n");
		return -EIO;
	}

	return 0;
}

static int uniphier_sd_dma_xfer(struct udevice *dev, struct mmc_data *data)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	size_t len = data->blocks * data->blocksize;
	void *buf;
	enum dma_data_direction dir;
	dma_addr_t dma_addr;
	u32 poll_flag, tmp;
	int ret;

	tmp = uniphier_sd_readl(priv, UNIPHIER_SD_DMA_MODE);

	if (data->flags & MMC_DATA_READ) {
		buf = data->dest;
		dir = DMA_FROM_DEVICE;
		/*
		 * The DMA READ completion flag position differs on Socionext
		 * and Renesas SoCs. It is bit 20 on Socionext SoCs and using
		 * bit 17 is a hardware bug and forbidden. It is bit 17 on
		 * Renesas SoCs and bit 20 does not work on them.
		 */
		poll_flag = (priv->caps & UNIPHIER_SD_CAP_RCAR) ?
			    UNIPHIER_SD_DMA_INFO1_END_RD :
			    UNIPHIER_SD_DMA_INFO1_END_RD2;
		tmp |= UNIPHIER_SD_DMA_MODE_DIR_RD;
	} else {
		buf = (void *)data->src;
		dir = DMA_TO_DEVICE;
		poll_flag = UNIPHIER_SD_DMA_INFO1_END_WR;
		tmp &= ~UNIPHIER_SD_DMA_MODE_DIR_RD;
	}

	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_DMA_MODE);

	dma_addr = __dma_map_single(buf, len, dir);

	uniphier_sd_dma_start(priv, dma_addr);

	ret = uniphier_sd_dma_wait_for_irq(dev, poll_flag, data->blocks);

	__dma_unmap_single(dma_addr, len, dir);

	return ret;
}

/* check if the address is DMA'able */
static bool uniphier_sd_addr_is_dmaable(unsigned long addr)
{
	if (!IS_ALIGNED(addr, UNIPHIER_SD_DMA_MINALIGN))
		return false;

#if defined(CONFIG_ARCH_UNIPHIER) && !defined(CONFIG_ARM64) && \
	defined(CONFIG_SPL_BUILD)
	/*
	 * For UniPhier ARMv7 SoCs, the stack is allocated in the locked ways
	 * of L2, which is unreachable from the DMA engine.
	 */
	if (addr < CONFIG_SPL_STACK)
		return false;
#endif

	return true;
}

static int uniphier_sd_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
				struct mmc_data *data)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	int ret;
	u32 tmp;

	if (uniphier_sd_readl(priv, UNIPHIER_SD_INFO2) & UNIPHIER_SD_INFO2_CBSY) {
		dev_err(dev, "command busy\n");
		return -EBUSY;
	}

	/* clear all status flags */
	uniphier_sd_writel(priv, 0, UNIPHIER_SD_INFO1);
	uniphier_sd_writel(priv, 0, UNIPHIER_SD_INFO2);

	/* disable DMA once */
	tmp = uniphier_sd_readl(priv, UNIPHIER_SD_EXTMODE);
	tmp &= ~UNIPHIER_SD_EXTMODE_DMA_EN;
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_EXTMODE);

	uniphier_sd_writel(priv, cmd->cmdarg, UNIPHIER_SD_ARG);

	tmp = cmd->cmdidx;

	if (data) {
		uniphier_sd_writel(priv, data->blocksize, UNIPHIER_SD_SIZE);
		uniphier_sd_writel(priv, data->blocks, UNIPHIER_SD_SECCNT);

		/* Do not send CMD12 automatically */
		tmp |= UNIPHIER_SD_CMD_NOSTOP | UNIPHIER_SD_CMD_DATA;

		if (data->blocks > 1)
			tmp |= UNIPHIER_SD_CMD_MULTI;

		if (data->flags & MMC_DATA_READ)
			tmp |= UNIPHIER_SD_CMD_RD;
	}

	/*
	 * Do not use the response type auto-detection on this hardware.
	 * CMD8, for example, has different response types on SD and eMMC,
	 * while this controller always assumes the response type for SD.
	 * Set the response type manually.
	 */
	switch (cmd->resp_type) {
	case MMC_RSP_NONE:
		tmp |= UNIPHIER_SD_CMD_RSP_NONE;
		break;
	case MMC_RSP_R1:
		tmp |= UNIPHIER_SD_CMD_RSP_R1;
		break;
	case MMC_RSP_R1b:
		tmp |= UNIPHIER_SD_CMD_RSP_R1B;
		break;
	case MMC_RSP_R2:
		tmp |= UNIPHIER_SD_CMD_RSP_R2;
		break;
	case MMC_RSP_R3:
		tmp |= UNIPHIER_SD_CMD_RSP_R3;
		break;
	default:
		dev_err(dev, "unknown response type\n");
		return -EINVAL;
	}

	dev_dbg(dev, "sending CMD%d (SD_CMD=%08x, SD_ARG=%08x)\n",
		cmd->cmdidx, tmp, cmd->cmdarg);
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_CMD);

	ret = uniphier_sd_wait_for_irq(dev, UNIPHIER_SD_INFO1,
				       UNIPHIER_SD_INFO1_RSP);
	if (ret)
		return ret;

	if (cmd->resp_type & MMC_RSP_136) {
		u32 rsp_127_104 = uniphier_sd_readl(priv, UNIPHIER_SD_RSP76);
		u32 rsp_103_72 = uniphier_sd_readl(priv, UNIPHIER_SD_RSP54);
		u32 rsp_71_40 = uniphier_sd_readl(priv, UNIPHIER_SD_RSP32);
		u32 rsp_39_8 = uniphier_sd_readl(priv, UNIPHIER_SD_RSP10);

		cmd->response[0] = ((rsp_127_104 & 0x00ffffff) << 8) |
				   ((rsp_103_72  & 0xff000000) >> 24);
		cmd->response[1] = ((rsp_103_72  & 0x00ffffff) << 8) |
				   ((rsp_71_40   & 0xff000000) >> 24);
		cmd->response[2] = ((rsp_71_40   & 0x00ffffff) << 8) |
				   ((rsp_39_8    & 0xff000000) >> 24);
		cmd->response[3] = (rsp_39_8     & 0xffffff)   << 8;
	} else {
		/* bit 39-8 */
		cmd->response[0] = uniphier_sd_readl(priv, UNIPHIER_SD_RSP10);
	}

	if (data) {
		/* use DMA if the HW supports it and the buffer is aligned */
		if (priv->caps & UNIPHIER_SD_CAP_DMA_INTERNAL &&
		    uniphier_sd_addr_is_dmaable((long)data->src))
			ret = uniphier_sd_dma_xfer(dev, data);
		else
			ret = uniphier_sd_pio_xfer(dev, data);

		ret = uniphier_sd_wait_for_irq(dev, UNIPHIER_SD_INFO1,
					       UNIPHIER_SD_INFO1_CMP);
		if (ret)
			return ret;
	}

	return ret;
}

static int uniphier_sd_set_bus_width(struct uniphier_sd_priv *priv,
				     struct mmc *mmc)
{
	u32 val, tmp;

	switch (mmc->bus_width) {
	case 0:
	case 1:
		val = UNIPHIER_SD_OPTION_WIDTH_1;
		break;
	case 4:
		val = UNIPHIER_SD_OPTION_WIDTH_4;
		break;
	case 8:
		val = UNIPHIER_SD_OPTION_WIDTH_8;
		break;
	default:
		return -EINVAL;
	}

	tmp = uniphier_sd_readl(priv, UNIPHIER_SD_OPTION);
	tmp &= ~UNIPHIER_SD_OPTION_WIDTH_MASK;
	tmp |= val;
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_OPTION);

	return 0;
}

static void uniphier_sd_set_ddr_mode(struct uniphier_sd_priv *priv,
				     struct mmc *mmc)
{
	u32 tmp;

	tmp = uniphier_sd_readl(priv, UNIPHIER_SD_IF_MODE);
	if (mmc->ddr_mode)
		tmp |= UNIPHIER_SD_IF_MODE_DDR;
	else
		tmp &= ~UNIPHIER_SD_IF_MODE_DDR;
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_IF_MODE);
}

static void uniphier_sd_set_clk_rate(struct uniphier_sd_priv *priv,
				     struct mmc *mmc)
{
	unsigned int divisor;
	u32 val, tmp;

	if (!mmc->clock)
		return;

	divisor = DIV_ROUND_UP(priv->mclk, mmc->clock);

	if (divisor <= 1)
		val = (priv->caps & UNIPHIER_SD_CAP_RCAR) ?
		      UNIPHIER_SD_CLKCTL_RCAR_DIV1 : UNIPHIER_SD_CLKCTL_DIV1;
	else if (divisor <= 2)
		val = UNIPHIER_SD_CLKCTL_DIV2;
	else if (divisor <= 4)
		val = UNIPHIER_SD_CLKCTL_DIV4;
	else if (divisor <= 8)
		val = UNIPHIER_SD_CLKCTL_DIV8;
	else if (divisor <= 16)
		val = UNIPHIER_SD_CLKCTL_DIV16;
	else if (divisor <= 32)
		val = UNIPHIER_SD_CLKCTL_DIV32;
	else if (divisor <= 64)
		val = UNIPHIER_SD_CLKCTL_DIV64;
	else if (divisor <= 128)
		val = UNIPHIER_SD_CLKCTL_DIV128;
	else if (divisor <= 256)
		val = UNIPHIER_SD_CLKCTL_DIV256;
	else if (divisor <= 512 || !(priv->caps & UNIPHIER_SD_CAP_DIV1024))
		val = UNIPHIER_SD_CLKCTL_DIV512;
	else
		val = UNIPHIER_SD_CLKCTL_DIV1024;

	tmp = uniphier_sd_readl(priv, UNIPHIER_SD_CLKCTL);
	if (tmp & UNIPHIER_SD_CLKCTL_SCLKEN &&
	    (tmp & UNIPHIER_SD_CLKCTL_DIV_MASK) == val)
		return;

	/* stop the clock before changing its rate to avoid a glitch signal */
	tmp &= ~UNIPHIER_SD_CLKCTL_SCLKEN;
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_CLKCTL);

	tmp &= ~UNIPHIER_SD_CLKCTL_DIV_MASK;
	tmp |= val | UNIPHIER_SD_CLKCTL_OFFEN;
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_CLKCTL);

	tmp |= UNIPHIER_SD_CLKCTL_SCLKEN;
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_CLKCTL);

	udelay(1000);
}

static void uniphier_sd_set_pins(struct udevice *dev)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);

	if (priv->vqmmc_dev) {
		if (mmc->signal_voltage == MMC_SIGNAL_VOLTAGE_180)
			regulator_set_value(priv->vqmmc_dev, 1800000);
		else
			regulator_set_value(priv->vqmmc_dev, 3300000);
		regulator_set_enable(priv->vqmmc_dev, true);
	}

	if (mmc->signal_voltage == MMC_SIGNAL_VOLTAGE_180)
		pinctrl_select_state(dev, "state_uhs");
	else
		pinctrl_select_state(dev, "default");
}

static int uniphier_sd_set_ios(struct udevice *dev)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	int ret;

	dev_dbg(dev, "clock %uHz, DDRmode %d, width %u\n",
		mmc->clock, mmc->ddr_mode, mmc->bus_width);

	ret = uniphier_sd_set_bus_width(priv, mmc);
	if (ret)
		return ret;
	uniphier_sd_set_ddr_mode(priv, mmc);
	uniphier_sd_set_clk_rate(priv, mmc);
	uniphier_sd_set_pins(dev);

#if CONFIG_IS_ENABLED(MMC_HS200_SUPPORT)
	if (priv->caps & UNIPHIER_SD_CAP_RCAR_UHS)
		uniphier_sd_reset_tuning(priv);
#endif

	return 0;
}

static int uniphier_sd_get_cd(struct udevice *dev)
{
	struct uniphier_sd_priv *priv = dev_get_priv(dev);

	if (priv->caps & UNIPHIER_SD_CAP_NONREMOVABLE)
		return 1;

	return !!(uniphier_sd_readl(priv, UNIPHIER_SD_INFO1) &
		  UNIPHIER_SD_INFO1_CD);
}

/*
 * Renesas RCar SDR104 / HS200
 */
#if CONFIG_IS_ENABLED(MMC_HS200_SUPPORT)

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

static void uniphier_sd_reset_tuning(struct uniphier_sd_priv *priv)
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

static int uniphier_sd_execute_tuning(struct udevice *dev, uint opcode)
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
		uniphier_sd_reset_tuning(priv);
	}

	return ret;
}
#endif

static const struct dm_mmc_ops uniphier_sd_ops = {
	.send_cmd = uniphier_sd_send_cmd,
	.set_ios = uniphier_sd_set_ios,
	.get_cd = uniphier_sd_get_cd,
#if CONFIG_IS_ENABLED(MMC_HS200_SUPPORT)
	.execute_tuning = uniphier_sd_execute_tuning,
#endif
};

static void uniphier_sd_host_init(struct uniphier_sd_priv *priv)
{
	u32 tmp;

	/* soft reset of the host */
	tmp = uniphier_sd_readl(priv, UNIPHIER_SD_SOFT_RST);
	tmp &= ~UNIPHIER_SD_SOFT_RST_RSTX;
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_SOFT_RST);
	tmp |= UNIPHIER_SD_SOFT_RST_RSTX;
	uniphier_sd_writel(priv, tmp, UNIPHIER_SD_SOFT_RST);

	/* FIXME: implement eMMC hw_reset */

	uniphier_sd_writel(priv, UNIPHIER_SD_STOP_SEC, UNIPHIER_SD_STOP);

	/*
	 * Connected to 32bit AXI.
	 * This register dropped backward compatibility at version 0x10.
	 * Write an appropriate value depending on the IP version.
	 */
	uniphier_sd_writel(priv, priv->version >= 0x10 ? 0x00000101 : 0x00000000,
			   UNIPHIER_SD_HOST_MODE);

	if (priv->caps & UNIPHIER_SD_CAP_DMA_INTERNAL) {
		tmp = uniphier_sd_readl(priv, UNIPHIER_SD_DMA_MODE);
		tmp |= UNIPHIER_SD_DMA_MODE_ADDR_INC;
		uniphier_sd_writel(priv, tmp, UNIPHIER_SD_DMA_MODE);
	}
}

static int uniphier_sd_bind(struct udevice *dev)
{
	struct uniphier_sd_plat *plat = dev_get_platdata(dev);

	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static int uniphier_sd_probe(struct udevice *dev)
{
	struct uniphier_sd_plat *plat = dev_get_platdata(dev);
	struct uniphier_sd_priv *priv = dev_get_priv(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	const u32 quirks = dev_get_driver_data(dev);
	fdt_addr_t base;
	struct clk clk;
	int ret;

	base = devfdt_get_addr(dev);
	if (base == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->regbase = devm_ioremap(dev, base, SZ_2K);
	if (!priv->regbase)
		return -ENOMEM;

#ifdef CONFIG_DM_REGULATOR
	ret = device_get_supply_regulator(dev, "vqmmc-supply", &priv->vqmmc_dev);
#endif

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret < 0) {
		dev_err(dev, "failed to get host clock\n");
		return ret;
	}

	/* set to max rate */
	priv->mclk = clk_set_rate(&clk, ULONG_MAX);
	if (IS_ERR_VALUE(priv->mclk)) {
		dev_err(dev, "failed to set rate for host clock\n");
		clk_free(&clk);
		return priv->mclk;
	}

	ret = clk_enable(&clk);
	clk_free(&clk);
	if (ret) {
		dev_err(dev, "failed to enable host clock\n");
		return ret;
	}

	ret = mmc_of_parse(dev, &plat->cfg);
	if (ret < 0) {
		dev_err(dev, "failed to parse host caps\n");
		return ret;
	}

	plat->cfg.name = dev->name;
	plat->cfg.host_caps |= MMC_MODE_HS_52MHz | MMC_MODE_HS;

	if (quirks)
		priv->caps = quirks;

	priv->version = uniphier_sd_readl(priv, UNIPHIER_SD_VERSION) &
						UNIPHIER_SD_VERSION_IP;
	dev_dbg(dev, "version %x\n", priv->version);
	if (priv->version >= 0x10) {
		priv->caps |= UNIPHIER_SD_CAP_DMA_INTERNAL;
		priv->caps |= UNIPHIER_SD_CAP_DIV1024;
	}

	if (fdt_get_property(gd->fdt_blob, dev_of_offset(dev), "non-removable",
			     NULL))
		priv->caps |= UNIPHIER_SD_CAP_NONREMOVABLE;

	uniphier_sd_host_init(priv);

	plat->cfg.voltages = MMC_VDD_165_195 | MMC_VDD_32_33 | MMC_VDD_33_34;
	plat->cfg.f_min = priv->mclk /
			(priv->caps & UNIPHIER_SD_CAP_DIV1024 ? 1024 : 512);
	plat->cfg.f_max = priv->mclk;
	plat->cfg.b_max = U32_MAX; /* max value of UNIPHIER_SD_SECCNT */

	upriv->mmc = &plat->mmc;

#if CONFIG_IS_ENABLED(MMC_HS200_SUPPORT)
	if (priv->caps & UNIPHIER_SD_CAP_RCAR_UHS)
		uniphier_sd_reset_tuning(priv);
#endif

	return 0;
}

#define RENESAS_SD_QUIRKS					\
	UNIPHIER_SD_CAP_64BIT | UNIPHIER_SD_CAP_RCAR | UNIPHIER_SD_CAP_RCAR_UHS

static const struct udevice_id uniphier_sd_match[] = {
	{ .compatible = "renesas,sdhi-r8a7795", .data = RENESAS_SD_QUIRKS },
	{ .compatible = "renesas,sdhi-r8a7796", .data = RENESAS_SD_QUIRKS },
	{ .compatible = "renesas,sdhi-r8a77970", .data = RENESAS_SD_QUIRKS },
	{ .compatible = "renesas,sdhi-r8a77995", .data = RENESAS_SD_QUIRKS },
	{ .compatible = "socionext,uniphier-sdhc", .data = 0 },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(uniphier_mmc) = {
	.name = "uniphier-mmc",
	.id = UCLASS_MMC,
	.of_match = uniphier_sd_match,
	.bind = uniphier_sd_bind,
	.probe = uniphier_sd_probe,
	.priv_auto_alloc_size = sizeof(struct uniphier_sd_priv),
	.platdata_auto_alloc_size = sizeof(struct uniphier_sd_plat),
	.ops = &uniphier_sd_ops,
};
