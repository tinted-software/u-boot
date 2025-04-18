/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Aaron <leafy.myeh@allwinnertech.com>
 *
 * MMC register definition for allwinner sunxi platform.
 */

#ifndef _SUNXI_MMC_H
#define _SUNXI_MMC_H

#include <linux/types.h>

struct sunxi_mmc {
	u32 gctrl;		/* 0x00 global control */
	u32 clkcr;		/* 0x04 clock control */
	u32 timeout;		/* 0x08 time out */
	u32 width;		/* 0x0c bus width */
	u32 blksz;		/* 0x10 block size */
	u32 bytecnt;		/* 0x14 byte count */
	u32 cmd;		/* 0x18 command */
	u32 arg;		/* 0x1c argument */
	u32 resp0;		/* 0x20 response 0 */
	u32 resp1;		/* 0x24 response 1 */
	u32 resp2;		/* 0x28 response 2 */
	u32 resp3;		/* 0x2c response 3 */
	u32 imask;		/* 0x30 interrupt mask */
	u32 mint;		/* 0x34 masked interrupt status */
	u32 rint;		/* 0x38 raw interrupt status */
	u32 status;		/* 0x3c status */
	u32 ftrglevel;		/* 0x40 FIFO threshold watermark*/
	u32 funcsel;		/* 0x44 function select */
	u32 cbcr;		/* 0x48 CIU byte count */
	u32 bbcr;		/* 0x4c BIU byte count */
	u32 dbgc;		/* 0x50 debug enable */
	u32 res0;		/* 0x54 reserved */
	u32 a12a;		/* 0x58 Auto command 12 argument */
	u32 ntsr;		/* 0x5c	New timing set register */
	u32 res1[6];
	u32 hwrst;		/* 0x78 Hardware Reset */
	u32 res5;
	u32 dmac;		/* 0x80 internal DMA control */
	u32 dlba;		/* 0x84 internal DMA descr list base address */
	u32 idst;		/* 0x88 internal DMA status */
	u32 idie;		/* 0x8c internal DMA interrupt enable */
	u32 chda;		/* 0x90 */
	u32 cbda;		/* 0x94 */
	u32 res2[26];
#if defined(CONFIG_SUNXI_GEN_SUN6I) || defined(CONFIG_SUN50I_GEN_H6) || defined(CONFIG_SUNXI_GEN_NCAT2)
	u32 thldc;		/* 0x100 Threshold control */
	u32 res3[16];
	u32 samp_dl;
	u32 res4[46];
#endif
	u32 fifo;		/* 0x100 / 0x200 FIFO access address */
};

#define SUNXI_MMC_CLK_POWERSAVE		(0x1 << 17)
#define SUNXI_MMC_CLK_ENABLE		(0x1 << 16)
#define SUNXI_MMC_CLK_DIVIDER_MASK	(0xff)

#define SUNXI_MMC_GCTRL		0x000
#define SUNXI_MMC_GCTRL_SOFT_RESET	(0x1 << 0)
#define SUNXI_MMC_GCTRL_FIFO_RESET	(0x1 << 1)
#define SUNXI_MMC_GCTRL_DMA_RESET	(0x1 << 2)
#define SUNXI_MMC_GCTRL_RESET		(SUNXI_MMC_GCTRL_SOFT_RESET|\
					 SUNXI_MMC_GCTRL_FIFO_RESET|\
					 SUNXI_MMC_GCTRL_DMA_RESET)
#define SUNXI_MMC_GCTRL_DMA_ENABLE	(0x1 << 5)
#define SUNXI_MMC_GCTRL_ACCESS_BY_AHB   (0x1 << 31)

#define SUNXI_MMC_CMD_RESP_EXPIRE	(0x1 << 6)
#define SUNXI_MMC_CMD_LONG_RESPONSE	(0x1 << 7)
#define SUNXI_MMC_CMD_CHK_RESPONSE_CRC	(0x1 << 8)
#define SUNXI_MMC_CMD_DATA_EXPIRE	(0x1 << 9)
#define SUNXI_MMC_CMD_WRITE		(0x1 << 10)
#define SUNXI_MMC_CMD_AUTO_STOP		(0x1 << 12)
#define SUNXI_MMC_CMD_WAIT_PRE_OVER	(0x1 << 13)
#define SUNXI_MMC_CMD_SEND_INIT_SEQ	(0x1 << 15)
#define SUNXI_MMC_CMD_UPCLK_ONLY	(0x1 << 21)
#define SUNXI_MMC_CMD_START		(0x1 << 31)

#define SUNXI_MMC_RINT_RESP_ERROR		(0x1 << 1)
#define SUNXI_MMC_RINT_COMMAND_DONE		(0x1 << 2)
#define SUNXI_MMC_RINT_DATA_OVER		(0x1 << 3)
#define SUNXI_MMC_RINT_TX_DATA_REQUEST		(0x1 << 4)
#define SUNXI_MMC_RINT_RX_DATA_REQUEST		(0x1 << 5)
#define SUNXI_MMC_RINT_RESP_CRC_ERROR		(0x1 << 6)
#define SUNXI_MMC_RINT_DATA_CRC_ERROR		(0x1 << 7)
#define SUNXI_MMC_RINT_RESP_TIMEOUT		(0x1 << 8)
#define SUNXI_MMC_RINT_DATA_TIMEOUT		(0x1 << 9)
#define SUNXI_MMC_RINT_VOLTAGE_CHANGE_DONE	(0x1 << 10)
#define SUNXI_MMC_RINT_FIFO_RUN_ERROR		(0x1 << 11)
#define SUNXI_MMC_RINT_HARD_WARE_LOCKED		(0x1 << 12)
#define SUNXI_MMC_RINT_START_BIT_ERROR		(0x1 << 13)
#define SUNXI_MMC_RINT_AUTO_COMMAND_DONE	(0x1 << 14)
#define SUNXI_MMC_RINT_END_BIT_ERROR		(0x1 << 15)
#define SUNXI_MMC_RINT_SDIO_INTERRUPT		(0x1 << 16)
#define SUNXI_MMC_RINT_CARD_INSERT		(0x1 << 30)
#define SUNXI_MMC_RINT_CARD_REMOVE		(0x1 << 31)
#define SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT      \
	(SUNXI_MMC_RINT_RESP_ERROR |		\
	 SUNXI_MMC_RINT_RESP_CRC_ERROR |	\
	 SUNXI_MMC_RINT_DATA_CRC_ERROR |	\
	 SUNXI_MMC_RINT_RESP_TIMEOUT |		\
	 SUNXI_MMC_RINT_DATA_TIMEOUT |		\
	 SUNXI_MMC_RINT_VOLTAGE_CHANGE_DONE |	\
	 SUNXI_MMC_RINT_FIFO_RUN_ERROR |	\
	 SUNXI_MMC_RINT_HARD_WARE_LOCKED |	\
	 SUNXI_MMC_RINT_START_BIT_ERROR |	\
	 SUNXI_MMC_RINT_END_BIT_ERROR) /* 0xbfc2 */
#define SUNXI_MMC_RINT_INTERRUPT_DONE_BIT	\
	(SUNXI_MMC_RINT_AUTO_COMMAND_DONE |	\
	 SUNXI_MMC_RINT_DATA_OVER |		\
	 SUNXI_MMC_RINT_COMMAND_DONE |		\
	 SUNXI_MMC_RINT_VOLTAGE_CHANGE_DONE)

#define SUNXI_MMC_STATUS_RXWL_FLAG		(0x1 << 0)
#define SUNXI_MMC_STATUS_TXWL_FLAG		(0x1 << 1)
#define SUNXI_MMC_STATUS_FIFO_EMPTY		(0x1 << 2)
#define SUNXI_MMC_STATUS_FIFO_FULL		(0x1 << 3)
#define SUNXI_MMC_STATUS_CARD_PRESENT		(0x1 << 8)
#define SUNXI_MMC_STATUS_CARD_DATA_BUSY		(0x1 << 9)
#define SUNXI_MMC_STATUS_DATA_FSM_BUSY		(0x1 << 10)
#define SUNXI_MMC_STATUS_FIFO_LEVEL(reg)	(((reg) >> 17) & 0x3fff)

#define SUNXI_MMC_NTSR_MODE_SEL_NEW		(0x1 << 31)

#define SUNXI_MMC_HWRST		0x078
#define SUNXI_MMC_HWRST_ASSERT		(0x0 << 0)
#define SUNXI_MMC_HWRST_DEASSERT	(0x1 << 0)

#define SUNXI_MMC_IDMAC_RESET		(0x1 << 0)
#define SUNXI_MMC_IDMAC_FIXBURST	(0x1 << 1)
#define SUNXI_MMC_IDMAC_ENABLE		(0x1 << 7)

#define SUNXI_MMC_IDIE_TXIRQ		(0x1 << 0)
#define SUNXI_MMC_IDIE_RXIRQ		(0x1 << 1)

#define SUNXI_MMC_COMMON_CLK_GATE		(1 << 16)
#define SUNXI_MMC_COMMON_RESET			(1 << 18)

#define SUNXI_MMC_THLDC		0x100
#define SUNXI_MMC_THLDC_READ_EN		(0x1 << 0)
#define SUNXI_MMC_THLDC_BSY_CLR_INT_EN	(0x1 << 1)
#define SUNXI_MMC_THLDC_WRITE_EN	(0x1 << 2)
#define SUNXI_MMC_THLDC_READ_THLD(x)	(((x) & 0xfff) << 16)

#define SUNXI_MMC_CAL_DL_SW_EN		(0x1 << 7)

#endif /* _SUNXI_MMC_H */
