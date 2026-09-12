// SPDX-License-Identifier: GPL-2.0+
/*
 * Google Tensor GS201 (Pixel 7a) DECON/DPU framebuffer driver for U-Boot
 *
 * The ABL bootloader leaves the whole display pipeline (DPP/DECON/MIPI-DSIM
 * and the panel) up and running, with the boot splash scanned out of the
 * RDMA plane.  This driver adopts that plane, forces a linear BGRX8888
 * scanout from a fixed scratch buffer in DRAM, and exposes it as a
 * driver-model video device so that the U-Boot console and boot menu are
 * rendered on the OLED panel.  The DSIM/panel is deliberately never touched:
 * as long as ABL brought it up, it keeps scanning.
 *
 * Ported from the Linux exynos_gs201_decon.c driver.
 */

#include <cpu_func.h>
#include <dm.h>
#include <log.h>
#include <video.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

#define GS201_DPU_DMA_PHYS	0x1C0B0000UL
#define GS201_DECON0_PHYS	0x1C240000UL
#define GS201_DECON0_SIZE	0x6000
#define GS201_DPU_DMA_SIZE	0x6000

/* ABL splash / linear scratch DRAM buffer. */
#define LINEAR_FB		0x86000000UL

#define DMA_SHD_OFFSET		0x0400
#define CHANNEL_STRIDE		0x1000
#define CHANNELS		6

#define RDMA_ENABLE		0x0000
#define IDMA_SFR_UPDATE_FORCE	BIT(4)
#define RDMA_IN_CTRL_0		0x0008
#define RDMA_SRC_SIZE		0x0010
#define RDMA_IMG_SIZE		0x0018
#define RDMA_BASEADDR_Y8	0x0040
#define RDMA_SRC_STRIDE_0	0x0050
#define RDMA_SRC_STRIDE_1	0x0054
#define IDMA_STRIDE_0_SEL	BIT(20)

#define IDMA_BLOCK_EN		BIT(0)
#define IDMA_AFBC_EN		BIT(1)
#define IDMA_SBWC_EN		BIT(2)
#define IDMA_IMG_FORMAT_MASK	(0x3f << 8)
#define IDMA_IMG_FORMAT_XRGB8888	(7 << 8)

#define GLOBAL_CON		0x0020
#define TRIG_CON		0x0030
#define TRIG_CON_SECURE		0x003C
#define SHD_REG_UP_REQ		0x0050
#define GLOBAL_CON_DECON_EN	BIT(1)
#define GLOBAL_CON_DECON_EN_F	BIT(0)
#define SW_TRIG_EN		BIT(8)
#define SW_TRIG_DET_EN		BIT(1)
#define HW_TRIG_EN		BIT(0)
#define HW_TRIG_MASK_DECON	BIT(4)
#define SHD_REG_UP_REQ_GLOBAL	BIT(31)
#define SHD_REG_UP_REQ_FOR_DECON	0x3f

struct gs201_decon_priv {
	void __iomem *dma;
	void __iomem *decon;
	unsigned int dma_off;
	bool marker_drawn;
};

/* Write a register together with its shadow (double-buffered) copy. */
static void gs201_wr32(void __iomem *base, unsigned int off, u32 val)
{
	writel(val, base + off);
	writel(val, base + off + DMA_SHD_OFFSET);
}

/*
 * Probe the RDMA window at @off.  Returns true when it is enabled and holds a
 * sane geometry + base address.
 */
static bool gs201_read_rdma(void __iomem *dma, unsigned int off,
			    unsigned int *width, unsigned int *height)
{
	u32 src, img;
	unsigned int w, h;

	src = readl(dma + off + RDMA_SRC_SIZE);
	img = readl(dma + off + RDMA_IMG_SIZE);
	w = img & 0x3fff;
	h = (img >> 16) & 0x3fff;
	if (!w || !h) {
		w = src & 0xffff;
		h = (src >> 16) & 0xffff;
	}
	if (w < 16 || h < 16 || w > 4096 || h > 4096)
		return false;
	if (!readl(dma + off + RDMA_BASEADDR_Y8))
		return false;

	*width = w;
	*height = h;
	return true;
}

static void gs201_force_linear(struct gs201_decon_priv *priv,
			       unsigned int width, unsigned int height)
{
	unsigned int off = priv->dma_off;
	u32 ctrl0, en, s0;
	u32 stride = width * 4;

	ctrl0 = readl(priv->dma + off + RDMA_IN_CTRL_0);
	ctrl0 &= ~(IDMA_AFBC_EN | IDMA_SBWC_EN | IDMA_BLOCK_EN |
		   IDMA_IMG_FORMAT_MASK);
	/*
	 * Match the format the Linux driver proved out: IDMA XRGB8888
	 * scans memory byte order X,R,G,B, which is VIDEO_BGRX8888 in
	 * U-Boot's pixel packing.
	 */
	ctrl0 |= IDMA_IMG_FORMAT_XRGB8888;
	gs201_wr32(priv->dma, off + RDMA_IN_CTRL_0, ctrl0);

	gs201_wr32(priv->dma, off + RDMA_BASEADDR_Y8, (u32)LINEAR_FB);

	s0 = readl(priv->dma + off + RDMA_SRC_STRIDE_0) | IDMA_STRIDE_0_SEL;
	gs201_wr32(priv->dma, off + RDMA_SRC_STRIDE_0, s0);
	gs201_wr32(priv->dma, off + RDMA_SRC_STRIDE_1, stride);

	gs201_wr32(priv->dma, off + RDMA_SRC_SIZE, (height << 16) | width);
	gs201_wr32(priv->dma, off + RDMA_IMG_SIZE, (height << 16) | width);

	en = readl(priv->dma + off + RDMA_ENABLE) | IDMA_SFR_UPDATE_FORCE;
	gs201_wr32(priv->dma, off + RDMA_ENABLE, en);
}

/* Trigger the DECON to latch the updated RDMA registers. */
static void gs201_kick(struct gs201_decon_priv *priv)
{
	u32 g, t;

	g = readl(priv->decon + GLOBAL_CON);
	if (!(g & GLOBAL_CON_DECON_EN))
		writel(g | GLOBAL_CON_DECON_EN | GLOBAL_CON_DECON_EN_F,
		       priv->decon + GLOBAL_CON);

	if (priv->dma) {
		u32 en = readl(priv->dma + priv->dma_off + RDMA_ENABLE);

		writel(en | IDMA_SFR_UPDATE_FORCE,
		       priv->dma + priv->dma_off + RDMA_ENABLE);
	}

	writel(SHD_REG_UP_REQ_GLOBAL | SHD_REG_UP_REQ_FOR_DECON,
	       priv->decon + SHD_REG_UP_REQ);

	t = readl(priv->decon + TRIG_CON_SECURE);
	writel(t & ~HW_TRIG_MASK_DECON, priv->decon + TRIG_CON_SECURE);

	t = readl(priv->decon + TRIG_CON);
	writel((t & ~HW_TRIG_MASK_DECON) | HW_TRIG_EN | SW_TRIG_EN |
	       SW_TRIG_DET_EN, priv->decon + TRIG_CON);
}

static int gs201_decon_probe(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct gs201_decon_priv *priv = dev_get_priv(dev);
	fdt_addr_t decon_addr;
	unsigned int width = 1080, height = 2400;
	unsigned int ch;
	bool found = false;

	decon_addr = dev_read_addr(dev);
	if (decon_addr == FDT_ADDR_T_NONE)
		decon_addr = GS201_DECON0_PHYS;

	/* Only decon0 is wired to the primary panel. */
	if ((ulong)decon_addr != GS201_DECON0_PHYS)
		return -ENODEV;

	priv->decon = map_physmem(decon_addr, GS201_DECON0_SIZE, MAP_NOCACHE);
	priv->dma = map_physmem(GS201_DPU_DMA_PHYS, GS201_DPU_DMA_SIZE,
				MAP_NOCACHE);
	if (!priv->decon || !priv->dma)
		return -ENOMEM;

	/* Find the RDMA window ABL left enabled. */
	for (ch = 0; ch < CHANNELS; ch++) {
		unsigned int base = ch * CHANNEL_STRIDE;
		unsigned int w, h;

		if (gs201_read_rdma(priv->dma, base, &w, &h) ||
		    gs201_read_rdma(priv->dma, base + DMA_SHD_OFFSET, &w, &h)) {
			width = w;
			height = h;
			priv->dma_off = base;
			found = true;
			break;
		}
	}
	if (!found) {
		priv->dma_off = 0;
		debug("%s: no active RDMA window, forcing L0\n", __func__);
	}

	if (width > 4096)
		width = 1080;
	if (height > 4096)
		height = 2400;

	plat->base = LINEAR_FB;
	plat->size = width * 4 * height;

	uc_priv->xsize = width;
	uc_priv->ysize = height;
	uc_priv->bpix = VIDEO_BPP32;
	uc_priv->format = VIDEO_BGRX8888;
	uc_priv->line_length = width * 4;

	gs201_force_linear(priv, width, height);
	gs201_kick(priv);

	log_info("gs201-decon: adopting ABL RDMA ch%u, %ux%u at 0x%lx\n",
		 priv->dma_off / CHANNEL_STRIDE, width, height,
		 (ulong)LINEAR_FB);

	return 0;
}

/*
 * Draw an unmistakable post-probe marker.  This is intentionally below the
 * normal vidconsole area and isolates scanout/cache failures from stdio setup.
 */
static void gs201_draw_marker(struct video_priv *uc_priv)
{
	u32 *fb = uc_priv->fb;
	unsigned int x, y;

	for (y = 80; y < 240; y++)
		for (x = 80; x < 240; x++)
			fb[y * uc_priv->xsize + x] = 0xffffff00;
}

static int gs201_decon_video_sync(struct udevice *dev)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct gs201_decon_priv *priv = dev_get_priv(dev);

	/* video_clear() runs before the vidconsole is bound; mark that point. */
	if (!priv->marker_drawn) {
		gs201_draw_marker(uc_priv);
		priv->marker_drawn = true;
	}

	/* The DPU DMA is not coherent; push the frame out before the kick. */
	if (uc_priv->fb && uc_priv->fb_size)
		flush_dcache_range((ulong)uc_priv->fb,
				   (ulong)uc_priv->fb + uc_priv->fb_size);
	gs201_kick(priv);

	return 0;
}

static const struct video_ops gs201_decon_ops = {
	.video_sync = gs201_decon_video_sync,
};

static const struct udevice_id gs201_decon_ids[] = {
	{ .compatible = "google,gs201-dpu" },
	{ .compatible = "samsung,exynos-decon" },
	{ }
};

U_BOOT_DRIVER(gs201_decon) = {
	.name		= "gs201_decon",
	.id		= UCLASS_VIDEO,
	.of_match	= gs201_decon_ids,
	.probe		= gs201_decon_probe,
	.priv_auto	= sizeof(struct gs201_decon_priv),
	.ops		= &gs201_decon_ops,
};
