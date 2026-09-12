// SPDX-License-Identifier: GPL-2.0+
/*
 * Google Pixel 7a (Lynx - Tensor GS201) Board Support
 *
 * Copyright (c) 2024-2026 Google LLC
 */

#include <asm/armv8/mmu.h>
#include <asm/io.h>
#include <cpu_func.h>
#include <blk.h>
#include <bootflow.h>
#include <ctype.h>
#include <dm/ofnode.h>
#include <dm/device.h>
#include <dm/uclass.h>
#include <efi.h>
#include <efi_loader.h>
#include <env.h>
#include <errno.h>
#include <init.h>
#include <linux/sizes.h>
#include <lmb.h>
#include <part.h>
#include <stdbool.h>
#include <string.h>

DECLARE_GLOBAL_DATA_PTR;

#define lmb_alloc(size, addr) \
	lmb_alloc_mem(LMB_MEM_ALLOC_ANY, SZ_2M, addr, size, LMB_NONE)

/*
 * Memory mapping for GS201 (Tensor G2):
 * - Peripheral MMIO block 1: 0x10000000 - 0x20000000 (UART, GIC, USI, Mailbox, PMU)
 * - Peripheral MMIO block 2: 0x20000000 - 0x30000000 (BTS, S2MPU)
 * - DRAM banks: dynamically populated from FDT
 */
static struct mm_region lynx_mem_map[CONFIG_NR_DRAM_BANKS + 3] = {
	{
		/* Peripheral MMIO block 1 */
		.virt = 0x10000000UL,
		.phys = 0x10000000UL,
		.size = 0x10000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE | PTE_BLOCK_PXN | PTE_BLOCK_UXN,
	},
	{
		/* Peripheral MMIO block 2 */
		.virt = 0x20000000UL,
		.phys = 0x20000000UL,
		.size = 0x10000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE | PTE_BLOCK_PXN | PTE_BLOCK_UXN,
	},
};

struct mm_region *mem_map = lynx_mem_map;

static const char *lynx_prev_bl_get_bootargs(void)
{
	void *prev_bl_fdt_base = (void *)get_prev_bl_fdt_addr();
	int chosen_node_offset, ret;
	const struct fdt_property *bootargs_prop;

	if (!prev_bl_fdt_base)
		return NULL;

	ret = fdt_check_header(prev_bl_fdt_base);
	if (ret < 0)
		return NULL;

	ret = fdt_path_offset(prev_bl_fdt_base, "/chosen");
	chosen_node_offset = ret;
	if (ret < 0)
		return NULL;

	bootargs_prop = fdt_get_property(prev_bl_fdt_base, chosen_node_offset,
					 "bootargs", &ret);
	if (!bootargs_prop)
		return NULL;

	return bootargs_prop->data;
}

static void lynx_parse_dram_banks(const void *fdt_base)
{
	u64 mem_addr, mem_size = 0;
	u32 na, ns, i;
	int index = 2;
	int offset;

	if (!fdt_base || fdt_check_header(fdt_base) < 0) {
		/* Fallback default memory mapping if FDT not parsed yet */
		lynx_mem_map[2].phys = 0x80000000UL;
		lynx_mem_map[2].virt = 0x80000000UL;
		lynx_mem_map[2].size = 0x200000000ULL; /* 8GB */
		lynx_mem_map[2].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
				       PTE_BLOCK_INNER_SHARE;
		return;
	}

	na = fdt_address_cells(fdt_base, 0);
	ns = fdt_size_cells(fdt_base, 0);

	fdt_for_each_subnode(offset, fdt_base, 0) {
		if (strncmp(fdt_get_name(fdt_base, offset, NULL), "memory", 6))
			continue;

		for (i = 0; ; i++) {
			if (index >= CONFIG_NR_DRAM_BANKS + 2)
				break;

			mem_addr = fdtdec_get_addr_size_fixed(fdt_base, offset,
							      "reg", i, na, ns,
							      &mem_size, false);
			if (mem_addr == FDT_ADDR_T_NONE)
				break;

			if (!mem_size)
				continue;

			lynx_mem_map[index].phys = mem_addr;
			lynx_mem_map[index].virt = mem_addr;
			lynx_mem_map[index].size = mem_size;
			lynx_mem_map[index].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
						   PTE_BLOCK_INNER_SHARE;
			index++;
		}
	}
}

static void lynx_env_setup(void)
{
	const char *bootargs = lynx_prev_bl_get_bootargs();
	char buf[128];
	int offset, ret;

	env_set("platform", "gs201");
	env_set("board", "google-lynx");
	env_set("fdtfile", "google/gs201-lynx.dtb");

	if (bootargs) {
		/* Parse serial number */
		ret = cmdline_get_arg(bootargs, "androidboot.serialno", &offset);
		if (ret > 0) {
			strlcpy(buf, bootargs + offset, ret + 1);
			env_set("serial#", buf);
		}

		/* Parse slot suffix */
		ret = cmdline_get_arg(bootargs, "androidboot.slot_suffix", &offset);
		if (ret > 0) {
			strlcpy(buf, bootargs + offset, ret + 1);
			env_set("slot_suffix", buf);
		}
	}
}

int board_fdt_blob_setup(void **fdtp)
{
	void *prev_bl = (void *)get_prev_bl_fdt_addr();

	if (prev_bl && fdt_check_header(prev_bl) == 0) {
		*fdtp = prev_bl;
		return 0;
	}

	if (*fdtp && fdt_check_header(*fdtp) == 0)
		return 0;

	return -EEXIST;
}
static void gs201_phase_square(unsigned int x0, unsigned int y0);

int timer_init(void)
{
	/* ABL does not reliably expose a usable FDT at this stage. */
	gd->arch.timer_rate_hz = 24576000;
	gs201_phase_square(560, 300);

	return 0;
}

/* GS201 cluster watchdog kick registers armed by ABL. */
#define GS201_WDT_CLUSTER0	0x10060000UL
#define GS201_WDT_CLUSTER1	0x10070000UL


/*
 * Early diagnostic path: this runs with the MMU and caches off, just like
 * the known-working Linux gs201_fb_poke().  It deliberately bypasses DM and
 * vidconsole so a visible square proves that the ABL display handoff works.
 */
#define GS201_DPU_L0		0x1c0b0000UL
#define GS201_DECON0		0x1c240000UL
#define GS201_LINEAR_FB	0x86000000UL
#define GS201_RDMA_ENABLE	0x0000
#define GS201_RDMA_CTRL0	0x0008
#define GS201_RDMA_SRC_SIZE	0x0010
#define GS201_RDMA_IMG_SIZE	0x0018
#define GS201_RDMA_BASE	0x0040
#define GS201_RDMA_STRIDE0	0x0050
#define GS201_RDMA_STRIDE1	0x0054
#define GS201_SHADOW		0x0400
#define GS201_CH_STRIDE	0x1000
#define GS201_DECON_GLOBAL	0x0020
#define GS201_DECON_TRIG	0x0030
#define GS201_DECON_TRIG_SEC	0x003c
#define GS201_DECON_SHD_REQ	0x0050

static inline u32 gs201_early_read(ulong addr)
{
	return *(volatile u32 *)addr;
}

static inline void gs201_early_write(ulong addr, u32 val)
{
	*(volatile u32 *)addr = val;
}


static void gs201_early_square(unsigned int x0)
{
	unsigned int x, y;
	u32 t;

	for (y = 80; y < 240; y++)
		for (x = x0; x < x0 + 160; x++)
			((volatile u32 *)GS201_LINEAR_FB)[y * 1080 + x] = 0xffffff00;

	gs201_early_write(GS201_DECON0 + GS201_DECON_SHD_REQ, BIT(31) | 0x3f);
	t = gs201_early_read(GS201_DECON0 + GS201_DECON_TRIG);
	gs201_early_write(GS201_DECON0 + GS201_DECON_TRIG,
			  (t & ~BIT(4)) | BIT(8) | BIT(1) | BIT(0));
}

static void gs201_phase_square(unsigned int x0, unsigned int y0)
{
	unsigned int x, y;
	u32 t;

	for (y = y0; y < y0 + 120; y++)
		for (x = x0; x < x0 + 120; x++)
			((volatile u32 *)GS201_LINEAR_FB)[y * 1080 + x] = 0xffffff00;

	flush_dcache_range(GS201_LINEAR_FB + y0 * 1080 * 4 + x0 * 4,
			   GS201_LINEAR_FB + (y0 + 120) * 1080 * 4 + (x0 + 120) * 4);
	gs201_early_write(GS201_DECON0 + GS201_DECON_SHD_REQ, BIT(31) | 0x3f);
	t = gs201_early_read(GS201_DECON0 + GS201_DECON_TRIG);
	gs201_early_write(GS201_DECON0 + GS201_DECON_TRIG,
			  (t & ~BIT(4)) | BIT(8) | BIT(1) | BIT(0));
}
void lowlevel_init(void)
{
	/* ABL's watchdog must be stopped before the framebuffer fill below. */
	gs201_early_write(GS201_WDT_CLUSTER0, 0);
	gs201_early_write(GS201_WDT_CLUSTER1, 0);

	ulong dma = GS201_DPU_L0;
	u32 img, src, width, height, ctrl, stride0, en, g, t;
	unsigned int ch, x, y;

	/* Find the ABL-owned RDMA plane. */
	for (ch = 0; ch < 6; ch++) {
		ulong candidate = GS201_DPU_L0 + ch * GS201_CH_STRIDE;

		img = gs201_early_read(candidate + GS201_RDMA_IMG_SIZE);
		src = gs201_early_read(candidate + GS201_RDMA_SRC_SIZE);
		width = img & 0x3fff;
		height = (img >> 16) & 0x3fff;
		if (!width || !height) {
			width = src & 0xffff;
			height = (src >> 16) & 0xffff;
		}
		if (width >= 16 && width <= 4096 && height >= 16 && height <= 4096 &&
		    gs201_early_read(candidate + GS201_RDMA_BASE)) {
			dma = candidate;
			break;
		}
	}
	if (ch == 6) {
		width = 1080;
		height = 2400;
	}

	/* Black background with a white 160x160 diagnostic square at (80,80). */
	for (y = 0; y < height; y++)
 		for (x = 0; x < width; x++)
 			((volatile u32 *)GS201_LINEAR_FB)[y * width + x] =
 				(x >= 80 && x < 240 && y >= 80 && y < 240) ?
 				0xffffff00 : 0;

	ctrl = gs201_early_read(dma + GS201_RDMA_CTRL0);
	ctrl &= ~((1 << 0) | (1 << 1) | (1 << 2) | (0x3f << 8));
	ctrl |= 7 << 8; /* XRGB8888: bytes X,R,G,B */
	gs201_early_write(dma + GS201_RDMA_CTRL0, ctrl);
	gs201_early_write(dma + GS201_RDMA_CTRL0 + GS201_SHADOW, ctrl);
	gs201_early_write(dma + GS201_RDMA_BASE, GS201_LINEAR_FB);
	gs201_early_write(dma + GS201_RDMA_BASE + GS201_SHADOW, GS201_LINEAR_FB);
	stride0 = gs201_early_read(dma + GS201_RDMA_STRIDE0) | BIT(20);
	gs201_early_write(dma + GS201_RDMA_STRIDE0, stride0);
	gs201_early_write(dma + GS201_RDMA_STRIDE0 + GS201_SHADOW, stride0);
	gs201_early_write(dma + GS201_RDMA_STRIDE1, width * 4);
	gs201_early_write(dma + GS201_RDMA_STRIDE1 + GS201_SHADOW, width * 4);
	img = (height << 16) | width;
	gs201_early_write(dma + GS201_RDMA_SRC_SIZE, img);
	gs201_early_write(dma + GS201_RDMA_SRC_SIZE + GS201_SHADOW, img);
	gs201_early_write(dma + GS201_RDMA_IMG_SIZE, img);
	gs201_early_write(dma + GS201_RDMA_IMG_SIZE + GS201_SHADOW, img);
	en = gs201_early_read(dma + GS201_RDMA_ENABLE) | BIT(4);
	gs201_early_write(dma + GS201_RDMA_ENABLE, en);
	gs201_early_write(GS201_DECON0 + GS201_DECON_SHD_REQ, BIT(31) | 0x3f);
	g = gs201_early_read(GS201_DECON0 + GS201_DECON_GLOBAL);
	gs201_early_write(GS201_DECON0 + GS201_DECON_GLOBAL, g | BIT(1) | BIT(0));
	t = gs201_early_read(GS201_DECON0 + GS201_DECON_TRIG_SEC);
	gs201_early_write(GS201_DECON0 + GS201_DECON_TRIG_SEC, t & ~BIT(4));
	t = gs201_early_read(GS201_DECON0 + GS201_DECON_TRIG);
	gs201_early_write(GS201_DECON0 + GS201_DECON_TRIG, (t & ~BIT(4)) | BIT(8) | BIT(1) | BIT(0));
}
int board_early_init_f(void)
{
	/* The watchdog was stopped in lowlevel_init(); avoid touching ABL's FDT. */
	gs201_early_square(260);

	lynx_mem_map[2].phys = 0x80000000UL;
	gs201_early_square(440);
	lynx_mem_map[2].virt = 0x80000000UL;
	gs201_early_square(620);
	lynx_mem_map[2].size = 0x200000000ULL;
	gs201_early_square(800);
	lynx_mem_map[2].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			       PTE_BLOCK_INNER_SHARE;
	gs201_early_square(900);

	return 0;
}

int board_early_init_r(void)
{
	/* Post-relocation checkpoint before stdio/console initialization. */
	gs201_phase_square(360, 460);
	return 0;
}

/*
 * Relocation occurs before U-Boot replaces ABL's translation tables.  Keep
 * the monitor below 4 GiB, which ABL identity-maps; all 8 GiB remain
 * available to U-Boot and the OS after U-Boot installs its own mappings.
 */
phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
{
	(void)total_size;
	return 0x100000000ULL;
}

int dram_init(void)
{
	unsigned int i;

	gd->ram_base = 0x80000000UL;
	gd->ram_size = 0x80000000UL; /* 2GB initial min */

	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
		if (gd->ram_size < lynx_mem_map[i + 2].size) {
			gd->ram_base = lynx_mem_map[i + 2].phys;
			gd->ram_size = lynx_mem_map[i + 2].size;
		}
	}

	/* Checkpoint 3: DRAM initialisation completed. */
	gs201_phase_square(80, 300);

	return 0;
}

int dram_init_banksize(void)
{
	unsigned int i;

	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
		gd->dram[i].start = lynx_mem_map[i + 2].phys;
		gd->dram[i].size = lynx_mem_map[i + 2].size;
	}


	/* Checkpoint: DRAM banks were published for relocation. */
	gs201_phase_square(80, 460);
	return 0;
}

int board_init(void)
{
	/* Checkpoint 4: board_init is reached. */
	gs201_phase_square(240, 300);
	return 0;
}

int misc_init_r(void)
{
	lynx_env_setup();
	struct udevice *video;

	/* Probe DECON directly: ABL's FDT has no stable video alias/sequence. */
	uclass_get_device_by_driver(UCLASS_VIDEO, DM_DRIVER_GET(gs201_decon),
				     &video);


	/* Checkpoint 5: post-relocation misc init is reached. */
	gs201_phase_square(400, 300);

	/*
	 * The GS201 video driver scans out of the fixed ABL splash buffer
	 * at 0x86000000 (1080x2400x4). Keep bootstd/LMB from allocating it.
	 */
	phys_addr_t fb = 0x86000000UL;

	lmb_alloc_mem(LMB_MEM_ALLOC_ADDR, 0, &fb, 1080 * 4 * 2400,
		     LMB_NONE);

	return 0;
}

int print_cpuinfo(void)
{
	printf("CPU:   Google Tensor G2 (GS201)\n");
	return 0;
}

unsigned long get_uart_clk(int dev_index)
{
	gs201_phase_square(700, 300);
	return 24576000;
}

