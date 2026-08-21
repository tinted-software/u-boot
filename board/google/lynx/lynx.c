// SPDX-License-Identifier: GPL-2.0+
/*
 * Google Pixel 7a (Lynx - Tensor GS201) Board Support
 *
 * Copyright (c) 2024-2026 Google LLC
 */

#include <asm/armv8/mmu.h>
#include <blk.h>
#include <bootflow.h>
#include <ctype.h>
#include <dm/ofnode.h>
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

	/* If bootloader provided FDT, prefer it */
	if (prev_bl && fdt_check_header(prev_bl) == 0) {
		*fdtp = prev_bl;
		return 0;
	}

	/* Otherwise use embedded FDT */
	if (*fdtp && fdt_check_header(*fdtp) == 0)
		return 0;

	return -EEXIST;
}

int timer_init(void)
{
	ofnode timer_node;

	timer_node = ofnode_by_compatible(ofnode_null(), "arm,armv8-timer");
	gd->arch.timer_rate_hz = ofnode_read_u32_default(timer_node,
							 "clock-frequency",
							 24576000);

	return 0;
}

int board_early_init_f(void)
{
	void *fdt = (void *)gd->fdt_blob;

	if (!fdt || fdt_check_header(fdt) < 0)
		fdt = (void *)get_prev_bl_fdt_addr();

	lynx_parse_dram_banks(fdt);

	return 0;
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

	return 0;
}

int dram_init_banksize(void)
{
	unsigned int i;

	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
		gd->bd->bi_dram[i].start = lynx_mem_map[i + 2].phys;
		gd->bd->bi_dram[i].size = lynx_mem_map[i + 2].size;
	}

	return 0;
}

int board_init(void)
{
	return 0;
}

int misc_init_r(void)
{
	lynx_env_setup();
	return 0;
}

int print_cpuinfo(void)
{
	printf("CPU:   Google Tensor G2 (GS201)\n");
	return 0;
}

unsigned long get_uart_clk(int dev_index)
{
	return 24576000;
}

