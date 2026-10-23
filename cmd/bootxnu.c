#include <cpu_func.h>
#include <asm/global_data.h>
#include <env.h>
#include <command.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <video.h>
#include <stdint.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_ARM64
	#include <asm/system.h>
#endif

#ifdef CONFIG_PHYS_64BIT
#define MACH_O_MAGIC            0xfeedfacf
#define LOAD_COMMAND_SEGMENT    0x19
#else
#define MACH_O_MAGIC            0xfeedface
#define LOAD_COMMAND_SEGMENT    0x1
#endif
#define LOAD_COMMAND_UNIXTHREAD 0x5
#define LOAD_COMMAND_MAIN       0x80000028
#define MACH_O_EXEC             0x2
#define MACH_O_FILESET          0xc

struct mach_o_header {
	u32 magic;
	u32 cpu_type;
	u32 cpu_subtype;
	u32 file_type;
	u32 commands_nb;
	u32 commands_len;
	u32 flags;
#ifdef CONFIG_PHYS_64BIT
	u32 reserved;
#endif
};

struct mach_o_load_command {
	u32 command;
	u32 command_size;
};

struct mach_o_segment_command {
	struct mach_o_load_command load_command;
	char segment_name[16];
	uintptr_t dst;
	uintptr_t dst_len;
	uintptr_t src_offset;
	uintptr_t src_len;
	u32 max_protection;
	u32 initial_protection;
	u32 sections_nb;
	u32 flags;
};

struct entry_point_command {
	struct mach_o_load_command load_command;
	uint64_t entryoff;
	uint64_t stacksize;
};

struct thread_command {
	struct mach_o_load_command load_command;
	u32 flavor;
	u32 count;
	struct {
#ifdef CONFIG_ARM64
		u64 x[29];
		u64 fp;
		u64 lr;
		u64 sp;
		u64 pc;
		u32 cpsr;
		u32 flags;
#else
		u64 pc;
#endif
	} state;
};

#define XNU_CMDLINE_LEN 1024

struct xnu_boot_arguments {
	u16 revision;
	u16 version;

	u64 virt_base;
	u64 phys_base;
	u64 mem_size;
	u64 phys_end;

	struct xnu_video_information {
		u64 base_addr;
		u64 display;
		u64 bytes_per_row;
		u64 width;
		u64 height;
		u64 depth;
	} video_information;

	u32 machine_type;
	uintptr_t afdt;
	u32 afdt_length;
	char command_line[XNU_CMDLINE_LEN];
	u64 boot_flags;
	u64 mem_size_actual;
};

struct afdt_node {
	u32 properties_nb;
	u32 children_nb;
};

struct afdt_property {
	char name[32];
	u32 length;
};

/*
 * The kernel maps itself with 32M L2 blocks (16K granule) as
 * round_down(phys_base, 32M) + (VA mod 32M), so the image must be loaded
 * at a physical address congruent to its link VA modulo 32M. The VMApple
 * kernelcache links at 0xfffffe0007004000, i.e. 0x1004000 mod 32M.
 */
/*
 * With a kernelcache (MH_FILESET) the image base is the collection header,
 * which link_cache places at a 32M-aligned address with the kernel at +0x8000;
 * loading the collection at 0x40000000 puts the kernel at 0x40008000, congruent
 * to its link address 0xfffffff004008000 modulo 32M.
 */
#define XNU_LOAD_OFFSET  0x0
#define XNU_LOAD_ADDR (CONFIG_SYS_LOAD_ADDR + XNU_LOAD_OFFSET)

u32 afdt_length(void *afdt)
{
	struct afdt_node *node = afdt;
	u32 offset = sizeof(*node);
	unsigned int i;

	for (i = 0; i < node->properties_nb; ++i) {
		struct afdt_property *property = (afdt + offset);

		offset += sizeof(*property) + roundup(property->length & 0x7fffffff, 4);
	}

	for (i = 0; i < node->children_nb; ++i)
		offset += afdt_length(afdt + offset);

	return offset;
}

#ifdef CONFIG_ARCH_QEMU
/*
 * XNU does not parse the standard devicetree; it wants iBoot's "Apple
 * Flattened Device Tree" (AFDT).  For QEMU's aarch64 "virt" machine we
 * synthesize one with the handful of nodes pexpert looks at: /chosen,
 * /arm-io (with the PL011 UART and the GIC), /cpus and /pram.
 *
 * QEMU virt memory map (see hw/arm/virt.c):
 *   GICD   0x08000000   GICC 0x08010000
 *   PL011  0x09000000   PL031 0x09010000
 * Properties in /arm-io are relative to the "ranges" base (soc base),
 * which pe_arm_get_soc_base_phys() reads as ranges[1].
 */
#define QEMU_VIRT_SOC_BASE	0x08000000ULL
#define QEMU_VIRT_GICD_BASE	0x08000000ULL
#define QEMU_VIRT_UART_BASE	0x09000000ULL
#define QEMU_VIRT_RTC_BASE	0x09010000ULL
#define QEMU_VIRT_UART_PHANDLE	0x100
#define QEMU_VIRT_TIMEBASE	62500000ULL

struct afdt_writer {
	u8 *base;
	u8 *cur;
};

static void afdt_w_u32(struct afdt_writer *w, u32 v)
{
	memcpy(w->cur, &v, sizeof(v));
	w->cur += sizeof(v);
}

static void afdt_w_u64(struct afdt_writer *w, u64 v)
{
	memcpy(w->cur, &v, sizeof(v));
	w->cur += sizeof(v);
}

static void afdt_w_bytes(struct afdt_writer *w, const void *p, size_t len)
{
	memcpy(w->cur, p, len);
	w->cur += len;
}

static void afdt_w_zero(struct afdt_writer *w, size_t len)
{
	memset(w->cur, 0, len);
	w->cur += len;
}

static void afdt_w_node(struct afdt_writer *w, u32 properties, u32 children)
{
	afdt_w_u32(w, properties);
	afdt_w_u32(w, children);
}

static void afdt_w_name(struct afdt_writer *w, const char *name)
{
	char padded[32] = { 0 };

	strlcpy(padded, name, sizeof(padded));
	afdt_w_bytes(w, padded, sizeof(padded));
}

static void afdt_w_string(struct afdt_writer *w, const char *name,
			  const char *value)
{
	size_t len = strlen(value) + 1;
	size_t padded = roundup(len, 4);

	afdt_w_name(w, name);
	afdt_w_u32(w, len);
	afdt_w_bytes(w, value, len - 1);
	afdt_w_zero(w, padded - (len - 1));
}

static void afdt_w_raw(struct afdt_writer *w, const char *name,
		       const void *data, size_t len)
{
	size_t padded = roundup(len, 4);

	afdt_w_name(w, name);
	afdt_w_u32(w, padded);
	afdt_w_bytes(w, data, len);
	afdt_w_zero(w, padded - len);
}

static void afdt_w_pair(struct afdt_writer *w, const char *name, u64 a, u64 b)
{
	afdt_w_name(w, name);
	afdt_w_u32(w, 16);
	afdt_w_u64(w, a);
	afdt_w_u64(w, b);
}

static void afdt_w_u32prop(struct afdt_writer *w, const char *name, u32 v)
{
	afdt_w_name(w, name);
	afdt_w_u32(w, 4);
	afdt_w_u32(w, v);
}

static void afdt_w_u64prop(struct afdt_writer *w, const char *name, u64 v)
{
	afdt_w_name(w, name);
	afdt_w_u32(w, 8);
	afdt_w_u64(w, v);
}

/*
 * Build the AFDT at @dst.  @ram_base/@ram_size describe the full physical
 * DRAM bank (panic log included); the kernel allocator only gets the part
 * described by boot_args.mem_size.
 */
static u32 build_qemu_virt_afdt(void *dst, u64 ram_base, u64 ram_size,
				u32 panic_log_size, const char *cmdline)
{
	struct afdt_writer w = { .base = dst, .cur = dst };
	u8 chip_id[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	u8 seed[256];
	size_t i;

	for (i = 0; i < sizeof(seed); i++)
		seed[i] = (u8)(i * 0x9d + 0x5a);

	/* / : name, compatible, model; 5 children */
	afdt_w_node(&w, 3, 5);
	afdt_w_string(&w, "name", "device-tree");
	afdt_w_string(&w, "compatible", "QEMU,virt");
	afdt_w_string(&w, "model", "QEMU1,1");

	/* /chosen: 11 properties, 1 child (memory-map) */
	afdt_w_node(&w, 11, 1);
	afdt_w_string(&w, "name", "chosen");
	afdt_w_u64prop(&w, "dram-base", ram_base);
	afdt_w_u64prop(&w, "dram-size", ram_size);
	afdt_w_string(&w, "firmware-version", "u-boot-xnu");
	afdt_w_string(&w, "system-firmware-version", "u-boot-xnu");
	afdt_w_string(&w, "boot-args", cmdline ? cmdline : "");
	afdt_w_raw(&w, "unique-chip-id", chip_id, sizeof(chip_id));
	afdt_w_u32prop(&w, "embedded-panic-log-size", panic_log_size);
	afdt_w_u32prop(&w, "kernel-ctrr-to-be-enabled", 0);
	afdt_w_u32prop(&w, "ephemeral-storage", 1);
	afdt_w_raw(&w, "random-seed", seed, sizeof(seed));

	/* /chosen/memory-map: no RAMDisk until a root image is provided */
	afdt_w_node(&w, 1, 0);
	afdt_w_string(&w, "name", "memory-map");

	/* /defaults: serial-device points at /arm-io/uart0's phandle */
	afdt_w_node(&w, 2, 0);
	afdt_w_string(&w, "name", "defaults");
	afdt_w_u32prop(&w, "serial-device", QEMU_VIRT_UART_PHANDLE);

	/* /arm-io: 3 properties, 3 children (uart0, gic, timer) */
	afdt_w_node(&w, 3, 3);
	afdt_w_string(&w, "name", "arm-io");
	afdt_w_string(&w, "device_type", "arm-io");
	afdt_w_pair(&w, "ranges", 0, QEMU_VIRT_SOC_BASE);

	/* /arm-io/uart0: PL011 */
	afdt_w_node(&w, 4, 0);
	afdt_w_string(&w, "name", "uart0");
	afdt_w_string(&w, "compatible", "arm,pl011");
	afdt_w_pair(&w, "reg", QEMU_VIRT_UART_BASE - QEMU_VIRT_SOC_BASE, 0x1000);
	afdt_w_u32prop(&w, "AAPL,phandle", QEMU_VIRT_UART_PHANDLE);

	/* /arm-io/gic: GICv2 distributor + CPU interface */
	afdt_w_node(&w, 4, 0);
	afdt_w_string(&w, "name", "gic");
	afdt_w_string(&w, "interrupt-controller", "master");
	afdt_w_string(&w, "compatible", "arm,gic-400");
	afdt_w_pair(&w, "reg", QEMU_VIRT_GICD_BASE - QEMU_VIRT_SOC_BASE, 0x1000);

	/* /arm-io/timer: PL031 RTC */
	afdt_w_node(&w, 3, 0);
	afdt_w_string(&w, "name", "timer");
	afdt_w_string(&w, "device_type", "timer");
	afdt_w_pair(&w, "reg", QEMU_VIRT_RTC_BASE - QEMU_VIRT_SOC_BASE, 0x1000);

	/* /cpus + /cpus/cpu0 */
	afdt_w_node(&w, 1, 1);
	afdt_w_string(&w, "name", "cpus");

	afdt_w_node(&w, 5, 0);
	afdt_w_string(&w, "name", "cpu0");
	afdt_w_string(&w, "state", "running");
	afdt_w_u32prop(&w, "reg", 0);
	afdt_w_u32prop(&w, "timebase-frequency", (u32)QEMU_VIRT_TIMEBASE);
	afdt_w_u32prop(&w, "clock-frequency", 1000000000);

	/* /pram: panic log at the top of DRAM */
	afdt_w_node(&w, 2, 0);
	afdt_w_string(&w, "name", "pram");
	afdt_w_pair(&w, "reg", ram_base + ram_size - panic_log_size,
		    panic_log_size);

	return (u32)(w.cur - w.base);
}
#endif /* CONFIG_ARCH_QEMU */

struct mach_o_load_info {
	uintptr_t base;
	uintptr_t entry;
	uintptr_t end;
};

static struct mach_o_load_info load_mach_o_image(void *mach_o_image)
{
	struct mach_o_header *header;
	struct mach_o_segment_command *sc;
	struct mach_o_load_command *lc;
	struct mach_o_load_info ret;
	void *dst, *src;
	int i;

	ret.end = 0;
	ret.base = ~0;

	header = mach_o_image;
	/*
	 * kernelcache is MH_FILESET: the top-level segments cover every
	 * kext, and the LC_UNIXTHREAD/LC_MAIN entry is the kernel's.
	 */
	if (header->magic != MACH_O_MAGIC ||
	    (header->file_type != MACH_O_EXEC && header->file_type != MACH_O_FILESET))
		return ret;

	lc = (struct mach_o_load_command *)(header + 1);
	for (i = 0; i < header->commands_nb; ++i) {
		if (lc->command == LOAD_COMMAND_SEGMENT) {
			sc = (struct mach_o_segment_command *)lc;

			/*
			 * Skip __PAGEZERO and any empty segment: XNU keeps
			 * zero-sized placeholders (__PRELINK_TEXT lives at
			 * 0xfffffff004004000, below __TEXT) that would
			 * otherwise become the image base and corrupt the
			 * segment destination addresses.
			 */
			if (sc->dst_len == 0 ||
			    (strncmp(sc->segment_name, "__PAGEZERO", 10) == 0 && sc->src_len == 0)) {
				lc = ((void *)lc) + lc->command_size;
				continue;
			}
			ret.end  = max_t(uintptr_t,
					 sc->dst + sc->dst_len, ret.end);
			ret.base = min_t(uintptr_t, sc->dst, ret.base);
		}
		lc = ((void *)lc) + lc->command_size;
	}

	lc = (struct mach_o_load_command *)(header + 1);
	for (i = 0; i < header->commands_nb; ++i) {
		if (lc->command == LOAD_COMMAND_SEGMENT) {
			sc = (struct mach_o_segment_command *)lc;

			if (sc->dst_len == 0 ||
			    (strncmp(sc->segment_name, "__PAGEZERO", 10) == 0 && sc->src_len == 0)) {
				lc = ((void *)lc) + lc->command_size;
				continue;
			}
			dst = (void *)(sc->dst - ret.base + XNU_LOAD_ADDR);
			src = (void *)mach_o_image + sc->src_offset;

			if (sc->src_len && sc->dst_len)
				memcpy(dst, src, sc->src_len);
			if (sc->src_len != sc->dst_len && sc->dst_len)
				memset(dst + sc->src_len, 0,
				       sc->dst_len - sc->src_len);
		} else if (lc->command == LOAD_COMMAND_UNIXTHREAD) {
			ret.entry = ((struct thread_command *)lc)->state.pc;
		} else if (lc->command == LOAD_COMMAND_MAIN) {
			struct entry_point_command *ep = (struct entry_point_command *)lc;
			ret.entry = ret.base + ep->entryoff;
		}
		lc = ((void *)lc) + lc->command_size;
	}

	return ret;
}

int do_bootxnu(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	void *kernel_image_addr, *fdt_image_addr;
	struct xnu_boot_arguments *boot_args;
	struct mach_o_load_info load_info;
	void *xnu_entry, *xnu_end;
	char *command_line;

	if (argc < 2) {
		printf("Usage: bootxnu kernel_addr [fdt_addr]\n");
		return 1;
	}
	kernel_image_addr = (void *)simple_strtoul(argv[1], NULL, 16);
	fdt_image_addr = argc >= 3 ? (void *)simple_strtoul(argv[2], NULL, 16) : NULL;

	load_info = load_mach_o_image(kernel_image_addr);
	if (load_info.base > load_info.end) {
		printf("No Mach-O image at address %p\n", kernel_image_addr);
		return 2;
	}
	xnu_entry = (void *)load_info.entry - load_info.base + XNU_LOAD_ADDR;
	xnu_end   = (void *)load_info.end   - load_info.base + XNU_LOAD_ADDR;

	boot_args = xnu_end;
	memset(boot_args, 0, sizeof(*boot_args));
	boot_args->revision = 2;
	boot_args->version = 2;
	boot_args->virt_base = load_info.base;
	boot_args->phys_base = XNU_LOAD_ADDR;
	/*
	 * mem_size is what XNU hands the allocator. On QEMU the panic log
	 * lives in the last 512K of DRAM (chosen/pram in the AFDT), so it
	 * must be excluded here. chosen/dram-size in the AFDT is the full
	 * bank and is what is_dram_addr() uses.
	 */
#ifdef CONFIG_ARCH_QEMU
	if (gd->ram_size > 0x80000)
		boot_args->mem_size = gd->ram_size - 0x80000;
	else
#endif
		boot_args->mem_size = gd->ram_size;
	boot_args->phys_end = (uintptr_t)(boot_args + 1);
	command_line = env_get("bootargs");
	if (command_line)
		strlcpy(boot_args->command_line, command_line, XNU_CMDLINE_LEN);

	struct udevice *vid_device = NULL;
	struct video_priv *vid_priv = NULL;
	int ret = uclass_first_device_err(UCLASS_VIDEO, &vid_device);
	if (ret == 0) {
		vid_priv = dev_get_uclass_priv(vid_device);
	}

	if (vid_priv) {
		boot_args->video_information.base_addr = (u64)(uintptr_t)vid_priv->fb;
		boot_args->video_information.display = 1;
		boot_args->video_information.bytes_per_row = ( (1 << vid_priv->bpix) * (vid_priv->xsize) ) / 8;
		boot_args->video_information.width = vid_priv->xsize;
		boot_args->video_information.height = vid_priv->ysize;
		boot_args->video_information.depth = (1 << vid_priv->bpix);
	}

	{
		uintptr_t afdt_phys = boot_args->phys_end;

#ifdef CONFIG_ARCH_QEMU
		/*
		 * QEMU's virt machine gives us a standard devicetree, which XNU
		 * cannot parse; synthesize the AFDT iBoot would have handed it.
		 * The panic log lives in the last 512K of DRAM (see mem_size).
		 */
		boot_args->afdt_length = build_qemu_virt_afdt((void *)afdt_phys,
							     gd->ram_base,
							     gd->ram_size,
							     0x80000,
							     boot_args->command_line);
#else
		if (!fdt_image_addr) {
			printf("bootxnu: no device tree provided\n");
			return 1;
		}
		boot_args->afdt_length = afdt_length(fdt_image_addr);
		memcpy((void *)afdt_phys, fdt_image_addr, boot_args->afdt_length);
#endif
		/*
		 * XNU's boot_args.deviceTreeP is a kernel virtual address,
		 * not the physical copy sitting after the Mach-O.
		 */
		boot_args->afdt = afdt_phys - XNU_LOAD_ADDR + load_info.base;
		boot_args->phys_end = roundup(afdt_phys + boot_args->afdt_length,
					      0x10000);
	}

	printf("bootxnu: entry=%p boot_args=%p phys_base=0x%llx virt_base=0x%llx\n",
	       xnu_entry, boot_args,
	       (unsigned long long)boot_args->phys_base,
	       (unsigned long long)boot_args->virt_base);

	dcache_disable();
#ifdef CONFIG_ARM64
	armv8_switch_to_el1((u64)boot_args, 0, 0, 0,
			    (u64)xnu_entry, ES_TO_AARCH64);
#else
	((void (*)(struct xnu_boot_arguments *))xnu_entry)(boot_args);
#endif
	return 3;
}

U_BOOT_CMD(
	bootxnu, 3, 0, do_bootxnu,
	"Boot XNU from a Mach-O image\n",
	" [kernel_address] - load address of XNU Mach-O image.\n"
	" [fdt_address] - load address of Apple flattened device tree image.\n"
);
