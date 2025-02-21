#include <common.h>
#include <cpu_func.h>
#include <env.h>
#include <command.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <video.h>
#include <stdint.h>

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
#define MACH_O_EXEC             0x2

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

#define XNU_CMDLINE_LEN 608

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

#define XNU_LOAD_OFFSET  0x4000
#define XNU_LOAD_ADDR CONFIG_SYS_LOAD_ADDR + XNU_LOAD_OFFSET

u32 afdt_length(void *afdt)
{
	struct afdt_node *node = afdt;
	u32 offset = sizeof(*node);
	unsigned int i;

	for (i = 0; i < node->properties_nb; ++i) {
		struct afdt_property *property = (afdt + offset);

		offset += sizeof(*property) + roundup(property->length, 4);
	}

	for (i = 0; i < node->children_nb; ++i)
		offset += afdt_length(afdt + offset);

	return offset;
}

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
	if (header->magic != MACH_O_MAGIC || header->file_type != MACH_O_EXEC)
		return ret;

	lc = (struct mach_o_load_command *)(header + 1);
	for (i = 0; i < header->commands_nb; ++i) {
		if (lc->command == LOAD_COMMAND_SEGMENT) {
			sc = (struct mach_o_segment_command *)lc;

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

			dst = (void *)(sc->dst - ret.base + XNU_LOAD_ADDR);
			src = (void *)mach_o_image + sc->src_offset;

			if (sc->src_len && sc->dst_len)
				memcpy(dst, src, sc->src_len);
			if (sc->src_len != sc->dst_len && sc->dst_len)
				memset(dst + sc->src_len, 0,
				       sc->dst_len - sc->src_len);
		} else if (lc->command == LOAD_COMMAND_UNIXTHREAD) {
			ret.entry = ((struct thread_command *)lc)->state.pc;
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

	if (argc < 3) {
		printf("Usage: bootxnu kernel_addr fdt_addr");
		return 1;
	}
	kernel_image_addr = (void *)simple_strtoul(argv[1], NULL, 16);
	fdt_image_addr = (void *)simple_strtoul(argv[2], NULL, 16);

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
	boot_args->mem_size = CONFIG_SYS_SDRAM_SIZE;
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

	boot_args->afdt = boot_args->phys_end;
	boot_args->afdt_length = afdt_length(fdt_image_addr);
	memcpy((void *)boot_args->afdt, fdt_image_addr, boot_args->afdt_length);
	boot_args->phys_end += boot_args->afdt_length;
	boot_args->phys_end = roundup(boot_args->phys_end, 0x10000);

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
