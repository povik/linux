// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/bits.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/io.h>

#include "leap.h"

#define REG_LEAP_CORE_ENABLE		0x8004
#define REG_LEAP_IO_PUT_TAKE(port)	(0xe00000 + ((port) - 0x40) * 8)
#define REG_LEAP_IO_UPDATE_PEEK(port)	(0xe00000 + ((port) - 0x40) * 8)

#define REG_LEAP_STATE_BASE(idx)	(0x00000 + (0x10000 * (idx)))
#define REG_LEAP_INST_BASE(idx)		(0x40000 + (0x10000 * (idx)))

#define REG_LEAP_ROUTINE_PCLIMITS(routidx)	(0x800ac + (0x4 * (routidx)))
#define LEAP_ROUTINE_PCLIMITS_TOP		GENMASK(31, 16)
#define LEAP_ROUTINE_PCLIMITS_BOT		GENMASK(15, 0)

#define REG_LEAP_WAITEMPTY_SIEVE(core, routidx, i) \
		(0xe00000 + ((core) * 0x400) + ((routidx) * 0x4) + ((i) * 0x40)

#define REG_LEAP_WAITFULL_SIEVE(core, routidx, i) \
		(0xe00020 + ((core) * 0x400) + ((routidx) * 0x4) + ((i) * 0x40)

#define LEAP_MAX_SECTION_SIZE	4096
#define LEAP_MAX_ROUTINES	8

struct leap_core {
	struct device *dev;
	struct leap_cluster *cluster;
	unsigned int no;
	__iomem void *base;

	u32 (*lookup_symbol)(const char *label);
};

struct leap_cluster {
	struct device *dev;
	__iomem void *base;
	struct reset_control *rstc;

	int ncores;
	struct leap_core cores[];
};

int leap_io_put(struct leap_cluster *cl, int port, u32 val)
{
	if (port < 0x40 || port >= 0x80)
		return -EINVAL;

	writel_relaxed(val, cl->base + REG_LEAP_IO_PUT_TAKE(port));
	return 0;
}

int leap_io_update(struct leap_cluster *cl, int port, u32 val)
{
	if (port < 0x40 || port >= 0x80)
		return -EINVAL;

	writel_relaxed(val, cl->base + REG_LEAP_IO_UPDATE_PEEK(port));
	return 0;
}

int leap_io_take(struct leap_cluster *cl, int port, u32 *val)
{
	if (port < 0x40 || port >= 0x80)
		return -EINVAL;

	*val = readl_relaxed(cl->base + REG_LEAP_IO_PUT_TAKE(port));
	return 0;
}

int leap_io_peek(struct leap_cluster *cl, int port, u32 *val)
{
	if (port < 0x40 || port >= 0x80)
		return -EINVAL;

	*val = readl_relaxed(cl->base + REG_LEAP_IO_UPDATE_PEEK(port));
	return 0;
}

void leap_enable(struct leap_core *core)
{
	writel_relaxed(3, core->base + REG_LEAP_CORE_ENABLE);
}

void leap_disable(struct leap_core *core)
{
	writel_relaxed(0, core->base + REG_LEAP_CORE_ENABLE);
}

u32 read_le32(u8 *p)
{
	return (u32) p[3] << 24 | (u32) p[2] << 16 | \
		(u32) p[1] << 8 | (u32) p[0];
}

static u8 *seek_in_firmware(u8 *p, size_t offset, u8 *end)
{
	if (offset > end - p)
		return end;

	return p + offset;
}

#define SECTION_TYPE(p)      read_le32(((u8 *) (p)) + 0)
#define SECTION_LOAD_BASE(p) read_le32(((u8 *) (p)) + 4)
#define SECTION_SIZE(p)      read_le32(((u8 *) (p)) + 8)
#define SECTION_FLAGS(p)     read_le32(((u8 *) (p)) + 12)

#define LEAPFROG_HDRSIZE		8
#define LEAPFROG_SECTION_HDRSIZE	16
#define LEAPFROG_SYM_HDRSIZE		12
#define LEAPFROG_SYM_LOCSIZE		8

#define LEAPFROG_SECTION_STATE0		0x10000
#define LEAPFROG_SECTION_STATE1		0x10001
#define LEAPFROG_SECTION_STATE2		0x10002
#define LEAPFROG_SECTION_STATE3		0x10003

#define LEAPFROG_SECTION_INST0		0x20000
#define LEAPFROG_SECTION_INST1		0x20001
#define LEAPFROG_SECTION_INST2		0x20002
#define LEAPFROG_SECTION_INST3		0x20003

#define LEAPFROG_SECTION_WAITEMPTY_LIST	0x30000
#define LEAPFROG_SECTION_WAITFULL_LIST	0x30001

#define LEAPFROG_SECTION_SYMBOLS	0x40000

#define LEAPFROG_SECTION_FLAG_ROUTINE	BIT(0)

#define for_leapfrog_section(image, end, p) \
	for (p = seek_in_firmware(image, LEAPFROG_HDRSIZE, end); \
	     p + LEAPFROG_SECTION_HDRSIZE < end; \
	     p = seek_in_firmware(p, LEAPFROG_SECTION_HDRSIZE \
	     			     + SECTION_SIZE(p) * 4, end))

#define SYM_NLOCATIONS(s)	read_le32(((u8 *) (s)) + 8)
#define SYM_DATALEN(s)		(SYM_NLOCATIONS(s) * 8)

#define LOC_OFFSET(l)	read_le32(((u8 *) (l)) + 0)
#define LOC_LSHIFT(l)	read_le32(((u8 *) (l)) + 4)

bool leapfrog_apply_symbols(struct leap_core *core,
			    u8 *image, u8 *image_end, u8 *sym, u8 *end)
{

	for (; sym + LEAPFROG_SYM_HDRSIZE < end;
			sym = seek_in_firmware(sym, LEAPFROG_SYM_HDRSIZE \
						    + SYM_DATALEN(sym), end)) {
		u8 *loc;
		u32 value;
		int i;

		if (!core->lookup_symbol) {
			dev_err(core->dev, "can't resolve firmware's symbols\n");
			return false;	
		}

		/*
		 * The symbol label must be a zero-terminated string.
		 */
		if (sym[7] != '\0') {
			dev_err(core->dev, "firmware bad symbol at +0x%zx\n", sym - image);
			return false;
		}

		value = core->lookup_symbol ? core->lookup_symbol(sym) : 0;
		loc = seek_in_firmware(sym, LEAPFROG_SYM_HDRSIZE, end);
		for (i = 0; i < SYM_NLOCATIONS(loc) && loc + LEAPFROG_SYM_LOCSIZE < end;
				i++, loc = seek_in_firmware(loc, LEAPFROG_SYM_LOCSIZE, end)) {
			u32 offset = LOC_OFFSET(loc);
			u8 *target;

			target = seek_in_firmware(image, offset, image_end);
			if (offset % 4 != 0 || target == image_end) {
				dev_err(core->dev, "firmware bad symbol location at +0x%zx\n",
					loc - image);
				return false;
			}

			*((u32 *) target) = value << LOC_LSHIFT(loc);
		}
	}

	return true;
}

int leap_routidx_by_base(struct leap_core *core, u32 routine_base)
{
	int i;

	for (i = 0; i < LEAP_MAX_ROUTINES; i++) {
		u32 span = readl_relaxed(core->base + \
					 REG_LEAP_ROUTINE_PCLIMITS(i));
		u32 base = FIELD_GET(LEAP_ROUTINE_PCLIMITS_BOT, span);
		u32 top = FIELD_GET(LEAP_ROUTINE_PCLIMITS_TOP, span);

		if (base == top)
			break;

		if (base == routine_base)
			return i;
	}

	return -1;
}

int leap_load_image(struct leap_core *core, u8 *image, size_t size)
{
	struct leap_cluster *cluster = core->cluster;
	u8 *p, *end = image + size;
	u32 magic;

	if (image + LEAPFROG_HDRSIZE > end)
		return -EINVAL;

	magic = read_le32(image);
	if (magic != 0x1ea9) {
		dev_err(core->dev, "firmware bad magic: %x\n", magic);
		return -EINVAL;
	}

	for_leapfrog_section(image, end, p) {
		u8 *data, *data_end;

		if (SECTION_TYPE(p) != LEAPFROG_SECTION_SYMBOLS)
			continue;

		data = seek_in_firmware(p, LEAPFROG_SECTION_HDRSIZE, end);
		data_end = seek_in_firmware(data, SECTION_SIZE(p), end);

		leapfrog_apply_symbols(core, image, end, data, data_end);
	}

	for_leapfrog_section(image, end, p) {
		u8 *data, *data_end;
		u32 base, load_end, stride, i;

		data = seek_in_firmware(p, LEAPFROG_SECTION_HDRSIZE, end);
		data_end = seek_in_firmware(data, SECTION_SIZE(p), end);

		switch (SECTION_TYPE(p)) {
		case LEAPFROG_SECTION_STATE0 ... LEAPFROG_SECTION_STATE3:
		case LEAPFROG_SECTION_INST0 ... LEAPFROG_SECTION_INST3:
			load_end = SECTION_LOAD_BASE(p) + SECTION_SIZE(p);

			if (load_end > LEAP_MAX_SECTION_SIZE) {
				dev_err(core->dev, "out of bounds section span (type %x size %x load base %x)\n",
					SECTION_TYPE(p), SECTION_SIZE(p), SECTION_LOAD_BASE(p));
				return -EINVAL;
			}
			/* Continues below */
			break;

		case LEAPFROG_SECTION_WAITEMPTY_LIST:
		case LEAPFROG_SECTION_WAITFULL_LIST:
		case LEAPFROG_SECTION_SYMBOLS:
			continue;

		default:
			dev_warn(core->dev, "skipping unknown section %x", SECTION_TYPE(p));
			continue;	
		}

		dev_dbg(core->dev, "loading firmware section %x load base %x size %x flags %x\n",
			SECTION_TYPE(p), SECTION_LOAD_BASE(p), SECTION_SIZE(p), SECTION_FLAGS(p));

		switch (SECTION_TYPE(p)) {
		case LEAPFROG_SECTION_STATE0 ... LEAPFROG_SECTION_STATE3:
			base = REG_LEAP_STATE_BASE(SECTION_TYPE(p) - LEAPFROG_SECTION_STATE0);
			stride = 8;
			break;

		case LEAPFROG_SECTION_INST0 ... LEAPFROG_SECTION_INST3:
			base = REG_LEAP_INST_BASE(SECTION_TYPE(p) - LEAPFROG_SECTION_INST0);
			stride = 4;
			break;
		}

		for (i = SECTION_LOAD_BASE(p); i < load_end && data < data_end; i++) {
			writel_relaxed(read_le32(data), core->base + base + stride * i);
			data += 4;
		}

		switch (SECTION_TYPE(p)) {
		case LEAPFROG_SECTION_INST0 ... LEAPFROG_SECTION_INST3:
			if ((SECTION_FLAGS(p) & LEAPFROG_SECTION_FLAG_ROUTINE) \
					&& leap_routidx_by_base(core, SECTION_LOAD_BASE(p)) == -1) {
				for (i = 0; i < LEAP_MAX_ROUTINES - 1; i++)
					if (!readl_relaxed(core->base + REG_LEAP_ROUTINE_PCLIMITS(i)))
						break;

				writel_relaxed(FIELD_PREP(LEAP_ROUTINE_PCLIMITS_BOT, SECTION_LOAD_BASE(p)) \
						| FIELD_PREP(LEAP_ROUTINE_PCLIMITS_TOP, load_end),
						core->base + REG_LEAP_ROUTINE_PCLIMITS(i));
			}
		}
	}

	for_leapfrog_section(image, end, p) {
		u8 *data, *data_end;
		u32 sieve[4];
		int routidx, i;

		data = seek_in_firmware(p, LEAPFROG_SECTION_HDRSIZE, end);
		data_end = seek_in_firmware(data, SECTION_SIZE(p), end);

		switch (SECTION_TYPE(p)) {
		case LEAPFROG_SECTION_WAITEMPTY_LIST:
		case LEAPFROG_SECTION_WAITFULL_LIST:
			break;

		case LEAPFROG_SECTION_STATE0 ... LEAPFROG_SECTION_STATE3:
		case LEAPFROG_SECTION_INST0 ... LEAPFROG_SECTION_INST3:
		case LEAPFROG_SECTION_SYMBOLS:
			continue;

		default:
			dev_warn(core->dev, "skipping unknown section %x", SECTION_TYPE(p));
			continue;	
		}

		dev_dbg(core->dev, "loading firmware section %x load base %x size %x flags %x\n",
			SECTION_TYPE(p), SECTION_LOAD_BASE(p), SECTION_SIZE(p), SECTION_FLAGS(p));

		routidx = leap_routidx_by_base(core, SECTION_LOAD_BASE(p) >> 16);

		if (routidx < 0) {
			dev_err(core->dev, "section %x base %x refers nonexistent routine\n",
				SECTION_TYPE(p), SECTION_LOAD_BASE(p));
			return -EINVAL;
		}

		for (; data < data_end; data += 4) {
			u32 port = read_le32(data);
			if (port >= 128)
				continue;
			sieve[port / 32] |= 1 << (port % 32);
		}

		switch (SECTION_TYPE(p)) {
		case LEAPFROG_SECTION_WAITEMPTY_LIST:
			for (i = 0; i < 4; i++)
				writel_relaxed(sieve[i], cluster->base + \
							REG_LEAP_WAITEMPTY_SIEVE(core->no, routidx, i)));
			break;

		case LEAPFROG_SECTION_WAITFULL_LIST:
			for (i = 0; i < 4; i++)
				writel_relaxed(sieve[i], cluster->base + \
							REG_LEAP_WAITFULL_SIEVE(core->no, routidx, i)));
			break;
		}
	}

	return 0;
}

struct leap_core *leap_get_core(struct leap_cluster *cluster, int index)
{
	return &cluster->cores[index];
}

static void apple_leap_release(struct leap_cluster *cl)
{
	reset_control_rearm(cl->rstc);
}

static int apple_leap_probe(struct platform_device *pdev)
{
	struct leap_cluster *cl;
	int i, ret, ncores;

	ncores = 4;
	cl = devm_kzalloc(&pdev->dev, struct_size(cl, cores, ncores), GFP_KERNEL);
	if (!cl)
		return -ENOMEM;
	cl->ncores = ncores;
	cl->dev = &pdev->dev;
	cl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cl->base))
		return PTR_ERR(cl->base);
	platform_set_drvdata(pdev, cl);
	cl->rstc = devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (IS_ERR(cl->rstc))
		return PTR_ERR(cl->rstc);

	reset_control_reset(cl->rstc);

	for (i = 0; i < ncores; i++) {
		struct leap_core *core = &cl->cores[i];

		core->dev = &pdev->dev;
		core->base = cl->base + i * 0x100000;
		core->cluster = cl;
	}

	return 0;

//err_release:
	apple_leap_release(cl);
	return ret;
}

static int apple_leap_remove(struct platform_device *pdev)
{
	struct leap_cluster *cl = platform_get_drvdata(pdev);

	apple_leap_release(cl);
	return 0;
}

static const struct of_device_id apple_leap_of_match[] = {
	{ .compatible = "apple,leap", },
	{}
};
MODULE_DEVICE_TABLE(of, apple_leap_of_match);

static struct platform_driver apple_leap_driver = {
	.driver = {
		.name = "apple-leap",
		.of_match_table = apple_leap_of_match,
	},
	.probe = apple_leap_probe,
	.remove = apple_leap_remove,
};
module_platform_driver(apple_leap_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Apple LEAP signal coprocessor driver");
MODULE_LICENSE("Dual MIT/GPL");
