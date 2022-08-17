// SPDX-License-Identifier: GPL-2.0
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */

#include <asm/tlbflush.h>
#include <linux/dma-mapping.h>
#include <linux/dma-map-ops.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/io-pgtable.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/of_address.h>
#include <linux/pci_regs.h>
#include <linux/sizes.h>

#define PPL_MAGIC 0x4b1d000000000002

#define UAT_NUM_CTX 64

#define UAT_PGBIT 14
#define UAT_PGSZ 16384

#define UAT_IAS 39
#define UAT_IAS_KERN 36
#define UAT_OAS 36

// TODO: get from DT
#define IOVA_TTBR1_BASE	0xffffff8000000000UL
#define IOVA_KERN_BASE	0xffffffa000000000UL
#define IOVA_KERN_TOP	0xffffffafffffffffUL

#define TTBR_VALID BIT(0)
#define TTBR_ASID(n) (n << 48)

// FIXME: this should not be global
static struct asahi_mmu *g_mmu;

struct uat_region {
	phys_addr_t base;
	size_t size;
	void *map;
};

struct flush_info_t {
	u64 state;
	u64 addr;
	u64 size;
} __packed;

struct handoff_t {
	u64 magic_ap;
	u64 magic_fw;

	u8 lock_ap;
	u8 lock_fw;
	u8 __pad[2];

	u32 turn;
	u32 unk;

	struct flush_info_t flush[UAT_NUM_CTX + 1];

	u8 unk2;
	u8 __pad2[7];
	u64 unk3;
} __packed;

struct ctx_t {
	u64 ttbr0;
	u64 ttbr1;
};

struct asahi_mmu {
	struct iova_domain iovad;
	struct io_pgtable_cfg pgtbl_cfg;
	struct io_pgtable_ops *pgtbl_ops;

	struct uat_region handoff_rgn;
	struct uat_region pagetables_rgn;
	struct uat_region contexts_rgn;

	struct handoff_t *handoff;
	struct ctx_t *contexts;
	u64 *kernel_l0;
};

static int mmu_map_region(struct device *dev, const char *name, struct uat_region *region)
{
	struct device_node *np;
	struct resource r;
	int ret;
	int idx = of_property_match_string(dev->of_node, "memory-region-names", name);

	np = of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!np) {
		dev_err(dev, "Missing %s region\n", name);
		return -EINVAL;
	}

	ret = of_address_to_resource(np, 0, &r);
	of_node_put(np);
	if (ret) {
		dev_err(dev, "Failed to get %s region\n", name);
		return ret;
	}

	region->base = r.start;
	region->size = resource_size(&r);
	region->map = devm_memremap(dev, r.start, region->size, MEMREMAP_WB);
	if (!region->map) {
		dev_err(dev, "Failed to map %s region\n", name);
		return -ENOMEM;
	}

	return 0;
}

static void mmu_tlb_flush_all(void *cookie)
{
	__tlbi(vmalle1os);
}

static void mmu_tlb_flush_walk(unsigned long iova, size_t size, size_t granule,
			       void *cookie)
{
	// TODO
	mmu_tlb_flush_all(cookie);
}

static const struct iommu_flush_ops mmu_tlb_ops = {
	.tlb_flush_all	= mmu_tlb_flush_all,
	.tlb_flush_walk = mmu_tlb_flush_walk,
};

static int dma_info_to_prot(enum dma_data_direction dir, bool coherent,
                     unsigned long attrs)
{
	int prot = coherent ? IOMMU_CACHE : 0;

	prot |= IOMMU_PRIV;

	switch (dir) {
	case DMA_BIDIRECTIONAL:
			return prot | IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
			return prot | IOMMU_READ;
	case DMA_FROM_DEVICE:
			return prot | IOMMU_WRITE;
	default:
		return 0;
	}
}

static dma_addr_t asahi_alloc_iova(struct iova_domain *iovad,
								   unsigned long size, unsigned long limit)
{
	unsigned long shift = iova_shift(iovad);
	unsigned long iova_len = iova_align(iovad, size) >> shift;
	unsigned long iova_pfn;

	iova_pfn = alloc_iova_fast(iovad, iova_len, limit >> shift, true);

	return (dma_addr_t)iova_pfn << shift;
}

static void asahi_free_iova(struct iova_domain *iovad,
								  dma_addr_t base, unsigned long size)
{
	unsigned long shift = iova_shift(iovad);
	unsigned long iova_len = iova_align(iovad, size) >> shift;

	WARN_ON(iova_offset(iovad, base));
	free_iova_fast(iovad, base >> shift, iova_len);
}

static int asahi_map_pages(struct asahi_mmu *mmu, phys_addr_t paddr, dma_addr_t iova, size_t size, int prot)
{
	struct io_pgtable_ops *ops = mmu->pgtbl_ops;

	printk("asahi_map_pages %llx %llx %llx %llx\n", paddr, iova, size, prot);

	if ((size | iova | paddr) & (UAT_PGSZ - 1))
		return -EINVAL;

	while (size) {
		size_t pgsize = UAT_PGSZ;

		ops->map(ops, iova - IOVA_KERN_BASE, paddr, pgsize, prot, GFP_KERNEL);
		iova += pgsize;
		paddr += pgsize;
		size -= pgsize;
	}

	mmu_tlb_flush_all(NULL);

	return 0;
}

static int asahi_unmap_pages(struct asahi_mmu *mmu, dma_addr_t iova, size_t size)
{
	struct io_pgtable_ops *ops = mmu->pgtbl_ops;

	printk("asahi_unmap_pages %llx %llx\n", iova, size);

	if ((size | iova) & (UAT_PGSZ - 1))
		return -EINVAL;

	while (size) {
		size_t pgsize = UAT_PGSZ;

		ops->unmap(ops, iova - IOVA_KERN_BASE, pgsize, NULL);
		iova += pgsize;
		size -= pgsize;
	}

	mmu_tlb_flush_all(NULL);

	return 0;
}

static dma_addr_t asahi_mmu_map_page(struct device *dev, struct page *page,
		unsigned long offset, size_t size, enum dma_data_direction dir,
		unsigned long attrs)
{
	struct iova_domain *iovad = &g_mmu->iovad;
	phys_addr_t phys = page_to_phys(page) + offset;
	int ioprot = dma_info_to_prot(dir, true, attrs);
	size_t iova_off = iova_offset(iovad, phys);
	dma_addr_t iova;

	size = iova_align(iovad, size + iova_off);

	iova = asahi_alloc_iova(iovad, size, IOVA_KERN_TOP);
	if (!iova)
		return DMA_MAPPING_ERROR;

	if (asahi_map_pages(g_mmu, phys - iova_off, iova, size, ioprot))
		return DMA_MAPPING_ERROR;

	return iova + iova_off;
}

static void asahi_mmu_unmap_page(struct device *dev, dma_addr_t dma_handle, size_t size, enum dma_data_direction dir, unsigned long attrs)
{
	struct iova_domain *iovad = &g_mmu->iovad;
	size_t iova_off = iova_offset(iovad, dma_handle);

	size = iova_align(iovad, size + iova_off);

	asahi_unmap_pages(g_mmu, dma_handle - iova_off, size);
	asahi_free_iova(iovad, dma_handle - iova_off, size);
}

static void *asahi_mmu_alloc(struct device *dev, size_t size,
		dma_addr_t *handle, gfp_t gfp, unsigned long attrs)
{
	struct iova_domain *iovad = &g_mmu->iovad;
	int ioprot = dma_info_to_prot(DMA_BIDIRECTIONAL, true, attrs);
	void *pages;
	dma_addr_t iova;

	size = iova_align(iovad, size);

	gfp |= __GFP_ZERO | __GFP_NOWARN;
	gfp &= ~__GFP_COMP;

	pages = alloc_pages_exact(size, gfp);
	iova = asahi_alloc_iova(iovad, size, IOVA_KERN_TOP);
	if (!iova)
		goto err_free;

	if (asahi_map_pages(g_mmu, virt_to_phys(pages), iova, size, ioprot))
		goto err_free_iova;

	*handle = iova;
	return pages;

err_free_iova:
	asahi_free_iova(iovad, iova, size);
err_free:
	free_pages_exact(pages, size);
	return NULL;
}

static void asahi_mmu_free(struct device *dev, size_t size, void *cpu_addr,
		dma_addr_t handle, unsigned long attrs)
{
	struct iova_domain *iovad = &g_mmu->iovad;

	size = iova_align(iovad, size);

	asahi_unmap_pages(g_mmu, handle, size);
	asahi_free_iova(iovad, handle, size);
	free_pages_exact(cpu_addr, size);
}

static const struct dma_map_ops asahi_dma_ops = {
	.map_page		= asahi_mmu_map_page,
	.unmap_page		= asahi_mmu_unmap_page,
	.alloc			= asahi_mmu_alloc,
	.free			= asahi_mmu_free,
	.alloc_pages	= dma_common_alloc_pages,
	.free_pages		= dma_common_free_pages,
	.mmap			= dma_common_mmap,
	.get_sgtable	= dma_common_get_sgtable,
};

void handoff_lock(struct asahi_mmu *mmu)
{
	mb();
	mmu->handoff->lock_ap = 1;
	mb();
	while (mmu->handoff->lock_fw != 0) {
		mb();
		if (mmu->handoff->turn != 0) {
			mb();
			mmu->handoff->lock_ap = 0;
			mb();
			while (mmu->handoff->turn != 0)
				mb();
			mb();
			mmu->handoff->lock_ap = 1;
			mb();
		}
	}
	mb();
}

void handoff_unlock(struct asahi_mmu *mmu)
{
	mb();
	mmu->handoff->turn = 1;
	wmb();
	mmu->handoff->lock_ap = 0;
	wmb();
}

int handoff_init(struct asahi_mmu *mmu)
{
	int i;

	mmu->handoff->magic_ap = PPL_MAGIC;
	mmu->handoff->unk = 0xffffffff;
	mmu->handoff->unk3 = 0;
	wmb();

	handoff_lock(mmu);

	while (mmu->handoff->magic_fw != PPL_MAGIC)
		mb();

	handoff_unlock(mmu);

	for(i = 0; i < UAT_NUM_CTX + 1; i++) {
		mmu->handoff->flush[i].state = 0;
		mmu->handoff->flush[i].addr = 0;
		mmu->handoff->flush[i].size = 0;
	}

	wmb();

	return 0;
}

int asahi_mmu_init(struct device *dev)
{
	int i, ret;
	struct asahi_mmu *priv = devm_kzalloc(dev, sizeof(struct asahi_mmu), GFP_KERNEL);

	if (!priv)
		return -ENOMEM;

	dev_info(dev, "MMU: Initializing...\n");

	if (mmu_map_region(dev, "handoff", &priv->handoff_rgn))
		return -EIO;
	if (mmu_map_region(dev, "contexts", &priv->contexts_rgn))
		return -EIO;
	if (mmu_map_region(dev, "pagetables", &priv->pagetables_rgn))
		return -EIO;

	priv->handoff = priv->handoff_rgn.map;
	priv->contexts = priv->contexts_rgn.map;
	priv->kernel_l0 = priv->pagetables_rgn.map;

	dev_info(dev, "MMU: Initializing handoff\n");
	if (handoff_init(priv))
		return -EIO;

	dev_info(dev, "MMU: Initializing tables\n");

	handoff_lock(priv);
	for (i = 0; i < UAT_NUM_CTX; i++) {
		priv->contexts[i].ttbr0 = 0;
		priv->contexts[i].ttbr1 = priv->pagetables_rgn.base | TTBR_VALID;
	}
	handoff_unlock(priv);

	wmb();

	dev_info(dev, "MMU: Initializing IOVA\n");

	init_iova_domain(&priv->iovad, UAT_PGSZ, IOVA_KERN_BASE >> UAT_PGBIT);
	ret = iova_domain_init_rcaches(&priv->iovad);
	if (ret)
		return ret;

	g_mmu = priv;

	dev_info(dev, "MMU: Initializing io_pgtable\n");

	priv->pgtbl_cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= UAT_PGSZ,
		.ias		= UAT_IAS_KERN,
		.oas		= UAT_OAS,
		.coherent_walk	= true,
		.tlb		= &mmu_tlb_ops,
		.iommu_dev	= dev,
	};

	priv->pgtbl_ops = alloc_io_pgtable_ops(APPLE_UAT, &priv->pgtbl_cfg,
					      priv);
	if (!priv->pgtbl_ops)
		return -EINVAL;

	priv->kernel_l0[2] = priv->pgtbl_cfg.apple_uat_cfg.ttbr | 3;
	wmb();

	set_dma_ops(dev, &asahi_dma_ops);

	dev_info(dev, "MMU: Initialized\n");

	return 0;
}

void asahi_mmu_fini(struct device *dev)
{

}
