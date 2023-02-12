// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io> */

#include <linux/align.h>
#include <linux/apple-mailbox.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>

#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_module.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "afk.h"
#include "dcp.h"
#include "dcp-internal.h"
#include "iomfb.h"
#include "parser.h"
#include "trace.h"

#define APPLE_DCP_COPROC_CPU_CONTROL	 0x44
#define APPLE_DCP_COPROC_CPU_CONTROL_RUN BIT(4)

#define DCP_BOOT_TIMEOUT msecs_to_jiffies(1000)

static bool show_notch;
module_param(show_notch, bool, 0644);
MODULE_PARM_DESC(show_notch, "Use the full display height and shows the notch");

/* HACK: moved here to avoid circular dependency between apple_drv and dcp */
void dcp_drm_crtc_vblank(struct apple_crtc *crtc)
{
	unsigned long flags;

	spin_lock_irqsave(&crtc->base.dev->event_lock, flags);
	if (crtc->event) {
		drm_crtc_send_vblank_event(&crtc->base, crtc->event);
		crtc->event = NULL;
	}
	spin_unlock_irqrestore(&crtc->base.dev->event_lock, flags);
}

void dcp_set_dimensions(struct apple_dcp *dcp)
{
	int i;

	/* Set the connector info */
	if (dcp->connector) {
		struct drm_connector *connector = &dcp->connector->base;

		mutex_lock(&connector->dev->mode_config.mutex);
		connector->display_info.width_mm = dcp->width_mm;
		connector->display_info.height_mm = dcp->height_mm;
		mutex_unlock(&connector->dev->mode_config.mutex);
	}

	/*
	 * Fix up any probed modes. Modes are created when parsing
	 * TimingElements, dimensions are calculated when parsing
	 * DisplayAttributes, and TimingElements may be sent first
	 */
	for (i = 0; i < dcp->nr_modes; ++i) {
		dcp->modes[i].mode.width_mm = dcp->width_mm;
		dcp->modes[i].mode.height_mm = dcp->height_mm;
	}
}

/*
 * Helper to send a DRM vblank event. We do not know how call swap_submit_dcp
 * without surfaces. To avoid timeouts in drm_atomic_helper_wait_for_vblanks
 * send a vblank event via a workqueue.
 */
static void dcp_delayed_vblank(struct work_struct *work)
{
	struct apple_dcp *dcp;

	dcp = container_of(work, struct apple_dcp, vblank_wq);
	mdelay(5);
	dcp_drm_crtc_vblank(dcp->crtc);
}

static void dcp_recv_msg(void *cookie, u8 endpoint, u64 message)
{
	struct apple_dcp *dcp = cookie;

	trace_dcp_recv_msg(dcp, endpoint, message);

	switch (endpoint) {
	case IOMFB_ENDPOINT:
		return iomfb_recv_msg(dcp, message);
	case SYSTEM_ENDPOINT:
		afk_receive_message(dcp->systemep, message);
		return;
	case DPTX_ENDPOINT:
		afk_receive_message(dcp->dptxep, message);
		return;
	default:
		WARN(endpoint, "unknown DCP endpoint %hhu", endpoint);
	}
}

static void dcp_rtk_crashed(void *cookie)
{
	struct apple_dcp *dcp = cookie;

	dcp->crashed = true;
	dev_err(dcp->dev, "DCP has crashed");
	if (dcp->connector) {
		dcp->connector->connected = 0;
		schedule_work(&dcp->connector->hotplug_wq);
	}
	complete(&dcp->start_done);
}

static int dcp_rtk_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_dcp *dcp = cookie;

	if (bfr->iova) {
		struct iommu_domain *domain =
			iommu_get_domain_for_dev(dcp->dev);
		phys_addr_t phy_addr;

		if (!domain)
			return -ENOMEM;

		// TODO: get map from device-tree
		phy_addr = iommu_iova_to_phys(domain,
					      bfr->iova & ~dcp->asc_dram_mask);
		if (!phy_addr)
			return -ENOMEM;

		// TODO: verify phy_addr, cache attribute
		bfr->buffer = memremap(phy_addr, bfr->size, MEMREMAP_WB);
		if (!bfr->buffer)
			return -ENOMEM;

		bfr->is_mapped = true;
		dev_info(dcp->dev,
			 "shmem_setup: iova: %lx -> pa: %lx -> iomem: %lx",
			 (uintptr_t)bfr->iova, (uintptr_t)phy_addr,
			 (uintptr_t)bfr->buffer);
	} else {
		bfr->buffer = dma_alloc_coherent(dcp->dev, bfr->size,
						 &bfr->iova, GFP_KERNEL);
		if (!bfr->buffer)
			return -ENOMEM;

		bfr->iova |= dcp->asc_dram_mask;

		dev_info(dcp->dev, "shmem_setup: iova: %lx, buffer: %lx",
			 (uintptr_t)bfr->iova, (uintptr_t)bfr->buffer);
	}

	return 0;
}

static void dcp_rtk_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_dcp *dcp = cookie;

	if (bfr->is_mapped)
		memunmap(bfr->buffer);
	else
		dma_free_coherent(dcp->dev, bfr->size, bfr->buffer,
				  bfr->iova & ~dcp->asc_dram_mask);
}

static struct apple_rtkit_ops rtkit_ops = {
	.crashed = dcp_rtk_crashed,
	.recv_message = dcp_recv_msg,
	.shmem_setup = dcp_rtk_shmem_setup,
	.shmem_destroy = dcp_rtk_shmem_destroy,
};

void dcp_send_message(struct apple_dcp *dcp, u8 endpoint, u64 message)
{
	trace_dcp_send_msg(dcp, endpoint, message);
	apple_rtkit_send_message(dcp->rtk, endpoint, message, NULL,
				 true);
}

int dcp_crtc_atomic_check(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct drm_plane_state *new_state, *old_state;
	struct drm_plane *plane;
	struct drm_crtc_state *crtc_state;
	int plane_idx, plane_count = 0;
	bool needs_modeset;

	if (dcp->crashed)
		return -EINVAL;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	needs_modeset = drm_atomic_crtc_needs_modeset(crtc_state) || !dcp->valid_mode;
	if (!needs_modeset && !dcp->connector->connected) {
		dev_err(dcp->dev, "crtc_atomic_check: disconnected but no modeset");
		return -EINVAL;
	}

	for_each_oldnew_plane_in_state(state, plane, old_state, new_state, plane_idx) {
		/* skip planes not for this crtc */
		if (new_state->crtc != crtc)
			continue;

		plane_count += 1;
	}

	if (plane_count > DCP_MAX_PLANES) {
		dev_err(dcp->dev, "crtc_atomic_check: Blend supports only 2 layers!");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dcp_crtc_atomic_check);

int dcp_get_connector_type(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return (dcp->connector_type);
}
EXPORT_SYMBOL_GPL(dcp_get_connector_type);

void dcp_hack(struct platform_device *pdev, struct phy *phy, struct mux_control *mux)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	dcp->dptxport[0].atcphy = phy;
	dcp->dptxport[0].mux = mux;
}
EXPORT_SYMBOL_GPL(dcp_hack);

int dcp_dptx_connect(struct platform_device *pdev, u32 port, struct phy *phy)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	dcp->dptxport[port].atcphy = phy;
	dptxport_validate_connection(dcp->dptxport[port].service, 0, 1, 0);
	dptxport_connect(dcp->dptxport[port].service, 0, 1, 0);
	dptxport_request_display(dcp->dptxport[port].service);
	dptxport_set_hpd(dcp->dptxport[port].service, true);

	return 0;
}
EXPORT_SYMBOL_GPL(dcp_dptx_connect);

int dcp_dptx_disconnect(struct platform_device *pdev, u32 port)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	dptxport_release_display(dcp->dptxport[port].service);
	dptxport_set_hpd(dcp->dptxport[port].service, false);

	return 0;
}
EXPORT_SYMBOL_GPL(dcp_dptx_disconnect);

void dcp_link(struct platform_device *pdev, struct apple_crtc *crtc,
	      struct apple_connector *connector)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	dcp->crtc = crtc;
	dcp->connector = connector;
}
EXPORT_SYMBOL_GPL(dcp_link);

int dcp_start(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	init_completion(&dcp->start_done);

	/* start RTKit endpoints */
	ret = systemep_init(dcp);
	if (ret) {
		dev_err(dcp->dev, "Failed to start system endpoint: %d", ret);
		return ret;
	}
	
	
	if (of_device_is_compatible(pdev->dev.of_node, "apple,dcpext")) {
		ret = dptxep_init(dcp);
		if (ret) {
			dev_err(dcp->dev, "Failed to start DPTX endpoint: %d", ret);
			return ret;
		}
	}

	ret = iomfb_start_rtkit(dcp);
	if (ret) {
		dev_err(dcp->dev, "Failed to start IOMFB endpoint: %d", ret);
	}

	dcp->iomfb_started = true;

	return ret;
}
EXPORT_SYMBOL(dcp_start);

int dcp_wait_ready(struct platform_device *pdev, u64 timeout)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	if (dcp->crashed)
		return -ENODEV;
	if (dcp->active)
		return 0;
	if (timeout <= 0)
		return -ETIMEDOUT;

	ret = wait_for_completion_timeout(&dcp->start_done, timeout);
	if (ret < 0)
		return ret;

	if (dcp->crashed)
		return -ENODEV;

	return dcp->active ? 0 : -ETIMEDOUT;
}
EXPORT_SYMBOL(dcp_wait_ready);

static void dcp_work_register_backlight(struct work_struct *work)
{
	int ret;
	struct apple_dcp *dcp;

	dcp = container_of(work, struct apple_dcp, bl_register_wq);

	mutex_lock(&dcp->bl_register_mutex);
	if (dcp->brightness.bl_dev)
		goto out_unlock;

	/* try to register backlight device, */
	ret = dcp_backlight_register(dcp);
	if (ret) {
		dev_err(dcp->dev, "Unable to register backlight device\n");
		dcp->brightness.maximum = 0;
	}

out_unlock:
	mutex_unlock(&dcp->bl_register_mutex);
}

static struct platform_device *dcp_get_dev(struct device *dev, const char *name)
{
	struct platform_device *pdev;
	struct device_node *node = of_parse_phandle(dev->of_node, name, 0);

	if (!node)
		return NULL;

	pdev = of_find_device_by_node(node);
	of_node_put(node);
	return pdev;
}

static int dcp_get_disp_regs(struct apple_dcp *dcp)
{
	struct platform_device *pdev = to_platform_device(dcp->dev);
	int count = pdev->num_resources - 1;
	int i;

	if (count <= 0 || count > MAX_DISP_REGISTERS)
		return -EINVAL;

	for (i = 0; i < count; ++i) {
		dcp->disp_registers[i] =
			platform_get_resource(pdev, IORESOURCE_MEM, 1 + i);
	}

	dcp->nr_disp_registers = count;
	return 0;
}

#define DCP_FW_VERSION_MIN_LEN	3
#define DCP_FW_VERSION_MAX_LEN	5
#define DCP_FW_VERSION_STR_LEN	(DCP_FW_VERSION_MAX_LEN * 4)

static int dcp_read_fw_version(struct device *dev, const char *name,
			       char *version_str)
{
	u32 ver[DCP_FW_VERSION_MAX_LEN];
	int len_str;
	int len;

	len = of_property_read_variable_u32_array(dev->of_node, name, ver,
						  DCP_FW_VERSION_MIN_LEN,
						  DCP_FW_VERSION_MAX_LEN);

	switch (len) {
	case 3:
		len_str = scnprintf(version_str, DCP_FW_VERSION_STR_LEN,
				    "%d.%d.%d", ver[0], ver[1], ver[2]);
		break;
	case 4:
		len_str = scnprintf(version_str, DCP_FW_VERSION_STR_LEN,
				    "%d.%d.%d.%d", ver[0], ver[1], ver[2],
				    ver[3]);
		break;
	case 5:
		len_str = scnprintf(version_str, DCP_FW_VERSION_STR_LEN,
				    "%d.%d.%d.%d.%d", ver[0], ver[1], ver[2],
				    ver[3], ver[4]);
		break;
	default:
		len_str = strscpy(version_str, "UNKNOWN",
				  DCP_FW_VERSION_STR_LEN);
		if (len >= 0)
			len = -EOVERFLOW;
		break;
	}

	if (len_str >= DCP_FW_VERSION_STR_LEN)
		dev_warn(dev, "'%s' truncated: '%s'\n", name, version_str);

	return len;
}

static enum dcp_firmware_version dcp_check_firmware_version(struct device *dev)
{
	char compat_str[DCP_FW_VERSION_STR_LEN];
	char fw_str[DCP_FW_VERSION_STR_LEN];
	int ret;

	/* firmware version is just informative */
	dcp_read_fw_version(dev, "apple,firmware-version", fw_str);

	ret = dcp_read_fw_version(dev, "apple,firmware-compat", compat_str);
	if (ret < 0) {
		dev_err(dev, "Could not read 'apple,firmware-compat': %d\n", ret);
		return DCP_FIRMWARE_UNKNOWN;
	}

	if (strncmp(compat_str, "12.3.0", sizeof(compat_str)) == 0)
		return DCP_FIRMWARE_V_12_3;

	dev_err(dev, "DCP firmware-compat %s (FW: %s) is not supported\n",
		compat_str, fw_str);

	return DCP_FIRMWARE_UNKNOWN;
}

static int dcp_comp_bind(struct device *dev, struct device *main, void *data)
{
	struct device_node *panel_np;
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;

	dcp->coproc_reg = devm_platform_ioremap_resource_byname(to_platform_device(dev), "coproc");
	if (IS_ERR(dcp->coproc_reg))
		return PTR_ERR(dcp->coproc_reg);

	dcp->reset = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(dcp->reset)) {
		return dev_err_probe(dev, PTR_ERR(dcp->reset),
				     "Failed to get reset control");
	}

	if (!show_notch)
		ret = of_property_read_u32(dev->of_node, "apple,notch-height",
					   &dcp->notch_height);

	if (dcp->notch_height > MAX_NOTCH_HEIGHT)
		dcp->notch_height = MAX_NOTCH_HEIGHT;
	if (dcp->notch_height > 0)
		dev_info(dev, "Detected display with notch of %u pixel\n", dcp->notch_height);

	/* intialize brightness scale to a sensible default to avoid divide by 0*/
	dcp->brightness.scale = 65536;
	panel_np = of_get_compatible_child(dev->of_node, "apple,panel-mini-led");
	if (panel_np)
		dcp->has_mini_led = true;
	else
		panel_np = of_get_compatible_child(dev->of_node, "apple,panel");

	if (panel_np) {
		const char height_prop[2][16] = { "adj-height-mm", "height-mm" };

		if (of_device_is_available(panel_np)) {
			ret = of_property_read_u32(panel_np, "apple,max-brightness",
						   &dcp->brightness.maximum);
			if (ret)
				dev_err(dev, "Missing property 'apple,max-brightness'\n");
		}

		of_property_read_u32(panel_np, "width-mm", &dcp->width_mm);
		/* use adjusted height as long as the notch is hidden */
		of_property_read_u32(panel_np, height_prop[!dcp->notch_height],
				     &dcp->height_mm);

		of_node_put(panel_np);
		dcp->connector_type = DRM_MODE_CONNECTOR_eDP;
		INIT_WORK(&dcp->bl_register_wq, dcp_work_register_backlight);
		mutex_init(&dcp->bl_register_mutex);
	} else if (of_property_match_string(dev->of_node, "apple,connector-type", "HDMI-A") >= 0)
		dcp->connector_type = DRM_MODE_CONNECTOR_HDMIA;
	else if (of_property_match_string(dev->of_node, "apple,connector-type", "USB-C") >= 0)
		dcp->connector_type = DRM_MODE_CONNECTOR_USB;
	else
		dcp->connector_type = DRM_MODE_CONNECTOR_Unknown;

	/*
	 * Components do not ensure the bind order of sub components but
	 * the piodma device is only used for its iommu. The iommu is fully
	 * initialized by the time dcp_piodma_probe() calls component_add().
	 */
	dcp->piodma = dcp_get_dev(dev, "apple,piodma-mapper");
	if (!dcp->piodma) {
		dev_err(dev, "failed to find piodma\n");
		return -ENODEV;
	}

	ret = dcp_get_disp_regs(dcp);
	if (ret) {
		dev_err(dev, "failed to find display registers\n");
		return ret;
	}

	dcp->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(dcp->clk))
		return dev_err_probe(dev, PTR_ERR(dcp->clk),
				     "Unable to find clock\n");

	ret = of_property_read_u64(dev->of_node, "apple,asc-dram-mask",
				   &dcp->asc_dram_mask);
	if (ret)
		dev_warn(dev, "failed read 'apple,asc-dram-mask': %d\n", ret);
	dev_dbg(dev, "'apple,asc-dram-mask': 0x%011llx\n", dcp->asc_dram_mask);

	bitmap_zero(dcp->memdesc_map, DCP_MAX_MAPPINGS);
	// TDOD: mem_desc IDs start at 1, for simplicity just skip '0' entry
	set_bit(0, dcp->memdesc_map);

	INIT_WORK(&dcp->vblank_wq, dcp_delayed_vblank);

	dcp->swapped_out_fbs =
		(struct list_head)LIST_HEAD_INIT(dcp->swapped_out_fbs);

	dcp->rtk = devm_apple_rtkit_init(dev, dcp, "mbox", 0, &rtkit_ops);
	if (IS_ERR(dcp->rtk))
		return dev_err_probe(dev, PTR_ERR(dcp->rtk),
				     "Failed to intialize RTKit");

	/* calling pm_runtime_force_resume() is equivalent to pm_runtime_enable
	 * if pm_runtime_force_suspend was not previously called
	 */
	ret = pm_runtime_force_resume(dev);
	if (ret)
		dev_err_probe(dev, ret, "pm_runtime_force_resume failed!\n");

	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		devm_apple_rtkit_free(dev, dcp->rtk);
		dcp->rtk = NULL;
		return dev_err_probe(dev, ret, "pm_runtime_resume_and_get failed!\n");
	}

	pm_runtime_set_autosuspend_delay(dev, 50);
	pm_runtime_use_autosuspend(dev);

	/*
	 * On probe, DCP is probably in charge of the boot framebuffer.
	 * Don't suspend it until we get an explicit poweroff request.
	 */
	dcp->awake = true;

	/* We are now a client of the DCP/DISP0 genpd, so we can remove the always on flag */
	if (dev->pm_domain) {
		struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
		genpd->flags &= ~GENPD_FLAG_ALWAYS_ON;
	}
	return ret;
}

/*
 * We need to shutdown DCP before tearing down the display subsystem. Otherwise
 * the DCP will crash and briefly flash a green screen of death.
 */
static void dcp_comp_unbind(struct device *dev, struct device *main, void *data)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	int ret;

	/*
	 * Get device but limit resume to RTKit
	 */
	dcp->iomfb_started = false;
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		dev_err(dev, "pm_runtime_resume_and_get failed: %d\n", ret);
	pm_runtime_dont_use_autosuspend(dev);

	/*
	 * force_suspend and shutdown RTKit
	 */
	dcp->active = true;
	dcp->shutdown = true;

	ret = pm_runtime_force_suspend(dev);
	if (ret)
		dev_err(dev, "pm_runtime_put_sync_suspend failed: %d\n", ret);

	dcp->rtk = NULL;

	dcp->shutdown = false;
	dcp->active = false;

	platform_device_put(dcp->piodma);
	dcp->piodma = NULL;
	dcp->clk = NULL;
	dcp->coproc_reg = NULL;
	iomfb_stop_rtkit(dcp);
}

static const struct component_ops dcp_comp_ops = {
	.bind	= dcp_comp_bind,
	.unbind	= dcp_comp_unbind,
};

static int dcp_platform_probe(struct platform_device *pdev)
{
	enum dcp_firmware_version fw_compat;
	struct device *dev = &pdev->dev;
	struct apple_dcp *dcp;
	int ret;

	fw_compat = dcp_check_firmware_version(dev);
	if (fw_compat == DCP_FIRMWARE_UNKNOWN)
		return -ENODEV;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	dcp = devm_kzalloc(dev, sizeof(*dcp), GFP_KERNEL);
	if (!dcp)
		return -ENOMEM;

	dcp->fw_compat = fw_compat;
	dcp->dev = dev;

	platform_set_drvdata(pdev, dcp);

	/* force suspend for symmetry with dcp_comp_unbind */
	ret = pm_runtime_force_suspend(dev);
	if (ret)
		dev_err(dev, "pm_runtime_put_sync_suspend failed: %d\n", ret);

	return component_add(&pdev->dev, &dcp_comp_ops);
}

static int dcp_platform_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcp_comp_ops);

	return 0;
}

static void dcp_platform_shutdown(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcp_comp_ops);
}

static int __maybe_unused dcp_runtime_suspend(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	u32 cpu_ctrl;
	int ret;

	if (!dcp->rtk || !dcp->coproc_reg)
		return 0;

	dcp_sleep(dcp);

	if (dcp->shutdown)
		ret = apple_rtkit_shutdown(dcp->rtk);
	else
		ret = apple_rtkit_idle(dcp->rtk);
	if (ret) {
		dev_err(dev, "Failed to shut down RTKit: %d", ret);
		return ret;
	}

	cpu_ctrl =
		readl_relaxed(dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);
	writel_relaxed(cpu_ctrl & ~APPLE_DCP_COPROC_CPU_CONTROL_RUN,
		       dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);

	dev_info(dev, "Sleep\n");

	return ret;
}

static int __maybe_unused dcp_runtime_resume(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	u32 cpu_ctrl;
	int ret;

	if (!dcp->rtk || !dcp->coproc_reg)
		return -ENODEV;

	cpu_ctrl =
		readl_relaxed(dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);
	writel_relaxed(cpu_ctrl | APPLE_DCP_COPROC_CPU_CONTROL_RUN,
		       dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);

	ret = apple_rtkit_wake(dcp->rtk);
	if (ret == -ETIME) {
		dev_err(dev, "RTKit wakeup timed out, trying reset. Please do not report bugs");
		add_taint(TAINT_FIRMWARE_WORKAROUND, LOCKDEP_STILL_OK);

		ret = reset_control_assert(dcp->reset);
		if (ret)
			dev_warn(dev, "reset_control_assert failed: %d\n", ret);

		ret = reset_control_deassert(dcp->reset);
		if (ret)
			dev_warn(dev, "reset_control_deassert failed: %d\n", ret);

		ret = apple_rtkit_wake(dcp->rtk);
	}
	if (ret) {
		dev_err(dev, "Failed to wake up RTKit: %d", ret);
		return ret;
	}

	if (!dcp->iomfb_started)
		return ret;

	ret = apple_rtkit_start_ep(dcp->rtk, IOMFB_ENDPOINT);
	if (ret < 0)
		dev_err(dev, "Failed to start IOMFB endpoint after wake: %d", ret);

	dev_info(dev, "Woke up\n");

	return ret;
}

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,dcp" },
	{ .compatible = "apple,dcpext" },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

DEFINE_RUNTIME_DEV_PM_OPS(dcp_pm_ops, dcp_runtime_suspend, dcp_runtime_resume, NULL);

static struct platform_driver dcp_driver = {
	.probe		= dcp_platform_probe,
	.remove		= dcp_platform_remove,
	.shutdown	= dcp_platform_shutdown,
	.driver	= {
		.name = "apple-dcp",
		.pm = pm_ptr(&dcp_pm_ops),
		.of_match_table	= of_match,
	},
};

drm_module_platform_driver(dcp_driver);

MODULE_AUTHOR("Alyssa Rosenzweig <alyssa@rosenzweig.io>");
MODULE_DESCRIPTION("Apple Display Controller DRM driver");
MODULE_LICENSE("Dual MIT/GPL");
