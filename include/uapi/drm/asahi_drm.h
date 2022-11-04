/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on asahi_drm.h which is
 *
 * Copyright © 2014-2018 Broadcom
 * Copyright © 2019 Collabora ltd.
 */
#ifndef _ASAHI_DRM_H_
#define _ASAHI_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define DRM_ASAHI_GET_PARAM			0x00
#define DRM_ASAHI_SUBMIT			0x01
#define DRM_ASAHI_WAIT				0x02
#define DRM_ASAHI_CREATE_BO			0x03
#define DRM_ASAHI_MMAP_BO			0x04
#define DRM_ASAHI_GET_BO_OFFSET			0x05

enum drm_asahi_param {
	// UAPI related
	DRM_ASAHI_PARAM_UNSTABLE_UABI_VERSION,

	// GPU identification
	DRM_ASAHI_PARAM_GPU_GENERATION,
	DRM_ASAHI_PARAM_GPU_VARIANT,
	DRM_ASAHI_PARAM_GPU_REVISION,
	DRM_ASAHI_PARAM_CHIP_ID,

	// GPU features
	DRM_ASAHI_PARAM_FEAT_COMPAT,
	DRM_ASAHI_PARAM_FEAT_INCOMPAT,
};

enum drm_asahi_generation {
	DRM_ASAHI_GENERATION_G13 = 13,
	DRM_ASAHI_GENERATION_G14 = 14,
};

enum drm_asahi_variant {
	DRM_ASAHI_VARIANT_P = 'P',
	DRM_ASAHI_VARIANT_G = 'G',
	DRM_ASAHI_VARIANT_S = 'S',
	DRM_ASAHI_VARIANT_C = 'C',
	DRM_ASAHI_VARIANT_D = 'D',
};

enum drm_asahi_revision {
	DRM_ASAHI_REV_A0 = 0x00,
	DRM_ASAHI_REV_A1 = 0x01,
	DRM_ASAHI_REV_B0 = 0x10,
	DRM_ASAHI_REV_B1 = 0x11,
	DRM_ASAHI_REV_C0 = 0x20,
	DRM_ASAHI_REV_C1 = 0x21,
};

/*
enum drm_asahi_feat_compat {
};
*/

enum drm_asahi_feat_incompat {
	DRM_ASAHI_FEAT_MANDATORY_ZS_COMPRESSION = (1UL) << 0,
};

struct drm_asahi_get_param {
	__u32 param;
	__u32 pad;
	__u64 value;
};

#define ASAHI_MAX_ATTACHMENTS 16

#define ASAHI_ATTACHMENT_C    0
#define ASAHI_ATTACHMENT_Z    1
#define ASAHI_ATTACHMENT_S    2

struct drm_asahi_attachment {
   __u32 type;
   __u32 size;
   __u64 pointer;
};

#define ASAHI_CMDBUF_NO_CLEAR_PIPELINE_TEXTURES (1UL << 0)
#define ASAHI_CMDBUF_SET_WHEN_RELOADING_Z_OR_S (1UL << 1)

struct drm_asahi_cmdbuf {
   __u64 flags;

   __u64 encoder_ptr;
   __u32 encoder_id;

   __u32 cmd_ta_id;
   __u32 cmd_3d_id;

   __u32 ppp_ctrl;
   __u64 zls_ctrl;

   __u64 depth_buffer_1;
   __u64 depth_buffer_2;
   __u64 depth_buffer_3;

   __u64 stencil_buffer_1;
   __u64 stencil_buffer_2;
   __u64 stencil_buffer_3;

   __u64 scissor_array;
   __u64 depth_bias_array;

   __u32 fb_width;
   __u32 fb_height;

   __u32 load_pipeline;
   __u32 load_pipeline_bind;

   __u32 store_pipeline;
   __u32 store_pipeline_bind;

   __u32 partial_reload_pipeline;
   __u32 partial_reload_pipeline_bind;

   __u32 partial_store_pipeline;
   __u32 partial_store_pipeline_bind;

   __u32 depth_dimensions;
   __u32 depth_clear_value;
   __u8 stencil_clear_value;
   __u8 pad2[3];

   struct drm_asahi_attachment attachments[ASAHI_MAX_ATTACHMENTS];
   __u32 attachment_count;
};

/**
 * struct drm_asahi_submit - ioctl argument for submitting commands to the 3D
 * engine.
 *
 * This asks the kernel to have the GPU execute a render command list.
 *
 * TODO: Make explicit sync from the start.
 */
struct drm_asahi_submit {
   /** User pointer to macOS-style command buffer with 12.3 ABI, TODO: this is
    * completely inappropriate for upstream.
    */
	__u64 cmdbuf;

	/** An optional array of sync objects to wait on before starting this job. */
	__u64 in_syncs;

	/** Number of sync objects to wait on before starting this job. */
	__u32 in_sync_count;

	/** An optional sync object to place the completion fence in. */
	__u32 out_sync;
};


/**
 * struct drm_asahi_wait - ioctl argument for waiting.
 */
struct drm_asahi_wait {
};

#define ASAHI_BO_PIPELINE	1

/**
 * struct drm_asahi_create_bo - ioctl argument for creating Panfrost BOs.
 *
 * There are currently no values for the flags argument, but it may be
 * used in a future extension.
 */
struct drm_asahi_create_bo {
	__u32 size;
	__u32 flags;
	/** Returned GEM handle for the BO. */
	__u32 handle;
	/* Pad, must be zero-filled. */
	__u32 pad;
	/**
	 * Returned offset for the BO in the GPU address space.  This offset
	 * is private to the DRM fd and is valid for the lifetime of the GEM
	 * handle.
	 *
	 * This offset value will always be nonzero, since various HW
	 * units treat 0 specially.
	 */
	__u64 offset;
};

/**
 * struct drm_asahi_mmap_bo - ioctl argument for mapping Panfrost BOs.
 *
 * This doesn't actually perform an mmap.  Instead, it returns the
 * offset you need to use in an mmap on the DRM device node.  This
 * means that tools like valgrind end up knowing about the mapped
 * memory.
 *
 * There are currently no values for the flags argument, but it may be
 * used in a future extension.
 */
struct drm_asahi_mmap_bo {
	/** Handle for the object being mapped. */
	__u32 handle;
	__u32 flags;
	/** offset into the drm node to use for subsequent mmap call. */
	__u64 offset;
};

/**
 * Returns the offset for the BO in the GPU address space for this DRM fd.
 * This is the same value returned by drm_asahi_create_bo, if that was called
 * from this DRM fd.
 */
struct drm_asahi_get_bo_offset {
	__u32 handle;
	__u32 pad;
	__u64 offset;
};

/* Note: this is an enum so that it can be resolved by Rust bindgen. */
enum {
   DRM_IOCTL_ASAHI_GET_PARAM        = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_GET_PARAM, struct drm_asahi_get_param),
   DRM_IOCTL_ASAHI_SUBMIT           = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_SUBMIT, struct drm_asahi_submit),
   DRM_IOCTL_ASAHI_WAIT             = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_WAIT, struct drm_asahi_wait),
   DRM_IOCTL_ASAHI_CREATE_BO        = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_CREATE_BO, struct drm_asahi_create_bo),
   DRM_IOCTL_ASAHI_MMAP_BO          = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_MMAP_BO, struct drm_asahi_mmap_bo),
   DRM_IOCTL_ASAHI_GET_BO_OFFSET    = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_GET_BO_OFFSET, struct drm_asahi_get_bo_offset),
};

#if defined(__cplusplus)
}
#endif

#endif /* _ASAHI_DRM_H_ */
