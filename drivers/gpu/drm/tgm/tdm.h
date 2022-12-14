/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 * Authors:
 *	Eunchul Kim <chulspro.kim@samsung.com>
 *	JinYoung Jeon <jy0.jeon@samsung.com>
 *	Taeheon Kim <th908.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _TDM_H_
#define _TDM_H_

#include <tgm_drv.h>
#include <drm/tgm_drm.h>
#ifdef CONFIG_DRM_TDM_PP
#include <tdm_pp.h>
#endif

#ifdef CONFIG_DRM_DMA_SYNC
#include <drm/drm_sync_helper.h>
#endif

enum tdm_notifier {
	TDM_NOTI_REGISTER,
#ifdef CONFIG_DRM_TDM_DPMS_CTRL
	TDM_NOTI_DPMS_CTRL,
#endif
	TDM_NOTI_VSYNC_CTRL,
	TDM_NOTI_MAX,
};

struct tdm_nb_event {
	void *data;
};

struct tdm_private {
	int num_crtcs;
	struct drm_device *drm_dev;
	struct hrtimer virtual_vblank;
	ktime_t	virtual_itv;
	ktime_t	virtual_rm;
#ifdef CONFIG_DRM_DMA_SYNC
	unsigned fence_context;
	atomic_t fence_seqno;
#endif
#ifdef CONFIG_DRM_TDM_DPMS_CTRL
	struct tdm_dpms_work	*dpms_work;
	struct completion	dpms_comp;
	struct mutex	dpms_lock;
	int	dpms[TDM_CRTC_MAX];
#endif
#ifdef CONFIG_DISPLAY_EARLY_DPMS
	struct notifier_block	nb_dpms_ctrl;
	bool	early_dpms;
	bool	early_dpms_force;
#endif
	u32	dbg_cnt;
	bool	dpms_working;
	u32     vblank_count;
};

#ifdef CONFIG_DRM_TDM_DPMS_CTRL
struct tdm_send_dpms_event {
	struct drm_pending_event	base;
	struct tdm_control_dpms_event	event;
};

struct tdm_dpms_work {
	struct work_struct	work;
	struct tdm_send_dpms_event	*event;
	struct tdm_private *tdm_priv;
#ifdef CONFIG_DISPLAY_EARLY_DPMS
	bool early_dpms;
#endif
};

#ifdef CONFIG_DRM_TDM_FRAME_DONE
struct tdm_frame_done_event {
	struct drm_event	base;
	enum tdm_crtc_id	crtc_id;
	__u32	num;
	__u32	user_data;
};
#endif

void tdm_dpms_work_ops(struct work_struct *work);
#endif

#ifdef CONFIG_DRM_TDM_FRAME_DONE
struct tdm_send_frame_done_event {
	struct drm_pending_event	base;
	struct tdm_frame_done_event	event;
};
#endif

int tdm_nb_register(struct notifier_block *nb);
int tdm_nb_unregister(struct notifier_block *nb);
int tdm_nb_send_event(unsigned long val, void *v);
#ifdef CONFIG_DRM_DMA_SYNC
struct fence *tdm_fence(void *fence_dev, struct dma_buf *buf);
int tdm_fence_signal(void *fence_dev, struct fence *fence);
#endif
#ifdef CONFIG_DRM_TDM_DPMS_CTRL
int tdm_dpms_ioctl(struct drm_device *dev, void *data, struct drm_file *file);
#endif

#ifdef CONFIG_DRM_TDM_FRAME_DONE
int tdm_set_frame_done(struct drm_device *dev, void *data, struct drm_file *file);
int tdm_send_frame_done(struct drm_device *dev, int req_num);
#endif
extern int tdm_get_irq(void);
extern void tdm_set_irq(int irq);
extern int tdm_init(struct drm_device *drm_dev);
extern void tdm_register_reset_block(void (*callback)(void));
#endif /* _TDM_H_ */
