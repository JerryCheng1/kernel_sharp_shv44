/* drivers/gpu/drm/msm/sharp/drm_diag_raven.h  (Display Driver)
 *
 * Copyright (C) 2017 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef DRM_DIAG_RAVEN_H
#define DRM_DIAG_RAVEN_H

#ifdef DRM_DIAG_PANEL_GMM_VOLTAGE
char drm_diag_raven_gmm_data[DRM_RAVEN_GMM_SIZE + 1];

// this ary is page, addr, payload
struct drm_diag_panel_pad drm_diag_raven_gmm[] = {
	{0x00, 0xC8, 61, &drm_diag_raven_gmm_data[0]},
};

char drm_diag_raven_volt_data_B7[6];
char drm_diag_raven_volt_data_BB[6];
char drm_diag_raven_volt_data_CF[8];
struct drm_diag_panel_pad drm_diag_raven_volt[] = {
	{0x00, 0xB7, 6, &drm_diag_raven_volt_data_B7[0]},
	{0x00, 0xBB, 6, &drm_diag_raven_volt_data_BB[0]},
	{0x00, 0xCF, 8, &drm_diag_raven_volt_data_CF[0]},
};

const struct drm_diag_pad_item drm_diag_raven_gmm_volt_pads[] = {
	{drm_diag_raven_volt,   ARRAY_SIZE(drm_diag_raven_volt)},
	{drm_diag_raven_gmm,  ARRAY_SIZE(drm_diag_raven_gmm)},
};
#endif /* DRM_DIAG_PANEL_GMM_VOLTAGE */

#endif /* DRM_DIAG_RAVEN_H */