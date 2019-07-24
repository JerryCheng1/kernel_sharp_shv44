/* include/sharp/shaudio_wcd934x_notifier.h  (Shaudio wcd934x Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
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
#ifndef SHAUDIO_WCD934X_NOTIFIER_H
#define SHAUDIO_WCD934X_NOTIFIER_H

enum wcd934x_notifier_cmd {

  WCD934X_NOTIFIER_MSM_HEADSET_HP_STATE = 1,
  WCD934X_NOTIFIER_MSM_HEADSET_BU_STATE,
  WCD934X_NOTIFIER_DIAG_CODEC_SET_BIAS_MODE,

};

#ifdef CONFIG_SH_AUDIO_DRIVER /* B-003 */
int register_sm8150_a2dp_snd_notifier(struct notifier_block *nb);
int unregister_sm8150_a2dp_snd_notifier(struct notifier_block *nb);
int snd_sm8150_proxy_msm_set_a2dp_mode(int mode);
int snd_sm8150_proxy_msm_get_is_music_play(void);

enum sm8150_notifier_cmd_ex {
        SM8150_NOTIFIER_MSM_GET_IS_MUSIC_PLAY = WCD934X_NOTIFIER_DIAG_CODEC_SET_BIAS_MODE + 1,
        SM8150_NOTIFIER_MSM_SET_A2DP_MODE,

};
#endif /* CONFIG_SH_AUDIO_DRIVER */ /* B-003 */
#endif /* SHAUDIO_WCD934X_NOTIFIER_H */
