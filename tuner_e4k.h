#ifndef _E4K_TUNER_H
#define _E4K_TUNER_H

/*
 * Elonics E4000 tuner driver
 *
 * (C) 2011-2012 by Harald Welte <laforge@gnumonks.org>
 * (C) 2012 by Sylvain Munaut <tnt@246tNt.com>
 * (C) 2012 by Hoernchen <la@tfc-server.de>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define E4K_I2C_ADDR	0xc8
#define E4K_CHECK_ADDR	0x02
#define E4K_CHECK_VAL	0x40

#define E4K_REG_FILT2 0x11
#define E4K_REG_FILT3 0x12

#define E4K_MASTER1_RESET	(1 << 0)
#define E4K_MASTER1_NORM_STBY	(1 << 1)
#define E4K_MASTER1_POR_DET	(1 << 2)

#define E4K_SYNTH1_PLL_LOCK	(1 << 0)
#define E4K_SYNTH1_BAND_SHIF	1

#define E4K_SYNTH7_3PHASE_EN	(1 << 3)

#define E4K_SYNTH8_VCOCAL_UPD	(1 << 2)

#define E4K_FILT3_DISABLE	(1 << 5)

#define E4K_AGC1_LIN_MODE	(1 << 4)
#define E4K_AGC1_LNA_UPDATE	(1 << 5)
#define E4K_AGC1_LNA_G_LOW	(1 << 6)
#define E4K_AGC1_LNA_G_HIGH	(1 << 7)

#define E4K_AGC6_LNA_CAL_REQ	(1 << 4)

#define E4K_AGC7_MIX_GAIN_AUTO	(1 << 0)
#define E4K_AGC7_GAIN_STEP_5dB	(1 << 5)

#define E4K_DC1_CAL_REQ		(1 << 0)

#define E4K_DC5_I_LUT_EN	(1 << 0)
#define E4K_DC5_Q_LUT_EN	(1 << 1)
#define E4K_DC5_RANGE_DET_EN	(1 << 2)
#define E4K_DC5_RANGE_EN	(1 << 3)
#define E4K_DC5_TIMEVAR_EN	(1 << 4)

#define E4K_CLKOUT_DISABLE	0x96

#define E4K_CHFCALIB_CMD	(1 << 0)

#define E4K_AGC1_MOD_MASK	0xF

enum e4k_agc_mode {
	E4K_AGC_MOD_SERIAL		= 0x0,
	E4K_AGC_MOD_IF_SERIAL_LNA_PWM	= 0x4,
	E4K_AGC_MOD_IF_SERIAL_LNA_AUTON	= 0x9,
	E4K_AGC_MOD_IF_SERIAL_LNA_SUPERV = 0xa,
};

#define E4K_BAND_VHF2 0
#define E4K_BAND_VHF3 1
#define E4K_BAND_UHF  2
#define E4K_BAND_L    3

#define E4K_IF_FILTER_MIX 0
#define E4K_IF_FILTER_RC 1
#define E4K_IF_FILTER_CHAN 2

struct e4k_state {
	uint8_t band;
	uint32_t fosc;
    uint8_t fil_cal_code;
	void *rtl_dev;
};

int e4k_init(struct e4k_state *e4k);
int e4k_standby(struct e4k_state *e4k, int enable);
int e4k_commonmode_set(struct e4k_state *e4k, uint8_t value);
uint32_t e4k_tune_freq(struct e4k_state *e4k, uint32_t freq);
int e4k_if_filter_bw_get(struct e4k_state *e4k, uint8_t filter);
int e4k_if_filter_bw_set(struct e4k_state *e4k, uint8_t filter, uint32_t bandwidth);
int e4k_if_filter_chan_enable(struct e4k_state *e4k, unsigned int on);
int e4k_manual_dc_offset(struct e4k_state *e4k, uint8_t iofs, uint8_t irange, uint8_t qofs, uint8_t qrange);
int e4k_dc_offset_calibrate(struct e4k_state *e4k, uint8_t *offs_i, uint8_t *offs_q, uint8_t *offs_range);

int e4k_enable_manual_gain(struct e4k_state *e4k, uint8_t manual);
int e4k_set_agc_params(struct e4k_state *e4k, uint16_t lna_agc, uint8_t mixer_agc, uint8_t agc_rate);
int e4k_set_lna_gain(struct e4k_state *e4k, uint32_t gain);
int e4k_set_mixer_gain(struct e4k_state *e4k, uint8_t value);

#endif /* _E4K_TUNER_H */
