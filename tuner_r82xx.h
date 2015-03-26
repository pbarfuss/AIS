/*
 * Rafael Micro R820T/R828D driver
 *
 * Copyright (C) 2013 Mauro Carvalho Chehab <mchehab@redhat.com>
 * Copyright (C) 2013 Steve Markgraf <steve@steve-m.de>
 *
 * This driver is a heavily modified version of the driver found in the
 * Linux kernel:
 * http://git.linuxtv.org/linux-2.6.git/history/HEAD:/drivers/media/tuners/r820t.c
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
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

#ifndef R82XX_H
#define R82XX_H

#define R820T_I2C_ADDR		0x34
#define R828D_I2C_ADDR		0x74
#define R82XX_MAX_I2C_MSG_LEN 8
#define R828D_XTAL_FREQ		16000000

#define R82XX_CHECK_ADDR	0x00
#define R82XX_CHECK_VAL		0x69

#define R82XX_IF_FREQ           3570000
#define R82XX_DEFAULT_IF_BW     2000

#define REG_SHADOW_START	5
#define NUM_REGS		30
#define VER_NUM			49

#define CHIP_R820T 0x00
#define CHIP_R828D 0x40

enum r82xx_xtal_cap_value {
	XTAL_LOW_CAP_30P = 0,
	XTAL_LOW_CAP_20P,
	XTAL_LOW_CAP_10P,
	XTAL_LOW_CAP_0P,
	XTAL_HIGH_CAP_0P
};

struct r82xx_priv {
	uint32_t xtal;
	uint8_t rafael_chip;

	uint8_t				regs[NUM_REGS];
	uint8_t				input;
    uint8_t             disable_dither;
	uint8_t             reg_cache;

	void *rtl_dev;
};

struct r82xx_freq_range {
	uint32_t	freq;
	uint8_t		rf_mux_ploy;
	uint8_t		tf_c;
};

int r82xx_standby(struct r82xx_priv *priv);
int r82xx_init(struct r82xx_priv *priv);
int r82xx_set_freq(struct r82xx_priv *priv, uint32_t freq);
int r82xx_enable_manual_gain(struct r82xx_priv *priv, int set_manual_gain);
int r82xx_set_agc_params(struct r82xx_priv *priv, uint8_t lna_agc, uint8_t mixer_agc, uint8_t agc_rate);
int r82xx_set_gain(struct r82xx_priv *priv, int gain);
int r82xx_set_bw(struct r82xx_priv *priv, uint32_t bw);
int r82xx_set_dither(struct r82xx_priv *priv, int dither);

#endif