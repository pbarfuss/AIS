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

#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "tuner_e4k.h"
#include "rtlsdr_i2c.h"

/* structure describing a field in a register */
struct reg_field {
    uint8_t reg;
    uint8_t shift;
    uint8_t width;
};

/* If this is defined, the limits are somewhat relaxed compared to what the
 * vendor claims is possible */
#define OUT_OF_SPEC

uint32_t unsigned_delta(uint32_t a, uint32_t b)
{
	if (a > b)
		return a - b;
	else
		return b - a;
}

/***********************************************************************
 * Register Access */

/*! \brief Write a register of the tuner chip
 *  \param[in] e4k reference to the tuner
 *  \param[in] reg number of the register
 *  \param[in] val value to be written
 *  \returns 0 on success, negative in case of error
 */
static int e4k_reg_write(struct e4k_state *e4k, uint8_t reg, uint8_t val)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = val;

	return rtlsdr_i2c_write_fn(e4k->rtl_dev, e4k->i2c_addr, data, 2);
}

/*! \brief Read a register of the tuner chip
 *  \param[in] e4k reference to the tuner
 *  \param[in] reg number of the register
 *  \returns positive 8bit register contents on success, negative in case of error
 */
static int e4k_reg_read(struct e4k_state *e4k, uint8_t reg)
{
	uint8_t data = reg;

	if (rtlsdr_i2c_write_fn(e4k->rtl_dev, e4k->i2c_addr, &data, 1) < 1)
		return -1;

	if (rtlsdr_i2c_read_fn(e4k->rtl_dev, e4k->i2c_addr, &data, 1) < 1)
		return -1;

	return data;
}

/*! \brief Set or clear some (masked) bits inside a register
 *  \param[in] e4k reference to the tuner
 *  \param[in] reg number of the register
 *  \param[in] mask bit-mask of the value
 *  \param[in] val data value to be written to register
 *  \returns 0 on success, negative in case of error
 */
static int e4k_reg_set_mask(struct e4k_state *e4k, uint8_t reg, uint8_t mask, uint8_t val)
{
	uint8_t tmp = e4k_reg_read(e4k, reg);

	if ((tmp & mask) == val)
		return 0;

	return e4k_reg_write(e4k, reg, (tmp & ~mask) | (val & mask));
}

/***********************************************************************
 * Filter Control */

static const uint32_t rf_filt_center_uhf[] = {
	MHZ(360), MHZ(380), MHZ(405), MHZ(425),
	MHZ(450), MHZ(475), MHZ(505), MHZ(540),
	MHZ(575), MHZ(615), MHZ(670), MHZ(720),
	MHZ(760), MHZ(840), MHZ(890), MHZ(970)
};

static const uint32_t rf_filt_center_l[] = {
	MHZ(1300), MHZ(1320), MHZ(1360), MHZ(1410),
	MHZ(1445), MHZ(1460), MHZ(1490), MHZ(1530),
	MHZ(1560), MHZ(1590), MHZ(1640), MHZ(1660),
	MHZ(1680), MHZ(1700), MHZ(1720), MHZ(1750)
};

static int closest_arr_idx(const uint32_t *arr, unsigned int arr_size, uint32_t freq)
{
	unsigned int i, bi = 0;
	uint32_t best_delta = 0xffffffff;

	/* iterate over the array containing a list of the center
	 * frequencies, selecting the closest one */
	for (i = 0; i < arr_size; i++) {
		uint32_t delta = unsigned_delta(freq, arr[i]);
		if (delta < best_delta) {
			best_delta = delta;
			bi = i;
		}
	}

	return bi;
}

/* Mixer Filter */
static uint32_t mix_filter_bw[] = {
	27000, 27000, 27000, 27000, 27000, 27000, 27000, 27000,
	4600, 4200, 3800, 3400, 3300, 2700, 2300, 1900
};

/* IF RC Filter */
static uint32_t ifrc_filter_bw[] = {
	21400, 21000, 17600, 14700, 12400, 10600, 9000, 7700,
	6400, 5300, 4400, 3400, 2600, 1800, 1200, 1000
};

/*! \brief Set the filter band-width of any of the IF filters
 *  \param[in] e4k reference to the tuner chip
 *  \param[in] filter filter to be configured
 *  \param[in] bandwidth bandwidth to be configured
 *  \returns positive actual filter band-width, negative in case of error
 */
int e4k_if_filter_bw_set(struct e4k_state *e4k, uint8_t filter, uint32_t bandwidth)
{
	int bw_idx;
    uint32_t *if_filter_bw_tab;
    uint8_t shift;

    if (filter > E4K_IF_FILTER_RC)
        return -1;

    if_filter_bw_tab = ((filter == E4K_IF_FILTER_RC) ? ifrc_filter_bw : mix_filter_bw);
    shift = ((filter == E4K_IF_FILTER_RC) ? 0 : 4);
	bw_idx = closest_arr_idx(if_filter_bw_tab, 16, bandwidth);
    return e4k_reg_set_mask(e4k, E4K_REG_FILT2, 0x0f << shift, bw_idx << shift);
}

int e4k_if_filter_bw_get(struct e4k_state *e4k, uint8_t filter)
{
	int rc;

    if (filter > E4K_IF_FILTER_RC)
        return -1;

    rc = e4k_reg_read(e4k, E4K_REG_FILT2);
    if (rc < 0)
        return rc;

    if (filter == E4K_IF_FILTER_RC) {
        return ifrc_filter_bw[rc & 0x0f];
    } else if (filter == E4K_IF_FILTER_MIX) {
        return mix_filter_bw[(rc >> 4) & 0x0f];
    } else {
        return -1;
    }
}


/***********************************************************************
 * Frequency Control */

#define E4K_PLL_Y		65536

#ifdef OUT_OF_SPEC
#define E4K_FVCO_MIN_KHZ	2400000UL /* 2.4 GHz; min FLO is 2400/48 = 50MHz */
#define E4K_FVCO_MAX_KHZ	4400000UL /* 4.4 GHz; max FLO is 4400/2  = 2200MHz  */
#else
/* NB: Datasheet values for RF input and LO ranges are 64 - 1700MHz.
 * The values below are from the slightly wider VCO ranges.
 */
#define E4K_FVCO_MIN_KHZ	2600000UL /* 2.6 GHz; min FLO is 2600/48 = 54MHz */
#define E4K_FVCO_MAX_KHZ	3900000UL /* 3.9 GHz; max FLO is 3900/2 = 1950MHz */
#endif

struct pll_settings {
	uint32_t freq;
	uint8_t reg_synth7;
	uint8_t mult;
};

static const struct pll_settings pll_vars[] = {
	{KHZ(72400),	(1 << 3) | 7,	48},
	{KHZ(81200),	(1 << 3) | 6,	40},
	{KHZ(108300),	(1 << 3) | 5,	32},
	{KHZ(162500),	(1 << 3) | 4,	24},
	{KHZ(216600),	(1 << 3) | 3,	16},
	{KHZ(325000),	(1 << 3) | 2,	12},
	{KHZ(350000),	(1 << 3) | 1,	8},
	{KHZ(432000),	(0 << 3) | 3,	8},
	{KHZ(667000),	(0 << 3) | 2,	6},
	{KHZ(1200000),	(0 << 3) | 1,	4}
};

static int is_fvco_valid(uint32_t fvco_z)
{
	/* check if the resulting fosc is valid */
	if (fvco_z/1000 < E4K_FVCO_MIN_KHZ ||
	    fvco_z/1000 > E4K_FVCO_MAX_KHZ) {
		rtlsdr_printf("[E4K] Fvco %u invalid\n", fvco_z);
		return 0;
	}

	return 1;
}

static int is_fosc_valid(uint32_t fosc)
{
	if (fosc < MHZ(16) || fosc > MHZ(30)) {
		rtlsdr_printf("[E4K] Fosc %u invalid\n", fosc);
		return 0;
	}

	return 1;
}

/*! \brief Determine if 3-phase mixing shall be used or not */
static int use_3ph_mixing(uint32_t flo)
{
	/* this is a magic number somewhre between VHF and UHF */
	if (flo < MHZ(350))
		return 1;

	return 0;
}

/* \brief compute Fvco based on Fosc, Z and X
 * \returns positive value (Fvco in Hz), 0 in case of error */
static int e4k_band_set(struct e4k_state *e4k, enum e4k_band band)
{
	int rc;

	switch (band) {
	case E4K_BAND_VHF2:
	case E4K_BAND_VHF3:
	case E4K_BAND_UHF:
		e4k_reg_write(e4k, E4K_REG_BIAS, 3);
		break;
	case E4K_BAND_L:
		e4k_reg_write(e4k, E4K_REG_BIAS, 0);
		break;
	}

	/* workaround: if we don't reset this register before writing to it,
	 * we get a gap between 325-350 MHz */
	rc = e4k_reg_set_mask(e4k, E4K_REG_SYNTH1, 0x06, 0);
	rc = e4k_reg_set_mask(e4k, E4K_REG_SYNTH1, 0x06, band << 1);
	if (rc >= 0)
		e4k->band = band;

	return rc;
}

/*! \brief Compute PLL parameters for givent target frequency
 *  \param[in] fosc Clock input frequency applied to the chip (Hz)
 *  \param[in] intended_flo target tuning frequency (Hz)
 *  \returns actual PLL frequency, as close as possible to intended_flo,
 *	     0 in case of error
 */
uint32_t e4k_compute_pll_params(struct e4k_state *e4k, uint32_t fosc, uint32_t intended_flo)
{
	uint32_t i;
	uint8_t r = 2, r_idx = 0;
	uint64_t intended_fvco, remainder;
	uint64_t z = 0;
	uint32_t x;
    uint32_t rf_filt = 0;

	if (!is_fosc_valid(fosc))
		return 0;

	for(i = 0; i < ARRAY_SIZE(pll_vars); ++i) {
		if(intended_flo < pll_vars[i].freq) {
			//three_phase_mixing = (pll_vars[i].reg_synth7 & 0x08) ? 1 : 0;
			r_idx = pll_vars[i].reg_synth7;
			r = pll_vars[i].mult;
			break;
		}
	}

	rtlsdr_printf("[E4K] Fint=%u, R=%u\n", intended_flo, r);

	/* flo(max) = 1700MHz, R(max) = 48, we need 64bit! */
	intended_fvco = (uint64_t)intended_flo * r;
	if (intended_fvco < KHZ(E4K_FVCO_MIN_KHZ)) {
		intended_fvco = KHZ(E4K_FVCO_MIN_KHZ);
	} else if (intended_fvco > KHZ(E4K_FVCO_MAX_KHZ)) {
		intended_fvco = KHZ(E4K_FVCO_MAX_KHZ);
	}

	/* compute integral component of multiplier */
	z = intended_fvco / fosc;

	/* compute fractional part.  this will not overflow,
	* as fosc(max) = 30MHz and z(max) = 255 */
	remainder = intended_fvco - (fosc * z);
	/* remainder(max) = 30MHz, E4K_PLL_Y = 65536 -> 64bit! */
	x = (remainder * E4K_PLL_Y) / fosc;
	/* x(max) as result of this computation is 65536 */

	/* program R + 3phase/2phase */
	e4k_reg_write(e4k, E4K_REG_SYNTH7, r_idx);
	/* program Z */
	e4k_reg_write(e4k, E4K_REG_SYNTH3, z);
	/* program X */
	e4k_reg_write(e4k, E4K_REG_SYNTH4, x & 0xff);
	e4k_reg_write(e4k, E4K_REG_SYNTH5, x >> 8);

	/* set the band */
	if (intended_flo < MHZ(140))
		e4k_band_set(e4k, E4K_BAND_VHF2);
	else if (intended_flo < MHZ(350))
		e4k_band_set(e4k, E4K_BAND_VHF3);
	else if (intended_flo < MHZ(1135))
		e4k_band_set(e4k, E4K_BAND_UHF);
	else
		e4k_band_set(e4k, E4K_BAND_L);

	/* select and set proper RF filter */
    if (e4k->band == E4K_BAND_UHF) {
        rf_filt = closest_arr_idx(rf_filt_center_uhf, 16, intended_flo);
    } else if (e4k->band == E4K_BAND_L) {
        rf_filt = closest_arr_idx(rf_filt_center_l, 16, intended_flo);
    }
	e4k_reg_set_mask(e4k, E4K_REG_FILT1, 0xF, rf_filt);

    return intended_flo;
}

/*! \brief High-level tuning API, just specify frquency
 *
 *  This function will compute matching PLL parameters, program them into the
 *  hardware and set the band as well as RF filter.
 *
 *  \param[in] e4k reference to tuner
 *  \param[in] freq frequency in Hz
 *  \param[out] lo_freq if non-NULL, set to actually tuned frequency in Hz
 *  \returns zero on success, negative on error
 */
int e4k_tune_freq(struct e4k_state *e4k, uint32_t freq)
{
	uint32_t rc;

	/* determine PLL parameters */
	/* actually tune to those parameters */
	rc = e4k_compute_pll_params(e4k, e4k->fosc, freq);
	if (!rc)
		return -1;

	/* check PLL lock */
	rc = e4k_reg_read(e4k, E4K_REG_SYNTH1);
	if (!(rc & 0x01)) {
		rtlsdr_printf("[E4K] PLL not locked for %u Hz!\n", freq);
		return -1;
	}

	return 0;
}

/***********************************************************************
 * Gain Control */

/*
 * According to the datasheet, 00x0 maps to -5dB and 00x1 maps to -2.5dB
 * We use 0010 and 0011 instead of 0000 and 0001 respectively here,
 * to make the mapping from gain to index a simple affine shift,
 * and thus, get rid of a lookup table.
 * Also, everywhere in this code we add the +5dB base mixer gain
 * (the datasheet says 4dB but 5dB better matches every dongle I've tested,
 *  and makes a lot more sense here, actually)
 * to the LNA gain, thus making the LNA gain strictly positive and both
 * LNA and mixer gain start at 0.
 */

int e4k_set_lna_gain(struct e4k_state *e4k, uint32_t gain)
{
    uint32_t i;
    for(i = 0; i < 13; ++i) {
        uint32_t lna_gain = 5*i;
        if(lna_gain >= gain) {
            return e4k_reg_set_mask(e4k, 0x14, 0xf, i+2);
        }
    }
    return -1;
}

int e4k_set_mixer_gain(struct e4k_state *e4k, uint8_t value)
{
    uint8_t bit = ((value > 0) ? 1 : 0);
    return e4k_reg_set_mask(e4k, 0x15, 1, bit);
}

int e4k_enable_manual_gain(struct e4k_state *e4k, uint8_t manual)
{
	if (manual) {
		/* Set LNA mode to manual */
		e4k_reg_set_mask(e4k, E4K_REG_AGC1, E4K_AGC1_MOD_MASK, E4K_AGC_MOD_SERIAL);

		/* Set Mixer Gain Control to manual */
		e4k_reg_set_mask(e4k, E4K_REG_AGC7, E4K_AGC7_MIX_GAIN_AUTO, 0);
	} else {
		/* Set LNA mode to auto */
		e4k_reg_set_mask(e4k, E4K_REG_AGC1, E4K_AGC1_MOD_MASK, E4K_AGC_MOD_IF_SERIAL_LNA_AUTON);

		/* Set Mixer Gain Control to auto */
		e4k_reg_set_mask(e4k, E4K_REG_AGC7, E4K_AGC7_MIX_GAIN_AUTO, 1);
	}

	return 0;
}

int e4k_read_gain(struct e4k_state *e4k, unsigned int *strength)
{
    int rssi = e4k_reg_read(e4k, E4K_REG_AGC3);
    int lna_gain = e4k_reg_read(e4k, E4K_REG_AGC1);

    *strength = 0;
    if ((rssi < 0) || (lna_gain < 0))
        return -1;

    lna_gain &= 0x0f;
    *strength = rssi;
    return (rssi - lna_gain);
}

int e4k_commonmode_set(struct e4k_state *e4k, int8_t value)
{
	if(value < 0)
		return -1;
	else if(value > 7)
		return -1;

	return e4k_reg_set_mask(e4k, E4K_REG_DC7, 7, value);
}

/***********************************************************************
 * DC Offset */

int e4k_manual_dc_offset(struct e4k_state *e4k, uint8_t iofs, uint8_t irange, uint8_t qofs, uint8_t qrange)
{
    int res;

    if(iofs > 0x3f)
        return -1;
    if(irange > 0x03)
        return -1;
    if(qofs > 0x3f)
        return -1;
    if(qrange > 0x03)
        return -1;

    res = e4k_reg_set_mask(e4k, 0x2a, 0x3f, iofs);
    if(res < 0)
        return res;

    res = e4k_reg_set_mask(e4k, 0x2b, 0x3f, qofs);
    if(res < 0)
        return res;

    res = e4k_reg_set_mask(e4k, 0x2c, 0x33, (qrange << 4) | irange);
    return res;
}

/***********************************************************************
 * Standby */

/*! \brief Enable/disable standby mode
 */
int e4k_standby(struct e4k_state *e4k, int enable)
{
	e4k_reg_set_mask(e4k, E4K_REG_MASTER1, E4K_MASTER1_NORM_STBY,
			 enable ? 0 : E4K_MASTER1_NORM_STBY);

	return 0;
}

/***********************************************************************
 * Initialization */

static void magic_init(struct e4k_state *e4k)
{
	e4k_reg_write(e4k, 0x7e, 0x01);
	e4k_reg_write(e4k, 0x7f, 0xfe);
	e4k_reg_write(e4k, 0x82, 0x00); /* ? not in data sheet */
	e4k_reg_write(e4k, 0x86, 0x50);	/* polarity A */
	e4k_reg_write(e4k, 0x87, 0x20); /* configure mixer */
	e4k_reg_write(e4k, 0x88, 0x01); /* configure mixer */
	e4k_reg_write(e4k, 0x9f, 0x7f); /* configure LNA */
	e4k_reg_write(e4k, 0xa0, 0x07); /* configure LNA */
}

/*! \brief Initialize the E4K tuner
 */
int e4k_init(struct e4k_state *e4k)
{
	/* make a dummy i2c read or write command, will not be ACKed! */
	e4k_reg_read(e4k, 0);

	/* Make sure we reset everything and clear POR indicator */
	e4k_reg_write(e4k, E4K_REG_MASTER1,
		E4K_MASTER1_RESET |
		E4K_MASTER1_NORM_STBY |
		E4K_MASTER1_POR_DET
	);

	/* Configure clock input */
	e4k_reg_write(e4k, E4K_REG_CLK_INP, 0x00);

	/* Disable clock output */
	e4k_reg_write(e4k, E4K_REG_REF_CLK, 0x00);
	e4k_reg_write(e4k, E4K_REG_CLKOUT_PWDN, 0x96);

	/* Write some magic values into registers */
	magic_init(e4k);
#if 0
	/* Set common mode voltage a bit higher for more margin 850 mv */
	e4k_commonmode_set(e4k, 4);

	/* Initialize DC offset lookup tables */
	e4k_dc_offset_gen_table(e4k);
#endif
	/* Enable time variant DC correction */
	e4k_reg_write(e4k, E4K_REG_DCTIME1, 0x01);
	e4k_reg_write(e4k, E4K_REG_DCTIME2, 0x01);

	/* Set LNA/Mixer gain control mode to manual */
	e4k_reg_write(e4k, E4K_REG_AGC4, 0x10); /* High threshold */
	e4k_reg_write(e4k, E4K_REG_AGC5, 0x04);	/* Low threshold */
	e4k_reg_write(e4k, E4K_REG_AGC6, 0x1a);	/* LNA calib + loop rate */
	e4k_enable_manual_gain(e4k, 1);

    /* Disable LNA gain enhancement */
	e4k_reg_set_mask(e4k, E4K_REG_AGC11, 0x7, 0);

	/* Use auto-gain as default */
	e4k_enable_manual_gain(e4k, 0);

	/* Set the most narrow filter we can possibly use
     * whoa there, let's not get absurd with the RC filter.
     * approximately slightly-more-than-doubled it so I now actually get most of a raw IQ dump @ 2.4MHz samplerate
     */
	e4k_if_filter_bw_set(e4k, E4K_IF_FILTER_MIX, 1900);
	e4k_if_filter_bw_set(e4k, E4K_IF_FILTER_RC, 2600);
    e4k_reg_set_mask(e4k, 0x12, 0x20, 0x20);

    /* make sure the DC range detector is enabled */
    e4k_reg_set_mask(e4k, 0x2d, 0x04, 0x04);

    /* perform DC offset calibration */
    e4k_reg_write(e4k, 0x29, 0x01);

	/* Disable DC LUT */
	e4k_reg_set_mask(e4k, 0x2d, 0x03, 0);

	return 0;
}

