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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "rtlsdr_i2c.h"
#include "tuner_r82xx.h"

/*
 * Static constants
 */

/*
 * These should be "safe" values, always. If we fail to get PLL lock in this range,
 * it's a hard error.
 */
#define PLL_SAFE_LOW 28e6
#define PLL_SAFE_HIGH 1845e6

/*
 * These are the initial, widest, PLL limits that we will try.
 *
 * Be cautious with lowering the low bound further - the PLL can claim to be locked
 * when configured to a lower frequency, but actually be running at around 26.6MHz
 * regardless of what it was configured for.
 *
 * This shows up as a tuning offset at low frequencies, and a "dead zone" about
 * 6MHz below the PLL lower bound where retuning within that region has no effect.
 */
#define PLL_INITIAL_LOW 26.7e6
#define PLL_INITIAL_HIGH 1885e6

/* We shrink the range edges by at least this much each time there is a soft PLL lock failure */
#define PLL_STEP_LOW 0.1e6
#define PLL_STEP_HIGH 1.0e6

/* Those initial values start from REG_SHADOW_START */
static const uint8_t r82xx_init_array[NUM_REGS] = {
    0x03, 0x12, 0x75,           /* 05 to 07 */
    0xc0, 0x40, 0xd0, 0x6c,         /* 08 to 0b */
    0x6b, 0x63, 0x75, 0x68,         /* 0c to 0f */
    0x6c, 0xbb, 0x80, 0x00,         /* 10 to 13 */
    0x0f, 0x00, 0xc0, 0x30,         /* 14 to 17 */
    0x48, 0xec, 0x60, 0x00,         /* 18 to 1b */
    0x24, 0xc5, 0x6a, 0x40          /* 1c to 1f */
};

/* Tuner frequency ranges */
static const struct r82xx_freq_range freq_ranges[] = {
    {
    /* .freq = */       0,  /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0xdf,   /* R27[7:0]  band2,band0 */
    }, {
    /* .freq = */       50, /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0xbe,   /* R27[7:0]  band4,band1  */
    }, {
    /* .freq = */       55, /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x8b,   /* R27[7:0]  band7,band4 */
    }, {
    /* .freq = */       60, /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x7b,   /* R27[7:0]  band8,band4 */
    }, {
    /* .freq = */       65, /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x69,   /* R27[7:0]  band9,band6 */
    }, {
    /* .freq = */       70, /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x58,   /* R27[7:0]  band10,band7 */
    }, {
    /* .freq = */       75, /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x44,   /* R27[7:0]  band11,band11 */
    }, {
    /* .freq = */       80, /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x44,   /* R27[7:0]  band11,band11 */
    }, {
    /* .freq = */       90, /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x34,   /* R27[7:0]  band12,band11 */
    }, {
    /* .freq = */       100,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x34,   /* R27[7:0]  band12,band11 */
    }, {
    /* .freq = */       110,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x24,   /* R27[7:0]  band13,band11 */
    }, {
    /* .freq = */       120,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x24,   /* R27[7:0]  band13,band11 */
    }, {
    /* .freq = */       140,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x14,   /* R27[7:0]  band14,band11 */
    }, {
    /* .freq = */       180,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x13,   /* R27[7:0]  band14,band12 */
    }, {
    /* .freq = */       220,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x13,   /* R27[7:0]  band14,band12 */
    }, {
    /* .freq = */       250,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x11,   /* R27[7:0]  highest,highest */
    }, {
    /* .freq = */       280,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x02,   /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
    /* .tf_c = */       0x00,   /* R27[7:0]  highest,highest */
    }, {
    /* .freq = */       310,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x41,   /* R26[7:6]=1 (bypass)  R26[1:0]=1 (middle) */
    /* .tf_c = */       0x00,   /* R27[7:0]  highest,highest */
    }, {
    /* .freq = */       450,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x41,   /* R26[7:6]=1 (bypass)  R26[1:0]=1 (middle) */
    /* .tf_c = */       0x00,   /* R27[7:0]  highest,highest */
    }, {
    /* .freq = */       588,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x40,   /* R26[7:6]=1 (bypass)  R26[1:0]=0 (highest) */
    /* .tf_c = */       0x00,   /* R27[7:0]  highest,highest */
    }, {
    /* .freq = */       650,    /* Start freq, in MHz */
    /* .rf_mux_ploy = */    0x40,   /* R26[7:6]=1 (bypass)  R26[1:0]=0 (highest) */
    /* .tf_c = */       0x00,   /* R27[7:0]  highest,highest */
    }
};

/*
 * I2C read/write code and shadow registers logic
 */
static void shadow_store(struct r82xx_priv *priv, uint8_t reg, const uint8_t *val, int len)
{
    int r = reg - REG_SHADOW_START;

    if (r < 0) {
        len += r;
        r = 0;
    }
    if (len <= 0)
        return;
    if (len > NUM_REGS - r)
        len = NUM_REGS - r;

    memcpy(&priv->regs[r], val, len);
}

static int r82xx_write(struct r82xx_priv *priv, uint8_t reg, const uint8_t *val, unsigned int len)
{
    int rc, size, pos = 0;
    uint8_t buf[NUM_REGS + 1];

    /* Store the shadow registers */
    shadow_store(priv, reg, val, len);

    do {
        if (len > 7)
            size = 7;
        else
            size = len;

        /* Fill I2C buffer */
        buf[0] = reg;
        memcpy(&buf[1], &val[pos], size);

        rc = rtlsdr_i2c_write_fn(priv->rtl_dev, R820T_I2C_ADDR, buf, size + 1);
        if (rc != size + 1) {
            rtlsdr_printf("r82xx_write: i2c wr failed=%d reg=%02x len=%d\n", rc, reg, size);
            if (rc < 0)
                return rc;
            return -1;
        }

        reg += size;
        len -= size;
        pos += size;
    } while (len > 0);

    return 0;
}

static int r82xx_read_cache_reg(struct r82xx_priv *priv, int reg)
{
    reg -= REG_SHADOW_START;

    if (reg >= 0 && reg < NUM_REGS)
        return priv->regs[reg];
    else
        return -1;
}

static int r82xx_write_reg(struct r82xx_priv *priv, uint8_t reg, uint8_t val)
{
    if (priv->reg_cache && r82xx_read_cache_reg(priv, reg) == val)
        return 0;
    return r82xx_write(priv, reg, &val, 1);
}

static int r82xx_write_reg_mask(struct r82xx_priv *priv, uint8_t reg, uint8_t val, uint8_t bit_mask)
{
    int rc = r82xx_read_cache_reg(priv, reg);

    if (rc < 0)
        return rc;

    val = (rc & ~bit_mask) | (val & bit_mask);
    return r82xx_write_reg(priv, reg, val);
}

static uint8_t r82xx_bitrev(uint8_t byte)
{
    const uint8_t lut[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
                              0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };
    return (lut[byte & 0xf] << 4) | lut[byte >> 4];
}

static int r82xx_read_reg(struct r82xx_priv *priv, uint8_t reg, uint8_t *val)
{
    int rc;
    uint8_t buf[5];

    rc = rtlsdr_i2c_read_fn(priv->rtl_dev, R820T_I2C_ADDR, buf, 5);
    if (rc < 0) {
        rtlsdr_printf("r82xx_read_reg: i2c rd failed=%d reg=%02x\n", rc, reg);
        if (rc < 0)
            return rc;
        return -1;
    }

    /* Copy data to the output buffer */
    *val = r82xx_bitrev(buf[reg]);
    return 0;
}

/*
 * r82xx tuning logic
 */

static int r82xx_set_mux(struct r82xx_priv *priv, uint32_t freq)
{
    const struct r82xx_freq_range *range;
    int rc;
    unsigned int i;

    /* Get the proper frequency range */
    freq = freq / 1000000;
    for (i = 0; i < ARRAY_SIZE(freq_ranges) - 1; i++) {
        if (freq < freq_ranges[i + 1].freq)
            break;
    }
    range = &freq_ranges[i];

    /* Open Drain */
    rc = r82xx_write_reg_mask(priv, 0x17, ((freq < 75) ? 0x08 : 0x00), 0x08);
    if (rc < 0)
        return rc;

    /* RF_MUX,Polymux */
    rc = r82xx_write_reg_mask(priv, 0x1a, range->rf_mux_ploy, 0xc3);
    if (rc < 0)
        return rc;

    /* TF BAND */
    rc = r82xx_write_reg(priv, 0x1b, range->tf_c);
    if (rc < 0)
        return rc;

    /* XTAL CAP & Drive */
    rc = r82xx_write_reg_mask(priv, 0x10, 0x00, 0x0b);
    if (rc < 0)
        return rc;

    r82xx_write_reg_mask(priv, 0x08, 0x00, 0x3f);
    r82xx_write_reg_mask(priv, 0x09, 0x00, 0x3f);
    return rc;
}

static int r82xx_set_pll(struct r82xx_priv *priv, uint32_t freq)
{
    int rc, i;
    uint64_t vco_freq;
    uint64_t vco_div;
    uint32_t vco_min = 1750000; /* kHz */
    uint32_t vco_max = vco_min * 2; /* kHz */
    uint32_t freq_khz, pll_ref;
    uint32_t sdm = 0;
    uint8_t mix_div = 2;
    uint8_t div_num = 0;
    uint8_t vco_power_ref = 2;
    uint8_t ni, si, nint, vco_fine_tune, val;
    uint8_t reg2 = 0;

    /* Frequency in kHz */
    freq_khz = (freq + 500) / 1000;
    pll_ref = priv->xtal;

    /* set pll autotune = 128kHz */
    rc = r82xx_write_reg_mask(priv, 0x1a, 0x00, 0x0c);
    if (rc < 0)
        return rc;

    /* set VCO current = 100 */
    rc = r82xx_write_reg_mask(priv, 0x12, 0x80, 0xe0);
    if (rc < 0)
        return rc;

    /* Calculate divider */
    for (mix_div = 2, div_num = 0; mix_div < 64; mix_div <<= 1, div_num++)
        if (((freq_khz * mix_div) >= vco_min) && ((freq_khz * mix_div) < vco_max))
            break;

    if (priv->rafael_chip == CHIP_R828D)
        vco_power_ref = 1;

    /*
    rc = r82xx_read(priv, 0x00, data, sizeof(data));
    if (rc < 0)
        return rc;
    vco_fine_tune = (data[4] & 0x30) >> 4;
    */
    vco_fine_tune = 2;

    if (vco_fine_tune > vco_power_ref)
        div_num = div_num - 1;
    else if (vco_fine_tune < vco_power_ref)
        div_num = div_num + 1;

    rc = r82xx_write_reg_mask(priv, 0x10, div_num << 5, 0xe0);
    if (rc < 0)
        return rc;

    vco_freq = (uint64_t)freq * (uint64_t)mix_div;

    /*
     * We want to approximate:
     *  vco_freq / (2 * pll_ref)
     * in the form:
     *  nint + sdm/65536
     * where nint,sdm are integers and 0 < nint, 0 <= sdm < 65536
     * Scaling to fixed point and rounding:
     *  vco_div = 65536*(nint + sdm/65536) = int( 0.5 + 65536 * vco_freq / (2 * pll_ref) )
     *  vco_div = 65536*nint + sdm         = int( (pll_ref + 65536 * vco_freq) / (2 * pll_ref) )
     */

    vco_div = (pll_ref + 65536 * vco_freq) / (2 * pll_ref);
    nint = (uint32_t) (vco_div / 65536);
    sdm = (uint32_t) (vco_div % 65536);

    if (nint < 13 ||
        (priv->rafael_chip == CHIP_R828D && nint > 127) ||
        (priv->rafael_chip != CHIP_R828D && nint > 76)) {
        rtlsdr_printf("[R82XX] No valid PLL values for %u Hz!\n", freq);
        return -1;
    }

    ni = (nint - 13) / 4;
    si = nint - 4 * ni - 13;

    rc = r82xx_write_reg(priv, 0x14, ni + (si << 6));
    if (rc < 0)
        return rc;

    /* pw_sdm */
    if (sdm == 0)
        val = 0x08;
    else
        val = 0x00;

    if (priv->disable_dither)
        val |= 0x10;

    rc = r82xx_write_reg_mask(priv, 0x12, val, 0x18);
    if (rc < 0)
        return rc;

    rc = r82xx_write_reg(priv, 0x16, sdm >> 8);
    if (rc < 0)
        return rc;
    rc = r82xx_write_reg(priv, 0x15, sdm & 0xff);
    if (rc < 0)
        return rc;

    for (i = 0; i < 2; i++) {
        /* Check if PLL has locked */
        reg2 = 0;
        rc = r82xx_read_reg(priv, 0x02, &reg2);
        if (rc < 0)
            return rc;
        if (reg2 & 0x40)
            break;

        /* Didn't lock. Increase VCO current */
        rc = r82xx_write_reg_mask(priv, 0x12, 0x60, 0xe0);
        if (rc < 0)
            return rc;
    }

    if (!(reg2 & 0x40)) {
        rtlsdr_printf("[r82xx] Failed to get PLL lock at %u Hz\n", freq);
        return -42;
    }

    /* set pll autotune = 8kHz */
    rc = r82xx_write_reg_mask(priv, 0x1a, 0x08, 0x08);

    return rc;
}

#if 0
static int r82xx_filter_calibrate(struct r82xx_priv *priv) {
    int rc;
    uint8_t data[5];
    unsigned int i;

    for (i = 0; i < 5; i++) {
        /* set cali clk =on */
        rc = r82xx_write_reg_mask(priv, 0x0f, 0x04, 0x04);
        if (rc < 0)
            return rc;

        rc = r82xx_set_pll(priv, 560001000u);
        if (rc < 0)
            return rc;

        /* Start Trigger */
        rc = r82xx_write_reg_mask(priv, 0x0b, 0x10, 0x10);
        if (rc < 0)
            return rc;

//      usleep(1000);

        /* Stop Trigger */
        rc = r82xx_write_reg_mask(priv, 0x0b, 0x00, 0x10);
        if (rc < 0)
            return rc;

        /* set cali clk =off */
        rc = r82xx_write_reg_mask(priv, 0x0f, 0x00, 0x04);
        if (rc < 0)
            return rc;

        /* Check if calibration worked */
        rc = r82xx_read(priv, 0x00, data, sizeof(data));
        if (rc < 0)
            return rc;

        priv->fil_cal_code = data[4] & 0x0f;
        rtlsdr_printf("r82xx_filter_calibrate: iteration %u, fil_cal_code 0x%02x\n", i, priv->fil_cal_code);
        if (priv->fil_cal_code && priv->fil_cal_code != 0x0f)
            break;
    }

    return rc;
}
#endif

int r82xx_set_bw(struct r82xx_priv *priv, uint32_t bw) {
    int rc;
    uint32_t hpf = (R82XX_IF_FREQ - bw/2)/1000U;
    uint8_t hp_cor = 0x00;

    if(hpf >= 4700)      hp_cor = 0x00;    /*         5 MHz */
    else if(hpf >= 3800) hp_cor = 0x01;    /*         4 MHz */
    else if(hpf >= 3000) hp_cor = 0x02;    /* -12dB @ 2.25 MHz */
    else if(hpf >= 2800) hp_cor = 0x03;    /*  -8dB @ 2.25 MHz */
    else if(hpf >= 2600) hp_cor = 0x04;    /*  -4dB @ 2.25 MHz */
    else if(hpf >= 2400) hp_cor = 0x05;    /* -12dB @ 1.75 MHz */
    else if(hpf >= 2200) hp_cor = 0x06;    /*  -8dB @ 1.75 MHz */
    else if(hpf >= 2000) hp_cor = 0x07;    /*  -4dB @ 1.75 MHz */
    else if(hpf >= 1800) hp_cor = 0x08;    /* -12dB @ 1.25 MHz */
    else if(hpf >= 1600) hp_cor = 0x09;    /*  -8dB @ 1.25 MHz */
    else if(hpf >= 1400) hp_cor = 0x0A;    /*  -4dB @ 1.25 MHz */
    else                 hp_cor = 0x0B;

    /* Set HP corner */
    rc = r82xx_write_reg_mask(priv, 0x0b, hp_cor, 0x0f);
    if (rc < 0)
        return rc;

    return 0;
}

int r82xx_set_dither(struct r82xx_priv *priv, int dither)
{
    priv->disable_dither = !dither;
    return 0;
}

int r82xx_read_gain(struct r82xx_priv *priv, unsigned int *gain0, unsigned int *gain1)
{
    uint8_t reg3 = 0;
    unsigned int strength = 0;
    int rc;

    rc = r82xx_read_reg(priv, 0x03, &reg3);
    if (rc < 0)
        return rc;

    rc = (((reg3 & 0xf0) << 1) + (reg3 & 0x0f));
    if (gain0) *gain0 = (reg3 & 0xf0);
    if (gain1) *gain1 = (reg3 & 0x0f);

    /* A higher gain at LNA means a lower signal strength */
    strength = (45 - rc) << 4 | 0xff;
    if (strength == 0xff)
        strength = 0;
    return strength;
}

/* measured with a Racal 6103E GSM test set at 928 MHz with -60 dBm
 * input power, for raw results see:
 * http://steve-m.de/projects/rtl-sdr/gain_measurement/r820t/
 */

const unsigned int r82xx_lna_gain_steps[]  = {
    0, 10, 13, 40, 38, 13, 31, 22, 26, 31, 26, 14, 19, 5, 35, 13
};

const unsigned int r82xx_lna_gain_values[16] = {
    0, 2, 5, 13, 20, 23, 29, 33, 39, 45, 50, 53, 57, 58, 65, 67
};

/*
 *  According to the info in the register file, this should *actually* look like:
 *  0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30
 */
unsigned int r82xx_mixer_gain_values[16] = {
    0, 1, 3, 5, 8, 11, 12, 17, 21, 23, 25, 28, 30, 32, 32, 31
};

int r82xx_set_agc_params(struct r82xx_priv *priv, uint8_t lna_agc, uint8_t mixer_agc, uint8_t agc_rate)
{
    int rc;

    /* LNA */
    rc = r82xx_write_reg(priv, 0x0d, lna_agc);
    if (rc < 0)
        return rc;

    /* Mixer */
    rc = r82xx_write_reg(priv, 0x0e, mixer_agc);
    if (rc < 0)
        return rc;

    /* AGC Clock
     * Settings: 0x30 -> 250hz, 0x20 -> 60hz,
     *           0x10 -> 16hz(?), 0x00 -> 1kHz(?)
     */
    rc = r82xx_write_reg_mask(priv, 0x1a, agc_rate << 4, 0x30);
    return rc;
}

int r82xx_enable_manual_gain(struct r82xx_priv *priv, unsigned int set_manual_gain)
{
    int rc = 0;
    uint8_t lna_manual = (set_manual_gain ? 0x10 : 0x00);
    uint8_t mixer_manual = (set_manual_gain ? 0x00 : 0x10);

    /* LNA */
    rc |= r82xx_write_reg_mask(priv, 0x05, lna_manual, 0x10);

    /* Mixer */
    rc |= r82xx_write_reg_mask(priv, 0x07, mixer_manual, 0x10);
    return rc;
}

int r82xx_set_lna_gain(struct r82xx_priv *priv, unsigned int gain)
{
    unsigned int i;
    uint8_t lna_index = 0;

    for (i = 0; i < 15; i++) {
        if (r82xx_lna_gain_values[lna_index] >= gain)
            break;
        lna_index++;
    }

    /* set LNA gain */
    return r82xx_write_reg_mask(priv, 0x05, lna_index, 0x0f);
}

int r82xx_set_mixer_gain(struct r82xx_priv *priv, unsigned int gain)
{
    uint8_t mix_index = ((gain >> 1) & 0x0f);

    /* set Mixer gain */
    return r82xx_write_reg_mask(priv, 0x07, mix_index, 0x0f);
}


int r82xx_set_freq(struct r82xx_priv *priv, uint32_t freq)
{
    int rc = -1;
    uint32_t lo_freq = freq + R82XX_IF_FREQ;
    uint8_t air_cable1_in;

    rc = r82xx_set_mux(priv, lo_freq);
    if (rc < 0)
        goto err;

    rc = r82xx_set_pll(priv, lo_freq);
    if (rc < 0)
        goto err;

    /* switch between 'Cable1' and 'Air-In' inputs on sticks with
     * R828D tuner. We switch at 345 MHz, because that's where the
     * noise-floor has about the same level with identical LNA
     * settings. The original driver used 320 MHz. */
    air_cable1_in = (freq > MHZ(345)) ? 0x00 : 0x60;

    if ((priv->rafael_chip == CHIP_R828D) &&
        (air_cable1_in != priv->input)) {
        priv->input = air_cable1_in;
        rc = r82xx_write_reg_mask(priv, 0x05, air_cable1_in, 0x60);
    }

err:
    if (rc < 0)
        rtlsdr_printf("r82xx_set_freq: failed=%d\n", rc);
    return rc;
}

/*
 * r82xx standby logic
 */

int r82xx_standby(struct r82xx_priv *priv)
{
    int rc;

    /* If device was not initialized yet, don't need to standby */
    /* Why on *earth* are you calling r82xx_standby() then? */
    /* if (!priv->init_done)
        return 0; */

    priv->reg_cache = 0;
    /* rc = r82xx_write_reg(priv, 0x06, 0xb1); */ // LNA Power Detector: Off, LNA Power Detector (Narrowband): Off,
                                                  // Cable2 LNA Off, LNA Power: Almost Max (Fixing this to min, below)
    rc = r82xx_write_reg(priv, 0x06, 0xb7); // LNA Power Detector: Off, LNA Power Detector (Narrowband): Off,
                                            // Cable2 LNA Off, LNA Power: Min
    rc |= r82xx_write_reg(priv, 0x05, 0x03); // Loop-Through On, Cable1 LNA Off, Air-In LNA Off, LNA Gain Mode: Auto
    rc |= r82xx_write_reg(priv, 0x07, 0x3a); // Image Negative, Mixer Power Off, Mixer Gain Mode: Auto
    rc |= r82xx_write_reg(priv, 0x0a, 0x36); // Channel Filter: Set Values to Default, Turn Off
    rc |= r82xx_write_reg(priv, 0x0f, 0x68); // LDO Power Off, Clk Output Off, Ring PLL Refclock Off,
                                             // Ch_Filt Calibration Clk Off, Internal AGC Clock ON(!)
    rc |= r82xx_write_reg(priv, 0x11, 0x03);
    rc |= r82xx_write_reg(priv, 0x17, 0xf4); // PLL digital low drop out regulator supply current Off, Prescale Current 150uA,
                                             // Open-Drain: High-Z, IQ Generator: Div_Mid, Buf_Min, Power Off
    rc |= r82xx_write_reg(priv, 0x19, 0x0c); // RingPLL VCO Power Max

    priv->reg_cache = 1;
    return rc;
}

/*
 * r82xx device init logic
 */

int r82xx_init(struct r82xx_priv *priv)
{
    int rc;

    /* TODO: R828D might need r82xx_xtal_check() */
    priv->xtal_cap_sel = XTAL_HIGH_CAP_0P;

    /* Initialize registers */
    priv->reg_cache = 0;
    rc = r82xx_write(priv, 0x05, r82xx_init_array, sizeof(r82xx_init_array));

    /* Initialize the shadow registers */
    memcpy(priv->regs, r82xx_init_array, sizeof(r82xx_init_array));

    /* Init Flag & Xtal_check Result, version */
    rc |= r82xx_write_reg_mask(priv, 0x13, VER_NUM, 0x3f);
    priv->input = 0x00;

    //r82xx_filter_calibrate(priv);
    priv->fil_cal_code = 0x00; /* seriously? */

    /* r82xx_set_bw will always be called by rtlsdr_set_sample_rate,
       so there's no need to call r82xx_set_if_filter here */
    priv->reg_cache = 1;

    /* lna   vth 0.84, vtl 0.64 */
    rc |= r82xx_write_reg(priv, 0x0d, 0x73);
    /* mixer vth 1.04, vtl 0.84 */
    rc |= r82xx_write_reg(priv, 0x0e, 0x73);

    if (rc < 0)
        rtlsdr_printf("r82xx_init: failed=%d\n", rc);
    return rc;
}

#if 0
/* Not used, for now */
static int r82xx_gpio(struct r82xx_priv *priv, int enable)
{
    return r82xx_write_reg_mask(priv, 0x0f, enable ? 1 : 0, 0x01);
}
#endif

