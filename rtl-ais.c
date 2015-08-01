#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <signal.h>
#include "rtl-sdr.h"
#define M_TWOPIf 6.2831853f
static const float inv_pi  =  0.3183098733;  /* 0x3ea2f984 */
static const float invpio2 =  6.3661980629e-01; /* 0x3f22f984 */

#include "rtl-ais.h"
#include "protodec.h"
#include "fast_atanf.h"
#include "filtertables.h"
#define RTLSDR_SAMPLE_RATE 288000
#define DOWNSAMPLE_FILTER_LENGTH 128
#define	INC	16

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#define usleep(x) Sleep(x/1000)
#endif

#define DEFAULT_BUF_LENGTH		16384

static volatile int do_exit = 0;
static rtlsdr_dev_t *dev = NULL;
static uint8_t buffer[DEFAULT_BUF_LENGTH]; /* We need this to be a multiple of 16K, as that's the USB URB size */

static const float lpf72k[20] = {
  -0.021042, -0.113812,  0.017416, 0.049604,
   0.024603, -0.064167, -0.080361, 0.072182,
   0.309384,  0.425102,  0.309384, 0.072182
  -0.080361, -0.064167,  0.024603, 0.049604,
   0.017416, -0.113812, -0.021042, 0.0
};

/**
 * 0th order modified bessel function of the first kind.
 */
float I0(float x){
    float v=1;
    float lastv=0;
    float t=1;
    int i;

    x= x*0.25f;
    for(i=1; v != lastv; i++){
        lastv=v;
        t *= x/(i*i);
        v += t;
    }
    return v;
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		write(2, "Signal caught, exiting!\n", 25);
		do_exit = 1;
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	write(2, "Signal caught, exiting!\n", 25);
	do_exit = 1;
}
#endif

static unsigned char *ais_bytearray;
static unsigned int ais_nalloc, ais_nbytes;

void ais_bytearray_append(uint8_t v0)
{
	//ch->bytearray[ch->nbytes++] = ch->outbits;
	if (ais_nalloc < ais_nbytes) {
		ais_bytearray = realloc(ais_bytearray, 2*ais_nalloc);
		ais_nalloc += ais_nbytes;
	}
	ais_bytearray[ais_nbytes++] = v0;
	/* AIS protocol decode */
	//protodec_decode(&fm.protodec);
}

/* 90 rotation is 1+0j, 0+1j, -1+0j, 0-1j
   or [0, 1, -3, 2, -4, -5, 7, -6] */
void hilbert(unsigned char *buf, uint32_t len)
{
	uint32_t i;
	unsigned char tmp;
	for (i=0; i<len; i+=8) {
		/* uint8_t negation = 255 - x */
		tmp = 255 - buf[i+3];
		buf[i+3] = buf[i+2];
		buf[i+2] = tmp;

		buf[i+4] = 255 - buf[i+4];
		buf[i+5] = 255 - buf[i+5];

		tmp = 255 - buf[i+6];
		buf[i+6] = buf[i+7];
		buf[i+7] = tmp;
	}
}

float scalarproduct_float_c(float *v1, float *v2, unsigned int len)
{
    float p = 0.0f;
    unsigned int i;
    for (i = 0; i < len; i++) {
        p += v1[i] * v2[i];
    }
    return p;
}

FFTComplex scalarproduct_iq_c(FFTComplex *v1, float *v2, unsigned int len)
{
	FFTComplex p;
    unsigned int i;
	p.re = 0.0f;
	p.im = 0.0f;
    for (i = 0; i < len; i++) {
		float t = v2[i];
		p.re += v1[i].re * t;
		p.im += v1[i].im * t;
    }
    return p;
}

float fast_atanf(float z)
{
  float alpha, angle, base_angle, z_in = z;
  unsigned int index;

  if (z > 1.0f) {
    z = 1.0f/z;
  }

  /* when ratio approaches the table resolution, the angle is */
  /* best approximated with the argument itself... */
  if(z < TAN_MAP_RES) {
    base_angle = z;
  } else {
    /* find index and interpolation value */
    alpha = z * (float)TAN_MAP_SIZE;
    index = ((unsigned int)alpha) & 0xff;
    alpha -= (float)index;
    /* determine base angle based on quadrant and */
    /* add or subtract table value from base angle based on quadrant */
    base_angle  =  fast_atan_table[index];
    base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
  }

  if(z_in < 1.0f) { /* -PI/4 -> PI/4 or 3*PI/4 -> 5*PI/4 */
    angle = base_angle; /* 0 -> PI/4, angle OK */
  }
  else { /* PI/4 -> 3*PI/4 or -3*PI/4 -> -PI/4 */
    angle = (float)M_PI*0.5f - base_angle; /* PI/4 -> PI/2, angle = PI/2 - angle */
  }

  return (angle);
}

float polar_disc_fast(FFTComplex a, FFTComplex b)
{
    float x = a.re*b.re + a.im*b.im;
    float y = a.im*b.re - a.re*b.im;
    float z;

    if ((y != y) || (x != x))
        return 0.0f;

    y += 1e-12f;
    if (x < 1e-12f) {
        z = (float)M_PI * 0.5f;
    } else {
        /* compute y/x */
        z=fast_atanf(fabsf(y/x));
        if (x < 0.0f) {
            z = (float)M_PI - z;
        }
    }

    if (z != z) {
        z = 0.0f;
    }

    if (y < 0.0f) {
        z = -z;
    }

    return (z * 0.31831f);
}

void pll_frequency_det(struct ais_state *d, FFTComplex *input_items, unsigned int ninput_items, float *optr)
{
    float sample_phase, error;
    unsigned int i = 0, n;

    while(i < ninput_items) {
        optr[i] = (d->d_freq * (float)M_1_PI);
        sample_phase = fast_atanf(fabsf(input_items[i].im / input_items[i].re));
		if (input_items[i].re < 0) sample_phase = (float)M_PI - sample_phase;
        error = (sample_phase - d->d_phase);
        n = (uint32_t)(error*inv_pi); n &= ~0x1;
        error -= ((float)M_PI * (float)n);
        d->d_freq += d->d_beta * error;
        d->d_phase = d->d_phase + d->d_freq + d->d_alpha * error;
        while(d->d_phase>M_TWOPIf)
            d->d_phase -= M_TWOPIf;
        while(d->d_phase<-M_TWOPIf)
            d->d_phase += M_TWOPIf;
        i++;
    }
}

void hpf(FFTComplex *out, FFTComplex *in, FFTComplex *mem, unsigned int n)
{
    unsigned int i;
    float tmp1, tmp2;

    for (i = 0; i < n; i++) {
        tmp1    = 0.989501953f * in[i].re - (-1.978881836f * mem[0].re + 0.979125977f * mem[1].re);
        tmp2    = 0.989501953f * in[i].im - (-1.978881836f * mem[0].im + 0.979125977f * mem[1].im);
        out[i].re = tmp1 - 2.0f*mem[0].re + mem[1].re;
        out[i].im = tmp2 - 2.0f*mem[0].im + mem[1].im;
        mem[1] = mem[0];
        mem[0].re = tmp1;
        mem[0].im = tmp2;
    }
}

static void full_demod(struct ais_state *fm, uint8_t *buffer, uint32_t signal_len)
{
	unsigned int i, N = 19;

	for (i = 0; i < signal_len; i++) {
		fm->fbuf[i + 19].re = ((float)buffer[2*i] - 127.5f);
		fm->fbuf[i + 19].im = ((float)buffer[2*i+1] - 127.5f);
	}
	pll_frequency_det(fm, fm->fbuf + 19, signal_len - 19, fm->freqdet);

	i = 0;
	fm->signal_len = 0;
    while (i < signal_len) {
        fm->signal[fm->signal_len++] = scalarproduct_iq_c(&fm->fbuf[i], lpf72k, 20);
        i += 4; /* 4->1 downsample */
    }
    for(i=0; i<N; i++) {
        fm->fbuf[i] = fm->fbuf[i+signal_len];
    }
	hpf(fm->signal, fm->signal, fm->dc_hpf_mem, fm->signal_len);
	demod_msk(&fm->sd1, fm->signal, fm->signal_len);

	/* Okay, so we have audio at 192kHz. AIS has channel widths of 12.5/25kHz, so we need at least
     * 96k samplerate for both. For now, just keep them at 264k.
     * Channel1: just lowpass. Channel2: freq shift first, then lowpass.
     */
	/* for(i = 0; i < fm->signal_len; i++) {
		fm->signal2[i] = resample_scalarproduct_iq_sse(&fm->signal[i], lpf96k, 256);
	} */

	for (i = 0; i < fm->signal_len; i++) {
		fm->signal3[i].re = fm->signal[i].re * ais_chan2_shift_to_baseband_table[i % 96].re;
		fm->signal3[i].im = fm->signal[i].im * ais_chan2_shift_to_baseband_table[i % 96].im;
	}
	demod_msk(&fm->sd2, fm->signal3, fm->signal_len);
}

/* standard suffixes */
float atofs(char *f)
{
	char last;
	int len;
	float suff = 1.0f;
	len = strlen(f);
	last = f[len-1];
	f[len-1] = '\0';
	switch (last) {
		case 'g':
		case 'G':
			suff *= 1e3f;
		case 'm':
		case 'M':
			suff *= 1e3f;
		case 'k':
		case 'K':
			suff *= 1e3f;
			suff *= (float)atof(f);
			f[len-1] = last;
			return suff;
	}
	f[len-1] = last;
	return atof(f);
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	struct ais_state fm;
	int r, opt;
	unsigned int ll, dev_index = 0;
	unsigned int lna_gain = 12, mixer_gain = 12;
	int ppm_error = 0;

	ais_protodec_initialize(&fm.decoder1, 1);
	ais_protodec_initialize(&fm.decoder2, 1);

	init_msk_demod(&fm.sd1, 288000/4);
	init_msk_demod(&fm.sd2, 48000);

	ais_bytearray = malloc(4096);
	ais_nalloc = 4096;
	ais_nbytes = 0;

	/*
	 * AIS is VHF marine 87B and 88B. We want to catch both.
	 * This mandates the following:
	 *
	 * We need to be able to capture both channels, which are,
     * surprisingly, only 50kHz apart. This means we can get by
     * with a relatively low samplerate (say, 1056kHz) and be fairly efficient.
     * It's FM so the actual frequency to tune to is slightly offset:
     * we're given the lower edge, but actually want the center.
     * Add Fs/4 to deal with this.
     */
	fm.freq = 162000000;
	fm.d_alpha = 0.04f;
	fm.d_beta = 0.0001f;
	//fm.d_freq = 0.0f;
	fm.d_freq = ((float)M_PI / 4800.0f);

	/*
     * We need at least 128kHz of spectrum. We also need the output rate
     * to be an integer submultiple of the samplerate (to make downsampling nice)
     * and an integer multiple of the symbol rate (which is 9600symbols/sec)
     * This gives us 264kHz and a downsampling ratio of 1/4
     */
	fm.fir_offset = 0;

	while ((opt = getopt(argc, argv, "d:L:M:p:h")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = rtlsdr_search_for_device(optarg);
			break;
		case 'L':
			lna_gain = (int)(atof(optarg) * 2);
			break;
		case 'M':
			mixer_gain = (int)(atof(optarg) * 2);
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'h':
		default:
			exit(1);
			break;
		}
	}

	fm.fbuf = malloc(2* (DOWNSAMPLE_FILTER_LENGTH + DEFAULT_BUF_LENGTH + 1) * sizeof(float));
	r = rtlsdr_open(&dev, dev_index);
	if (r < 0) {
		rtlsdr_printf("Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}

#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigact.sa_handler = SIG_IGN;
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler((PHANDLER_ROUTINE) sighandler, TRUE);
#endif

	/* Disable PLL dithering, needed to get reasonable fingerprinting IDs. */
	rtlsdr_set_dithering(dev, 0);

	/* Set the sample rate */
	rtlsdr_printf("Sampling at %u Hz.\n", RTLSDR_SAMPLE_RATE);
	rtlsdr_printf("Buffer size: %0.2fms\n", 0.5f * (float)DEFAULT_BUF_LENGTH / 1536.0f);
	r = rtlsdr_set_sample_rate(dev, RTLSDR_SAMPLE_RATE);
	if (r < 0) {
		rtlsdr_printf("WARNING: Failed to set sample rate.\n");
	}
	r = rtlsdr_set_freq_correction(dev, ppm_error);

	/* Set the frequency */
	r = rtlsdr_set_center_freq(dev, (uint32_t)fm.freq);
	if (r < 0) {
		rtlsdr_printf("WARNING: Failed to set center freq.\n");
	} else {
		rtlsdr_printf("Tuned to %u Hz.\n", fm.freq);
	}

	/* Set the tuner gain */
	//r = rtlsdr_set_tuner_gain_mode(dev, 0);
	r = rtlsdr_set_tuner_gain_mode(dev, 1);
	if (r != 0) {
		rtlsdr_printf("WARNING: Failed to set gain mode to manual.\n");
	} else {
		lna_gain  = rtlsdr_set_tuner_lna_gain(dev, lna_gain);
		mixer_gain = rtlsdr_set_tuner_mixer_gain(dev, mixer_gain);
		rtlsdr_printf("Tuner gain set to (LNA: %0.2f dB, Mixer: %.5fdB)\n",
                      lna_gain * 0.1, mixer_gain * 0.1);
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	rtlsdr_reset_buffer(dev);

	while (!do_exit) {
		unsigned int n_read;
		int r2 = rtlsdr_read_sync(dev, buffer, DEFAULT_BUF_LENGTH, &n_read);
		if (r2 < 0) {
			rtlsdr_printf("WARNING: sync read failed.\n");
			break;
		}

		/* do a Hilbert transform on the 8-bit I/Q data */
		hilbert(buffer, n_read);

		/* downsample, MSK demod */
		full_demod(&fm, buffer, (n_read >> 1));

		/* AIS protocol decode */
		//protodec_decode(&fm.protodec);

		/* Dump phase offset/delay from MSK decoder, and frequency offset from the I/Q PLL here
         * and cross-reference/compare.
         */
		for (ll = 0; ll < (n_read >> 3); ll++) {
			char buf2[1024];
			int len2 = snprintf(buf2, 1023, "%.10f %.10f %.10f %.10f\n",
								fm.freqdet[4*ll], fm.freqdet[4*ll+1], fm.freqdet[4*ll+2], fm.freqdet[4*ll+3]);
			write(1, buf2, len2);
		}
	}

	rtlsdr_printf("\nUser cancel, exiting...\n");
	rtlsdr_close(dev);
	return r >= 0 ? r : -r;
}

// vim: tabstop=4:softtabstop=4:shiftwidth=4:noexpandtab
