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
#include "ais.h"
#define WINDOW_TYPE 9
#define ALPHA_I0INV_FRAC 0.9363670349f /* fractional part of 1.0 / I0(WINDOW_TYPE * WINDOW_TYPE) */
#define FFMAX(a,b) ((a) > (b) ? (a) : (b))
#define FFMIN(a,b) ((a) > (b) ? (b) : (a))

#ifndef FFTCOMPLEX_T_DEFINED
typedef struct FFTComplex {
    float re, im;
} FFTComplex;
#define FFTCOMPLEX_T_DEFINED
#endif
FFTComplex resample_scalarproduct_iq_sse(FFTComplex *v1, float *v2, uint32_t len);
FFTComplex resample_scalarproduct_iq_neon(FFTComplex *v1, float *v2, uint32_t len);

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#define usleep(x) Sleep(x/1000)
#endif

#define DEFAULT_BUF_LENGTH		16384
#define AUTO_GAIN			-100

static volatile int do_exit = 0;
static rtlsdr_dev_t *dev = NULL;
static uint8_t buffer[DEFAULT_BUF_LENGTH]; /* We need this to be a multiple of 16K, as that's the USB URB size */
extern float *lpf96k;

struct ais_state
{
    uint32_t num_rate;
	uint32_t fir_offset;
    float *filter_bank;
    uint32_t filter_length;
	FFTComplex *fbuf;
	FFTComplex signal[DEFAULT_BUF_LENGTH];  /* float i/q pairs */
	unsigned int signal_len;
	FFTComplex *buffer2[DEFAULT_BUF_LENGTH];
	FFTComplex *buffer3[DEFAULT_BUF_LENGTH];
	uint32_t freq;
	uint32_t sample_rate;
	uint32_t output_rate;
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

static const float inv_pi  =  0.3183098733;  /* 0x3ea2f984 */

static const float
S1  = -1.66666666666666324348e-01, /* 0xBFC55555, 0x55555549 */
S2  =  8.33333333332248946124e-03, /* 0x3F811111, 0x1110F8A6 */
S3  = -1.98412698298579493134e-04, /* 0xBF2A01A0, 0x19C161D5 */
S4  =  2.75573137070700676789e-06, /* 0x3EC71DE3, 0x57B1FE7D */
S5  = -2.50507602534068634195e-08, /* 0xBE5AE5E6, 0x8A2B9CEB */
S6  =  1.58969099521155010221e-10; /* 0x3DE5D93A, 0x5ACFD57C */

// Differs from libc sinf on [0, pi/2] by at most 0.0000001192f
// Differs from libc sinf on [0, pi] by at most 0.0000170176f
static inline float k_sinf(float x)
{
    float z = x*x;
    return x*(1.0f+z*(S1+z*(S2+z*(S3+z*(S4+z*(S5+z*S6))))));
}

static float sinc(float x)
{
    float fn, y, z;
    uint32_t n;
    if (x < 0.0f) {
        x = -x;
    }

    if (x < 1e-5f) {
        z = 1.0f;
    } else {
		n = (uint32_t)(x*inv_pi);
		fn = (float)n;
		y = x - fn*(float)M_PI;
        z = k_sinf(y) / x;
        if (n&1) {
            z = -z;
        }
    }
    return z;
}

static void build_filter(struct ais_state *fm, unsigned int num_rate, unsigned int alpha)
{
    int32_t j, N2;
    //float cutoff= FFMAX(1.0f - 6.5f/(fm->filter_length+8.0f), 0.9f);
    float cutoff= FFMAX(1.0f - 6.5f/(fm->filter_length+8.0f), 0.8f);
    cutoff *= (1.0f / (float)num_rate);
	fm->filter_length *= num_rate;
	fm->filter_length &= ~3;
	fm->filter_bank = malloc((fm->filter_length + 1) * sizeof(float));

    N2 = (fm->filter_length >> 1);
    for (j=0;j<fm->filter_length;j++) {
        float x = (float)(N2-j-1);
        float tmp = fabsf(x/(float)N2);
        tmp = (1.0f - tmp*tmp);
        fm->filter_bank[j] = 0.015625f * cutoff * sinc((float)M_PI * x * cutoff) * I0(alpha * alpha * tmp) * ALPHA_I0INV_FRAC;
    }
}

static void full_demod(struct ais_state *fm, uint8_t *buffer, uint32_t buffer_len)
{
	float freqshift = 2 * (float)M_PI * (50000.0f / 264000.0f); // radians per sample
	float phase = -(float)M_PI;
	float psd = 0.0f;
	unsigned int i, N = fm->filter_length;

	for (i = 0; i < buffer_len; i++) {
		fm->fbuf[i + fm->fir_offset].re = ((float)buffer[2*i] - 127.5f);
		fm->fbuf[i + fm->fir_offset].im = ((float)buffer[2*i+1] - 127.5f);
	}

	if (!fm->fir_offset) {
		fm->fir_offset = fm->filter_length;
	}

    //fm->signal_len = resample2(fm, fm->signal, fm->fbuf, buffer_len);

	i = 0;
	fm->signal_len = 0;
    while (i < buffer_len) {
        fm->signal[fm->signal_len++] = resample_scalarproduct_iq_sse(&fm->fbuf[i], fm->filter_bank, N);
        i += fm->num_rate;
    }
    for(i=0; i<N; i++) {
        fm->fbuf[i] = fm->fbuf[i+buffer_len];
    }

	/* Okay, so we have audio at 264kHz. AIS has channel widths of 12.5/25kHz, so we need at least
     * 96k samplerate for both. For now, just keep them at 264k.
     * Channel1: just lowpass. Channel2: freq shift first, then lowpass.
     */
	for(i = 0; i < fm->signal_len; i++) {
		fm->signal2[i] = resample_scalarproduct_iq_sse(&fm->signal[i], lpf96k, 256);
	}

	for (i = 0; i < fm->signal_len; i++) {
		fm->signal3[i].re = fm->signal[i].re * fast_cosf(phase);
		fm->signal3[i].im = fm->signal[i].im * fast_sinf(phase);
		/* Advance local oscillator phase */
		phase += freqshift;
		if (phase >= (float)M_PI) phase -= 2 * (float)M_PI;
	}
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
	char *filename = NULL;
	int r, opt, write_wav_hdr = 0;
	int out_fd = -1;
	int dev_index = 0, dev_given = 0;
	int gain = AUTO_GAIN; // tenths of a dB
	unsigned int ppm_error = 0, capture_freq, sample_rate_khz;
	unsigned int aislen1 = 0, aislen2 = 0;
	unsigned char aisout1[1024];
	unsigned char aisout2[1024];

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
	fm.sample_rate = 1056000;
	fm.freq = 161975000;
	fm.freq += (fm.sample_rate >> 2);

	/*
     * We need at least 128kHz of spectrum. We also need the output rate
     * to be an integer submultiple of the samplerate (to make downsampling nice)
     * and an integer multiple of the symbol rate (which is 9600symbols/sec)
     * This gives us 264kHz and a downsampling ratio of 1/4
     */
	fm.output_rate = 264000;
	fm.filter_length = 16;
	fm.fir_offset = 0;

	while ((opt = getopt(argc, argv, "d:g:o:t:r:p:wEA:DNWMULS:CF:h")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = rtlsdr_search_for_device(optarg);
			dev_given = 1;
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10);
			break;
		case 'r':
			fm.sample_rate = (uint32_t)atofs(optarg);
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'F':
			fm.filter_length = strtoul(optarg, NULL, 10);
			break;
		case 'h':
		default:
			exit(1);
			break;
		}
	}

	if (argc <= optind) {
		filename = "-";
	} else {
		filename = argv[optind];
	}

    fm.num_rate= 4;
    build_filter(&fm, fm.num_rate, WINDOW_TYPE);

	fm.fbuf = malloc(2* (fm.filter_length + DEFAULT_BUF_LENGTH + 1) * sizeof(float));
	if (!dev_given) {
		dev_index = rtlsdr_search_for_device("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

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

	/* Set the frequency */
	r = rtlsdr_set_center_freq(dev, (uint32_t)fm.freq);
	if (r < 0) {
		rtlsdr_printf("WARNING: Failed to set center freq.\n");
	} else {
		rtlsdr_printf("Tuned to %u Hz.\n", fm.freq);
	}

	/* Set the sample rate */
	rtlsdr_printf("Sampling at %u Hz.\n", fm.sample_rate);
	rtlsdr_printf("Output at %u Hz.\n", fm.output_rate);
	sample_rate_khz = fm.sample_rate / 1000U;
	rtlsdr_printf("Buffer size: %0.2fms\n", 0.5f * (float)DEFAULT_BUF_LENGTH / (float)sample_rate_khz);
	r = rtlsdr_set_sample_rate(dev, (uint32_t)fm.sample_rate);
	if (r < 0) {
		rtlsdr_printf("WARNING: Failed to set sample rate.\n");
	}

	/* Set the tuner gain */
	if (gain == AUTO_GAIN) {
		r = rtlsdr_set_tuner_gain_mode(dev, 0);
	} else {
		r = rtlsdr_set_tuner_gain_mode(dev, 1);
		gain = rtlsdr_get_nearest_gain(dev, gain);
		r = rtlsdr_set_tuner_gain(dev, gain);
	}

	if (r != 0) {
		rtlsdr_printf("WARNING: Failed to set tuner gain.\n");
	} else if (gain == AUTO_GAIN) {
		rtlsdr_printf("Tuner gain set to automatic.\n");
	} else {
		rtlsdr_printf("Tuner gain set to %0.2f dB.\n", gain/10.0);
	}
	r = rtlsdr_set_freq_correction(dev, ppm_error);

	if (strcmp(filename, "-") == 0) { /* Write samples to stdout */
		out_fd = 1;
#ifdef _WIN32
		_setmode(out_fd, _O_BINARY);
#endif
	} else {
		out_fd = open(filename, O_WRONLY|O_CREAT|O_APPEND, 0644);
		if (out_fd < 0) {
			rtlsdr_printf("Failed to open %s\n", filename);
			exit(1);
		}
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	r = rtlsdr_reset_buffer(dev);
	if (r < 0) {
		rtlsdr_printf("WARNING: Failed to reset buffers.\n");}

	while (!do_exit) {
		unsigned int n_read;
		int r = rtlsdr_read_sync(dev, buffer, DEFAULT_BUF_LENGTH, &n_read);
		if (r < 0) {
			rtlsdr_printf("WARNING: sync read failed.\n");
			break;
		}

		/* do a Hilbert transform on the 8-bit I/Q data */
		hilbert(buffer, n_read);

		/* downsample, MSK demod */
		full_demod(&fm, buffer, (n_read >> 1));

		/* AIS protocol decode */
		aislen1 = ais_protodec(fm.buffer2, fm.buffer_len, aisout1, 1024);
		aislen2 = ais_protodec(fm.buffer3, fm.buffer_len, aisout2, 1024);

		write(out_fd, aisout1, aislen1);
		write(out_fd, aisout2, aislen2);
	}

	rtlsdr_printf("\nUser cancel, exiting...\n");
	if ((out_fd >= 0) && (out_fd != 1))
		close(out_fd);

	rtlsdr_close(dev);
	return r >= 0 ? r : -r;
}

// vim: tabstop=4:softtabstop=4:shiftwidth=4:noexpandtab
