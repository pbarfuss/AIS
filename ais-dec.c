#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <signal.h>
#include "ais-dec.h"
#include "filtertables.h"
#include "fast_atanf.h"
#define M_TWOPIf 6.2831853f
static const float inv_pi  =  0.3183098733;  /* 0x3ea2f984 */
static const float invpio2 =  6.3661980629e-01; /* 0x3f22f984 */

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#define usleep(x) Sleep(x/1000)
#endif
#define DEFAULT_BUF_LENGTH		16384

static volatile int do_exit = 0;
static FFTComplex buffer[DEFAULT_BUF_LENGTH]; /* We need this to be a multiple of 16K, as that's the USB URB size */

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
static struct demod_state_t *ais_demod;

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

void ais_bytearray_append(uint8_t v0)
{
	//ch->bytearray[ch->nbytes++] = ch->outbits;
	if (ais_nalloc < ais_nbytes) {
		ais_bytearray = realloc(ais_bytearray, 2*ais_nalloc);
		ais_nalloc += ais_nbytes;
	}
	ais_bytearray[ais_nbytes++] = v0;
	/* AIS protocol decode - we need at least ~512b for a packet, worst-case we wind up splitting a multipart one.
	 * Thankfully this *is* correctly handled in the decoder, as long as it's less than 1K that's split off,
     * so we can just feed it 512b every time and reset the buffer pointer afterwards.
	 */
	if (ais_nbytes > 512) {
		ais_protodec_decode(ais_bytearray, ais_nbytes, ais_demod);
		ais_nbytes = 0;
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

static void full_demod(struct ais_state *fm, FFTComplex *buf, uint32_t signal_len)
{
	unsigned int i;

	hpf(fm->signal, buf, fm->dc_hpf_mem, signal_len);
	demod_msk(&fm->sd1, fm->signal, signal_len);

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

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	struct ais_state fm;
	int r, in_fd = -1;

	if (argc < 2) {
		printf("Usage: ais-dec <input SDR capture in interleaved I/Q float format>\n");
		return -1;
	}

	if ((in_fd = open(argv[1], O_RDONLY)) < 0) {
		printf("Error: cannot open input file %s!\n", argv[1]);
		return -1;
	}

	ais_protodec_initialize(&fm.decoder1, 1);
	ais_protodec_initialize(&fm.decoder2, 1);
	ais_demod = &(fm.decoder1);

	init_msk_demod(&fm.sd1, 288000/4);
	init_msk_demod(&fm.sd2, 48000);

	ais_bytearray = malloc(4096);
	ais_nalloc = 4096;
	ais_nbytes = 0;

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

	while (1) {
		int r2 = read(in_fd, buffer, DEFAULT_BUF_LENGTH*sizeof(FFTComplex));
		if (r2 <= 0) {
			break;
		}

		/* The data is still modulated, but it's been Hilbert-transformed, is at baseband,
         * and has been downsampled to something reasonable. (Hopefully, at least, on that last point).
		 * So we skip directly to the MSK demodulation. */
		full_demod(&fm, buffer, (r2 >> 1));

		/* Dump phase offset/delay from MSK decoder, and frequency offset from the I/Q PLL here
         * and cross-reference/compare.
         */
#if 0
		for (ll = 0; ll < (n_read >> 3); ll++) {
			char buf2[1024];
			int len2 = snprintf(buf2, 1023, "%.10f %.10f %.10f %.10f\n",
								fm.freqdet[4*ll], fm.freqdet[4*ll+1], fm.freqdet[4*ll+2], fm.freqdet[4*ll+3]);
			write(1, buf2, len2);
		}
#endif
	}

	close(in_fd);
	return r >= 0 ? r : -r;
}

// vim: tabstop=4:softtabstop=4:shiftwidth=4:noexpandtab
