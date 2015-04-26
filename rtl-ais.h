#ifndef __RTL_AIS_H__
#define __RTL_AIS_H__

#include <stdint.h>
#include <math.h>
#include "protodec.h"

typedef struct FFTComplex {
    float re, im;
} FFTComplex;
#define FFTCOMPLEX_T_DEFINED

#define DEFAULT_BUF_LENGTH		16384
#define RRC_BUFLEN 1024
#define RRC_COEFFS_L 81
#define INVGAIN 0.7f
#define FFMAX(a,b) ((a) > (b) ? (a) : (b))
#define FFMIN(a,b) ((a) > (b) ? (b) : (a))

typedef struct _msk_t
{
    float MskPhi;
    float MskFreq,MskDf;
    float Mska,MskKa;
    float MskClk;
    unsigned int MskS;

    unsigned int flen, idx;
    float I[127];
    float Q[127];
    float h[127];

    float hpf_mem[2];
    unsigned int outbits, nbits;

    unsigned char *bytearray;
    unsigned int nalloc, nbytes;
} msk_t;

typedef struct _ais_receiver_t {
    float rrc_filter_buffer[RRC_BUFLEN];
    unsigned int rrc_filter_bufidx;
	int lastbit;
	unsigned int pll;
	unsigned int pllinc;
	struct demod_state_t decoder;
	int prev;
	time_t last_levellog;
} ais_receiver_t;

struct ais_state
{
	FFTComplex prev1, prev2;
	uint32_t fir_offset;
	FFTComplex *fbuf;
	FFTComplex signal[DEFAULT_BUF_LENGTH];  /* float i/q pairs */
	unsigned int signal_len;
	FFTComplex signal2[DEFAULT_BUF_LENGTH];
	FFTComplex signal3[DEFAULT_BUF_LENGTH];
	float demod2[DEFAULT_BUF_LENGTH];
	float demod3[DEFAULT_BUF_LENGTH];
	float freqdet[DEFAULT_BUF_LENGTH];
	uint32_t freq;
    msk_t sd1, sd2;
	ais_receiver_t rx1, rx2;

    float d_alpha, d_beta;
    float d_phase, d_freq;
};

void init_msk_demod(msk_t *ch, unsigned int samplerate);
void demod_msk(msk_t *ch, float *input, unsigned int ninput);

#endif

