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

    FFTComplex hpf_mem[2];
    unsigned int outbits, nbits;
} msk_t;

struct ais_state
{
	uint32_t fir_offset;
	FFTComplex signal[DEFAULT_BUF_LENGTH];  /* float i/q pairs */
	unsigned int signal_len;
	FFTComplex signal2[DEFAULT_BUF_LENGTH];
	FFTComplex signal3[DEFAULT_BUF_LENGTH];
    FFTComplex dc_hpf_mem[2];
	uint32_t freq;
    msk_t sd1, sd2;
    unsigned char *bytearray;
    unsigned int bytearray_nalloc, bytearray_nbytes;

    float d_alpha, d_beta;
    float d_phase, d_freq;

	struct demod_state_t decoder1;
	struct demod_state_t decoder2;
};

void ais_bytearray_append(uint8_t v0);
void init_msk_demod(msk_t *ch, unsigned int samplerate);
void demod_msk(msk_t *ch, FFTComplex *input, unsigned int ninput);

#endif

