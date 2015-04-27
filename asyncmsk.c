#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "rtl-ais.h"
float fast_atanf(float x);

static float rcos_filter_coeffs[84] = {
    -0.0195706589, -0.0019463516, +0.0174879305, +0.0376500445,
    +0.0573162726, +0.0751855693, +0.0899533523, +0.1003910752,
    +0.1054272579, +0.1042253298, +0.0962535644, +0.0813425924,
    +0.0597264537, +0.0320638792, -0.0005625513, -0.0366705982,
    -0.0744258758, -0.1117113668, -0.1462163400, -0.1755408960,
    -0.1973113466, -0.2093008549, -0.2095492535, -0.1964757729,
    -0.1689785575, -0.1265153337, -0.0691603972, +0.0023658260,
    +0.0866970749, +0.1818523559, +0.2852972395, +0.3940331739,
    +0.5047111565, +0.6137647860, +0.7175566441, +0.8125311731,
    +0.8953667896, +0.9631199126, +1.0133539072, +1.0442466315,
    +1.0546712948, +1.0442466315, +1.0133539072, +0.9631199126,
    +0.8953667896, +0.8125311731, +0.7175566441, +0.6137647860,
    +0.5047111565, +0.3940331739, +0.2852972395, +0.1818523559,
    +0.0866970749, +0.0023658260, -0.0691603972, -0.1265153337,
    -0.1689785575, -0.1964757729, -0.2095492535, -0.2093008549,
    -0.1973113466, -0.1755408960, -0.1462163400, -0.1117113668,
    -0.0744258758, -0.0366705982, -0.0005625513, +0.0320638792,
    +0.0597264537, +0.0813425924, +0.0962535644, +0.1042253298,
    +0.1054272579, +0.1003910752, +0.0899533523, +0.0751855693,
    +0.0573162726, +0.0376500445, +0.0174879305, -0.0019463516,
    -0.0195706589, 0.0, 0.0, 0.0
};

static float rrc_filter_run_buf(asynchronous_msk_demod_t *f, float *in, float *out, unsigned int len)
{
    float maxval = 0.0f;
    unsigned int id = 0, filtidx = f->rrc_filter_bufidx;

	while (id < len) {
	    f->rrc_filter_buffer[f->rrc_filter_bufidx] = in[id];

		// look for peak volume
		if (in[id] > maxval)
			maxval = in[id];

		out[id++] = INVGAIN*scalarproduct_float_c(&f->rrc_filter_buffer[filtidx - RRC_COEFFS_L], rcos_filter_coeffs, 84);

		/* the buffer is much smaller than the incoming chunks */
		if (filtidx == RRC_BUFLEN-1) {
			memcpy(f->rrc_filter_buffer,
			       f->rrc_filter_buffer + RRC_BUFLEN - RRC_COEFFS_L,
			       RRC_COEFFS_L * sizeof(float));
            filtidx = RRC_COEFFS_L - 1;
		}
        filtidx++;
	}

    f->rrc_filter_bufidx = filtidx;
	return maxval;
}

void putbit(asynchronous_msk_demod_t *ch, unsigned char v)
{
    unsigned char b = 0;
    ch->outbits <<= 1;
    if(v>0) {
        ch->outbits |= 1;
        b = 1;
    }
    if (ch->nbits >= 8) {
	    /* feed to the decoder */
        ais_bytearray_append(ch->outbits);
        ch->nbits = 0;
        ch->outbits = 0;
    }
	/* feed to the decoder */
	//protodec_decode(&b, 1, &fm->decoder);
}

void init_async_msk(asynchronous_msk_demod_t *async_msk)
{
	async_msk->rrc_filter_bufidx = RRC_COEFFS_L;
	async_msk->lastbit = 0;
	async_msk->pll = 0;
	async_msk->pllinc = 0x10000 / 5;
	async_msk->prev = 0;
	async_msk->last_levellog = 0;
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

void async_msk_decode(asynchronous_msk_demod_t *async_msk, FFTComplex *inbuf, unsigned int len)
{
	float out, maxval = 0.0f;
    unsigned int i;
	int curr, bit;
	unsigned char b;
	float filtered[4096];

    async_msk->demod[0] = polar_disc_fast(inbuf[0], async_msk->prev_fm);
    for (i = 1; i < inbuf_len; i++) {
        async_msk->demod[i] = polar_disc_fast(inbuf[i], inbuf[i-1]);
    }
    async_msk->prev_fm = inbuf[inbuf_len - 1];

	maxval = rrc_filter_run_buf(async_msk, buf, filtered, len);
	for (i = 0; i < len; i++) {
		out = filtered[i];
		curr = (out > 0);

		if ((curr ^ async_msk->prev) == 1) {
			if (async_msk->pll < (0x10000 / 2)) {
				async_msk->pll += async_msk->pllinc / INC;
			} else {
				async_msk->pll -= async_msk->pllinc / INC;
			}
		}
		async_msk->prev = curr;
		async_msk->pll += async_msk->pllinc;

		if (async_msk->pll > 0xffff) {
			/* slice */
			bit = (out > 0);
			/* nrzi decode */
			b = !(bit ^ async_msk->lastbit);
            putbut(async_mask, b);
			async_msk->lastbit = bit;
			async_msk->pll &= 0xffff;
		}
	}
}

