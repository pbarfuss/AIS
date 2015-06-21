#include <stdint.h>
#include <math.h>

/*
 * Cascaded Integrator-Comb, i.e. coarse lowpass.
 *
 * The RTL needs to be run at a reasonably high rate (at least 900kHz or so) in order to get non-terrible output.
 * It's honestly only half due to a terrible LPF - it actually does support doing a 16-order symmetric FIR lowpass
 * on the device, which is better than you'd expect. The other half is that at very low samplerates you run into
 * the strange and fun issue where the capacitors in the delay buffers discharge enough to drop between levels in the
 * extremely coarse 8-bit quantizer between each sampling period.
 *
 * The problem with this is that it's a pain to use in an actual decoder for, well, any digital mode.
 * Essentially at 1MHz, decoding AIS (each channel of which only has a bandwidth of 25kHz) is an extremely sparse
 * computation. Downsampling turns this into a dense computation over a much smaller array, which is both faster and more accruate.
 *
 * That being said, the classical windowed sinc()-based FIR filters you're familiar with from the audio world don't work very well at high orders - for decimation by N you need to rescale your coefficients by 1/N and multiply the delay line length by N.
 * This can get to a point where if you aren't careful numerical error means you'll get inferior output to a box lowpass
 * (i.e. a zero-order hold).
 *
 * Turns out ZOH's aren't always bad, especially if you modify them just slightly. Enter the Cascased Integrator-Comb Filter.
 * A good intro to their theory and operating principles is here: http://www.embedded.com/design/configurable-systems/4006446/Understanding-cascaded-integrator-comb-filters
 *
 * This is used to get to a "ballpark" frequency: it's 4x the width of an AIS channel, and 2x what we want.
 * We *then* do freq translation/downsampling on that signal, this is basically an AF-style problem instead of an RF-style one,
 * and unsurprisingly AF-style tools (such as windowed sinc() FIR lpfs) work well there in comparison, notably better than
 * if we just used the CIC to decimate all the way down to baseband.
 *
 * CICs depend on sane cancellation behaviour, so they must be done in integer arithmetic and I can only guarantee them working
 * in 1s/2s-complement implementations, so no, the float conversion must happen afterwards, not beforehand.
 * Do not change this, a naive floating-point implementation of a CIC filter just gives utter absurdity.
 * (Also they technically need signed overflow to have wrap semantics, I've used -fwrapv for everything for ages already,
 *  but they didn't break the one time I tried to compile without that flag. That being said, caveat emptor, especially with gcc).
 */

#ifndef FFTCOMPLEX_T_DEFINED
typedef struct FFTComplex {
    float re, im;
} FFTComplex;
#define FFTCOMPLEX_T_DEFINED
#endif

typedef struct _comb_filter_t {
    int32_t d_l[4];
    uint32_t M, cur_d;
} comb_filter_t;

typedef struct _cic_filter_t {
    int32_t d_sum;
    comb_filter_t comb;
} cic_filter_t;

static int32_t cycle_comb(comb_filter_t *c, uint32_t c1, uint32_t idx) {
    int32_t t1, i = c->cur_d - idx;
    if (i < 0)
        i += c->M;
    t1 = c->d_l[i];
    c->d_l[c->cur_d++] = c1;
    c->cur_d &= (idx-1);
    return (c1 - t1);
}

void cic_init(cic_filter_t *cic, unsigned int M)
{
    cic->d_sum = 0;
    cic->comb.M = M;
    cic->comb.d_l[0] = 0;
    cic->comb.d_l[1] = 0;
    cic->comb.d_l[2] = 0;
    cic->comb.d_l[3] = 0;
    cic->comb.cur_d = 0;
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

unsigned int cic4_decimate(cic_filter_t *cic, float *out, uint8_t *samples_in, unsigned int samples_count)
{
    unsigned int i, j = 0, decim_ctr = 0;
    for (i = 0; i < samples_count; i++) {
        int32_t sample, sample_out;
        cic->d_sum = ((int32_t)(samples_in[i] ^ 0x80) - cic->d_sum);
        sample = cic->d_sum;
        if (++decim_ctr < 16) continue;
        {
            sample_out = cycle_comb(&cic->comb, sample, 4);
            out[j++] = (float)sample_out;
        }
    }
    return j;
}

unsigned int cic1_decimate(cic_filter_t *cic, float *out, uint8_t *samples_in, unsigned int samples_count)
{
    unsigned int i, j = 0, decim_ctr = 0;
    for (i = 0; i < samples_count; i++) {
        int32_t sample, sample_out;
        cic->d_sum = ((int32_t)(samples_in[i] ^ 0x80) - cic->d_sum);
        sample = cic->d_sum;
        if (++decim_ctr < 16) continue;
        {
            sample_out = cycle_comb(&cic->comb, sample, 2);
            out[j++] = (float)sample_out;
        }
    }
    return j;
}

/* Compensation filters for the above CIC to get approximately linear behaviour instead of a very clear sinc(x) bias.
 * These are annoyingly hard to compute, but have the nice property that they're basically invariant over absurdly
 * large parts of their input domains: for instace, if you wake the command for generating the fir1 table and make
 * it generate a 32-coeff table instead of a 16-coeff one, it's identical to fir4 up to the fifth decimal digit.
 * C(0) = RM, where R is your decimation factor and M is your differential delay.
 * C(1) = N, where N is the number of sequential stages you have, but for RM > 4 it might as well always be 1.
 * Or 42. Or whatever, really.
 *
 * You must have 2/Fc < 2/RM, or else the filter will be unstable (this is easy to see - you'll get negative coeffs).
 */

/* FIR compensation for the M=4, R=16 decimator, computed using:
 * b = firceqrip(32,0.025,[0.05 0.03],'invsinc',[32 1])
 */
static const float fir4[33] = {
   0.01862255, 0.0080027, 0.0095611, 0.0111938,
   0.01287605, 0.0145805, 0.0162777, 0.0179370,
   0.01952699, 0.0210172, 0.0223778, 0.0235812,
   0.02460248, 0.0254203, 0.0260172, 0.0263806,
   0.02650262, 0.0263806, 0.0260172, 0.0254203,
   0.02460248, 0.0235812, 0.0223778, 0.0210172,
   0.01952700, 0.0179370, 0.0162777, 0.0145805,
   0.01287605, 0.0111938, 0.0095611, 0.0080027,
   0.01862255
};

/* FIR compensation for the M=1, R=16 decimator, computed using:
 * b = firceqrip(16,0.1,[0.05 0.03],'invsinc',[8 1])
 */
static const float fir1[17] = {
   0.00996307, 0.03474873, 0.03449451, 0.05410495,
   0.06718299, 0.08154095, 0.09233568, 0.09960537,
   0.10204750, 0.09960537, 0.09233568, 0.08154095,
   0.06718299, 0.05410495, 0.03449451, 0.03474873,
   0.00996307
};

void cic_compensate(FFTComplex *v, unsigned int len)
{
	FFTComplex p;
    unsigned int i, j;
	p.re = 0.0f;
	p.im = 0.0f;
    for (i = 0; i < len; i++) {
        for (j = 0; j < 17; j++) {
		    float t = fir1[j];
		    p.re += v[i+j].re * t;
		    p.im += v[i+j].im * t;
        }
        v[i] = p;
    }
}

