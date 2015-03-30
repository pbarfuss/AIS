#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#define FFMAX(a,b) ((a) > (b) ? (a) : (b))
#define FFMIN(a,b) ((a) > (b) ? (b) : (a))

#ifndef FFTCOMPLEX_T_DEFINED
typedef struct FFTComplex {
    float re, im;
} FFTComplex;
#define FFTCOMPLEX_T_DEFINED
#endif

static const float inv_pi  =  0.3183098733;  /* 0x3ea2f984 */
static const float invpio2 =  6.3661980629e-01; /* 0x3f22f984 */

static const float
C1  = -5.0000000000e-01,
C2  =  4.1666667908e-02, /* 0x3d2aaaab */
C3  = -1.3888889225e-03, /* 0xbab60b61 */
C4  =  2.4801587642e-05, /* 0x37d00d01 */
C5  = -2.7557314297e-07, /* 0xb493f27c */
C6  =  2.0875723372e-09, /* 0x310f74f6 */
C7  = -1.1359647598e-11; /* 0xad47d74e */

// Differs from libc cosf on [0, pi/2] by at most 0.0000001229f
// Differs from libc cosf on [0, pi] by at most 0.0000035763f
static inline float k_cosf(float x)
{
    float z = x*x;
    return (1.0f+z*(C1+z*(C2+z*(C3+z*(C4+z*(C5+z*(C6+z*C7)))))));
}

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

float fast_cosf(float x) {
    float y = fabsf(x), z;
    uint32_t n = (uint32_t)(y*inv_pi);
    z = k_cosf(y - (float)M_PI*(float)n);
    return ((n&1) ? -z : z);
}

float fast_sinf(float x)
{
    float y = fabsf(x), z;
    uint32_t n = (uint32_t)(y*inv_pi);
    z = k_sinf(y - (float)M_PI*(float)n);
    z = ((x < 0.0f) ? -z : z);
    return ((n&1) ? -z : z);
}

void gen_shifttable(FFTComplex *shifttab, unsigned int signal_len, unsigned int shift_amount, unsigned int samplerate)
{
	float freqshift = 2 * (float)M_PI * ((float)shift_amount / (float)samplerate); // radians per sample
	float phase = -(float)M_PI;
    unsigned int i;

	for (i = 0; i < signal_len; i++) {
		shifttab[i].re = fast_cosf(phase);
		shifttab[i].im = fast_sinf(phase);
		/* Advance local oscillator phase */
		phase += freqshift;
		if (phase >= (float)M_PI) phase -= 2 * (float)M_PI;
	}
}

static void write_complex_float_array(const char *name, const FFTComplex *data, unsigned int len)
{
    unsigned int i;
    printf("static float %s[%u] = {\n", name, len);
    printf("   ");
    for (i = 0; i < len - 1; i++) {
       printf("{ %.5f, %.5f }, ", data[i].re, data[i].im);
       if ((i & 3) == 3) printf("\n   ");
    }
    printf("{ %.5f, %.5f },\n};\n\n", data[i].re, data[i].im);
}

/* standard suffixes */
float atofs(char *f)
{
    char last;
    int len;
    float suff = 1.0f;
    len = strlen(f);
    last = f[len-3];
    f[len-3] = '\0';
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
    unsigned freq, samplerate, buffer_length;
    FFTComplex *freqshift_table = NULL;

    if (argc < 3) {
        printf("Usage: freqshift_tablegen [freq] [samplerate]\n"
               "       shifts frequency [freq] to baseband via multiplication\n"
               "       (also gives you a second copy at 2*freq that you need to LPF out)\n\n");
        return -1;
    }

    freq = (uint32_t)atofs(argv[1]);
    samplerate = (uint32_t)atofs(argv[2]);
    buffer_length = 2*lrintf(4.0f * (float)M_PI * ((float)samplerate / (float)freq));
    freqshift_table = malloc((buffer_length+1) * sizeof(FFTComplex));

    gen_shifttable(freqshift_table, buffer_length, freq, samplerate);
    write_complex_float_array("ais_chan2_shift_to_baseband_table", freqshift_table, buffer_length);
    return 0;
}


