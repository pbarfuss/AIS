#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#define FFMAX(a,b) ((a) > (b) ? (a) : (b))
#define FFMIN(a,b) ((a) > (b) ? (b) : (a))
#define WINDOW_TYPE 9

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

/*
 * inv_pi:  24 bits of 1/pi
 * pi_1:   first 17 bit of pi
 * pi_1t:  pi - pi_1
 */
static const float
inv_pi  =  0.3183098733,  /* 0x3ea2f984 */
pi_1    =  3.1415710449,  /* 0x40490f80 */
pi_1t   =  2.1696090698e-5; /* 0x37b60000 */

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
    float z;
    if (x < 0.0f) {
        x = -x;
    }
    if (x < 1e-12f) {
        z = 1.0f;
    } else {
        float y, fn;
        uint32_t n  = (int32_t)(x*inv_pi);
        fn = (float)n;
        y = x-fn*pi_1-fn*pi_1t; /* 1st round good to 40 bit */
        z = k_sinf(y) / x;
        if (n&1) {
            z = -z;
        }
    }
    return z;
}

uint32_t ff_div32(uint32_t num, uint32_t den)
{
    uint32_t bit = 1;
    uint32_t quo;

    /* normalize to make den >= num, but not overflow */
    while ((den < num) && !(den >> 31)) {
        bit <<= 1;
        den <<= 1;
    }
    quo = 0;

    /* generate quotient, one bit at a time */
    while (bit) {
        if (den <= num) {
            num -= den;
            quo += bit;
        }
        bit >>= 1;
        den >>= 1;
    }

    return quo;
}

unsigned int ff_gcd(unsigned int u, unsigned int v) {
     unsigned int k = 0;
     while ((u & 1) == 0  &&  (v & 1) == 0) { /* while both u and v are even */
         u >>= 1;   /* shift u right, dividing it by 2 */
         v >>= 1;   /* shift v right, dividing it by 2 */
         k++;       /* add a power of 2 to the final result */
     }
     /* At this point either u or v (or both) is odd */
     do {
         if ((u & 1) == 0)      /* if u is even */
             u  >>= 1;           /* divide u by 2 */
         else if ((v &  1) == 0) /* else if v is even */
             v  >>= 1;           /* divide v by 2 */
         else if (u >= v)       /* u and v are both odd */
             u = (u-v) >> 1;
         else                   /* u and v both odd, v > u */
             v = (v-u) >> 1;
     } while (u > 0);
     return v << k;  /* returns v * 2^k */
}


float *build_filter(unsigned int in_rate, unsigned int out_rate, unsigned int alpha, uint32_t *filter_length)
{
    uint32_t i;
    uint32_t a = ff_gcd(in_rate, out_rate);
    uint32_t num_rate = ff_div32(in_rate, a);
    uint32_t den_rate = ff_div32(out_rate, a);
    int32_t j, N2, N = *filter_length;
    float alpha_i0inv = 1.0f / I0(alpha * alpha);
    float cutoff= FFMAX(1.0f - 6.5f/(N+8.0f), 0.9f);
    float *filter_bank = NULL;

    cutoff = (cutoff * den_rate / num_rate);
    N = ff_div32((N*num_rate), den_rate);
    N &= (~0x03);
    *filter_length = N;

    filter_bank= malloc(((den_rate+1)*N)*sizeof(float));

    N2 = (N >> 1);
    for (i=0;i<den_rate;i++) {
        for (j=0;j<N;j++) {
            //float x = (float)((j-N2+1)-((float)i)/den_rate);
            float x = (float)((N2-j-1)+((float)i)/den_rate);
            if (fabsf(x) > N2) {
                filter_bank[i*N+j] = 0.0f;
            } else {
                float tmp = fabsf(x/(float)N2);
                tmp = (1.0f - tmp*tmp);
                filter_bank[i*N+j] = cutoff * sinc((float)M_PI * x * cutoff) * I0(alpha * alpha * tmp) * alpha_i0inv;
            }
        }
    }

    return filter_bank;
}

static void write_float_array(const char *name, const float *data, unsigned int len)
{
    unsigned int i;
    printf("static float %s[%u] = {\n", name, len);
    printf("   ");
    for (i = 0; i < len - 1; i++) {
       printf("%.10f, ", data[i]);
       if ((i & 3) == 3) printf("\n   ");
    }
    printf("%.10f\n};\n\n", data[len-1]);
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
    unsigned in_rate, out_rate, filter_length = 16;
    float *filter = NULL;

    if (argc < 3) {
        printf("Usage: downsample_filtergen input_samplerate output_samplerate <filter_length (defaults to 16 if not specified>\n");
        return -1;
    }

    in_rate = (uint32_t)atofs(argv[1]);
    out_rate = (uint32_t)atofs(argv[2]);
    if (argc > 3) {
        filter_length = strtoul(argv[3], NULL, 10);
    }

    if (out_rate > in_rate) {
        printf("Error: output rate > input rate. Just use bicubic interpolation here, there's no advantage to using bandlimited sinc for upsampling.\n");
        return 0;
    }

    filter = build_filter(in_rate, out_rate, WINDOW_TYPE, &filter_length);
    write_float_array("downsample_filter", filter, filter_length-1);
    return 0;
}


