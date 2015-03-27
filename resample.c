#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "resample.h"
#define WINDOW_TYPE 9
float resample_scalarproduct_sse(float *v1, float *v2, uint32_t len);

typedef struct _ResampleContext{
    uint32_t num_rate;
    uint32_t den_rate;
    float cutoff;
    uint32_t int_advance;
    uint32_t frac_advance;
    float *filter_bank;
    uint32_t filter_length;
}ResampleContext;

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
    float z;
    z = x*x;
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
        float y, z, fn;
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

static void build_filter(float *filter, float cutoff, unsigned int alpha, int32_t N, uint32_t den_rate)
{
   uint32_t i;
   int32_t j, N2;
   float alpha_i0inv = 1.0f / I0(alpha * alpha);

   N2 = (N >> 1);
   for (i=0;i<den_rate;i++) {
       for (j=0;j<N;j++) {
           //float x = (float)((j-N2+1)-((float)i)/den_rate);
           float x = (float)((N2-j-1)+((float)i)/den_rate);
           if (fabsf(x) > N2) {
               filter[i*N+j] = 0.0f;
           } else {
               float xx = (float)M_PI * x * cutoff;
               float tmp = fabsf(x/(float)N2);
               tmp = (1.0f - tmp*tmp);
               filter[i*N+j] = cutoff * sinc(xx) * I0(alpha * alpha * tmp) * alpha_i0inv;
           }
       }
   }
}

uint32_t ff_div32(uint32_t num, uint32_t den, uint32_t *rem)
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

    if (rem) {
        *rem = num;
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

ResampleContext *resample_init(uint32_t out_rate, uint32_t in_rate, uint32_t filter_size){
    float cutoff= FFMAX(1.0f - 6.5f/(filter_size+8.0f), 0.9f);
    uint32_t a = ff_gcd(in_rate, out_rate);
    ResampleContext *c = malloc(sizeof(struct _ResampleContext));
    c->den_rate= ff_div32(out_rate, a, NULL);
    c->num_rate= ff_div32(in_rate, a, NULL);

    c->filter_length= filter_size;

    /* if upsampling, only need to interpolate, no filter */
    if (c->den_rate > c->num_rate) {
        c->cutoff = 0.95f;
    } else {
        c->cutoff = (cutoff * c->den_rate / c->num_rate);
        c->filter_length= ff_div32((c->filter_length*c->num_rate), c->den_rate, NULL);
        c->filter_length &= (~0x03);
    }
    c->filter_bank= malloc(((c->den_rate+1)*c->filter_length)*sizeof(float));
    build_filter(c->filter_bank, c->cutoff, WINDOW_TYPE, c->filter_length, c->den_rate);

    c->int_advance = ff_div32(c->num_rate, c->den_rate, &(c->frac_advance));
    return c;
}

void resample_close(ResampleContext *c){
    free(c->filter_bank);
}

static uint32_t resample_internal(ResampleContext *c, ResampleBufferContext *b, float *dst, uint32_t src_size){
    uint32_t j, dst_index = 0, N = c->filter_length;
    int index= b->index;
    int frac= b->frac;

    while (index < src_size) {
        float *filter_bank = &c->filter_bank[frac * N];
        float val = ff_resample_scalarproduct_sse(&b->buf[index], filter_bank, N);
        dst[dst_index++] = val;
        frac += c->frac_advance;
        index += c->int_advance;
        if(frac >= c->den_rate) {
            frac -= c->den_rate;
            index++;
        }
    }
    b->index= (index - src_size);
    b->frac= frac;

    for (j = 0; j < N; j++) {
        b->buf[j] = b->buf[j+src_size];
    }
    return dst_index;
}

uint32_t resample(ResampleContext *c, ResampleBufferContext *b, float *dst, float *src, uint32_t src_size){
    uint32_t ilen = src_size, dst_len = 0;
    while (ilen) {
        uint32_t ichunk = (ilen > b->bufsize) ? b->bufsize : ilen;
        memcpy(b->buf+c->filter_length, src, ichunk*sizeof(float));
        dst_len += resample_internal(c, b, &dst[dst_len], ichunk);
        ilen -= ichunk;
        src += ichunk;
    }
    if (!src_size) {
        dst_len += resample_internal(c, b, &dst[dst_len], c->filter_length);
    }
    return dst_len;
}

