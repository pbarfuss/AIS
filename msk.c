#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "rtl-ais.h"
#define TWOPIf 6.2831853f
float fast_atanf(float x);

/* complex multiplication: p = a * b */
#define CMUL(pre, pim, are, aim, bre, bim) \
{\
    float _are = (are);\
    float _aim = (aim);\
    float _bre = (bre);\
    float _bim = (bim);\
    (pre) = _are * _bre - _aim * _bim;\
    (pim) = _are * _bim + _aim * _bre;\
}

static float fast_atan2f(float y, float x)
{
    float z;

    if ((y != y) || (x != x) || (y < 1e-12f))
        return 0.0f;

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
    return z;
}

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

static void fast_sincosf(float x, float *sinx, float *cosx)
{
    unsigned int n = (uint32_t)(fabsf(x)*invpio2);
    float y = (fabsf(x) - (float)M_PI*0.5f*(float)n);
    float s = k_sinf(y);
    float c = k_cosf(y);
    if (n & 2) {
        c = -c;
        s = -s;
    }
    *sinx = ((n & 1) ? c : s);
    *cosx = ((n & 1) ? -s : c);
}

static inline float fast_sinf(float x) {
    float y = fabsf(x), z;
    uint32_t n = (uint32_t)(y*inv_pi);
    z = k_sinf(y - ((float)M_PI * (float)n));
    if (x < 0.0f) x = -x;
    return ((n&1) ? -z : z);
}

/*
 * Highpass filter with cutoff @ 31Hz, used for DC removal.
 * A bit higher might be better, should test.
static const float hpf_31_poles[2]  = { -1.978881836, 0.979125977 };
static const float hpf_31_gain      = 0.989501953;
void hpf(float *out, float *in, float *mem, unsigned int n, float pole_coeffs[2], float gain)
{
    unsigned int i;
    float tmp;

    for (i = 0; i < n; i++) {
        tmp    = gain * in[i] - (pole_coeffs[0] * mem[0] + pole_coeffs[1] * mem[1]);
        out[i] = tmp - 2.0f*mem[0] + mem[1];
        mem[1] = mem[0];
        mem[0] = tmp;
    }
}
*/

#define PLLKa 1.8991680918e+02f
#define PLLKb 9.8503292076e-01f
#define PLLKc 0.9995f

void init_msk_demod(msk_t *ch, unsigned int samplerate)
{
    unsigned int i;

    ch->MskFreq=(2.0f * (float)M_PI * 4800.0f)/(float)samplerate;
    ch->MskPhi=ch->MskClk=0;
    ch->MskS=0;

    ch->MskKa=PLLKa/(float)samplerate;
    ch->MskDf=ch->Mska=0;

#ifdef __arm__
    ch->flen=(unsigned int)(samplerate/4800.0f);
#else
    ch->flen=lrintf(samplerate/4800.0f);
#endif
    ch->idx=0;

    ch->outbits = 0;
    ch->nbits = 0;

    for(i=0;i<ch->flen;i++) {
        ch->h[i]=k_sinf(2.0f*(float)M_PI*4800.0f/(float)samplerate*(i+1));
        ch->I[i]=ch->Q[i]=0;
    }
}

void putbit(msk_t *ch, float v)
{
    unsigned char b = 0;
    ch->outbits <<= 1;
    if(v>0) {
        ch->outbits |= 1;
        b = 1;
    }
    if (ch->nbits >= 8) {
        char out[5];
	    /* feed to the decoder */
        ais_bytearray_append(ch->outbits);
        sprintf(out, "0x%02x, ", ch->outbits);
        ch->nbits = 0;
        ch->outbits = 0;
        write(1, out, 5);
    }
}

void demod_msk(msk_t *ch, FFTComplex *input, unsigned int ninput)
{
    float iv,qv,c,s,bit,tmp;
    float dphi;
    float p,sp,cp;
    int i, j, idx;

    for (i = 0; i < ninput; i++) {
        p=ch->MskFreq+ch->MskDf;
        ch->MskClk+=p;

        /* Wrap phase mod 2PI */
        p=ch->MskPhi+p;
        if(p>=2.0f*(float)M_PI){
            p-=2.0f*(float)M_PI;
        }
        ch->MskPhi=p;

        if(ch->MskClk>3.0f*(float)M_PI/2.0f) {
            ch->MskClk-=3.0f*(float)M_PI/2.0f;
            idx=ch->idx;

            /* matched filter */
            for(j=0,iv=qv=0;j<39;j++) {
                int k=(idx+1+j)%ch->flen;
                iv+=ch->h[j]*ch->I[k];
                qv+=ch->h[j]*ch->Q[k];
            }

            if((ch->MskS&1)==0) {
                if(iv>=0)
                    dphi=fast_atan2f(-qv,iv);
                else
                    dphi=fast_atan2f(qv,-iv);
                bit=-iv;
            } else {
                if(qv>=0)
                    dphi=fast_atan2f(iv,qv);
                else
                    dphi=fast_atan2f(-iv,-qv);
                bit=qv;
            }
            if (ch->MskS&2) bit = -bit;
            putbit(ch, bit);
            ch->MskS=(ch->MskS+1)&3;

            /* PLL */
            dphi*=ch->MskKa;
            ch->MskDf=PLLKc*ch->MskDf+dphi-PLLKb*ch->Mska;
            ch->Mska=dphi;
        }

        /* DC blocking - now done in main decode f'n immediate defined above this one. */

        /* FI */
        fast_sincosf(p,&sp,&cp);
        ch->I[ch->idx]=input[i].re*cp;
        ch->Q[ch->idx]=input[i].im*sp;
        ch->idx=(ch->idx+1)%ch->flen;
    }
}

