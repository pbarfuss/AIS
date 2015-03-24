#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
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
    float z;
    z = x*x;
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
    float z;
    z = x*x;
    return x*(1.0f+z*(S1+z*(S2+z*(S3+z*(S4+z*(S5+z*S6))))));
}

static const float
invpio2 =  6.3661980629e-01, /* 0x3f22f984 */
pio2_1  =  1.5707855225e+00, /* 0x3fc90f80 */
pio2_1t =  1.0804334124e-05; /* 0x37354443 */

uint32_t
__ieee754_rem_pio2f(float x, float *y)
{
    float fn, t;
    uint32_t n;

    t = fabsf(x);
    n  = (uint32_t)(t*invpio2);
    fn = (float)n;
    y[0] = t-fn*pio2_1-fn*pio2_1t; /* 1st round good to 40 bit */
    return n;
}

void fft_sincosf(float x, float *sinx, float *cosx)
{
    float y[2];
    unsigned int n = __ieee754_rem_pio2f(x, y);
    float s = k_sinf(y[0]);
    float c = k_cosf(y[0]);
    switch (n & 3) {
        case 0:
          *sinx = s;
          *cosx = c;
          break;
        case 1:
          *sinx =  c;
          *cosx = -s;
          break;
        case 2:
          *sinx = -s;
          *cosx = -c;
          break;
        default:
          *sinx = -c;
          *cosx =  s;
          break;
    }
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

typedef struct _msk_t
{
    float MskPhi;
    float MskFreq,MskDf;
    float Mska,MskKa;
    float MskClk;
    unsigned int MskS;

    float Mskdc, Mskdcf;

    unsigned int flen, idx;
    float I[40];
    float Q[40];

    unsigned int outbits, nbits;

    unsigned char *bytearray;
    unsigned int nalloc, nbytes;
} msk_t;

#define PLLKa 1.8991680918e+02f
#define PLLKb 9.8503292076e-01f
#define PLLKc 0.9995f

static const float h_48k[40] = {
   0.0784590989, 0.1564344764, 0.2334453762, 0.3090170026,
   0.3826834261, 0.4539904892, 0.5224985480, 0.5877853036,
   0.6494480968, 0.7071067691, 0.7604059577, 0.8090170026,
   0.8526401520, 0.8910064697, 0.9238795638, 0.9510564804,
   0.9723699689, 0.9876883626, 0.9969173670, 0.9999998808,
   0.9969172478, 0.9876882434, 0.9723700285, 0.9510565996,
   0.9238795042, 0.8910064101, 0.8526402116, 0.8090170026,
   0.7604058981, 0.7071069479, 0.6494483352, 0.5877856612,
   0.5224992633, 0.4539917111, 0.3826852441, 0.3090200126,
   0.2334503829, 0.1564420015, 0.0784704387, 0.0000000000
};

static const float h_24k[20] = {
   0.1564344764, 0.3090170026, 0.4539904892, 0.5877853036,
   0.7071067691, 0.8090170026, 0.8910064697, 0.9510564804,
   0.9876883626, 0.9999998808, 0.9876882434, 0.9510565996,
   0.8910064101, 0.8090170026, 0.7071069479, 0.5877856612,
   0.4539917111, 0.3090200126, 0.1564420015, 0.0000000000
};

static float h[256];

void initMsk(msk_t *ch, unsigned int samplerate)
{
    unsigned int i;

    //ch->MskFreq=(2.0f * (float)M_PI * 1200.0f)/(float)samplerate;
    ch->MskFreq=(2.0f * (float)M_PI * 2400.0f)/(float)samplerate;
    ch->MskPhi=ch->MskClk=0;
    ch->MskS=0;

    ch->MskKa=PLLKa/(float)samplerate;
    ch->MskDf=ch->Mska=0;

    ch->Mskdc = 0.0f;
    ch->Mskdcf = 120.0f/(float)samplerate;

    ch->flen=40;
    ch->idx=0;

    ch->outbits = 0;
    ch->nbits = 0;

    for(i=0;i<40;i++) {
        //h[i] = k_sinf((float)(i+1) * (2.0f * (float)M_PI * 1200.0f)/(float)samplerate);
        h[i] = k_sinf((float)(i+1) * (2.0f * (float)M_PI * 1200.0f)/(float)samplerate);
        ch->I[i]=ch->Q[i]=0;
    }
}

void putbit(msk_t *ch, float v)
{
    ch->outbits>>=1;
    if(v>0) {
        ch->outbits|=0x80;
    }
    if (ch->nbits >= 8) {
        ch->bytearray[ch->nbytes++] = ch->outbits;
        ch->nbits = 0;
    }
}

void demodMsk(msk_t *ch, float input)
{
    int idx,j;

    float iv,qv,s,bit;
    float dphi;
    float p,sp,cp;

    /* oscilator */
    p=ch->MskFreq+ch->MskDf;
    ch->MskClk+=p;
    p=ch->MskPhi+p;
    if(p>=2.0f*(float)M_PI){
        p-=2.0f*(float)M_PI;
    }
    ch->MskPhi=p;

    idx=ch->idx;

    if(ch->MskClk>3.0f*(float)M_PI/2.0f) {
        ch->MskClk-=3.0f*(float)M_PI/2.0f;

        /* matched filter */
        for(j=0,iv=qv=0;j<39;j++) {
            int k=(idx+1+j)%40;
            //iv+=h_48k[j]*ch->I[k];
            //qv+=h_48k[j]*ch->Q[k];
            iv+=h[j]*ch->I[k];
            qv+=h[j]*ch->Q[k];
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
        //fprintf(stderr, "qv: %.5f, iv: %.5f, bit: %c\n", qv, iv, ((bit > 0) ? 0x31 : 0x30));
        putbit(ch, bit);
        ch->MskS=(ch->MskS+1)&3;

        /* PLL */
        dphi*=ch->MskKa;
        ch->MskDf=PLLKc*ch->MskDf+dphi-PLLKb*ch->Mska;
        ch->Mska=dphi;
    }

    /* DC blocking */
    //s=input-ch->Mskdc;
    //ch->Mskdc=(1.0f-ch->Mskdcf)*ch->Mskdc+ch->Mskdcf*input;

    /* FI */
    s = input;
    fft_sincosf(p,&sp,&cp);
    ch->I[idx]=s*cp;
    ch->Q[idx]=s*sp;
    ch->idx=(idx+1)%40;
}

