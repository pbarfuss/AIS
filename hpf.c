#include <stdint.h>
#include <math.h>

static const float inv_pi  =  0.3183098733;  /* 0x3ea2f984 */

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

static inline float fast_cosf(float x) {
    float y = fabsf(x), z;
    uint32_t n = (uint32_t)(y*inv_pi);
    z = k_cosf(y - ((float)M_PI * (float)n));
    return ((n&1) ? -z : z);
}

// zeroes are 1, -2, 1 always, as it's a highpass filter */
float get_hpf_poles_and_gain(float cutoff_freq, float samplerate, float a[3])
{
    float omega = (2.0f * (float)M_PI * cutoff_freq) / (float)samplerate;
    float gain = 0.5f*(1 + fast_cosf(omega));
    const float Q = 0.5f*(float)M_SQRT2;
    float alpha = (fast_cosf((float)M_PI*0.5f - omega) / (2*Q));
    a[0] =  1 + alpha;
    a[1] = -2 * fast_cosf(omega);
    a[2] =  1 - alpha;

    a[1] /= a[0];
    a[2] /= a[0];
    gain /= a[0];
    return gain;
}

// actually run the highpass filter on some samples. in and out must overlap either entirely or not at all.
void hpf(float *out, const float *in, float *mem, unsigned int n, float pole_coeffs[2], float gain)
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

#ifdef STANDALONE

#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    float Fc = strtod(argv[1], NULL);
    float Fs = strtod(argv[2], NULL);
    float a[3];
    float gain = get_hpf_poles_and_gain(Fc, Fs, a);
    printf("Samplerate: %.5f, Cutoff Frequency: %.5f, a[0]: %.5f, a[1]: %.5f, a[2]: %.5f, overall gain: %.5f\n", Fs, Fc, a[0], a[1], a[2], gain);
    return 0;
}

#endif

