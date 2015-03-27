#ifndef __RESAMPLE_H__
#define __RESAMPLE_H__

#define FFMAX(a,b) ((a) > (b) ? (a) : (b))
#define FFMIN(a,b) ((a) > (b) ? (b) : (a))

typedef struct _ResampleContext ResampleContext;
typedef struct _ResampleBufferContext{
    float *buf;
    uint32_t bufsize;
    int index;
    int frac;
}ResampleBufferContext;

static inline void resample_ch_init(ResampleBufferContext *b){
    b->index= 0;
    b->frac= 0;
    b->bufsize= 128;
    b->buf= malloc(b->bufsize * sizeof(float));
}

static inline void resample_ch_free(ResampleBufferContext *b){
    free(b->buf);
}

ResampleContext *resample_init(uint32_t out_rate, uint32_t in_rate, uint32_t filter_size);
void resample_close(ResampleContext *c);
uint32_t resample(ResampleContext *c, ResampleBufferContext *b, float *dst, float *src, uint32_t src_size);

#endif
