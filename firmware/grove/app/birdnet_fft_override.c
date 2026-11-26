#ifdef ARM_MATH_MVEI
#undef ARM_MATH_MVEI
#endif
#ifdef ARM_MATH_MVEF
#undef ARM_MATH_MVEF
#endif

#define kiss_fft_alloc birdnet_kiss_fft_alloc
#define kiss_fft_stride birdnet_kiss_fft_stride
#define kiss_fft birdnet_kiss_fft
#define kiss_fft_cleanup birdnet_kiss_fft_cleanup
#define kiss_fft_next_fast_size birdnet_kiss_fft_next_fast_size
#define kiss_fftr_alloc birdnet_kiss_fftr_alloc
#define kiss_fftr birdnet_kiss_fftr
#define kiss_fftri birdnet_kiss_fftri

#include "../../../library/inference/tflmtag2209_u55tag2205/third_party/kissfft/kiss_fft.h"
#include "../../../library/inference/tflmtag2209_u55tag2205/third_party/kissfft/kiss_fft.c"
#include "../../../library/inference/tflmtag2209_u55tag2205/third_party/kissfft/tools/kiss_fftr.c"

typedef struct {
    float r;
    float i;
} birdnet_kiss_fft_cpx;

size_t birdnet_kiss_fftr_init_for_nfft(int nfft, void *mem, size_t mem_bytes)
{
    size_t len = mem_bytes;
    birdnet_kiss_fftr_alloc(nfft, 0, mem, &len);
    return len;
}

void birdnet_kiss_fftr_run(void *cfg, const float *timedata, birdnet_kiss_fft_cpx *freqdata)
{
    birdnet_kiss_fftr(cfg, timedata, (kiss_fft_cpx *)freqdata);
}
