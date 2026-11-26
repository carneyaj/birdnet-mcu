#include "frontend.h"
#include <cstdlib>
#include <new>
#include "esp_nn_multicore.h"
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif
#include <algorithm>
#include <cmath>
#include <cstring>
#include "generated/hann.h"
#include "birdnet_frontend_assets.h"
#include "birdnet_frontend_tables.h"
#include "dsps_fft4r.h"

namespace {
using namespace birdnet_frontend;
constexpr int kFft = 1024;
float background_window[kFft], background_bins[kFft / 2 + 1];
struct Branch {
    const uint16_t *starts, *lengths, *offsets;
    const float *weights, *mantissa, *exp2;
};
constexpr Branch branches[] = {
    {kBranch1MelStarts, kBranch1MelLengths, kBranch1MelOffsets, kBranch1MelWeights,
     birdnet_tables::kMantissa0, birdnet_tables::kExp0},
    {kBranch2MelStarts, kBranch2MelLengths, kBranch2MelOffsets, kBranch2MelWeights,
     birdnet_tables::kMantissa1, birdnet_tables::kExp1}
};
int bins[2];
static_assert(510 * kBranch1Hop + 2 * (kFft - 1) < kSamples);
static_assert(510 * kBranch2Hop + kFft <= kSamples);

// Direct projection coefficients for values close to a quantization boundary.
// Symmetry reduces each 1024-point real projection to 513 integer products.
constexpr int kHalf = kFft/2;
using Projection = int32_t[kHalf+1];
Projection* kernels;
float weight_sum[2][kNumMels], boundaries[2][256];
struct Split { int32_t hi, lo; };
Split precise_cos[kFft];
struct FrameScratch {
    alignas(16) float fft[2*kFft];
    float real_bins[kHalf+1], checked_bins[kHalf+1];
    int32_t paired[kHalf+1];
    Split original_pairs[kHalf+1];
    bool bin_valid[kHalf+1];
};
FrameScratch primary;

void init_projection() {
    constexpr size_t bytes = 2*kNumMels*sizeof(Projection);
#ifdef ESP_PLATFORM
    kernels = static_cast<Projection*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
#else
    kernels = static_cast<Projection*>(std::malloc(bytes));
#endif
    ESP_ERROR_CHECK(kernels ? ESP_OK : ESP_ERR_NO_MEM);
    for (int j = 0; j < kFft; ++j) {
        double scaled = xiao_frontend::cosine[j] * (1<<22);
        int32_t hi = std::lround(scaled);
        precise_cos[j] = {hi, int32_t(std::lround((scaled-hi)*(1<<22)))};
    }
    for (int ch = 0; ch < 2; ++ch) {
        const Branch& b = branches[ch];
        for (int m = 0; m < kNumMels; ++m) {
            double total = 0;
            for (int k = 0; k < b.lengths[m]; ++k) total += b.weights[b.offsets[m]+k];
            weight_sum[ch][m] = total;
            for (int j = 0; j <= kHalf; ++j) {
                double value = 0;
                for (int k = 0; k < b.lengths[m]; ++k)
                    value += b.weights[b.offsets[m]+k] * xiao_frontend::cosine[((b.starts[m]+k)*j) % kFft];
                kernels[ch*kNumMels+m][j] = total ? std::lround(value * (1<<28) / total) : 0;
            }
        }
    }
}

// These finite, in-range values use round-to-nearest-even in the reference
// frontend. Keep the conversion in the S3 FPU, including exact half steps.
inline int nearest_integer(float value) {
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32S3)
    int32_t result;
    asm("round.s %0, %1, 0" : "=a"(result) : "f"(value));
    return result;
#else
    return static_cast<int32_t>(std::nearbyint(value));
#endif
}

void pair_samples(FrameScratch& scratch, const int16_t* src, int step, float norm, float offset) {
    auto sample = [&](int j) {
        float value = (float(src[step*j])*norm + offset) * xiao_frontend::hann[j];
        return nearest_integer(value * (1<<28));
    };
    scratch.paired[0] = sample(0);
    scratch.paired[kHalf] = sample(kHalf);
    for (int j = 1; j < kHalf; ++j) scratch.paired[j] = sample(j) + sample(kFft-j);
}

float precise_projection(const FrameScratch& scratch, int ch, int m) {
    const auto& coefficients = kernels[ch*kNumMels+m];
    int64_t total = 0;
    // Group eight Q56 products before shifting, leaving accumulator headroom.
    for (int j = 0; j < kHalf; j += 8) {
        int64_t part = 0;
        for (int k = 0; k < 8; ++k) part += int64_t(scratch.paired[j+k]) * coefficients[j+k];
        total += part >> 8;
    }
    total += (int64_t(scratch.paired[kHalf])*coefficients[kHalf]) >> 8;
    return float(total) * (1.f / (1LL<<48)) * weight_sum[ch][m] * (ch == 0 ? 2.f : 1.f);
}

// In the small region where integer dot-product rounding can still affect a
// byte, reproduce the reference order: round each real FFT bin to float, then
// multiply by its mel weight. Neighboring mel filters share the bin cache.
void exact_pairs(FrameScratch& scratch, const int16_t* src, int step, float norm, float offset) {
    auto sample = [&](int j) -> Split {
        float value = (float(src[j*step])*norm + offset) * xiao_frontend::hann[j];
        float scaled = value * (1<<22);
        int32_t hi = nearest_integer(scaled);
        return {hi, nearest_integer((scaled-float(hi))*(1<<22))};
    };
    scratch.original_pairs[0] = sample(0);
    scratch.original_pairs[kHalf] = sample(kHalf);
    for (int j = 1; j < kHalf; ++j) {
        Split a = sample(j), b = sample(kFft-j);
        scratch.original_pairs[j] = {a.hi+b.hi, a.lo+b.lo};
    }
    std::fill_n(scratch.bin_valid, kHalf+1, false);
}

template<int Sign>
void folded_dot(const Split* pairs, int bin, int64_t& high, int64_t& cross) {
    for (int j = 0, phase = 0; j < kHalf/2; ++j, phase = (phase+bin) % kFft) {
        const auto a = pairs[j], b = pairs[kHalf-j], w = precise_cos[phase];
        const int32_t h = a.hi + Sign*b.hi, l = a.lo + Sign*b.lo;
        high += int64_t(h)*w.hi;
        cross += int64_t(h)*w.lo + int64_t(l)*w.hi;
    }
    const auto x = pairs[kHalf/2], w = precise_cos[(bin*(kHalf/2)) % kFft];
    high += int64_t(x.hi)*w.hi;
    cross += int64_t(x.hi)*w.lo + int64_t(x.lo)*w.hi;
    // Independently rounded mirrored cosines can differ by one Q44 count.
    // The exporter derives these signed sample indices from the same table.
    for (int i = xiao_frontend::mirror_offsets[bin]; i < xiao_frontend::mirror_offsets[bin+1]; ++i) {
        const int correction = xiao_frontend::mirror_corrections[i];
        cross += correction > 0 ? pairs[correction].hi : -pairs[-correction].hi;
    }
}

float reference_projection(FrameScratch& scratch, int ch, int m) {
    const Branch& b = branches[ch];
    float sum = 0;
    for (int k = 0; k < b.lengths[m]; ++k) {
        int bin = b.starts[m]+k;
        if (!scratch.bin_valid[bin]) {
            // Q22 high parts and Q44 remainders keep the S3 hot loop in
            // integer MACs. Omitted low × low terms total less than 2e-11.
            int64_t high = 0, cross = 0;
            if (bin & 1) folded_dot<-1>(scratch.original_pairs, bin, high, cross);
            else folded_dot<1>(scratch.original_pairs, bin, high, cross);
            double real = double(high)*0x1p-44 + double(cross)*0x1p-66;
            scratch.checked_bins[bin] = float(real) * (ch == 0 ? 2.f : 1.f);
            scratch.bin_valid[bin] = true;
        }
        sum = std::fma(scratch.checked_bins[bin], b.weights[b.offsets[m]+k], sum);
    }
    return sum;
}

bool close_to_boundary(int q, int ch, float magnitude, float error) {
    return (q > -128 && magnitude - boundaries[ch][q+127] <= error) ||
           (q < 127 && boundaries[ch][q+128] - magnitude <= error);
}

int prior_equal_frame(const int16_t* slow, const int16_t* fast, int frame) {
    // Periodic or gated audio can repeat whole frames. Check every input byte
    // before reusing a result. The two branches have different frame hops.
    for (int lag : {1, 2, 8}) {
        if (frame < lag) break;
        const int16_t* prev_slow = slow - lag*kBranch1Hop;
        const int16_t* prev_fast = fast - lag*kBranch2Hop;
        if (slow[0] == prev_slow[0] && fast[0] == prev_fast[0] &&
            slow[1010] == prev_slow[1010] && fast[505] == prev_fast[505] &&
            !std::memcmp(fast, prev_fast, kFft*sizeof(int16_t)) &&
            !std::memcmp(slow, prev_slow, (2*kFft-1)*sizeof(int16_t))) return lag;
    }
    return 0;
}

inline float compress(float energy, const Branch& b, int ch) {
    if (energy == 0) return 0;
    uint32_t bits;
    __builtin_memcpy(&bits, &energy, sizeof(bits));
    const int exp = int(bits >> 23) - 126;
    if (exp < -64 || exp > 64) return std::pow(energy, kBranchPowerExponent[ch]);
    const int idx = (bits >> 15) & 255;
    const float fraction = (bits & 32767) * (1.f / 32768);
    return (b.mantissa[idx] + fraction * (b.mantissa[idx+1] - b.mantissa[idx])) * b.exp2[exp+64];
}

struct FrameJob {
    FrameScratch* scratch;
    const int16_t* pcm;
    int8_t* out;
    float norm, offset, scale;
    int zero;
    bool constant;
    int begin, end;
    float multiplier[2], bias[2];
};

void process_frames(void* arg) {
    auto& job = *static_cast<FrameJob*>(arg);
    auto& scratch = *job.scratch;
    auto& fft = scratch.fft;
    auto& real_bins = scratch.real_bins;
    const auto* pcm = job.pcm;
    auto* out = job.out;
    const float norm = job.norm, offset = job.offset;
    const bool constant = job.constant;
    for (int frame = job.begin; frame < job.end; ++frame) {
        const int16_t* slow = pcm + frame * kBranch1Hop;
        const int16_t* fast = pcm + frame * kBranch2Hop;
        if (int lag = prior_equal_frame(slow, fast, frame-job.begin)) {
            for (int m = 0; m < kNumMels; ++m) {
                int8_t* dst = out + 2*(m*kNumFrames+frame);
                dst[0] = dst[-2*lag];
                dst[1] = dst[1-2*lag];
            }
            continue;
        }
        float peak = 0;
        if (!constant) {
            for (int j = 0; j < kFft; ++j) {
                fft[2*j] = (float(slow[2*j]) * norm + offset) * xiao_frontend::hann[j] - background_window[j];
                fft[2*j+1] = (float(fast[j]) * norm + offset) * xiao_frontend::hann[j] - background_window[j];
                peak = std::max(peak, std::max(std::abs(fft[2*j]), std::abs(fft[2*j+1])));
            }
            ESP_ERROR_CHECK(dsps_fft4r_fc32(fft, kFft));
            ESP_ERROR_CHECK(dsps_bit_rev4r_fc32(fft, kFft));
        }
        for (int ch = 0; ch < 2; ++ch) {
            // For Z = FFT(a + ib), adding mirrored real or imaginary bins
            // recovers 2 Re(A) or 2 Re(B). Keep branch one's decimation gain.
            const float gain = ch == 0 ? 1.0f : 0.5f;
            for (int k = 0; k < bins[ch]; ++k)
                real_bins[k] = gain * ((constant ? 0.f : fft[2*k+ch] + fft[2*((kFft-k) % kFft)+ch]) +
                                          2 * background_bins[k]);
            const auto& b = branches[ch];
            bool paired_ready = false, exact_ready = false;
            for (int m = 0; m < kNumMels; ++m) {
                const float* spectrum = real_bins + b.starts[m];
                const float* weight = b.weights + b.offsets[m];
                float sum = 0;
                for (int k = 0; k < b.lengths[m]; ++k) sum += spectrum[k] * weight[k];
                float feature = compress(sum * sum, b, ch);
                int q = std::clamp(nearest_integer(feature * job.multiplier[ch] + job.bias[ch]), -128, 127);
                // Only recompute bins for which FFT float roundoff could switch the input byte.
                float magnitude = std::abs(sum);
                float error = (ch == 0 ? 2.f : 1.f) * weight_sum[ch][m] * peak * 0.000012f +
                              magnitude * 0.000004f;
                if (!constant && close_to_boundary(q, ch, magnitude, error)) {
                    if (!paired_ready) {
                        pair_samples(scratch, ch == 0 ? slow : fast, ch == 0 ? 2 : 1, norm, offset);
                        paired_ready = true;
                    }
                    sum = precise_projection(scratch, ch, m);
                    magnitude = std::abs(sum);
                    feature = std::pow(sum * sum, kBranchPowerExponent[ch]);
                    q = std::clamp(nearest_integer((feature*kBatchNormMul[ch]+kBatchNormAdd[ch])/job.scale+job.zero), -128, 127);
                    if (close_to_boundary(q, ch, magnitude,
                                          weight_sum[ch][m]*0.00000018f + magnitude*0.000002f)) {
                        if (!exact_ready) {
                            exact_pairs(scratch, ch == 0 ? slow : fast, ch == 0 ? 2 : 1, norm, offset);
                            exact_ready = true;
                        }
                        sum = reference_projection(scratch, ch, m);
                        feature = std::pow(sum * sum, kBranchPowerExponent[ch]);
                        q = nearest_integer((feature*kBatchNormMul[ch]+kBatchNormAdd[ch])/job.scale+job.zero);
                    }
                }
                int8_t* dst = out + ((kNumMels-1-m) * kNumFrames + frame) * 2 + ch;
                *dst = std::clamp(q, -128, 127);
                if (constant)
                    for (int i = 1; i < kNumFrames; ++i) dst[2*i] = *dst;
            }
        }
    }
}

}

void init_frontend() {
    ESP_ERROR_CHECK(dsps_fft4r_init_fc32(nullptr, kFft));
    init_projection();
    for (int ch = 0; ch < 2; ++ch) {
        const auto& b = branches[ch];
        for (int m = 0; m < kNumMels; ++m)
            bins[ch] = std::max(bins[ch], b.starts[m] + b.lengths[m]);
        ESP_ERROR_CHECK(bins[ch] <= kFft / 2 + 1 ? ESP_OK : ESP_ERR_INVALID_SIZE);
    }
}

void make_features(const int16_t* pcm, int8_t* out, float scale, int zero) {
    int low = pcm[0], high = pcm[0];
    int64_t pcm_sum = 0;
    for (int i = 0; i < kSamples; ++i) {
        int v = pcm[i];
        low = std::min(low, v);
        high = std::max(high, v);
        pcm_sum += v;
    }
    auto& fft = primary.fft;
    const float norm = float(2.0 / (double(high - low) + 1e-6));
    // Match the reference's rounding order, including its double offset calculation.
    const float offset = float(-double(low) * norm - 1.0);
    const float background = float(pcm_sum / kSamples) * norm + offset;
    // Subtract the common windowed DC level before packing. Otherwise its
    // leakage from float FFT arithmetic dominates real low-energy bird bands.
    for (int j = 0; j < kFft; ++j) {
        background_window[j] = background * xiao_frontend::hann[j];
        fft[2*j] = float(double(background_window[j]) - double(background) * xiao_frontend::ideal_hann[j]);
        fft[2*j+1] = 0;
    }
    ESP_ERROR_CHECK(dsps_fft4r_fc32(fft, kFft));
    ESP_ERROR_CHECK(dsps_bit_rev4r_fc32(fft, kFft));
    for (int k = 0; k <= kFft/2; ++k) background_bins[k] = fft[2*k];
    background_bins[0] += background * (kFft / 2);
    background_bins[1] -= background * (kFft / 4);
    static float last_scale = 0;
    static int last_zero = 0;
    if (last_scale != scale || last_zero != zero) {
        for (int ch = 0; ch < 2; ++ch)
            for (int i = 0; i < 256; ++i) {
                double value = ((i - 128 + 0.5 - zero) * scale - double(kBatchNormAdd[ch])) / kBatchNormMul[ch];
                boundaries[ch][i] = value <= 0 ? -1.f : std::pow(value, 0.5 / kBranchPowerExponent[ch]);
            }
        last_scale = scale;
        last_zero = zero;
    }
    const bool constant = low == high;
    FrameJob main{&primary, pcm, out, norm, offset, scale, zero, constant, 0, constant ? 1 : kNumFrames,
                  {kBatchNormMul[0]/scale, kBatchNormMul[1]/scale},
                  {kBatchNormAdd[0]/scale+zero, kBatchNormAdd[1]/scale+zero}};
    if (!constant && esp_nn_dual_core_begin()) {
        // Reuse ESP-NN's worker scratch while it is claimed. Invoke starts
        // only after the frontend finishes and releases the worker.
        void* storage = esp_nn_dual_core_scratch(sizeof(FrameScratch));
        if (storage) {
            FrameJob other = main;
            other.scratch = new(storage) FrameScratch;
            other.begin = kNumFrames/2;
            if (esp_nn_dual_core_run(process_frames, &other)) {
                main.end = other.begin;
                process_frames(&main);
                esp_nn_dual_core_wait();
                esp_nn_dual_core_end();
                return;
            }
        }
        esp_nn_dual_core_end();
    }
    process_frames(&main);
}
