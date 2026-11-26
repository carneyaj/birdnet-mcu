#include "swish.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include "esp_nn.h"
#include "esp_nn_multicore.h"
#include "tensorflow/lite/micro/kernels/internal/common.h"
#include "tensorflow/lite/micro/kernels/internal/quantization_util.h"
#include "tensorflow/lite/micro/kernels/internal/reference/integer_ops/logistic.h"
#include "tensorflow/lite/micro/kernels/kernel_util.h"

namespace {
using namespace tflite::micro;
struct Swish { const int8_t* lut; int size; int8_t zero; };

void* init(TfLiteContext* ctx, const char*, size_t) {
    return ctx->AllocatePersistentBuffer(ctx, sizeof(Swish));
}

TfLiteStatus prepare_lookup(TfLiteContext* ctx, TfLiteNode* node, bool padded, int outputs = 1) {
    TF_LITE_ENSURE(ctx, node->inputs->size == 1 && node->outputs->size == outputs);
    auto* micro = GetMicroContext(ctx);
    auto* in = micro->AllocateTempInputTensor(node, 0);
    auto* out = micro->AllocateTempOutputTensor(node, 0);
    TF_LITE_ENSURE(ctx, node->user_data && in && out && node->custom_initial_data &&
                   node->custom_initial_data_size == 256);
    TF_LITE_ENSURE(ctx, in->type == kTfLiteInt8 && out->type == kTfLiteInt8);
    if (padded) {
        TF_LITE_ENSURE(ctx, in->dims->size == 4 && out->dims->size == 4);
        const int* x = in->dims->data;
        const int* y = out->dims->data;
        TF_LITE_ENSURE(ctx, x[0] == 1 && y[0] == 1 && y[1] == x[1] + 2 &&
                       y[2] == x[2] + 2 && y[3] == x[3]);
    } else {
        TF_LITE_ENSURE(ctx, in->bytes == out->bytes);
    }
    auto* op = static_cast<Swish*>(node->user_data);
    op->size = in->bytes;
    op->zero = out->params.zero_point;
    op->lut = static_cast<const int8_t*>(node->custom_initial_data);

    // Use the exact LOGISTIC and non-broadcast MUL arithmetic from the linked
    // target kernels. Check every signed byte against the exported LiteRT table.
    alignas(16) int8_t domain[256], sigmoid[256], target[256];
    for (int i = 0; i < 256; ++i) domain[i] = static_cast<int8_t>(i - 128);
    int left_shift;
    const double mantissa = std::frexp(std::ldexp(double(in->params.scale), 27), &left_shift);
    const int multiplier = std::lround(std::ldexp(mantissa, 31));
    reference_integer_ops::Logistic(in->params.zero_point, CalculateInputRadius(4, left_shift, 31),
                                   multiplier, left_shift, 256, domain, sigmoid);
    int32_t mul;
    int shift;
    QuantizeMultiplier(double(in->params.scale) / 256.0 / out->params.scale, &mul, &shift);
    esp_nn_mul_elementwise_s8(domain, sigmoid, -in->params.zero_point, 128, target,
                              out->params.zero_point, mul, shift, -128, 127, 256);
    if (std::memcmp(target, op->lut, 256)) {
        auto* exact = static_cast<int8_t*>(ctx->AllocatePersistentBuffer(ctx, 256));
        TF_LITE_ENSURE(ctx, exact);
        std::memcpy(exact, target, 256);
        op->lut = exact;
    }
    micro->DeallocateTempTfLiteTensor(out);
    micro->DeallocateTempTfLiteTensor(in);
    return kTfLiteOk;
}

TfLiteStatus prepare(TfLiteContext* ctx, TfLiteNode* node) { return prepare_lookup(ctx, node, false); }
TfLiteStatus prepare_padded(TfLiteContext* ctx, TfLiteNode* node) { return prepare_lookup(ctx, node, true); }

struct MeanSwish {
    Swish activation;
    int positions, channels, shift, zero;
    int32_t multiplier;
};
void* init_mean(TfLiteContext* ctx, const char*, size_t) {
    return ctx->AllocatePersistentBuffer(ctx, sizeof(MeanSwish));
}
TfLiteStatus prepare_mean(TfLiteContext* ctx, TfLiteNode* node) {
    TF_LITE_ENSURE_OK(ctx, prepare_lookup(ctx, node, false, 2));
    auto* micro = GetMicroContext(ctx);
    auto* activation = micro->AllocateTempOutputTensor(node, 0);
    auto* mean = micro->AllocateTempOutputTensor(node, 1);
    TF_LITE_ENSURE(ctx, activation && mean && activation->dims->size == 4 && mean->dims->size == 4);
    const auto* x = activation->dims->data;
    const auto* y = mean->dims->data;
    TF_LITE_ENSURE(ctx, mean->type == kTfLiteInt8 && x[0] == 1 && x[1] > 0 && x[2] > 0 &&
                   x[3] > 0 && y[0] == 1 && y[1] == 1 && y[2] == 1 && y[3] == x[3] &&
                   int64_t(x[1])*x[2] <= INT32_MAX/255);
    auto& op = *static_cast<MeanSwish*>(node->user_data);
    op.positions = x[1]*x[2];
    op.channels = x[3];
    op.zero = mean->params.zero_point;
    // Match the two-step multiplier adjustment in the linked TFLM MEAN.
    QuantizeMultiplier(double(activation->params.scale)/mean->params.scale, &op.multiplier, &op.shift);
    const int adjust = std::min(31 - __builtin_clz(unsigned(op.positions)), 31 + op.shift);
    TF_LITE_ENSURE(ctx, adjust >= 0);
    op.multiplier = (int64_t(op.multiplier) << adjust) / op.positions;
    op.shift -= adjust;
    micro->DeallocateTempTfLiteTensor(mean);
    micro->DeallocateTempTfLiteTensor(activation);
    return kTfLiteOk;
}

struct MeanJob {
    const MeanSwish& op;
    const int8_t *lut, *input;
    int8_t *activation, *mean;
    int begin, end;
};
void lookup_mean(void* arg) {
    const auto& job = *static_cast<MeanJob*>(arg);
    const auto& op = job.op;
    constexpr int tile = 128;
    int32_t sums[tile];
    for (int base = job.begin; base < job.end; base += tile) {
        const int n = std::min(tile, job.end - base);
        std::fill_n(sums, n, -int(op.activation.zero)*op.positions);
        for (int pos = 0; pos < op.positions; ++pos) {
            const int at = pos*op.channels + base;
            for (int c = 0; c < n; ++c) {
                const int8_t v = job.lut[job.input[at+c]];
                job.activation[at+c] = v;
                sums[c] += v;
            }
        }
        for (int c = 0; c < n; ++c) {
            int v = MultiplyByQuantizedMultiplier(sums[c], op.multiplier, op.shift) + op.zero;
            job.mean[base+c] = std::clamp(v, -128, 127);
        }
    }
}
TfLiteStatus eval_mean(TfLiteContext* ctx, TfLiteNode* node) {
    const auto& op = *static_cast<const MeanSwish*>(node->user_data);
    alignas(16) int8_t lut[256];
    std::memcpy(lut, op.activation.lut, sizeof(lut));
    MeanJob job{op, lut+128, GetEvalInput(ctx,node,0)->data.int8, GetEvalOutput(ctx,node,0)->data.int8,
                GetEvalOutput(ctx,node,1)->data.int8, 0, op.channels};
    const int split = (op.channels / 2) & ~63;
    if (split && esp_nn_dual_core_begin()) {
        MeanJob first = job;
        first.end = split;
        if (esp_nn_dual_core_run(lookup_mean, &first)) {
            job.begin = split;
            lookup_mean(&job);
            esp_nn_dual_core_wait();
            esp_nn_dual_core_end();
            return kTfLiteOk;
        }
        esp_nn_dual_core_end();
    }
    lookup_mean(&job);
    return kTfLiteOk;
}

inline void lookup_span(const int8_t* lut, const int8_t* in, int8_t* out, int n) {
    const int8_t* biased = lut + 128;
    #pragma GCC unroll 8
    for (int i = 0; i < n; ++i) out[i] = biased[in[i]];
}

struct Job { const int8_t *lut, *in; int8_t* out; int n; };
void lookup(void* arg) {
    const auto& job = *static_cast<Job*>(arg);
    lookup_span(job.lut, job.in, job.out, job.n);
}

TfLiteStatus eval(TfLiteContext* ctx, TfLiteNode* node) {
    const auto& op = *static_cast<const Swish*>(node->user_data);
    alignas(16) int8_t local[256];
    std::memcpy(local, op.lut, sizeof(local));
    Job job{local, GetEvalInput(ctx, node, 0)->data.int8, GetEvalOutput(ctx, node, 0)->data.int8, op.size};
    if (job.n >= 32768 && esp_nn_dual_core_begin()) {
        Job first = job;
        first.n = (job.n / 2) & ~63;
        if (esp_nn_dual_core_run(lookup, &first)) {
            job.in += first.n;
            job.out += first.n;
            job.n -= first.n;
            lookup(&job);
            esp_nn_dual_core_wait();
            esp_nn_dual_core_end();
            return kTfLiteOk;
        }
        esp_nn_dual_core_end();
    }
    lookup(&job);
    return kTfLiteOk;
}

struct RowJob {
    const int8_t *lut, *in;
    int8_t* out;
    int rows, row_bytes, channels;
    int8_t zero;
};

void lookup_rows(void* arg) {
    const auto& job = *static_cast<RowJob*>(arg);
    const int8_t* src = job.in;
    int8_t* dst = job.out;
    for (int row = 0; row < job.rows; ++row) {
        std::memset(dst, job.zero, job.channels);
        lookup_span(job.lut, src, dst + job.channels, job.row_bytes);
        std::memset(dst + job.channels + job.row_bytes, job.zero, job.channels);
        src += job.row_bytes;
        dst += job.row_bytes + 2 * job.channels;
    }
}

TfLiteStatus eval_padded(TfLiteContext* ctx, TfLiteNode* node) {
    const auto& op = *static_cast<const Swish*>(node->user_data);
    const auto* in = GetEvalInput(ctx, node, 0);
    auto* dst = GetEvalOutput(ctx, node, 0)->data.int8;
    const int rows = in->dims->data[1], channels = in->dims->data[3];
    const int row_bytes = in->dims->data[2] * channels;
    const int stride = row_bytes + 2 * channels;
    std::memset(dst, op.zero, stride);
    std::memset(dst + (rows + 1) * stride, op.zero, stride);
    alignas(16) int8_t lut[256];
    std::memcpy(lut, op.lut, sizeof(lut));
    RowJob job{lut, in->data.int8, dst + stride, rows, row_bytes, channels, op.zero};
    if (rows >= 2 && op.size >= 32768 && esp_nn_dual_core_begin()) {
        RowJob first = job;
        first.rows = rows / 2;
        if (esp_nn_dual_core_run(lookup_rows, &first)) {
            job.in += first.rows * row_bytes;
            job.out += first.rows * stride;
            job.rows -= first.rows;
            lookup_rows(&job);
            esp_nn_dual_core_wait();
            esp_nn_dual_core_end();
            return kTfLiteOk;
        }
        esp_nn_dual_core_end();
    }
    lookup_rows(&job);
    return kTfLiteOk;
}
}

TFLMRegistration register_birdnet_swish() { return tflite::micro::RegisterOp(init, prepare, eval); }
TFLMRegistration register_birdnet_swish_pad() { return tflite::micro::RegisterOp(init, prepare_padded, eval_padded); }
TFLMRegistration register_birdnet_swish_mean() { return tflite::micro::RegisterOp(init_mean, prepare_mean, eval_mean); }

