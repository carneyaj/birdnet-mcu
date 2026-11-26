#include "stem_pool.h"
#include <algorithm>
#include <cstring>
#include "tensorflow/lite/micro/kernels/kernel_util.h"

namespace {
constexpr int kTableBytes = 256 + 511;

TfLiteStatus prepare(TfLiteContext* ctx, TfLiteNode* node) {
    TF_LITE_ENSURE(ctx, node->inputs->size == 1 && node->outputs->size == 1 &&
                   node->custom_initial_data && node->custom_initial_data_size == kTableBytes);
    auto* in = tflite::micro::GetEvalInput(ctx, node, 0);
    auto* out = tflite::micro::GetEvalOutput(ctx, node, 0);
    TF_LITE_ENSURE(ctx, in->type == kTfLiteInt8 && out->type == kTfLiteInt8 &&
                   in->dims->size == 4 && out->dims->size == 4);
    const int* x = in->dims->data;
    const int* y = out->dims->data;
    TF_LITE_ENSURE(ctx, x[0] == y[0] && x[1] == y[1] && x[2] == 2*y[2] && y[3] == 2*x[3]);
    return kTfLiteOk;
}

TfLiteStatus eval(TfLiteContext* ctx, TfLiteNode* node) {
    auto* in = tflite::micro::GetEvalInput(ctx, node, 0);
    auto* out = tflite::micro::GetEvalOutput(ctx, node, 0);
    // Keep both small tables in internal SRAM during the traversal of PSRAM.
    alignas(16) int8_t tables[kTableBytes];
    std::memcpy(tables, node->custom_initial_data, sizeof(tables));
    const int8_t* max_table = tables + 128;
    const int8_t* sum_table = tables + 256 + 256;
    const int channels = in->dims->data[3];
    const int pairs = out->dims->data[0] * out->dims->data[1] * out->dims->data[2];
    const int8_t* src = in->data.int8;
    int8_t* dst = out->data.int8;
    for (int pair = 0; pair < pairs; ++pair, src += 2*channels, dst += 2*channels) {
        for (int c = 0; c < channels; ++c) {
            const int a = src[c], b = src[channels+c];
            dst[c] = max_table[std::max(a, b)];
            dst[channels+c] = sum_table[a+b];
        }
    }
    return kTfLiteOk;
}
}

TFLMRegistration register_birdnet_stem_pool() {
    return tflite::micro::RegisterOp(nullptr, prepare, eval);
}
