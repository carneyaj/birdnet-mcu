#include "packed_model.h"

#include <cstring>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "miniz.h"
#include "tensorflow/lite/micro/schema/schema_generated.h"

namespace {
struct Weight {
    const tflite::Buffer* buffer;
    void* data;
};
constexpr size_t kMaxWeights = 2;
Weight weights[kMaxWeights]{};
}

// The model and any resolved weights live for the lifetime of the interpreter.
extern "C" void* xiao_resolve_model_buffer(const void* buffer) {
    for (const auto& weight : weights)
        if (weight.buffer == buffer) return weight.data;
    return nullptr;
}

bool prepare_packed_weights(const uint8_t* data, size_t bytes) {
    const auto* buffers = tflite::GetModel(data)->buffers();
    Weight prepared[kMaxWeights]{};
    size_t count = 0, allocated = 0;
    auto fail = [&]() {
        for (const auto& weight : prepared) heap_caps_free(weight.data);
        ESP_LOGE("weights", "could not load packed model constants");
        return false;
    };
    const auto start = esp_timer_get_time();
    for (const auto* buffer : *buffers) {
        if (!buffer->offset() && !buffer->size()) continue;
        const uint64_t offset = buffer->offset(), size = buffer->size();
        if (buffer->data() || offset < 8 || offset > bytes || bytes - offset < sizeof(uint32_t) ||
            !size || size > SIZE_MAX || count == kMaxWeights) return fail();
        uint32_t compressed;
        std::memcpy(&compressed, data + offset, sizeof(compressed));
        if (!compressed || compressed > bytes - offset - sizeof(compressed)) return fail();
        void* dest = heap_caps_aligned_alloc(16, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!dest) return fail();
        prepared[count++] = {buffer, dest};
        if (tinfl_decompress_mem_to_mem(dest, size, data + offset + sizeof(compressed), compressed,
                                        TINFL_FLAG_PARSE_ZLIB_HEADER) != size) return fail();
        allocated += size;
    }
    const size_t inflated = count;
    // With inline constants the same hot weights can be copied directly from
    // quad flash to octal PSRAM. All other tensors still point into flash.
    const auto* tensors = tflite::GetModel(data)->subgraphs()->Get(0)->tensors();
    for (const auto* tensor : *tensors) {
        const auto* shape = tensor->shape();
        if (!shape || tensor->type() != tflite::TensorType_INT8) continue;
        const bool embedding = shape->size() == 4 && shape->Get(0) == 1024 &&
            shape->Get(1) == 1 && shape->Get(2) == 1 && shape->Get(3) == 1728;
        const bool classifier = shape->size() == 2 && shape->Get(0) > 1 && shape->Get(1) == 1024;
        if (!embedding && !classifier) continue;
        const auto* buffer = buffers->Get(tensor->buffer());
        const auto* raw = buffer->data();
        if (!raw || !raw->size() || count == kMaxWeights) continue;
        // Leave room for capture, frontend coefficients and the recorder on smaller heaps.
        if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) < raw->size() + 2 * 1024 * 1024) continue;
        void* dest = heap_caps_aligned_alloc(16, raw->size(), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!dest) continue;
        std::memcpy(dest, raw->data(), raw->size());
        prepared[count++] = {buffer, dest};
        allocated += raw->size();
    }
    std::memcpy(weights, prepared, sizeof(weights));
    if (count) ESP_LOGI("weights", "cached=%u inflated=%u bytes=%u startup_ms=%lld", unsigned(count), unsigned(inflated),
                        unsigned(allocated), (esp_timer_get_time() - start) / 1000);
    return true;
}
