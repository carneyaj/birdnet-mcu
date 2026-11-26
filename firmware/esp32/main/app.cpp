#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "driver/i2s_pdm.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_nn.h"
#include "esp_nn_multicore.h"
#include "esp_pm.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "frontend.h"
#include "packed_model.h"
#include "oled.h"
#include "recorder.h"
#include "diagnostics/timing.h"
#include "birdnet_species_profile.h"
#include "swish.h"
#include "stem_pool.h"

extern const uint8_t model_data[] asm("_binary_classifier_tflite_start");
extern const char labels_begin[] asm("_binary_field_labels_start");
extern const char labels_end[] asm("_binary_field_labels_end");

extern const uint8_t model_end[] asm("_binary_classifier_tflite_end");

namespace {
constexpr char kTag[] = "birdnet";
constexpr int kLocalCount = birdnet_species::kCount;
constexpr size_t kArena = 2 * 1024 * 1024;
struct Prediction { int index; float probability; char label[160]; };
struct Window { int16_t* pcm; uint32_t number; int64_t ready_us; };
QueueHandle_t available, latest;
i2s_chan_handle_t mic;
uint32_t overflows;

void* external(size_t bytes) {
    void* p = heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) ESP_LOGE(kTag, "PSRAM allocation failed: need=%u largest=%u", unsigned(bytes),
                     unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)));
    return p;
}

void initialize_mic() {
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan.dma_desc_num = 8;
    chan.dma_frame_num = 480;
    ESP_ERROR_CHECK(i2s_new_channel(&chan, nullptr, &mic));
    i2s_pdm_rx_config_t cfg = {};
    cfg.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(48000);
    cfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
    cfg.gpio_cfg.clk = GPIO_NUM_42;
    cfg.gpio_cfg.din = GPIO_NUM_41;
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(mic, &cfg));
    i2s_event_callbacks_t callbacks = {};
    callbacks.on_recv_q_ovf = [](i2s_chan_handle_t, i2s_event_data_t*, void*) {
        __atomic_fetch_add(&overflows, 1u, __ATOMIC_RELAXED);
        return false;
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(mic, &callbacks, nullptr));
}

void capture(void*) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_ERROR_CHECK(i2s_channel_enable(mic));
    uint32_t seq = 0;
    while (true) {
        Window w;
        xQueueReceive(available, &w, portMAX_DELAY);
        const auto before = __atomic_load_n(&overflows, __ATOMIC_RELAXED);
        size_t got = 0;
        esp_err_t err = i2s_channel_read(mic, w.pcm, kSamples * sizeof(int16_t), &got, 4000);
        w.ready_us = esp_timer_get_time();
        w.number = ++seq;
        if (err != ESP_OK || got != kSamples * sizeof(int16_t) || before != __atomic_load_n(&overflows, __ATOMIC_RELAXED)) {
            ESP_LOGW(kTag, "discarded incomplete capture %lu (%s, %u bytes)", (unsigned long)seq, esp_err_to_name(err), unsigned(got));
            xQueueSend(available, &w, portMAX_DELAY);
            continue;
        }
        // While inference runs, keep only the newest complete, contiguous window.
        Window stale;
        if (xQueueReceive(latest, &stale, 0)) xQueueSend(available, &stale, portMAX_DELAY);
        xQueueSend(latest, &w, portMAX_DELAY);
    }
}

float probability(const TfLiteTensor* scores, int index) {
    const float logit = (scores->data.int8[index] - scores->params.zero_point) * scores->params.scale;
    return 1.0f / (1.0f + std::exp(-logit));
}

int winning_index(const TfLiteTensor* scores) {
    const int8_t* data = scores->data.int8;
    if constexpr (!birdnet_species::kVocalGuardOnly) {
        return std::max_element(data, data + kLocalCount) - data;
    }
    int best = birdnet_species::kHumanVocal == 0 ? 1 : 0;
    for (int i = best + 1; i < kLocalCount; ++i) {
        if (birdnet_species::Visible(i) && data[i] > data[best]) best = i;
    }
    return best;
}

Prediction local_prediction(const TfLiteTensor* scores, int index) {
    Prediction p = {};
    p.index = index;
    p.probability = probability(scores, index);
    const char* begin = labels_begin;
    for (int i = 0; i < p.index; ++i) {
        const void* end = std::memchr(begin, '\n', labels_end - begin);
        if (!end) return p;
        begin = static_cast<const char*>(end) + 1;
    }
    const char* end = std::find(begin, labels_end, '\n');
    std::memcpy(p.label, begin, std::min<size_t>(end - begin, sizeof(p.label) - 1));
    return p;
}

const char* record_hit(Window& w, const TfLiteTensor* scores, const Prediction& top) {
    if (top.probability < kDetectionThreshold) return "idle";
    if (probability(scores, birdnet_species::kHumanVocal) >= 0.30f) return "vocal-veto";
    if (!birdnet_species::CanTriggerAudio(top.index)) {
        int best = -1;
        for (int i = 0; i < kLocalCount; ++i) {
            if ((best < 0 || scores->data.int8[i] > scores->data.int8[best]) &&
                birdnet_species::CanTriggerAudio(i)) best = i;
        }
        if (best < 0 || probability(scores, best) < kDetectionThreshold) return "sound-only";
        const auto animal = local_prediction(scores, best);
        return queue_recording(w.pcm, w.number, w.ready_us, animal.index, animal.probability, animal.label);
    }
    return queue_recording(w.pcm, w.number, w.ready_us, top.index, top.probability, top.label);
}

void configure_scratch() {
    constexpr uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    for (size_t size : {57920, 53312, 32768, 24576, 16384}) {
        void* scratch = heap_caps_aligned_alloc(16, size, caps);
        if (!scratch) continue;
        esp_nn_set_conv_preferred_scratch_buf_esp32s3(scratch, size);
        esp_nn_set_depthwise_conv_preferred_scratch_buf_esp32s3(scratch, size);
        break;
    }
}

bool run() {
    // FreeRTOS holds CPU_FREQ_MAX while either core has runnable work.
    // The enabled PDM receiver keeps its peripheral clock throughout capture.
    const esp_pm_config_t power = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = 80,
        .light_sleep_enable = false,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&power));
    const auto mb = esp_psram_get_size();
    oled_update();
    ESP_LOGI(kTag, "BirdNET trained V2 / XIAO ESP32S3 Sense, PSRAM=%u", unsigned(mb));
    auto* arena = static_cast<uint8_t*>(external(kArena));
    if (!arena) return false;
    tflite::MicroMutableOpResolver<18> ops;
    const auto swish = register_birdnet_swish();
    const auto swish_pad = register_birdnet_swish_pad();
    const auto swish_mean = register_birdnet_swish_mean();
    const auto stem_pool = register_birdnet_stem_pool();
    if (ops.AddConv2D() || ops.AddDepthwiseConv2D() || ops.AddAveragePool2D() || ops.AddMaxPool2D() ||
        ops.AddQuantize() || ops.AddConcatenation() || ops.AddMul() || ops.AddLogistic() || ops.AddPad() ||
        ops.AddMean() || ops.AddAdd() || ops.AddFullyConnected() || ops.AddReshape() || ops.AddGather() ||
        ops.AddCustom("BirdnetSwish", &swish) || ops.AddCustom("BirdnetSwishPad", &swish_pad) ||
        ops.AddCustom("BirdnetSwishMean", &swish_mean) || ops.AddCustom("BirdnetStemPool", &stem_pool)) return false;
    if (!prepare_packed_weights(model_data, uintptr_t(model_end) - uintptr_t(model_data))) return false;
    tflite::MicroInterpreter net(tflite::GetModel(model_data), ops, arena, kArena);
    if (net.AllocateTensors() != kTfLiteOk) { ESP_LOGE(kTag, "AllocateTensors failed"); return false; }
    auto* in = net.input(0);
    auto* local = net.output(0);
    if (!in || !local || net.outputs_size() != 1 || in->type != kTfLiteInt8 || local->type != kTfLiteInt8 ||
        in->bytes != kFeatures || local->bytes != kLocalCount) {
        ESP_LOGE(kTag, "unexpected model contract"); return false;
    }
    init_frontend();
    latest = xQueueCreate(1, sizeof(Window));
    available = xQueueCreate(3, sizeof(Window));
    if (!latest || !available) return false;
    for (int i = 0; i < 3; ++i) {
        Window buffer{static_cast<int16_t*>(external(kSamples * sizeof(int16_t))), 0, 0};
        if (!buffer.pcm) return false;
        xQueueSend(available, &buffer, 0);
    }

    initialize_mic();
    TaskHandle_t capture_task = nullptr;
    if (xTaskCreatePinnedToCore(capture, "pdm48", 4096, nullptr, configMAX_PRIORITIES - 1,
                                &capture_task, 0) != pdPASS) return false;
    init_recorder();
    esp_nn_dual_core_enable();
    configure_scratch();
    xTaskNotifyGive(capture_task);
    ESP_LOGI(kTag, "ready, %d classes at 48 kHz", kLocalCount);
    uint32_t last = 0;
    while (true) {
        Window window;
        xQueueReceive(latest, &window, portMAX_DELAY);
        BirdnetTiming timing(window.ready_us);
        make_features(window.pcm, in->data.int8, in->params.scale, in->params.zero_point);
        timing.frontend_done();
        if (net.Invoke() != kTfLiteOk) {
            ESP_LOGE(kTag, "inference failed");
            return false;
        }
        timing.inference_done();
        const auto top = local_prediction(local, winning_index(local));
        const bool hit = top.probability >= kDetectionThreshold;
        const char* recording = record_hit(window, local, top);
        oled_update(hit ? top.label : nullptr, top.probability);
        const unsigned long skipped = window.number - last - 1;
        if (skipped) ESP_LOGW(kTag, "missed %lu audio window(s)", skipped);
        ESP_LOGI(kTag, "window=%lu top=%s p=%.3f hit=%d record=%s",
                 (unsigned long)window.number, top.label, top.probability, hit, recording);
        timing.report(window.number, __atomic_load_n(&overflows, __ATOMIC_RELAXED));
        last = window.number;
        xQueueSend(available, &window, portMAX_DELAY);
    }
}
} // namespace

extern "C" void app_main() {
    if (xTaskCreatePinnedToCore([](void*) {
        if (!run()) {
            oled_update();
            ESP_LOGE(kTag, "classifier stopped; reset after checking the preceding error");
        }
        vTaskDelete(nullptr);
    }, "birdnet", 24 * 1024, nullptr, 3, nullptr, 1) != pdPASS) ESP_LOGE(kTag, "could not start classifier task");
}
