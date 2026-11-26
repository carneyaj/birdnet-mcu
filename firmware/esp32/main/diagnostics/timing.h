#pragma once
#include <cstdint>

#if BIRDNET_DIAGNOSTICS
#include "esp_log.h"
#include "esp_timer.h"

class BirdnetTiming {
public:
    explicit BirdnetTiming(int64_t ready)
        : ready_(ready), start_(esp_timer_get_time()) {}
    void frontend_done() { frontend_ = esp_timer_get_time(); }
    void inference_done() { invoke_ = esp_timer_get_time(); }
    void report(uint32_t window, uint32_t overflows) const {
        ESP_LOGI("timing",
                 "window=%lu frontend_us=%lld invoke_us=%lld capture_to_result_us=%lld dma_overflows=%lu",
                 (unsigned long)window, frontend_ - start_, invoke_ - frontend_,
                 esp_timer_get_time() - ready_, (unsigned long)overflows);
    }

private:
    int64_t ready_, start_, frontend_ = 0, invoke_ = 0;
};
#else
struct BirdnetTiming {
    explicit BirdnetTiming(int64_t) {}
    void frontend_done() {}
    void inference_done() {}
    void report(uint32_t, uint32_t) const {}
};
#endif
