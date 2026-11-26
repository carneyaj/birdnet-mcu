#include "recorder.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include "birdnet_opus_codec.h"
#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "frontend.h"
#include "recording_profile.h"
#include "sdmmc_cmd.h"

namespace {
constexpr char kTag[] = "recorder";
constexpr char kMount[] = "/sd";
bool bus_ready;
constexpr gpio_num_t kCardSelect[] = {GPIO_NUM_3, GPIO_NUM_21}; // OLED base D2, Sense daughterboard
struct Clip {
    int16_t* pcm;
    uint32_t window;
    uint64_t capture_end_ms;
    uint16_t species, confidence;
    char label[160];
};
QueueHandle_t jobs, spare;
sdmmc_card_t* card;

bool mount_card() {
    if (card) {
        if (sdmmc_get_status(card) == ESP_OK) return true;
        esp_vfs_fat_sdcard_unmount(kMount, card);
        card = nullptr;
    }
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    const auto port = static_cast<spi_host_device_t>(host.slot);
    if (!bus_ready) {
        for (auto cs : kCardSelect) {
            gpio_set_level(cs, 1);
            gpio_set_direction(cs, GPIO_MODE_OUTPUT);
        }
        spi_bus_config_t bus = {};
        bus.mosi_io_num = 9;
        bus.miso_io_num = 8;
        bus.sclk_io_num = 7;
        bus.quadwp_io_num = bus.quadhd_io_num = -1;
        if (spi_bus_initialize(port, &bus, SDSPI_DEFAULT_DMA) != ESP_OK) return false;
        bus_ready = true;
    }
    esp_vfs_fat_mount_config_t fs = {};
    fs.max_files = 2;
    fs.format_if_mount_failed = false;
    for (auto cs : kCardSelect) {
        sdspi_device_config_t dev = SDSPI_DEVICE_CONFIG_DEFAULT();
        dev.host_id = port;
        dev.gpio_cs = cs;
        if (esp_vfs_fat_sdspi_mount(kMount, &host, &dev, &fs, &card) == ESP_OK) {
            ESP_LOGI(kTag, "mounted SD on GPIO%d", int(cs));
            return true;
        }
        gpio_set_level(cs, 1);
        gpio_set_direction(cs, GPIO_MODE_OUTPUT);
    }
    return false;
}

bool write_block(void* context, const uint8_t* data, size_t size) {
    return std::fwrite(data, 1, size, static_cast<FILE*>(context)) == size;
}

bool save_clip(const Clip& clip, uint32_t session, uint32_t& sequence) {
    char path[32], temporary[32];
    int fd;
    do {
        ++sequence;
        std::snprintf(path, sizeof(path), "/sd/%08lX.OGG", (unsigned long)sequence);
        std::snprintf(temporary, sizeof(temporary), "/sd/%08lX.TMP", (unsigned long)sequence);
        struct stat existing;
        if (stat(path, &existing) == 0) continue;
        if (errno != ENOENT) return false;
        fd = open(temporary, O_WRONLY | O_CREAT | O_EXCL, 0666);
        if (fd >= 0) break;
        if (errno != EEXIST) return false;
    } while (true);
    FILE* file = fdopen(fd, "wb");
    if (!file) {
        close(fd);
        unlink(temporary);
        return false;
    }
    birdnet_clip::Metadata detection{};
    detection.count = 1;
    detection.session = session;
    detection.threshold_q1e4 = kDetectionThresholdQ1e4;
    detection.species[0] = clip.species;
    detection.probability[0] = clip.confidence;
    constexpr auto board_name = kDeployment;
    birdnet_opus_codec::Metadata tags{clip.capture_end_ms, sequence, board_name, &detection};
    birdnet_opus_codec::EncodeStats stats{};
    const bool encoded = birdnet_opus_codec::EncodeOggFrom48kRing(
        clip.pcm, kSamples, 0, kSamples, tags, write_block, file, &stats);
    const bool closed = std::fclose(file) == 0;
    if (!encoded || !closed || rename(temporary, path) != 0) {
        unlink(temporary);
        return false;
    }
    FILE* index = std::fopen("/sd/HITS.CSV", "a");
    bool indexed = false;
    if (index) {
        const int n = std::fprintf(index, "%s,%lu,%llu,%u,%u,\"%s\"\n", path + 4,
            (unsigned long)clip.window, (unsigned long long)clip.capture_end_ms,
            clip.species, clip.confidence, clip.label);
        indexed = std::fclose(index) == 0 && n > 0;
    }
    ESP_LOGI(kTag, "saved window=%lu file=%s bytes=%u indexed=%d",
             (unsigned long)clip.window, path, unsigned(stats.file_bytes), indexed);
    return true;
}


void writer(void*) {
    const uint32_t session = esp_random();
    uint32_t sequence = session;
    Clip clip;
    for (;;) {
        xQueueReceive(jobs, &clip, portMAX_DELAY);
        if (!mount_card()) {
            ESP_LOGW(kTag, "window=%lu SD unavailable, recording skipped", (unsigned long)clip.window);
        } else if (!save_clip(clip, session, sequence)) {
            ESP_LOGW(kTag, "window=%lu SD save failed, retrying on the next hit", (unsigned long)clip.window);
            esp_vfs_fat_sdcard_unmount(kMount, card);
            card = nullptr;
        }
        xQueueSend(spare, &clip.pcm, 0);
    }
}
} // namespace

void init_recorder() {
    auto* pcm = static_cast<int16_t*>(heap_caps_aligned_alloc(16, kSamples * sizeof(int16_t),
                                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    jobs = xQueueCreate(1, sizeof(Clip));
    spare = xQueueCreate(1, sizeof(pcm));
    if (pcm && jobs && spare &&
        xTaskCreatePinnedToCore(writer, "sd-opus", 72 * 1024, nullptr, 1, nullptr, 0) == pdPASS) {
        xQueueSend(spare, &pcm, 0);
        return;
    }
    if (jobs) vQueueDelete(jobs);
    if (spare) vQueueDelete(spare);
    jobs = spare = nullptr;
    heap_caps_free(pcm);
    ESP_LOGW(kTag, "recording unavailable, continuing classification");
}

const char* queue_recording(int16_t*& pcm, uint32_t window, int64_t ready_us,
                            int species, float probability, const char* label) {
    if (!spare) return "unavailable";
    int16_t* replacement;
    if (xQueueReceive(spare, &replacement, 0) != pdTRUE) return "busy";
    Clip clip{pcm, window, uint64_t(ready_us / 1000), uint16_t(species),
              uint16_t(probability * 10000 + 0.5f), {}};
    strlcpy(clip.label, label, sizeof(clip.label));
    if (xQueueSend(jobs, &clip, 0) != pdTRUE) {
        xQueueSend(spare, &replacement, 0);
        return "busy";
    }
    pcm = replacement;
    return "queued";
}

