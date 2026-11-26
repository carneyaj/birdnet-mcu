#include "oled.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string_view>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "oled_font.h"

namespace {
constexpr int kWidth = 128, kHeight = 64;
constexpr int kColumns = kWidth / 6;
constexpr int kAddress = 0x3c;
constexpr int kTimeoutMs = 50;
i2c_master_bus_handle_t bus;
i2c_master_dev_handle_t panel;
enum class PanelState { Unknown, Off, On };
PanelState state = PanelState::Unknown;
int64_t retry_after_us;
uint8_t frame[1 + kWidth * kHeight / 8] = {0x40};  // SSD1306 data control byte
char displayed_label[160], displayed_score[8];

esp_err_t connect_panel() {
    if (!bus) {
        i2c_master_bus_config_t cfg = {};
        cfg.i2c_port = I2C_NUM_0;
        cfg.sda_io_num = GPIO_NUM_5;  // XIAO D4
        cfg.scl_io_num = GPIO_NUM_6;  // XIAO D5
        cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        cfg.glitch_ignore_cnt = 7;
        cfg.flags.enable_internal_pullup = true;
        esp_err_t err = i2c_new_master_bus(&cfg, &bus);
        if (err != ESP_OK) return err;
    }
    esp_err_t err = i2c_master_probe(bus, kAddress, kTimeoutMs);
    if (err != ESP_OK) return err;
    if (!panel) {
        i2c_device_config_t cfg = {};
        cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        cfg.device_address = kAddress;
        cfg.scl_speed_hz = 400000;
        err = i2c_master_bus_add_device(bus, &cfg, &panel);
        if (err != ESP_OK) return err;
    }
    // Reverse both segment and COM scan directions to rotate the display 180°.
    constexpr uint8_t init[] = {0x00, 0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00,
        0x40, 0x8d, 0x14, 0x20, 0x00, 0xa1, 0xc8, 0xda, 0x12, 0x81, 0xcf,
        0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0x2e};
    return i2c_master_transmit(panel, init, sizeof(init), kTimeoutMs);
}

void draw_text(std::string_view text, int x, int y, int xscale, int yscale) {
    for (unsigned char ch : text) {
        if (x + 5 * xscale > kWidth) break;
        const auto* glyph = kFont5x7 + 5 * ((ch >= 32 && ch <= 126 ? ch : '?') - 32);
        for (int col = 0; col < 5 * xscale; ++col) {
            for (int dy = 0; dy < 8 * yscale; ++dy) {
                if (glyph[col / xscale] & (1u << (dy / yscale))) {
                    const int row = y + dy;
                    frame[1 + (row / 8) * kWidth + x + col] |= 1u << (row % 8);
                }
            }
        }
        x += 6 * xscale;
    }
}

// Prefer whole words, including keeping a hyphen with its preceding syllable.
std::string_view take_line(std::string_view& name, size_t columns) {
    size_t n = std::min(columns, name.size());
    if (n < name.size() && name[n] != ' ') {
        size_t split = name.substr(0, n).find_last_of(" -");
        if (split != name.npos) n = split + (name[split] == '-');
    }
    auto line = name.substr(0, n);
    name.remove_prefix(n);
    while (!name.empty() && name.front() == ' ') name.remove_prefix(1);
    return line;
}

void render(const char* label, const char* score) {
    std::memset(frame + 1, 0, sizeof(frame) - 1);
    draw_text("Detected", 0, 0, 1, 1);
    const char* separator = std::strchr(label, '_');
    std::string_view name(separator ? separator + 1 : label);
    auto rest = name;
    const auto first = take_line(rest, kColumns / 2);
    const bool whole_word = rest.empty() || name[first.size()] == ' ' || first.back() == '-';
    const int xscale = whole_word && rest.size() <= kColumns / 2 ? 2 : 1;
    for (int y : {16, 32, 48}) {
        if (name.empty()) break;
        draw_text(take_line(name, kColumns / xscale), 0, y, xscale, 2);
    }
    // Keep the percentage visible even when the name takes three lines.
    draw_text(score, kWidth - 6 * int(std::strlen(score)), 0, 1, 1);
}
}  // namespace

void oled_update(const char* detected_label, float probability) {
    if (!detected_label && state == PanelState::Off) return;
    if (state == PanelState::Unknown && esp_timer_get_time() < retry_after_us) return;

    // Initialization leaves the panel off. Reinitialize on wake in case the
    // expansion base was disconnected while the display was asleep.
    char score[sizeof(displayed_score)] = {};
    if (detected_label) {
        std::snprintf(score, sizeof(score), "%.1f%%", double(probability) * 100);
        if (state == PanelState::On && !std::strcmp(score, displayed_score) &&
            !std::strcmp(detected_label, displayed_label)) return;
    }
    esp_err_t err = state == PanelState::On ? ESP_OK : connect_panel();
    if (err == ESP_OK && detected_label) {
        render(detected_label, score);
        constexpr uint8_t window[] = {0x00, 0x21, 0, kWidth - 1, 0x22, 0, kHeight / 8 - 1};
        err = i2c_master_transmit(panel, window, sizeof(window), kTimeoutMs);
        if (err == ESP_OK) err = i2c_master_transmit(panel, frame, sizeof(frame), kTimeoutMs);
        if (err == ESP_OK && state != PanelState::On) {
            constexpr uint8_t on[] = {0x00, 0xaf};
            err = i2c_master_transmit(panel, on, sizeof(on), kTimeoutMs);
        }
    } else if (err == ESP_OK && state == PanelState::On) {
        constexpr uint8_t off[] = {0x00, 0xae};
        err = i2c_master_transmit(panel, off, sizeof(off), kTimeoutMs);
    }
    if (err != ESP_OK) {
        if (state != PanelState::Unknown || retry_after_us == 0) ESP_LOGW("oled", "Expansion OLED unavailable (%s), retrying during live capture", esp_err_to_name(err));
        retry_after_us = esp_timer_get_time() + 5000000;
        state = PanelState::Unknown;
        return;
    }
    const auto next = detected_label ? PanelState::On : PanelState::Off;
    if (detected_label) {
        strlcpy(displayed_label, detected_label, sizeof(displayed_label));
        strlcpy(displayed_score, score, sizeof(displayed_score));
    }
    if (next != state) ESP_LOGI("oled", "Expansion OLED %s", detected_label ? "on" : "off");
    state = next;
}
