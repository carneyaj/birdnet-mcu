#include "birdnet_format.h"
#include "birdnet_oled.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "config.h"

extern "C" {
#include "WE2_device_addr.h"
#include "hx_drv_iic.h"
}

namespace birdnet_oled {
namespace {

constexpr int kDisplayWidth = 128;
constexpr int kDisplayHeight = cfg::kOledHeight;
constexpr int kDisplayPages = kDisplayHeight / 8;
constexpr int kFontFirstChar = 0x20;
constexpr int kFontLastChar = 0x7E;
constexpr int kFontCharWidth = 5;
constexpr int kFontAdvance = 6;
constexpr int kMaxCharsPerLine = kDisplayWidth / kFontAdvance;
constexpr int kMaxLines = 4;
constexpr int kLineSpacingPixels = kDisplayHeight / kMaxLines;
constexpr int kChunkBytes = 16;
constexpr int kEnvLineBufferBytes = 32;

// ASCII 0x20..0x7E from the classic Adafruit_GFX 5x7 font.
constexpr uint8_t kFont5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x07,
    0x00, 0x07, 0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x24, 0x2A, 0x7F, 0x2A,
    0x12, 0x23, 0x13, 0x08, 0x64, 0x62, 0x36, 0x49, 0x56, 0x20, 0x50, 0x00,
    0x08, 0x07, 0x03, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x41, 0x22,
    0x1C, 0x00, 0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x08, 0x08, 0x3E, 0x08, 0x08,
    0x00, 0x80, 0x70, 0x30, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00,
    0x60, 0x60, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x51, 0x49, 0x45,
    0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x72, 0x49, 0x49, 0x49, 0x46, 0x21,
    0x41, 0x49, 0x4D, 0x33, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x27, 0x45, 0x45,
    0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x31, 0x41, 0x21, 0x11, 0x09, 0x07,
    0x36, 0x49, 0x49, 0x49, 0x36, 0x46, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00,
    0x14, 0x00, 0x00, 0x00, 0x40, 0x34, 0x00, 0x00, 0x00, 0x08, 0x14, 0x22,
    0x41, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x41, 0x22, 0x14, 0x08, 0x02,
    0x01, 0x59, 0x09, 0x06, 0x3E, 0x41, 0x5D, 0x59, 0x4E, 0x7C, 0x12, 0x11,
    0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
    0x7F, 0x41, 0x41, 0x41, 0x3E, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09,
    0x09, 0x09, 0x01, 0x3E, 0x41, 0x41, 0x51, 0x73, 0x7F, 0x08, 0x08, 0x08,
    0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F,
    0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x7F, 0x02, 0x1C,
    0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E, 0x41, 0x41, 0x41, 0x3E,
    0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09,
    0x19, 0x29, 0x46, 0x26, 0x49, 0x49, 0x49, 0x32, 0x03, 0x01, 0x7F, 0x01,
    0x03, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F,
    0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63, 0x03, 0x04, 0x78,
    0x04, 0x03, 0x61, 0x59, 0x49, 0x4D, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x41,
    0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41, 0x41, 0x7F, 0x04, 0x02,
    0x01, 0x02, 0x04, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x03, 0x07, 0x08,
    0x00, 0x20, 0x54, 0x54, 0x78, 0x40, 0x7F, 0x28, 0x44, 0x44, 0x38, 0x38,
    0x44, 0x44, 0x44, 0x28, 0x38, 0x44, 0x44, 0x28, 0x7F, 0x38, 0x54, 0x54,
    0x54, 0x18, 0x00, 0x08, 0x7E, 0x09, 0x02, 0x18, 0xA4, 0xA4, 0x9C, 0x78,
    0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x20, 0x40,
    0x40, 0x3D, 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x41, 0x7F, 0x40,
    0x00, 0x7C, 0x04, 0x78, 0x04, 0x78, 0x7C, 0x08, 0x04, 0x04, 0x78, 0x38,
    0x44, 0x44, 0x44, 0x38, 0xFC, 0x18, 0x24, 0x24, 0x18, 0x18, 0x24, 0x24,
    0x18, 0xFC, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x48, 0x54, 0x54, 0x54, 0x24,
    0x04, 0x04, 0x3F, 0x44, 0x24, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x1C, 0x20,
    0x40, 0x20, 0x1C, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x44, 0x28, 0x10, 0x28,
    0x44, 0x4C, 0x90, 0x90, 0x90, 0x7C, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00,
    0x08, 0x36, 0x41, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x41, 0x36,
    0x08, 0x00, 0x02, 0x01, 0x02, 0x04, 0x02,
};

EnsureI2cReadyFn g_ensure_i2c_ready = nullptr;
alignas(4) uint8_t g_page[kDisplayWidth]{};
char g_last_lines[kMaxLines][kMaxCharsPerLine + 1]{};
bool g_lines_valid = false;
uint8_t g_oled_addr = cfg::kDetectionOledI2cAddr;
bool g_available = false;
bool g_display_on = false;

bool WritePrefixed(uint8_t control, const uint8_t* payload, size_t len) {
  if (!payload && len != 0) {
    return false;
  }
  uint8_t prefix = control;
  const bool ok = hx_drv_i2cm_write_data(USE_DW_IIC_0,
                                g_oled_addr,
                                &prefix,
                                1,
                                const_cast<uint8_t*>(payload),
                                static_cast<uint32_t>(len)) == IIC_ERR_OK;
  if (!ok) g_available = g_lines_valid = false;
  return ok;
}

bool WriteCommands(const uint8_t* commands, size_t len) {
  return WritePrefixed(0x00, commands, len);
}

bool WriteData(const uint8_t* data, size_t len) {
  return WritePrefixed(0x40, data, len);
}

int BuildAddressCandidates(uint8_t* out, size_t max_out) {
  if (!out || max_out == 0) {
    return 0;
  }

  int count = 0;
  const auto append_unique = [&](uint8_t value) {
    for (int i = 0; i < count; ++i) {
      if (out[i] == value) {
        return;
      }
    }
    if (static_cast<size_t>(count) < max_out) {
      out[count++] = value;
    }
  };

  append_unique(cfg::kDetectionOledI2cAddr);
  // The SDK uses seven-bit targets. Do not probe reserved 0x78/0x7a addresses.
  append_unique((cfg::kDetectionOledI2cAddr == 0x3C) ? 0x3D : 0x3C);
  return count;
}

bool EnableChargePump(bool on) {
  const uint8_t commands[] = {0x8D, static_cast<uint8_t>(on ? 0x14 : 0x10)};
  return WriteCommands(commands, sizeof(commands));
}

bool FlushPage(int page) {
  const uint8_t commands[] = {static_cast<uint8_t>(0xB0 + page), 0x00, 0x10};
  if (!WriteCommands(commands, sizeof(commands))) return false;
  for (int offset = 0; offset < kDisplayWidth; offset += kChunkBytes) {
    const size_t chunk = static_cast<size_t>(std::min(kChunkBytes, kDisplayWidth - offset));
    if (!WriteData(g_page + offset, chunk)) return false;
  }
  return true;
}

void DrawChar(int x, char c) {
  if (x < 0 || x + kFontCharWidth >= kDisplayWidth) {
    return;
  }
  uint8_t glyph_index = static_cast<uint8_t>(c);
  if (glyph_index < kFontFirstChar || glyph_index > kFontLastChar) {
    glyph_index = '?';
  }
  const size_t font_offset = static_cast<size_t>(glyph_index - kFontFirstChar) * kFontCharWidth;
  uint8_t* dst = g_page + x;
  for (int col = 0; col < kFontCharWidth; ++col) {
    dst[col] = kFont5x7[font_offset + col];
  }
  dst[kFontCharWidth] = 0x00;
}

void DrawTextLine(const char* text) {
  if (!text) {
    return;
  }
  size_t len = 0;
  while (text[len] != '\0') {
    ++len;
  }
  if (len == 0) {
    return;
  }
  const int text_width = static_cast<int>(len) * kFontAdvance;
  int x = (kDisplayWidth - text_width) / 2;
  if (x < 0) {
    x = 0;
  }
  for (size_t i = 0; i < len && x + kFontCharWidth < kDisplayWidth; ++i) {
    DrawChar(x, text[i]);
    x += kFontAdvance;
  }
}

void ExtractCommonName(const char* label, char* out, size_t out_size) {
  if (!out || out_size == 0) {
    return;
  }
  out[0] = '\0';
  if (!label) {
    return;
  }

  const char* start = label;
  for (const char* p = label; *p != '\0'; ++p) {
    if (*p == '_') {
      start = p + 1;
      break;
    }
  }

  size_t out_len = 0;
  for (const char* p = start; *p != '\0' && out_len + 1 < out_size; ++p) {
    char ch = *p;
    if (ch == '_') {
      ch = ' ';
    } else if (static_cast<unsigned char>(ch) < 0x20u ||
               static_cast<unsigned char>(ch) > 0x7Eu) {
      ch = '?';
    }
    out[out_len++] = ch;
  }
  while (out_len > 0 && out[out_len - 1] == ' ') {
    --out_len;
  }
  out[out_len] = '\0';
}

void AddEllipsis(char* line) {
  if (!line) {
    return;
  }
  size_t len = 0;
  while (line[len] != '\0') {
    ++len;
  }
  if (len >= 3) {
    line[len - 3] = '.';
    line[len - 2] = '.';
    line[len - 1] = '.';
  } else {
    while (len < 3) {
      line[len++] = '.';
    }
    line[len] = '\0';
  }
}

int WrapLines(const char* text,
              char lines[kMaxLines][kMaxCharsPerLine + 1],
              int max_lines) {
  if (max_lines < 0) {
    max_lines = 0;
  } else if (max_lines > kMaxLines) {
    max_lines = kMaxLines;
  }

  for (int i = 0; i < kMaxLines; ++i) {
    lines[i][0] = '\0';
  }
  if (!text || text[0] == '\0' || max_lines == 0) {
    return 0;
  }

  int line = 0;
  int col = 0;
  const char* p = text;

  while (*p != '\0' && line < max_lines) {
    while (*p == ' ') {
      ++p;
    }
    if (*p == '\0') {
      break;
    }

    const char* word = p;
    int word_len = 0;
    while (p[word_len] != '\0' && p[word_len] != ' ') {
      ++word_len;
    }
    p += word_len;

    int consumed = 0;
    while (consumed < word_len && line < max_lines) {
      if (col != 0) {
        if (col >= kMaxCharsPerLine) {
          ++line;
          col = 0;
          continue;
        }
        lines[line][col++] = ' ';
        lines[line][col] = '\0';
      }

      const int room = kMaxCharsPerLine - col;
      if (room <= 0) {
        ++line;
        col = 0;
        continue;
      }

      int chunk = word_len - consumed;
      if (chunk > room) {
        if (col != 0) {
          ++line;
          col = 0;
          continue;
        }
        chunk = room;
      }

      std::memcpy(lines[line] + col, word + consumed, static_cast<size_t>(chunk));
      col += chunk;
      lines[line][col] = '\0';
      consumed += chunk;
      if (consumed < word_len) {
        ++line;
        col = 0;
      }
    }
    if (consumed < word_len) AddEllipsis(lines[max_lines - 1]);
  }

  if (*p != '\0' && max_lines > 0) {
    AddEllipsis(lines[max_lines - 1]);
  }

  int count = 0;
  for (int i = 0; i < max_lines; ++i) {
    if (lines[i][0] != '\0') {
      count = i + 1;
    }
  }
  return count;
}

void FormatTenths(char* out, size_t out_size, int value) {
  if (!out || out_size == 0) {
    return;
  }

  long long magnitude = value;
  const bool negative = magnitude < 0;
  if (magnitude < 0) {
    magnitude = -magnitude;
  }

  const long long tenths = (magnitude + 5) / 10;
  birdnet_format::Snprintf(out,
                out_size,
                negative ? "-%lld.%lld" : "%lld.%lld",
                tenths / 10,
                tenths % 10);
}

void BuildEnvironmentLine(char* out,
                     size_t out_size,
                     bool env_ok,
                     int temp_c_q1e2,
                     int rh_q1e2) {
  if (!out || out_size == 0) {
    return;
  }

  char temp[16]{};
  char rh[16]{};
  if (cfg::kEnableSht41Sensor && env_ok) {
    FormatTenths(temp, sizeof(temp), temp_c_q1e2);
    FormatTenths(rh, sizeof(rh), rh_q1e2);
  } else {
    birdnet_format::Snprintf(temp, sizeof(temp), "--.-");
    birdnet_format::Snprintf(rh, sizeof(rh), "--");
  }

  birdnet_format::Snprintf(out, out_size, "%.7sC %.5s%%RH", temp, rh);
}

void BuildProbabilityBatteryLine(char* out, size_t out_size, float probability,
                                 bool battery_ok, int percent_q1e2, int mv = 0) {
  const float bounded = std::isfinite(probability) ?
      std::max(0.0f, std::min(1.0f, probability)) : 0.0f;
  const int probability_percent = std::max(0, std::min(100,
      static_cast<int>(bounded * 100.0f + 0.5f)));
  if (battery_ok) {
    const int tenths = (std::max(0, std::min(10000, percent_q1e2)) + 5) / 10;
    const unsigned cv = static_cast<unsigned>(std::max(0, std::min(9999, mv))) / 10u;
    birdnet_format::Snprintf(out, out_size, "P:%d%% B:%d.%d%% %u.%02uV", probability_percent,
                  tenths / 10, tenths % 10, cv / 100u, cv % 100u);
  } else {
    birdnet_format::Snprintf(out, out_size, "P:%d%% Bat:--", probability_percent);
  }
}

bool TryInitAtAddress(uint8_t addr) {
  g_lines_valid = false;
  g_oled_addr = addr;
  const uint8_t init_commands[] = {
      0xAE, 0xD5, 0x80, 0xA8, kDisplayHeight - 1, 0xD3, 0x00, 0x40,
      static_cast<uint8_t>(cfg::kDetectionOledFlip180 ? 0xA1 : 0xA0),
      static_cast<uint8_t>(cfg::kDetectionOledFlip180 ? 0xC8 : 0xC0),
      0xDA, static_cast<uint8_t>(kDisplayHeight == 32 ? 0x02 : 0x12),
      0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40,
      0x20, 0x02, 0xA4, 0xA6, 0x2E,
  };
  if (!WriteCommands(init_commands, sizeof(init_commands))) {
    return false;
  }
  if (!EnableChargePump(false)) {
    return false;
  }
  g_display_on = false;
  return true;
}

bool EnsurePanelReady() {
  if constexpr (!cfg::kEnableDetectionOled) {
    return false;
  }
  if (g_available) {
    return true;
  }

  if (!g_ensure_i2c_ready || !g_ensure_i2c_ready()) {
    g_available = false;
    return false;
  }

  uint8_t candidates[4]{};
  const int candidate_count = BuildAddressCandidates(candidates, sizeof(candidates));
  g_available = false;
  for (int i = 0; i < candidate_count; ++i) {
    if (TryInitAtAddress(candidates[i])) {
      g_available = true;
      break;
    }
  }
  return g_available;
}

bool ShowLines(const char lines[kMaxLines][kMaxCharsPerLine + 1]) {
  constexpr int stride = kLineSpacingPixels / 8;
  static_assert(stride * kMaxLines == kDisplayPages);
  const bool incremental = g_lines_valid && g_display_on;
  if (!incremental && !EnableChargePump(true)) return false;
  for (int page = 0; page < kDisplayPages; ++page) {
    const int line = page / stride;
    const bool spacer = page % stride != 0;
    if (incremental && (spacer || std::strcmp(lines[line], g_last_lines[line]) == 0)) continue;
    std::memset(g_page, 0, sizeof(g_page));
    if (!spacer) DrawTextLine(lines[line]);
    if (!FlushPage(page)) return false;
  }
  const uint8_t on = 0xAF;
  if (!incremental && !WriteCommands(&on, 1)) return false;
  std::memcpy(g_last_lines, lines, sizeof(g_last_lines));
  g_display_on = g_lines_valid = true;
  return true;
}

}  // namespace

void SetEnsureI2cReadyFn(EnsureI2cReadyFn fn) {
  g_ensure_i2c_ready = fn;
}

bool Init() {
  return EnsurePanelReady();
}

bool Resume(uint8_t address) {
  g_lines_valid = false;
  if ((address != 0x3c && address != 0x3d) ||
      !g_ensure_i2c_ready || !g_ensure_i2c_ready()) return false;
  g_oled_addr = address;
  g_available = g_display_on = true;
  return true;
}

uint8_t Address() { return g_oled_addr; }

bool ShowStatus(bool env_ok, int temp_c_q1e2, int rh_q1e2,
                bool battery_ok, int battery_percent_q1e2, int battery_mv, bool paused,
                uint64_t uptime_seconds) {
  if (!EnsurePanelReady()) return false;
  char lines[kMaxLines][kMaxCharsPerLine + 1]{};
  const uint64_t days = uptime_seconds / 86400u;
  const unsigned hours = (uptime_seconds / 3600u) % 24u;
  const unsigned minutes = (uptime_seconds / 60u) % 60u;
  if (days) {
    birdnet_format::Snprintf(lines[0], sizeof(lines[0]), "Up %llud %02u:%02u",
        static_cast<unsigned long long>(days), hours, minutes);
  } else {
    birdnet_format::Snprintf(lines[0], sizeof(lines[0]), "Up %02u:%02u:%02u",
        hours, minutes, static_cast<unsigned>(uptime_seconds % 60u));
  }
  if (battery_ok) {
    const int tenths = (std::max(0, std::min(10000, battery_percent_q1e2)) + 5) / 10;
    const unsigned cv = static_cast<unsigned>(std::max(0, std::min(9999, battery_mv))) / 10u;
    birdnet_format::Snprintf(lines[1], sizeof(lines[1]), "Bat:%d.%d%% %u.%02uV", tenths / 10, tenths % 10,
                  cv / 100u, cv % 100u);
  } else {
    birdnet_format::Snprintf(lines[1], sizeof(lines[1]), "Battery: --");
  }
  BuildEnvironmentLine(lines[2], sizeof(lines[2]), env_ok, temp_c_q1e2, rh_q1e2);
  birdnet_format::Snprintf(lines[3], sizeof(lines[3]), "%s", paused ? "Low battery: paused" : "Listening...");
  return ShowLines(lines);
}

bool ShowCounts(const char* const labels[3], const uint32_t counts[3],
                unsigned page, unsigned pages) {
  if (!EnsurePanelReady()) return false;
  char lines[kMaxLines][kMaxCharsPerLine + 1]{};
  birdnet_format::Snprintf(lines[0], sizeof(lines[0]), "SINCE BOOT %u/%u", page, pages);
  for (int i = 0; i < 3; ++i) {
    if (!labels[i]) continue;
    char name[96]{}, count[12]{};
    ExtractCommonName(labels[i], name, sizeof(name));
    birdnet_format::Snprintf(count, sizeof(count), "%lu", static_cast<unsigned long>(counts[i]));
    const int width = kMaxCharsPerLine - 1 - int(std::strlen(count));
    birdnet_format::Snprintf(lines[i + 1], sizeof(lines[i + 1]), "%.*s %s", width, name, count);
  }
  if (!labels[0]) birdnet_format::Snprintf(lines[1], sizeof(lines[1]), "No birds yet");
  return ShowLines(lines);
}

bool ShowDetection(const char* label,
                   float probability,
                   bool env_ok,
                   int temp_c_q1e2,
                   int rh_q1e2,
                   bool battery_ok,
                   int battery_percent_q1e2,
                   int battery_mv) {
  if (!EnsurePanelReady()) {
    return false;
  }

  char common_name[96]{};
  ExtractCommonName(label, common_name, sizeof(common_name));
  if (common_name[0] == '\0') {
    std::memcpy(common_name, "Detection", sizeof("Detection"));
  }

  char env_line[kEnvLineBufferBytes]{};
  BuildEnvironmentLine(env_line, sizeof(env_line), env_ok, temp_c_q1e2, rh_q1e2);

  char lines[kMaxLines][kMaxCharsPerLine + 1]{};
  int line_count = WrapLines(common_name, lines, 2);
  if (line_count <= 0) {
    std::memcpy(lines[0], "Detection", sizeof("Detection"));
    line_count = 1;
  }
  BuildProbabilityBatteryLine(lines[2], sizeof(lines[2]), probability,
                              battery_ok, battery_percent_q1e2, battery_mv);
  birdnet_format::Snprintf(lines[3], sizeof(lines[3]), "%s", env_line);

  return ShowLines(lines);
}

bool PowerOff() {
  if (!EnsurePanelReady()) {
    return false;
  }
  if (!g_display_on) {
    return true;
  }
  const uint8_t off = 0xAE;
  if (!WriteCommands(&off, 1)) {
    return false;
  }
  if (!EnableChargePump(false)) {
    return false;
  }
  g_display_on = g_lines_valid = false;
  return true;
}

}  // namespace birdnet_oled
