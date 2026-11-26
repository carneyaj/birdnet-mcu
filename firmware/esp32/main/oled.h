#pragma once

// Display calls run on the classifier task. The expansion base is optional.
// A null label puts the SSD1306 in display-off mode.
void oled_update(const char* detected_label = nullptr, float probability = 0.0f);
