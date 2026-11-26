#pragma once
#include <cstdint>
constexpr int kSamples = 144000;
constexpr int kFeatures = 96 * 511 * 2;
void init_frontend();
void make_features(const int16_t* pcm, int8_t* out, float scale, int zero);
