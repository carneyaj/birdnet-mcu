#pragma once

#include <cstddef>
#include <cstdint>

namespace birdnet_frontend {

constexpr int kNumMels = 96;
constexpr int kNumFrames = 511;
constexpr int kNumChannels = 2;
constexpr int kBranch1FftSize = 2048;
constexpr int kBranch1Bins = 1025;
constexpr int kBranch1Hop = 278;
constexpr int kBranch2FftSize = 1024;
constexpr int kBranch2Bins = 513;
constexpr int kBranch2Hop = 280;

extern const float kBranchPowerExponent[2];
extern const float kBatchNormMul[2];
extern const float kBatchNormAdd[2];

extern const uint16_t kBranch1MelStarts[kNumMels];
extern const uint16_t kBranch1MelLengths[kNumMels];
extern const uint16_t kBranch1MelOffsets[kNumMels];
extern const float kBranch1MelWeights[252];

extern const uint16_t kBranch2MelStarts[kNumMels];
extern const uint16_t kBranch2MelLengths[kNumMels];
extern const uint16_t kBranch2MelOffsets[kNumMels];
extern const float kBranch2MelWeights[609];

}  // namespace birdnet_frontend
