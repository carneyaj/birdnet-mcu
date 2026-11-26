#pragma once

#include "inference_pipeline.h"

namespace birdnet_lora {

using LogFn = void (*)(const char* fmt, ...);

bool Init(LogFn log_fn);
bool SendDetection(const InferenceResult& result);
bool SendTestPacket(uint64_t timestamp_ms);

}  // namespace birdnet_lora
