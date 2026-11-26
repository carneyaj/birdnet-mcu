#pragma once
#include <cstdarg>
#include <cstddef>
#include "config.h"

// One bounded logger for the application and inference runtime. File-service
// frames use their own transport and never pass through this formatter.
namespace birdnet_log {
using Sink = void (*)(const char*, size_t);
void SetSink(Sink sink);
int VWrite(const char* format, va_list args);
void Write(const char* format, ...) __attribute__((format(printf, 1, 2)));
template <typename... Args>
inline void Status(const char* format, Args... args) {
  if constexpr (cfg::kEnableStatusLogs) Write(format, args...);
}
}  // namespace birdnet_log

#include "birdnet_format.h"
