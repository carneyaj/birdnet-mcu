#include "birdnet_log.h"
#include <cstdio>
#include <cstring>

#if defined(__arm__) || defined(__thumb__)
extern "C" int vsniprintf(char*, size_t, const char*, va_list);
#endif

namespace birdnet_format {
#if defined(__arm__) || defined(__thumb__)
int Vsnprintf(char* output, size_t capacity, const char* format, va_list args) {
  return ::vsniprintf(output, capacity, format, args);
}
int Snprintf(char* output, size_t capacity, const char* format, ...) {
  va_list args;
  va_start(args, format);
  const int result = Vsnprintf(output, capacity, format, args);
  va_end(args);
  return result;
}
#endif
}  // namespace birdnet_format

namespace birdnet_log {
namespace { Sink current_sink; }
void SetSink(Sink sink) { current_sink = sink; }
int VWrite(const char* format, va_list args) {
  if (!current_sink || !format) return 0;
  char buffer[256];
  const int length = birdnet_format::Vsnprintf(buffer, sizeof(buffer), format, args);
  if (length > 0) current_sink(buffer, size_t(length) < sizeof(buffer) ? length : sizeof(buffer) - 1);
  return length;
}
void Write(const char* format, ...) {
  va_list args;
  va_start(args, format);
  VWrite(format, args);
  va_end(args);
}
}  // namespace birdnet_log

// Ethos-U diagnostics and newlib assertions use these entry points. Preserve
// their messages and assert/abort behavior without pulling in a second printf
// engine or floating-point conversion. No application FILE* is a libc stream:
// SD storage uses FatFs, and binary export bypasses stdio entirely.
extern "C" int __wrap_fprintf(FILE*, const char* format, ...) {
  va_list args;
  va_start(args, format);
  const int result = birdnet_log::VWrite(format, args);
  va_end(args);
  return result;
}
extern "C" int __wrap_fiprintf(FILE*, const char* format, ...) {
  va_list args;
  va_start(args, format);
  const int result = birdnet_log::VWrite(format, args);
  va_end(args);
  return result;
}
