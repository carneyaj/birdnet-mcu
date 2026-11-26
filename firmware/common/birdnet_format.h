#pragma once
#include <cstdarg>
#include <cstddef>
#include <cstdio>
namespace birdnet_format {
// Release formats are integer/string only; keep newlib's proven integer
// formatter (64-bit values, padding, precision, truncation), not a custom parser.
#if defined(__arm__) || defined(__thumb__)
int Vsnprintf(char* output, size_t capacity, const char* format, va_list args);
int Snprintf(char* output, size_t capacity, const char* format, ...)
    __attribute__((format(printf, 3, 4)));
#else
inline int Vsnprintf(char* out, size_t n, const char* format, va_list args) {
  return std::vsnprintf(out, n, format, args);
}
inline int Snprintf(char* out, size_t n, const char* format, ...) __attribute__((format(printf, 3, 4)));
inline int Snprintf(char* out, size_t n, const char* format, ...) {
  va_list args;
  va_start(args, format);
  const int result = std::vsnprintf(out, n, format, args);
  va_end(args);
  return result;
}
#endif
}  // namespace birdnet_format
