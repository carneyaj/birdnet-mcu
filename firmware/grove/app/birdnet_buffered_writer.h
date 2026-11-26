#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "birdnet_clip_metadata.h"

namespace birdnet_storage {
using WriteFn = bool (*)(void*, const uint8_t*, size_t);
using SeekFn = bool (*)(void*, uint32_t);
// Non-owning buffer; single writer at a time. Error state is sticky.
class BufferedWriter {
 public:
  BufferedWriter(void* ctx, WriteFn write, SeekFn seek, uint8_t* buffer, size_t capacity,
                 size_t header_bytes = 0)
      : ctx_(ctx), write_(write), seek_(seek), buffer_(buffer), capacity_(capacity),
        header_bytes_(header_bytes) {
    ok_ = ctx && write && seek && buffer && capacity && header_bytes <= sizeof(header_);
  }
  bool Flush() {
    if (!ok_) return false;
    if (used_ && !write_(ctx_, buffer_, used_)) return ok_ = false;
    used_ = 0;
    return true;
  }
  bool Write(const uint8_t* data, size_t size) {
    if (!ok_ || (!data && size) || size > UINT32_MAX - position_) return ok_ = false;
    if (position_ < header_bytes_) {
      const size_t n = std::min<size_t>(size, header_bytes_ - position_);
      std::memmove(header_ + position_, data, n);
    }
    const size_t skip = position_ < header_bytes_ ? std::min<size_t>(size, header_bytes_ - position_) : 0;
    if (header_bytes_ && size > skip) crc_ = birdnet_clip::Crc(crc_, data + skip, size - skip);
    position_ += size;
    while (size) {
      const size_t n = std::min(size, capacity_ - used_);
      std::memcpy(buffer_ + used_, data, n); used_ += n; data += n; size -= n;
      if (used_ == capacity_ && !Flush()) return false;
    }
    return true;
  }
  bool Seek(uint32_t offset) {
    if (!Flush() || !seek_(ctx_, offset)) return ok_ = false;
    position_ = offset;
    return true;
  }
  // Call exactly once after encoding. CRC covers audio payload, excluding header.
  bool FinishMetadata(bool brc, uint32_t audio_file_bytes, const birdnet_clip::Metadata& metadata) {
    if (!ok_ || position_ != audio_file_bytes || header_bytes_ != (brc ? 32u : 60u) ||
        audio_file_bytes < header_bytes_ || audio_file_bytes > UINT32_MAX - 264u) return false;
    uint8_t packed[birdnet_clip::kBytes];
    if (!brc) birdnet_clip::Put32(header_ + 4, audio_file_bytes + 264u - 8u);
    if (!birdnet_clip::Pack(metadata, ~crc_, audio_file_bytes - header_bytes_, packed,
                           brc ? 0 : ~birdnet_clip::Crc(~0u, header_, header_bytes_))) return false;
    if (!brc) {
      uint8_t chunk[8] = {'b', 'n', 'm', 'd'};
      birdnet_clip::Put32(chunk + 4, sizeof(packed));
      if (!Write(chunk, sizeof(chunk))) return false;
    }
    if (!Write(packed, sizeof(packed))) return false;
    const uint32_t end = position_;
    if (brc) {
      std::memcpy(header_, "BRC2", 4); header_[4] = 2;
      birdnet_clip::Put32(header_ + 20, sizeof(packed));
      birdnet_clip::Put32(header_ + 28, ~birdnet_clip::Crc(~0u, header_, 28));
    } else {
      birdnet_clip::Put32(header_ + 4, end - 8);
    }
    // A failed final rewrite is never indexed as a complete recording.
    return Seek(0) && Write(header_, header_bytes_) && Seek(end);
  }
 private:
  void* ctx_; WriteFn write_; SeekFn seek_; uint8_t* buffer_; size_t capacity_;
  size_t header_bytes_, used_ = 0;
  uint32_t position_ = 0, crc_ = ~0u;
  bool ok_ = false;
  uint8_t header_[60]{};
};
} // namespace birdnet_storage
