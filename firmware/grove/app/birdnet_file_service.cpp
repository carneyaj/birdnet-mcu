#include "birdnet_format.h"
#include "config.h"
#if BIRDNET_ENABLE_SERIAL_FILES
#include "birdnet_file_service.h"
#include <cstdio>
extern "C" {
#include "ff.h"
}

namespace birdnet_files {
namespace {
// No allocator, file-sized buffers, or retained state. Reinitialized each session.
alignas(32) uint8_t buffer[kChunkBytes]
#if defined(__arm__)
    __attribute__((section(".scratch_dtcm.files")))
#endif
    ;
class Service {
 public:
  explicit Service(const Platform& p)
      : p_(p), start_(p.millis()), last_(start_), skipped_start_(p.skipped_windows()) {}
  ~Service() {
    if (file_open_) f_close(&file_);
    if (dir_open_) f_closedir(&dir_);
    p_.unmount();
  }
  bool Alive() {
    const uint64_t now = p_.millis();
    return now - start_ < kSessionMs && now - last_ < kIdleMs && p_.progress();
  }
  bool Reply(uint16_t id, Status status, const uint8_t* data = nullptr, size_t size = 0) {
    uint8_t header[12] = {'B', 'N', 'R', '2', uint8_t(id), uint8_t(id >> 8), uint8_t(status), 0};
    Put32(header + 8, uint32_t(size));
    uint32_t crc = CrcUpdate(~0u, header, sizeof(header));
    crc = CrcUpdate(crc, data, size) ^ ~0u;
    uint8_t trailer[4];
    Put32(trailer, crc);
    return p_.write(header, sizeof(header)) && (!size || p_.write(data, size)) &&
           p_.write(trailer, sizeof(trailer));
  }
  bool Receive(uint8_t* packet) {
    // Eight-byte credits fit even the 16-byte hardware FIFO. The host never
    // transmits while FatFs is busy or while a response is being serialized.
    for (size_t base = 0; base < kRequestBytes; base += 8) {
      const size_t count = (kRequestBytes - base < 8) ? kRequestBytes - base : 8;
      size_t have = 0;
      while (have < count) {
        if (!Alive()) return false;
        int got = p_.read(packet + base + have, count - have);
        if (got < 0 || size_t(got) > count - have) return false;
        have += size_t(got);
      }
      const uint8_t credit = 6;
      if (!p_.write(&credit, 1)) return false;
    }
    return true;
  }
  void Run(const char* deployment) {
    // HELLO is sent before potentially slow SD initialization. Subsequent
    // replies are always CRC-framed; no printf may share this UART now.
    if (!Reply(0, Busy)) return;
    if (!p_.mount()) { Reply(0, IoError); return; }
    last_ = p_.millis();
    if (!Alive() || !Reply(0, Ok, reinterpret_cast<const uint8_t*>(deployment), std::strlen(deployment))) return;
    uint8_t packet[kRequestBytes];
    while (Receive(packet)) {
      Request r{};
      if (!Decode(packet, &r)) { Reply(0, Invalid); return; }
      last_ = p_.millis(); // Only a complete CRC-valid command extends inactivity.
      if (r.op == Quit) {
        Put32(buffer, uint32_t(p_.millis() - start_));
        Put32(buffer + 4, p_.skipped_windows() - skipped_start_);
        Reply(r.id, Ok, buffer, 8);
        return;
      }
      if (!Command(r)) return;
    }
  }
 private:
  bool Command(const Request& r) {
    if (r.op == List) {
      if (dir_open_) { f_closedir(&dir_); dir_open_ = false; }
      std::memcpy(directory_, r.path, sizeof(directory_));
      if (f_opendir(&dir_, directory_) != FR_OK) return Reply(r.id, IoError);
      dir_open_ = true;
    }
    if (r.op == List || r.op == Next) {
      if (!dir_open_) return Reply(r.id, End);
      size_t used = 0;
      while (used + kEntryBytes <= kChunkBytes) {
        if (!Alive()) return false;
        FILINFO info{};
        if (f_readdir(&dir_, &info) != FR_OK) return Reply(r.id, IoError);
        if (!info.fname[0]) { f_closedir(&dir_); dir_open_ = false; break; }
        char path[kPathBytes];
        const int n = birdnet_format::Snprintf(path, sizeof(path), "%s%s%s", directory_, *directory_ ? "/" : "", info.fname);
        if (n < 0 || n >= int(sizeof(path))) continue;
        const bool directory = (info.fattrib & AM_DIR) != 0;
        if (directory ? !Directory(path) : !File(path)) continue;
        std::memset(buffer + used, 0, kEntryBytes);
        std::memcpy(buffer + used, path, size_t(n));
        Put32(buffer + used + kPathBytes, uint32_t(info.fsize));
        buffer[used + kPathBytes + 4] = directory ? 1 : 0;
        used += kEntryBytes;
      }
      return Reply(r.id, used ? Ok : End, buffer, used);
    }
    if (r.op == Open) {
      if (file_open_) { f_close(&file_); file_open_ = false; }
      if (f_open(&file_, r.path, FA_READ) != FR_OK) return Reply(r.id, IoError);
      file_open_ = true;
      size_ = uint32_t(f_size(&file_));
      uint32_t crc = ~0u, pos = 0;
      uint64_t notified = p_.millis();
      while (pos < size_) {
        // Long hashes are active work, not an idle host; the absolute session
        // deadline and independent capture watchdog still apply.
        last_ = p_.millis();
        if (!Alive()) return false;
        UINT got = 0;
        const UINT count = (size_ - pos < kChunkBytes) ? size_ - pos : kChunkBytes;
        if (f_read(&file_, buffer, count, &got) != FR_OK || got != count) {
          f_close(&file_); file_open_ = false;
          return Reply(r.id, IoError);
        }
        crc = CrcUpdate(crc, buffer, got);
        pos += got;
        if (p_.millis() - notified >= 1000) {
          if (!Reply(r.id, Busy)) return false;
          notified = p_.millis();
        }
      }
      Put32(buffer, size_);
      Put32(buffer + 4, crc ^ ~0u);
      last_ = p_.millis();
      return Reply(r.id, Ok, buffer, 8);
    }
    if (r.op == Read) {
      if (!file_open_ || r.offset > size_ || r.length > size_ - r.offset)
        return Reply(r.id, Invalid);
      UINT got = 0;
      if (f_lseek(&file_, r.offset) != FR_OK ||
          f_read(&file_, buffer, r.length, &got) != FR_OK || got != r.length)
        return Reply(r.id, IoError);
      return Reply(r.id, Ok, buffer, got);
    }
    return Reply(r.id, Invalid);
  }
  const Platform& p_;
  uint64_t start_, last_;
  uint32_t skipped_start_;
  FIL file_{};
  DIR dir_{};
  bool file_open_ = false, dir_open_ = false;
  uint32_t size_ = 0;
  char directory_[kPathBytes]{};
};
}  // namespace
void Run(const Platform& p, const char* deployment) { Service service(p); service.Run(deployment); }
}  // namespace birdnet_files
#endif
