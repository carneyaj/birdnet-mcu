// Full model-flash readback once per cold boot; no steady-state scanning.
#pragma once
uint64_t PlatformMillis();
__attribute__((optimize("Os"), noinline)) bool VerifyInstalledModel() {
  const uint64_t started = PlatformMillis();
  // QSPI XIP byte reads are expensive. One aligned word fetch supplies four
  // FNV bytes while preserving the same little-endian file checksum.
  static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);
  const auto *words = reinterpret_cast<const uint32_t *>(cfg::kModelFlashAddr);
  uint32_t hash = 2166136261u;
  for (uint32_t i = 0; i < birdnet_contract::kModelBytes / 4u; ++i) {
    const uint32_t word = words[i];
    hash = (hash ^ (word & 255u)) * 16777619u;
    hash = (hash ^ ((word >> 8) & 255u)) * 16777619u;
    hash = (hash ^ ((word >> 16) & 255u)) * 16777619u;
    hash = (hash ^ (word >> 24)) * 16777619u;
  }
  const auto* bytes = reinterpret_cast<const uint8_t*>(words);
  for (uint32_t i = birdnet_contract::kModelBytes & ~3u; i < birdnet_contract::kModelBytes; ++i)
    hash = (hash ^ bytes[i]) * 16777619u;
  const bool ok = hash == birdnet_contract::kModelFnv;
  LogLine("[bnmodel] bytes=%lu fnv1a=%08lx ok=%u ms=%lu\r\n",
          (unsigned long)birdnet_contract::kModelBytes, (unsigned long)hash,
          unsigned(ok), (unsigned long)(PlatformMillis() - started));
  return ok;
}
