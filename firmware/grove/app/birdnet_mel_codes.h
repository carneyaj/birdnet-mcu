// Float32 mel-to-int8 shortcuts; ambiguous intervals use the ordinary path.
namespace birdnet_quant {
inline bool enabled() {
  return true;
}
constexpr unsigned first=(127-48)*16, count=60*16;
struct Table {
  int16_t q[count], zero;
#if BIRDNET_FIELD_MEL_CODES
  uint32_t cut[count];
#endif
  float mul, add;
  bool ready;
};
inline Table table[2];
inline int reference(float x, int ch) {
  const float c=CompressMelEnergy(x,ch,birdnet_frontend::kBranchPowerExponent[ch]);
  return Clamp(int(std::nearbyint(c*g_rt.input_quant_mul[ch]+g_rt.input_quant_add[ch])),-128,127);
}
inline uint32_t bits(float x) { uint32_t u; std::memcpy(&u,&x,4); return u; }
inline float value(uint32_t u) { float x; std::memcpy(&x,&u,4); return x; }
inline void prepare(int ch) {
  auto& t=table[ch];
  if (t.ready && t.mul==g_rt.input_quant_mul[ch] && t.add==g_rt.input_quant_add[ch]) return;
  t.mul=g_rt.input_quant_mul[ch]; t.add=g_rt.input_quant_add[ch]; t.ready=true;
  t.zero=reference(0,ch);
  const float* lut=ch?birdnet_tables::kMantissa1:birdnet_tables::kMantissa0;
  bool monotone=cfg::kUseCompressionLut && t.mul>0;
  for (int i=1;i<=kPowMantissaLutSize;++i) monotone &= lut[i]>=lut[i-1];
  for (unsigned i=0;i<count;++i) {
    float lo=value((i+first)<<19), hi=value(((i+first+1)<<19)-1);
    // Within a normal energy exponent the mantissa interpolation is monotone.
    // Exclude exponent transitions, including the subnormal range.
    const unsigned e=bits(lo*lo)>>23;
    int q=reference(lo,ch);
    const bool segment=monotone && e!=0 && e!=255 && e==(bits(hi*hi)>>23);
    const int last=reference(hi,ch);
#if BIRDNET_FIELD_MEL_CODES
    t.cut[i]=~0u;
    if (segment && last==q+1) {
      // Find the first binary32 magnitude that rounds above the lower code.
      uint32_t l=bits(lo),h=bits(hi);
      while (h-l>1) {
        uint32_t m=l+(h-l)/2;
        if (reference(value(m),ch)==q) l=m; else h=m;
      }
      t.cut[i]=h;
    } else
#endif
    if (!segment || q!=last) q=256;
    t.q[i]=q;
  }
}
inline bool store(int index,float x,int ch) {
  uint32_t mag=bits(x)&0x7fffffff;
  unsigned i=(mag>>19)-first;
  const auto& t=table[ch];
  int q=!mag?t.zero:i<count?t.q[i]:256;
  if (q==256) return false;
#if BIRDNET_FIELD_MEL_CODES
  if (mag && mag>=t.cut[i]) ++q;
#endif
  g_rt.input_int8_data[index]=int8_t(q);
  return true;
}
}
