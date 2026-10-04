#include "NvsCalibrationStore.h"

#include <Preferences.h>

#include <cstddef>

namespace qspider {

namespace {
constexpr const char* kNamespace = "quadcal";
constexpr const char* kKey = "trims";
constexpr uint32_t kMagic = 0x51434C31;  // "QCL1"

struct Blob {
  uint32_t magic;
  float trimDeg[kServoCount];
  uint32_t checksum;
};

uint32_t checksum(const Blob& b) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&b);
  uint32_t h = 2166136261u;  // FNV-1a over everything but the checksum
  for (size_t i = 0; i < offsetof(Blob, checksum); ++i) h = (h ^ p[i]) * 16777619u;
  return h;
}
}  // namespace

bool NvsCalibrationStore::load(CalibrationData& out) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, true)) return false;
  Blob b{};
  const size_t n = prefs.getBytes(kKey, &b, sizeof(b));
  prefs.end();
  if (n != sizeof(b) || b.magic != kMagic || b.checksum != checksum(b)) return false;
  for (int i = 0; i < kServoCount; ++i) out.trimDeg[i] = b.trimDeg[i];
  return true;
}

bool NvsCalibrationStore::save(const CalibrationData& data) {
  Blob b{};
  b.magic = kMagic;
  for (int i = 0; i < kServoCount; ++i) b.trimDeg[i] = data.trimDeg[i];
  b.checksum = checksum(b);
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) return false;
  const size_t n = prefs.putBytes(kKey, &b, sizeof(b));
  prefs.end();
  return n == sizeof(b);
}

}  // namespace qspider
