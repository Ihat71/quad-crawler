#pragma once

#include <cstdint>

#include "common/Types.h"

namespace qspider {

struct CalibrationData {
  float trimDeg[kServoCount] = {};
};

// Persistent storage for servo trims (NVS flash on the ESP32).
class ICalibrationStore {
 public:
  virtual ~ICalibrationStore() = default;
  virtual bool load(CalibrationData& out) = 0;  // false = nothing stored / corrupt
  virtual bool save(const CalibrationData& data) = 0;
};

}  // namespace qspider
