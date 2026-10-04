#pragma once

#include "hal/ICalibrationStore.h"

namespace qspider {

// Servo trims in the ESP32's NVS flash (survives reboots and re-flashing,
// unless the flash is fully erased).
class NvsCalibrationStore : public ICalibrationStore {
 public:
  bool load(CalibrationData& out) override;
  bool save(const CalibrationData& data) override;
};

}  // namespace qspider
