#pragma once

#include <cstdint>

#include "config/RobotConfig.h"

namespace qspider {

enum class BatteryStatus : uint8_t { Unavailable, Ok, Low, Critical, Overvoltage, Unstable, SensorFault };
const char* toString(BatteryStatus s);

// Pure battery supervision logic (no ADC access): filtering, thresholds with
// hysteresis, hold times and ripple ("irregular power") detection.
class BatteryMonitor {
 public:
  explicit BatteryMonitor(const BatteryConfig& cfg);

  void addSample(float volts, uint32_t nowMs);
  void markReadError(uint32_t nowMs);

  BatteryStatus status() const { return status_; }
  float voltage() const { return ema_; }
  float lastRaw() const { return lastRaw_; }
  float ripple() const { return ripple_; }
  int percent() const;
  bool hasData() const { return hasData_; }

 private:
  static constexpr int kWindow = 32;

  void evaluate(uint32_t nowMs);

  const BatteryConfig& cfg_;
  BatteryStatus status_ = BatteryStatus::Unavailable;
  bool hasData_ = false;
  float ema_ = 0.0f;
  float lastRaw_ = 0.0f;
  float ripple_ = 0.0f;
  float window_[kWindow] = {};
  uint32_t windowTime_[kWindow] = {};
  int windowCount_ = 0;
  int windowHead_ = 0;
  uint32_t belowCriticalSinceMs_ = 0;
  bool belowCritical_ = false;
  uint32_t aboveOverSinceMs_ = 0;
  bool aboveOver_ = false;
  bool criticalLatched_ = false;
  bool lowLatched_ = false;
};

}  // namespace qspider
