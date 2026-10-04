#pragma once

#include <cstdint>

#include "common/Types.h"
#include "config/RobotConfig.h"
#include "hal/IServoDriver.h"
#include "servo/AngleGuard.h"
#include "telemetry/EventLog.h"

namespace qspider {

// The only path from robot logic to the servo driver. Applies, in order:
// calibration trim -> angle guard (10..170) -> slew-rate limit -> channel map,
// then writes all channels in a single I2C burst.
class ServoBus {
 public:
  ServoBus(const RobotConfig& cfg, IServoDriver& driver, EventLog& log);

  // Nominal angle (before trim) for a logical servo index 0..11.
  void setTarget(int servo, float nominalDeg);
  float target(int servo) const { return target_[servo]; }

  void setTrim(int servo, float trimDeg);
  float trim(int servo) const { return trim_[servo]; }

  void setEnabled(int servo, bool enabled);
  void setLegEnabled(int leg, bool enabled);
  void disableAll();
  bool enabled(int servo) const { return enabled_[servo]; }
  bool anyEnabled() const;

  // Computes outputs and writes them. Returns false on a driver error.
  bool flush(float dtSec);

  // Angle actually sent to the servo (after trim/guard/slew). NaN if disabled.
  float output(int servo) const { return output_[servo]; }
  uint8_t channel(int servo) const { return channel_[servo]; }

  uint32_t clampEvents() const { return clampEvents_; }
  const AngleGuard& guard() const { return guard_; }

 private:
  const RobotConfig& cfg_;
  IServoDriver& driver_;
  EventLog& log_;
  AngleGuard guard_;
  uint8_t channel_[kServoCount];
  float target_[kServoCount];
  float trim_[kServoCount];
  float output_[kServoCount];
  bool enabled_[kServoCount];
  bool wasClamped_[kServoCount];
  uint32_t clampEvents_ = 0;
};

}  // namespace qspider
