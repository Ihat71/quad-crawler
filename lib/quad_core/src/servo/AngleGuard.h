#pragma once

#include <cmath>

#include "config/RobotConfig.h"

namespace qspider {

// Enforces the servo angle envelope. By requirement the envelope is 10..170 deg;
// a configuration may only widen it when allowLimitOverride is explicitly set.
class AngleGuard {
 public:
  static constexpr float kRequiredMinDeg = 10.0f;
  static constexpr float kRequiredMaxDeg = 170.0f;

  explicit AngleGuard(const ServoLimitConfig& cfg) {
    min_ = cfg.minDeg;
    max_ = cfg.maxDeg;
    if (!cfg.allowLimitOverride) {
      if (min_ < kRequiredMinDeg) { min_ = kRequiredMinDeg; configAdjusted_ = true; }
      if (max_ > kRequiredMaxDeg) { max_ = kRequiredMaxDeg; configAdjusted_ = true; }
    }
    if (!(min_ < max_)) {  // nonsensical config: fall back to the required envelope
      min_ = kRequiredMinDeg;
      max_ = kRequiredMaxDeg;
      configAdjusted_ = true;
    }
  }

  // Returns the angle limited to the envelope. NaN/inf is mapped to `fallback`.
  float apply(float deg, float fallback, bool* clamped = nullptr) const {
    bool c = false;
    if (!std::isfinite(deg)) {
      deg = fallback;
      c = true;
    }
    if (deg < min_) { deg = min_; c = true; }
    if (deg > max_) { deg = max_; c = true; }
    if (clamped) *clamped = c;
    return deg;
  }

  float minDeg() const { return min_; }
  float maxDeg() const { return max_; }
  // True if the configured limits violated the 10..170 rule and were tightened.
  bool configAdjusted() const { return configAdjusted_; }

 private:
  float min_;
  float max_;
  bool configAdjusted_ = false;
};

}  // namespace qspider
