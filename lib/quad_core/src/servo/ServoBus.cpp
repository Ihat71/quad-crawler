#include "servo/ServoBus.h"

#include <cmath>

#include "common/MathUtil.h"

namespace qspider {

ServoBus::ServoBus(const RobotConfig& cfg, IServoDriver& driver, EventLog& log)
    : cfg_(cfg), driver_(driver), log_(log), guard_(cfg.servoLimits) {
  for (int s = 0; s < kServoCount; ++s) {
    channel_[s] = cfg.legs[legOfServo(s)].servo[jointOfServo(s)].channel;
    target_[s] = cfg.geometry.servoCentreDeg;
    trim_[s] = 0.0f;
    output_[s] = NAN;
    enabled_[s] = false;
    wasClamped_[s] = false;
  }
}

void ServoBus::setTarget(int servo, float nominalDeg) { target_[servo] = nominalDeg; }

void ServoBus::setTrim(int servo, float trimDeg) {
  const float lim = cfg_.servoLimits.maxTrimDeg;
  trim_[servo] = clampf(trimDeg, -lim, lim);
}

void ServoBus::setEnabled(int servo, bool enabled) {
  if (enabled_[servo] == enabled) return;
  enabled_[servo] = enabled;
  // A freshly enabled SG90 jumps straight to its command (no position feedback),
  // so the slew limiter restarts from the target instead of an unknown position.
  output_[servo] = NAN;
}

void ServoBus::setLegEnabled(int leg, bool enabled) {
  for (int j = 0; j < kJointsPerLeg; ++j) setEnabled(servoIndex(leg, j), enabled);
}

void ServoBus::disableAll() {
  for (int s = 0; s < kServoCount; ++s) setEnabled(s, false);
}

bool ServoBus::anyEnabled() const {
  for (bool e : enabled_)
    if (e) return true;
  return false;
}

bool ServoBus::flush(float dtSec) {
  float frame[IServoDriver::kChannels];
  for (float& f : frame) f = NAN;

  const float maxStep = cfg_.servoLimits.maxSlewDegPerSec * dtSec;
  for (int s = 0; s < kServoCount; ++s) {
    if (!enabled_[s]) {
      output_[s] = NAN;
      continue;
    }
    bool clamped = false;
    const float fallback = std::isfinite(output_[s]) ? output_[s] : cfg_.geometry.servoCentreDeg;
    const float wanted = guard_.apply(target_[s] + trim_[s], fallback, &clamped);
    if (clamped) {
      ++clampEvents_;
      if (!wasClamped_[s]) {
        log_.log(LogLevel::Warn, LogCategory::Safety, "Servo %d (%s %s) clamped: %.1f -> %.1f deg", s,
                 legName(legOfServo(s)), jointName(jointOfServo(s)), target_[s] + trim_[s], wanted);
      }
    }
    wasClamped_[s] = clamped;
    output_[s] = std::isfinite(output_[s]) ? approach(output_[s], wanted, maxStep) : wanted;
    frame[channel_[s]] = output_[s];
  }
  return driver_.writeAll(frame);
}

}  // namespace qspider
