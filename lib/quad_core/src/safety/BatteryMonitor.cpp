#include "safety/BatteryMonitor.h"

#include <cmath>

#include "common/MathUtil.h"

namespace qspider {

const char* toString(BatteryStatus s) {
  switch (s) {
    case BatteryStatus::Unavailable: return "UNAVAILABLE";
    case BatteryStatus::Ok: return "OK";
    case BatteryStatus::Low: return "LOW";
    case BatteryStatus::Critical: return "CRITICAL";
    case BatteryStatus::Overvoltage: return "OVERVOLTAGE";
    case BatteryStatus::Unstable: return "UNSTABLE";
    case BatteryStatus::SensorFault: return "SENSOR_FAULT";
  }
  return "?";
}

BatteryMonitor::BatteryMonitor(const BatteryConfig& cfg) : cfg_(cfg) {}

void BatteryMonitor::markReadError(uint32_t) { status_ = BatteryStatus::SensorFault; }

void BatteryMonitor::addSample(float volts, uint32_t nowMs) {
  lastRaw_ = volts;
  if (!std::isfinite(volts) || volts < cfg_.plausibleMinVoltage || volts > cfg_.plausibleMaxVoltage) {
    status_ = BatteryStatus::SensorFault;
    return;
  }

  if (!hasData_) {
    ema_ = volts;
    hasData_ = true;
  } else {
    ema_ += cfg_.emaAlpha * (volts - ema_);
  }

  window_[windowHead_] = volts;
  windowTime_[windowHead_] = nowMs;
  windowHead_ = (windowHead_ + 1) % kWindow;
  if (windowCount_ < kWindow) ++windowCount_;

  float lo = volts, hi = volts;
  int used = 0;
  for (int i = 0; i < windowCount_; ++i) {
    if (nowMs - windowTime_[i] > cfg_.rippleWindowMs) continue;
    lo = std::fmin(lo, window_[i]);
    hi = std::fmax(hi, window_[i]);
    ++used;
  }
  ripple_ = used >= 3 ? hi - lo : 0.0f;

  evaluate(nowMs);
}

void BatteryMonitor::evaluate(uint32_t nowMs) {
  // Over-voltage (wrong pack / charger connected).
  if (ema_ > cfg_.overVoltage) {
    if (!aboveOver_) {
      aboveOver_ = true;
      aboveOverSinceMs_ = nowMs;
    }
  } else {
    aboveOver_ = false;
  }
  const bool over = aboveOver_ && nowMs - aboveOverSinceMs_ >= cfg_.criticalHoldMs;

  // Critical under-voltage must persist; it is released only once the pack is
  // back above the warning level (a resting pack recovers a little).
  if (ema_ < cfg_.criticalVoltage) {
    if (!belowCritical_) {
      belowCritical_ = true;
      belowCriticalSinceMs_ = nowMs;
    }
    if (nowMs - belowCriticalSinceMs_ >= cfg_.criticalHoldMs) criticalLatched_ = true;
  } else {
    belowCritical_ = false;
    if (ema_ > cfg_.warnVoltage + cfg_.hysteresisVoltage) criticalLatched_ = false;
  }

  if (ema_ < cfg_.warnVoltage)
    lowLatched_ = true;
  else if (ema_ > cfg_.warnVoltage + cfg_.hysteresisVoltage)
    lowLatched_ = false;

  if (over)
    status_ = BatteryStatus::Overvoltage;
  else if (ripple_ > cfg_.rippleMaxVoltage)
    status_ = BatteryStatus::Unstable;
  else if (criticalLatched_)
    status_ = BatteryStatus::Critical;
  else if (lowLatched_)
    status_ = BatteryStatus::Low;
  else
    status_ = BatteryStatus::Ok;
}

int BatteryMonitor::percent() const {
  if (!hasData_) return -1;
  const float span = cfg_.fullVoltage - cfg_.criticalVoltage;
  if (span <= 0.0f) return -1;
  return static_cast<int>(clampf((ema_ - cfg_.criticalVoltage) / span, 0.0f, 1.0f) * 100.0f + 0.5f);
}

}  // namespace qspider
