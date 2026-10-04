#include "AdcPowerSensor.h"

#include <Arduino.h>

namespace qspider {

void AdcPowerSensor::begin() {
  if (!cfg_.enabled) return;
  analogReadResolution(12);
  analogSetPinAttenuation(cfg_.adcPin, ADC_11db);  // ~0.15..2.45 V linear range
}

bool AdcPowerSensor::readVoltage(float& volts) {
  if (!cfg_.enabled) return false;
  const uint8_t n = cfg_.samplesPerRead > 0 ? cfg_.samplesPerRead : 1;
  uint32_t sumMv = 0;
  for (uint8_t i = 0; i < n; ++i) sumMv += analogReadMilliVolts(cfg_.adcPin);  // eFuse-calibrated
  volts = (sumMv / static_cast<float>(n)) / 1000.0f * cfg_.dividerRatio * cfg_.calibrationScale;
  return true;
}

}  // namespace qspider
