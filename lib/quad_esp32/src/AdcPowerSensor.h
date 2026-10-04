#pragma once

#include "config/RobotConfig.h"
#include "hal/IPowerSensor.h"

namespace qspider {

// Battery voltage through a resistor divider on an ESP32 ADC1 pin. Reports
// "not available" when the divider is disabled in the configuration.
class AdcPowerSensor : public IPowerSensor {
 public:
  explicit AdcPowerSensor(const BatteryConfig& cfg) : cfg_(cfg) {}
  void begin();
  bool available() const override { return cfg_.enabled; }
  bool readVoltage(float& volts) override;

 private:
  const BatteryConfig& cfg_;
};

}  // namespace qspider
