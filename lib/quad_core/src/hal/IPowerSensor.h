#pragma once

namespace qspider {

class IPowerSensor {
 public:
  virtual ~IPowerSensor() = default;
  // False when no sensor is fitted; battery protection is then disabled.
  virtual bool available() const = 0;
  virtual bool readVoltage(float& volts) = 0;
};

// Used when the optional voltage divider is not wired.
class NullPowerSensor : public IPowerSensor {
 public:
  bool available() const override { return false; }
  bool readVoltage(float&) override { return false; }
};

}  // namespace qspider
