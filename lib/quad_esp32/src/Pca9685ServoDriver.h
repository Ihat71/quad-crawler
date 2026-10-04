#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "config/RobotConfig.h"
#include "hal/IServoDriver.h"

namespace qspider {

// PCA9685 16-channel PWM driver over I2C. All channels are written in a single
// auto-increment burst per control tick; pulse start times are staggered per
// channel to spread the servos' current peaks over the 20 ms frame.
class Pca9685ServoDriver : public IServoDriver {
 public:
  explicit Pca9685ServoDriver(const PcaConfig& cfg, TwoWire& wire = Wire);

  bool begin() override;
  bool writeAll(const float (&angleDeg)[kChannels]) override;
  bool allOff() override;
  bool verify() override;
  bool readChannel(int channel, float& angleDeg, bool& off) override;
  bool sleep() override;
  uint32_t errorCount() const override { return errors_; }

  bool present();  // I2C address acknowledges

 private:
  bool writeReg(uint8_t reg, uint8_t value);
  bool readRegs(uint8_t reg, uint8_t* out, uint8_t count);
  bool end(bool ok);
  uint16_t degToTicks(float deg) const;
  float ticksToDeg(uint16_t ticks) const;
  uint16_t phaseOf(int channel) const;
  void setOutputEnable(bool enabled);

  const PcaConfig& cfg_;
  TwoWire& wire_;
  bool wireStarted_ = false;
  uint8_t prescale_ = 0;
  float usPerTick_ = 4.88f;
  uint32_t errors_ = 0;
};

}  // namespace qspider
