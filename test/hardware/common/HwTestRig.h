#pragma once
// Shared helpers for the on-device test firmware (test/hardware/*).
//
// The hardware suites drive the REAL PCA9685 and servos, and verify every
// result by reading the PCA9685 registers back over I2C. Inputs that cannot be
// produced on demand (controller presses, a dying battery) are scripted through
// the fake controller / power sensor of RobotHarness, running in real time.
//
// Safety: put the robot on a stand with its legs in the air for test_servos and
// test_legs. test_poses and test_estop can run with the robot on the floor.

#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

#include <cmath>
#include <memory>

#include "Pca9685ServoDriver.h"
#include "RobotHarness.h"
#include "quad_core.h"
#include "robot_config.h"

namespace hw {

using namespace qspider;
using namespace qspider::testing;

inline RobotConfig& config() {
  static RobotConfig c = makeRobotConfig();
  return c;
}

inline Pca9685ServoDriver& driver() {
  static Pca9685ServoDriver d(config().pca);
  return d;
}

inline void realDelay(uint32_t ms) { delay(ms); }

// A complete Robot on the real servo driver with a scripted controller/battery.
inline std::unique_ptr<RobotHarness> makeRobot(const RobotConfig& cfg = config(), IServoDriver* drv = nullptr) {
  return std::unique_ptr<RobotHarness>(new RobotHarness(cfg, drv ? drv : &driver(), &realDelay));
}

inline void banner(const char* text) { Serial.printf("\n>>> %s\n", text); }

inline void beginSuite(const char* name) {
  Serial.begin(115200);
  delay(2500);  // give the test runner time to attach to the serial port
  Serial.printf("\n==== QuadSpider hardware test: %s ====\n", name);
}

// Asserts the channel is driving a pulse that corresponds to `deg`.
inline void assertChannelAt(int ch, float deg, float tol = 1.5f) {
  float got = 0;
  bool off = true;
  char msg[64];
  snprintf(msg, sizeof(msg), "PCA9685 channel %d readback", ch);
  TEST_ASSERT_TRUE_MESSAGE(driver().readChannel(ch, got, off), msg);
  TEST_ASSERT_FALSE_MESSAGE(off, msg);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tol, deg, got, msg);
}

inline void assertChannelOff(int ch) {
  float got = 0;
  bool off = false;
  char msg[64];
  snprintf(msg, sizeof(msg), "PCA9685 channel %d should be OFF", ch);
  TEST_ASSERT_TRUE_MESSAGE(driver().readChannel(ch, got, off), msg);
  TEST_ASSERT_TRUE_MESSAGE(off, msg);
}

inline void assertAllServosOff() {
  for (int s = 0; s < kServoCount; ++s) assertChannelOff(config().legs[legOfServo(s)].servo[jointOfServo(s)].channel);
  if (config().pca.outputEnablePin >= 0)
    TEST_ASSERT_EQUAL_MESSAGE(HIGH, digitalRead(config().pca.outputEnablePin), "OE pin should disable outputs");
}

// Every enabled servo's PCA9685 register matches what the ServoBus sent.
inline void assertOutputsMatch(const ServoBus& bus) {
  for (int s = 0; s < kServoCount; ++s) {
    if (bus.enabled(s))
      assertChannelAt(bus.channel(s), bus.output(s));
    else
      assertChannelOff(bus.channel(s));
  }
}

// Wraps the real driver so I2C failures can be injected on demand.
class FaultInjectingDriver : public IServoDriver {
 public:
  explicit FaultInjectingDriver(IServoDriver& real) : real_(real) {}
  bool failWrites = false;

  bool begin() override { return real_.begin(); }
  bool writeAll(const float (&a)[kChannels]) override {
    if (failWrites) return false;
    return real_.writeAll(a);
  }
  bool allOff() override { return real_.allOff(); }  // the kill path still reaches the chip
  bool verify() override { return real_.verify(); }
  bool readChannel(int ch, float& d, bool& off) override { return real_.readChannel(ch, d, off); }
  bool sleep() override { return real_.sleep(); }
  uint32_t errorCount() const override { return real_.errorCount(); }

 private:
  IServoDriver& real_;
};

// PCA9685 "software reset" general call: the chip returns to its power-on
// state exactly like after a brown-out of its supply.
inline void resetPca9685ViaGeneralCall() {
  Wire.beginTransmission(0x00);
  Wire.write(0x06);
  Wire.endTransmission();
  delay(2);
}

}  // namespace hw
