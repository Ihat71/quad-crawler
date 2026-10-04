// HARDWARE TEST - low battery behaviour.
//  1. Simulated discharge on the real servos: the robot must sit down first and
//     then switch every output off (verified by PCA9685 readback).
//  2. Bench test with the real voltage divider (only when battery.enabled):
//     lower the bench supply below the critical voltage when prompted.
//   pio test -e esp32dev -f hardware/test_battery

#include "../common/HwTestRig.h"

#include "AdcPowerSensor.h"

using namespace hw;

void tearDown() { driver().allOff(); }

void test_simulated_discharge_sits_then_goes_limp() {
  RobotConfig cfg = config();
  cfg.battery.enabled = true;  // fake sensor supplies the voltage
  auto robot = makeRobot(cfg);
  robot->power.volts = 7.8f;
  TEST_ASSERT_TRUE(robot->bootAndArm());
  robot->tap(Button::Cross);
  TEST_ASSERT_TRUE(robot->waitIdle());

  banner("Simulating a discharge 7.8 V -> 6.3 V");
  for (float v = 7.8f; v > 6.3f; v -= 0.05f) {
    robot->power.volts = v;
    robot->runMs(100);
  }
  TEST_ASSERT_TRUE(robot->runUntil([&] { return robot->robot.lifecycle() == Lifecycle::Stopping; }, 6000));
  banner("Sitting down before power-off");
  TEST_ASSERT_TRUE(robot->runUntil([&] { return robot->robot.mode() == Mode::EmergencyStop; }, 6000));
  TEST_ASSERT_TRUE(robot->robot.hasFault(Fault::BatteryCritical));
  TEST_ASSERT_EQUAL(Posture::Sitting, robot->robot.motion().posture());
  assertAllServosOff();

  // Must refuse to re-arm while the battery is still low.
  robot->hold(Button::PlayStation, 2100);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, robot->robot.mode());
  assertAllServosOff();
}

void test_low_battery_at_boot_keeps_outputs_off() {
  RobotConfig cfg = config();
  cfg.battery.enabled = true;
  auto robot = makeRobot(cfg);
  robot->power.volts = 6.8f;
  robot->boot();
  robot->connectController();
  robot->runMs(2000);
  TEST_ASSERT_EQUAL(Lifecycle::WaitingForController, robot->robot.lifecycle());
  assertAllServosOff();
}

void test_bench_supply_undervoltage_with_real_divider() {
  const BatteryConfig& b = config().battery;
  if (!b.enabled) TEST_IGNORE_MESSAGE("battery.enabled = false in config/robot_config.h (no divider fitted)");

  AdcPowerSensor sensor(b);
  sensor.begin();
  BatteryMonitor monitor(b);
  float v = 0;
  TEST_ASSERT_TRUE(sensor.readVoltage(v));
  Serial.printf("  measured %.2f V\n", v);
  TEST_ASSERT_TRUE_MESSAGE(v > b.plausibleMinVoltage && v < b.plausibleMaxVoltage, "implausible reading - check divider");

  Serial.printf(">>> Slowly lower the bench supply below %.2f V within 60 s...\n", b.criticalVoltage);
  const uint32_t start = millis();
  while (millis() - start < 60000 && monitor.status() != BatteryStatus::Critical) {
    if (sensor.readVoltage(v)) monitor.addSample(v, millis());
    if ((millis() - start) % 2000 < b.samplePeriodMs)
      Serial.printf("  %.2f V  %s\n", monitor.voltage(), toString(monitor.status()));
    delay(b.samplePeriodMs);
  }
  TEST_ASSERT_EQUAL_MESSAGE(BatteryStatus::Critical, monitor.status(), "critical voltage not detected");
  Serial.println(">>> Detected. Restore the supply voltage.");
}

void setup() {
  beginSuite("low battery");
  TEST_ASSERT_TRUE(driver().begin());
  UNITY_BEGIN();
  RUN_TEST(test_simulated_discharge_sits_then_goes_limp);
  RUN_TEST(test_low_battery_at_boot_keeps_outputs_off);
  RUN_TEST(test_bench_supply_undervoltage_with_real_divider);
  UNITY_END();
}

void loop() {}
