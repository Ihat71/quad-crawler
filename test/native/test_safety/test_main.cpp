// Angle guard (10..170), servo bus (trim, slew, channel map) and battery monitor.

#include <unity.h>

#include "Fakes.h"
#include "RobotHarness.h"
#include "quad_core.h"
#include "robot_config.h"

using namespace qspider;
using namespace qspider::testing;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- guard --

void test_guard_clamps_to_10_170() {
  ServoLimitConfig lim{10, 170, false, 450, 25};
  AngleGuard g(lim);
  bool clamped = false;
  TEST_ASSERT_EQUAL_FLOAT(10.0f, g.apply(0.0f, 90.0f, &clamped));
  TEST_ASSERT_TRUE(clamped);
  TEST_ASSERT_EQUAL_FLOAT(170.0f, g.apply(200.0f, 90.0f, &clamped));
  TEST_ASSERT_TRUE(clamped);
  TEST_ASSERT_EQUAL_FLOAT(95.0f, g.apply(95.0f, 90.0f, &clamped));
  TEST_ASSERT_FALSE(clamped);
}

void test_guard_rejects_wider_limits_without_override() {
  ServoLimitConfig lim{0, 180, false, 450, 25};
  AngleGuard g(lim);
  TEST_ASSERT_TRUE(g.configAdjusted());
  TEST_ASSERT_EQUAL_FLOAT(10.0f, g.minDeg());
  TEST_ASSERT_EQUAL_FLOAT(170.0f, g.maxDeg());
}

void test_guard_allows_override_when_explicit() {
  ServoLimitConfig lim{5, 175, true, 450, 25};
  AngleGuard g(lim);
  TEST_ASSERT_FALSE(g.configAdjusted());
  TEST_ASSERT_EQUAL_FLOAT(5.0f, g.apply(0.0f, 90.0f));
}

void test_guard_maps_nan_to_fallback() {
  ServoLimitConfig lim{10, 170, false, 450, 25};
  AngleGuard g(lim);
  bool clamped = false;
  TEST_ASSERT_EQUAL_FLOAT(77.0f, g.apply(NAN, 77.0f, &clamped));
  TEST_ASSERT_TRUE(clamped);
}

// ------------------------------------------------------------ servo bus --

void test_bus_applies_trim_guard_and_channel_map() {
  RobotConfig cfg = makeRobotConfig();
  FakeServoDriver drv;
  EventLog log(&fakeClock);
  ServoBus bus(cfg, drv, log);

  bus.setEnabled(4, true);  // FL femur -> channel 4
  bus.setTarget(4, 100.0f);
  bus.setTrim(4, 3.0f);
  TEST_ASSERT_TRUE(bus.flush(0.02f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 103.0f, drv.channels[cfg.legs[1].servo[1].channel]);
  TEST_ASSERT_EQUAL_INT(1, drv.onCount());  // disabled servos stay off

  bus.setTarget(4, 250.0f);  // way out of range
  for (int i = 0; i < 50; ++i) bus.flush(0.02f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 170.0f, drv.channels[4]);
  TEST_ASSERT_TRUE(bus.clampEvents() > 0);
}

void test_bus_trim_is_limited() {
  RobotConfig cfg = makeRobotConfig();
  FakeServoDriver drv;
  EventLog log(&fakeClock);
  ServoBus bus(cfg, drv, log);
  bus.setTrim(0, 99.0f);
  TEST_ASSERT_EQUAL_FLOAT(cfg.servoLimits.maxTrimDeg, bus.trim(0));
}

void test_bus_slew_limits_motion() {
  RobotConfig cfg = makeRobotConfig();
  FakeServoDriver drv;
  EventLog log(&fakeClock);
  ServoBus bus(cfg, drv, log);
  bus.setEnabled(0, true);
  bus.setTarget(0, 60.0f);
  bus.flush(0.02f);  // first frame after enabling jumps (no position feedback)
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, drv.channels[0]);
  bus.setTarget(0, 120.0f);
  bus.flush(0.02f);
  const float maxStep = cfg.servoLimits.maxSlewDegPerSec * 0.02f;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f + maxStep, drv.channels[0]);
}

void test_bus_disable_turns_outputs_off() {
  RobotConfig cfg = makeRobotConfig();
  FakeServoDriver drv;
  EventLog log(&fakeClock);
  ServoBus bus(cfg, drv, log);
  for (int s = 0; s < kServoCount; ++s) bus.setEnabled(s, true);
  bus.flush(0.02f);
  TEST_ASSERT_EQUAL_INT(kServoCount, drv.onCount());
  bus.disableAll();
  bus.flush(0.02f);
  TEST_ASSERT_FALSE(drv.anyOn());
}

// -------------------------------------------------------------- battery --

static BatteryConfig batteryCfg() {
  RobotConfig c = makeRobotConfig();
  c.battery.enabled = true;
  return c.battery;
}

void test_battery_ok_low_critical_with_hold_time() {
  const BatteryConfig cfg = batteryCfg();
  BatteryMonitor m(cfg);
  uint32_t t = 0;
  for (int i = 0; i < 20; ++i) m.addSample(7.8f, t += 100);
  TEST_ASSERT_EQUAL(BatteryStatus::Ok, m.status());

  for (int i = 0; i < 40; ++i) m.addSample(6.9f, t += 100);
  TEST_ASSERT_EQUAL(BatteryStatus::Low, m.status());

  // Critical only after criticalHoldMs below the threshold.
  int samplesUntilCritical = 0;
  while (m.status() != BatteryStatus::Critical && samplesUntilCritical < 100) {
    m.addSample(6.3f, t += 100);
    ++samplesUntilCritical;
  }
  TEST_ASSERT_EQUAL(BatteryStatus::Critical, m.status());
  TEST_ASSERT_TRUE(samplesUntilCritical * 100 >= static_cast<int>(cfg.criticalHoldMs));
}

void test_battery_short_dip_is_not_critical() {
  const BatteryConfig cfg = batteryCfg();
  BatteryMonitor m(cfg);
  uint32_t t = 0;
  for (int i = 0; i < 20; ++i) m.addSample(7.6f, t += 100);
  for (int i = 0; i < 3; ++i) m.addSample(6.0f, t += 100);  // 300 ms sag
  for (int i = 0; i < 20; ++i) m.addSample(7.6f, t += 100);
  TEST_ASSERT_NOT_EQUAL(BatteryStatus::Critical, m.status());
}

void test_battery_critical_releases_only_above_warn() {
  const BatteryConfig cfg = batteryCfg();
  BatteryMonitor m(cfg);
  uint32_t t = 0;
  for (int i = 0; i < 60; ++i) m.addSample(6.2f, t += 100);
  TEST_ASSERT_EQUAL(BatteryStatus::Critical, m.status());
  for (int i = 0; i < 40; ++i) m.addSample(6.8f, t += 100);  // above critical, below warn
  TEST_ASSERT_EQUAL(BatteryStatus::Critical, m.status());
  for (int i = 0; i < 40; ++i) m.addSample(7.6f, t += 100);
  TEST_ASSERT_EQUAL(BatteryStatus::Ok, m.status());
}

void test_battery_ripple_is_irregular_power() {
  const BatteryConfig cfg = batteryCfg();
  BatteryMonitor m(cfg);
  uint32_t t = 0;
  for (int i = 0; i < 20; ++i) m.addSample(7.6f, t += 100);
  for (int i = 0; i < 6; ++i) m.addSample(i % 2 ? 8.2f : 6.6f, t += 100);
  TEST_ASSERT_EQUAL(BatteryStatus::Unstable, m.status());
}

void test_battery_overvoltage() {
  const BatteryConfig cfg = batteryCfg();
  BatteryMonitor m(cfg);
  uint32_t t = 0;
  for (int i = 0; i < 40; ++i) m.addSample(9.5f, t += 100);
  TEST_ASSERT_EQUAL(BatteryStatus::Overvoltage, m.status());
}

void test_battery_implausible_reading_is_sensor_fault() {
  const BatteryConfig cfg = batteryCfg();
  BatteryMonitor m(cfg);
  m.addSample(0.1f, 100);
  TEST_ASSERT_EQUAL(BatteryStatus::SensorFault, m.status());
}

void test_battery_percent() {
  const BatteryConfig cfg = batteryCfg();
  BatteryMonitor m(cfg);
  TEST_ASSERT_EQUAL_INT(-1, m.percent());
  m.addSample(cfg.fullVoltage, 100);
  TEST_ASSERT_EQUAL_INT(100, m.percent());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_guard_clamps_to_10_170);
  RUN_TEST(test_guard_rejects_wider_limits_without_override);
  RUN_TEST(test_guard_allows_override_when_explicit);
  RUN_TEST(test_guard_maps_nan_to_fallback);
  RUN_TEST(test_bus_applies_trim_guard_and_channel_map);
  RUN_TEST(test_bus_trim_is_limited);
  RUN_TEST(test_bus_slew_limits_motion);
  RUN_TEST(test_bus_disable_turns_outputs_off);
  RUN_TEST(test_battery_ok_low_critical_with_hold_time);
  RUN_TEST(test_battery_short_dip_is_not_critical);
  RUN_TEST(test_battery_critical_releases_only_above_warn);
  RUN_TEST(test_battery_ripple_is_irregular_power);
  RUN_TEST(test_battery_overvoltage);
  RUN_TEST(test_battery_implausible_reading_is_sensor_fault);
  RUN_TEST(test_battery_percent);
  return UNITY_END();
}
