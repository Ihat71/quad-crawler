// HARDWARE TEST - error handling.
// Injects real faults (PCA9685 reset as after a brown-out, I2C write failures,
// controller loss) and checks detection, the safe state and recovery. Also
// measures the control-loop execution time on the target.
//   pio test -e esp32dev -f hardware/test_errors

#include "../common/HwTestRig.h"

using namespace hw;

void setUp() { TEST_ASSERT_TRUE(driver().begin()); }
void tearDown() { driver().allOff(); }

void test_shipped_config_passes_validation_on_target() {
  BodyKinematics kin(config());
  EventLog log(&fakeClock);
  const ConfigReport r = validateConfig(config(), kin, log);
  TEST_ASSERT_EQUAL_INT(0, r.errors);
}

void test_pca9685_reset_is_detected_and_recoverable() {
  auto robot = makeRobot();
  TEST_ASSERT_TRUE(robot->bootAndArm());
  banner("Resetting the PCA9685 (simulated servo-rail brown-out)");
  resetPca9685ViaGeneralCall();
  TEST_ASSERT_TRUE(robot->runUntil([&] { return robot->robot.mode() == Mode::EmergencyStop; },
                                   robot->cfg.safety.driverVerifyPeriodMs + 500));
  TEST_ASSERT_TRUE(robot->robot.hasFault(Fault::ServoDriverReset));
  assertAllServosOff();

  banner("Recovering with a 2 s PS hold");
  robot->hold(Button::PlayStation, 2100);
  TEST_ASSERT_TRUE(robot->runUntil([&] { return robot->robot.lifecycle() == Lifecycle::Active; }, 4000));
  TEST_ASSERT_FALSE(robot->robot.hasFault(Fault::ServoDriverReset));
  assertOutputsMatch(robot->robot.servos());
}

void test_i2c_write_failures_stop_the_robot() {
  FaultInjectingDriver faulty(driver());
  auto robot = makeRobot(config(), &faulty);
  TEST_ASSERT_TRUE(robot->bootAndArm());
  faulty.failWrites = true;
  robot->runMs(200);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, robot->robot.mode());
  TEST_ASSERT_TRUE(robot->robot.hasFault(Fault::ServoDriverComm));
  assertAllServosOff();  // the kill path does not depend on the failing write path
}

void test_controller_loss_sits_then_powers_down() {
  RobotConfig cfg = config();
  cfg.safety.controllerLostShutdownMs = 3000;  // shortened for the test
  auto robot = makeRobot(cfg);
  TEST_ASSERT_TRUE(robot->bootAndArm());
  robot->tap(Button::Cross);
  TEST_ASSERT_TRUE(robot->waitIdle());

  banner("Controller disconnected");
  robot->pad.disconnect();
  robot->tick();
  TEST_ASSERT_TRUE(robot->robot.hasFault(Fault::ControllerLost));
  TEST_ASSERT_TRUE(robot->waitIdle());
  TEST_ASSERT_EQUAL(Posture::Sitting, robot->robot.motion().posture());
  assertOutputsMatch(robot->robot.servos());

  TEST_ASSERT_TRUE(robot->runUntil([&] { return robot->robot.lifecycle() == Lifecycle::Shutdown; }, 6000));
  assertAllServosOff();
}

void test_control_tick_fits_the_servo_period() {
  auto robot = makeRobot();
  TEST_ASSERT_TRUE(robot->bootAndArm());
  robot->tap(Button::Cross);
  robot->pad.sticks(0, 1);  // walking = heaviest path (gait + 4x IK + I2C burst)
  uint32_t maxUs = 0, sumUs = 0;
  const int n = 250;
  for (int i = 0; i < n; ++i) {
    fakeNow() += robot->cfg.timing.controlPeriodMs;
    const uint32_t t0 = micros();
    robot->robot.tick(fakeNow());
    const uint32_t us = micros() - t0;
    maxUs = us > maxUs ? us : maxUs;
    sumUs += us;
    delay(robot->cfg.timing.controlPeriodMs);
  }
  robot->pad.sticks(0, 0);
  Serial.printf("  tick: avg %lu us, max %lu us (period %u ms)\n", static_cast<unsigned long>(sumUs / n),
                static_cast<unsigned long>(maxUs), robot->cfg.timing.controlPeriodMs);
  TEST_ASSERT_TRUE(maxUs < robot->cfg.timing.controlPeriodMs * 1000u / 4u);  // < 25% of the budget
}

void setup() {
  beginSuite("error handling");
  UNITY_BEGIN();
  RUN_TEST(test_shipped_config_passes_validation_on_target);
  RUN_TEST(test_pca9685_reset_is_detected_and_recoverable);
  RUN_TEST(test_i2c_write_failures_stop_the_robot);
  RUN_TEST(test_controller_loss_sits_then_powers_down);
  RUN_TEST(test_control_tick_fits_the_servo_period);
  UNITY_END();
}

void loop() {}
