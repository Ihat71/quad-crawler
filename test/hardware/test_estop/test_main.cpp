// HARDWARE TEST - emergency stop.
// Robot on the floor (it drops when the servos go limp). Verifies through PCA9685
// readback that D-pad left / dashboard E-stop really switch every servo off,
// that nothing can move it while stopped, and that only a 2 s PS hold recovers.
//   pio test -e esp32dev -f hardware/test_estop

#include "../common/HwTestRig.h"

using namespace hw;

static std::unique_ptr<RobotHarness> robot;

void setUp() {
  robot = makeRobot();
  TEST_ASSERT_TRUE_MESSAGE(robot->bootAndArm(), "robot did not arm");
  robot->tap(Button::Cross);
  TEST_ASSERT_TRUE(robot->waitIdle());
  TEST_ASSERT_EQUAL(Posture::Standing, robot->robot.motion().posture());
}
void tearDown() {
  robot.reset();
  driver().allOff();
}

void test_dpad_left_cuts_all_outputs_within_one_tick() {
  banner("D-pad left while standing");
  robot->pad.press(Button::Left);
  fakeNow() += robot->cfg.timing.controlPeriodMs;
  const uint32_t t0 = micros();
  robot->robot.tick(fakeNow());
  const uint32_t latencyUs = micros() - t0;
  robot->pad.release(Button::Left);
  Serial.printf("  emergency stop handled in %lu us (one control tick)\n", static_cast<unsigned long>(latencyUs));
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, robot->robot.mode());
  assertAllServosOff();
  TEST_ASSERT_TRUE(latencyUs < 20000);
}

void test_nothing_moves_while_stopped() {
  robot->tap(Button::Left);
  robot->tap(Button::Cross);
  robot->tap(Button::Up);
  robot->tap(Button::Circle);
  robot->pad.sticks(0, 1, 1, 0);
  robot->runMs(1500);
  robot->pad.sticks(0, 0, 0, 0);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, robot->robot.mode());
  assertAllServosOff();
}

void test_short_ps_press_does_not_clear_but_2s_hold_does() {
  robot->tap(Button::Left);
  robot->hold(Button::PlayStation, 1000);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, robot->robot.mode());
  assertAllServosOff();

  banner("Holding PS for 2 s - robot re-arms into the sit pose");
  robot->hold(Button::PlayStation, 2100);
  TEST_ASSERT_TRUE(robot->runUntil([] { return robot->robot.lifecycle() == Lifecycle::Active; }, 4000));
  TEST_ASSERT_EQUAL(Mode::Normal, robot->robot.mode());
  TEST_ASSERT_TRUE(robot->waitIdle());
  assertOutputsMatch(robot->robot.servos());
}

void test_dashboard_estop() {
  robot->submit(CommandType::EmergencyStop);
  TEST_ASSERT_EQUAL(Mode::EmergencyStop, robot->robot.mode());
  assertAllServosOff();
}

void setup() {
  beginSuite("emergency stop");
  UNITY_BEGIN();
  RUN_TEST(test_dpad_left_cuts_all_outputs_within_one_tick);
  RUN_TEST(test_nothing_moves_while_stopped);
  RUN_TEST(test_short_ps_press_does_not_clear_but_2s_hold_does);
  RUN_TEST(test_dashboard_estop);
  UNITY_END();
}

void loop() {}
