// HARDWARE TEST - standing and sitting.
// Robot on the floor. The full robot stack runs on the real servos with a
// scripted controller: arm -> stand (X) -> sit (X), verifying every output.
//   pio test -e esp32dev -f hardware/test_poses

#include "../common/HwTestRig.h"

using namespace hw;

static std::unique_ptr<RobotHarness> robot;

void setUp() {
  robot = makeRobot();
  TEST_ASSERT_TRUE_MESSAGE(robot->bootAndArm(), "robot did not arm");
}
void tearDown() {
  robot.reset();
  driver().allOff();
}

static void expectPosture(Posture p, const char* name) {
  const uint32_t start = millis();
  TEST_ASSERT_TRUE(robot->waitIdle(4000));
  Serial.printf("  %s reached in %lu ms\n", name, static_cast<unsigned long>(millis() - start));
  TEST_ASSERT_EQUAL(p, robot->robot.motion().posture());
  assertOutputsMatch(robot->robot.servos());
  TEST_ASSERT_EQUAL_UINT32(0, robot->robot.activeFaults() & kCriticalFaultMask);
}

void test_arms_into_sit_pose() {
  banner("Armed: robot should be sitting");
  expectPosture(Posture::Sitting, "sit");
  TEST_ASSERT_EQUAL(Mode::Normal, robot->robot.mode());
}

void test_stand_and_sit_with_cross_button() {
  banner("X -> stand");
  robot->tap(Button::Cross);
  expectPosture(Posture::Standing, "stand");
  robot->runMs(1500);
  banner("X -> sit");
  robot->tap(Button::Cross);
  expectPosture(Posture::Sitting, "sit");
}

void test_up_and_down_and_rocking() {
  robot->tap(Button::Cross);
  expectPosture(Posture::Standing, "stand");
  for (const char* name : {"up_down", "rock"}) {
    Serial.printf("  dance %s\n", name);
    Command c;
    c.type = CommandType::Dance;
    c.source = CommandSource::Test;
    for (int i = 0; i < animations::danceCount(); ++i)
      if (strcmp(animations::dance(i).name, name) == 0) c.i = i;
    robot->robot.submit(c);
    robot->tick();
    TEST_ASSERT_NOT_NULL(robot->robot.motion().animation());
    expectPosture(Posture::Standing, name);
  }
  robot->tap(Button::Cross);
  expectPosture(Posture::Sitting, "sit");
}

void setup() {
  beginSuite("standing and sitting");
  UNITY_BEGIN();
  RUN_TEST(test_arms_into_sit_pose);
  RUN_TEST(test_stand_and_sit_with_cross_button);
  RUN_TEST(test_up_and_down_and_rocking);
  UNITY_END();
}

void loop() {}
