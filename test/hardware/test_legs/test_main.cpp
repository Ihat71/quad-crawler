// HARDWARE TEST - each leg.
// Robot on a stand, legs free. Each leg alone traces UP -> OUT -> FORWARD ->
// back to neutral using inverse kinematics. Watch the leg: if a joint moves the
// wrong way, flip that joint's direction in config/robot_config.h.
//   pio test -e esp32dev -f hardware/test_legs

#include "../common/HwTestRig.h"

using namespace hw;

void setUp() { TEST_ASSERT_TRUE_MESSAGE(driver().begin(), "PCA9685 not responding"); }
void tearDown() { driver().allOff(); }

static void moveLegTo(ServoBus& bus, const BodyKinematics& kin, int leg, const Vec3& foot, const char* label) {
  float s[kJointsPerLeg];
  char msg[64];
  snprintf(msg, sizeof(msg), "%s %s unreachable", legName(leg), label);
  TEST_ASSERT_TRUE_MESSAGE(kin.footToServos(leg, foot, s), msg);
  for (int j = 0; j < kJointsPerLeg; ++j) bus.setTarget(servoIndex(leg, j), s[j]);
  for (int i = 0; i < 50; ++i) {
    TEST_ASSERT_TRUE(bus.flush(0.02f));
    delay(20);
  }
  Serial.printf("  %-8s coxa %6.1f  femur %6.1f  tibia %6.1f\n", label, bus.output(servoIndex(leg, 0)),
                bus.output(servoIndex(leg, 1)), bus.output(servoIndex(leg, 2)));
  TEST_ASSERT_EQUAL_UINT32(0, bus.clampEvents());  // the test path stays inside 10..170
  assertOutputsMatch(bus);
}

static void exerciseLeg(int leg) {
  const RobotConfig& cfg = config();
  BodyKinematics kin(cfg);
  EventLog log(&fakeClock);
  ServoBus bus(cfg, driver(), log);

  const Vec3 neutral = kin.neutralFoot(leg, cfg.posture.standReach, cfg.posture.standHeight);
  const Vec3 out = kin.legToBody(leg, Vec3{cfg.posture.standReach + 15.0f, 0, -cfg.posture.standHeight}) - neutral;

  Serial.printf("Leg %s: neutral -> up -> out -> forward -> neutral\n", legName(leg));
  bus.setLegEnabled(leg, true);
  moveLegTo(bus, kin, leg, neutral, "neutral");
  moveLegTo(bus, kin, leg, neutral + Vec3{0, 0, 20}, "up");
  moveLegTo(bus, kin, leg, neutral + Vec3{0, 0, 20} + out, "out");
  moveLegTo(bus, kin, leg, neutral + Vec3{15, 0, 20}, "forward");
  moveLegTo(bus, kin, leg, neutral, "neutral");
  bus.disableAll();
  bus.flush(0.02f);
}

void test_leg_front_right() { exerciseLeg(0); }
void test_leg_front_left() { exerciseLeg(1); }
void test_leg_rear_left() { exerciseLeg(2); }
void test_leg_rear_right() { exerciseLeg(3); }

void setup() {
  beginSuite("each leg");
  UNITY_BEGIN();
  RUN_TEST(test_leg_front_right);
  RUN_TEST(test_leg_front_left);
  RUN_TEST(test_leg_rear_left);
  RUN_TEST(test_leg_rear_right);
  UNITY_END();
}

void loop() {}
