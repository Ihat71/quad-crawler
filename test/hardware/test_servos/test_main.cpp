// HARDWARE TEST - each servo.
// Robot on a stand, legs free. Every servo is moved on its own around the
// calibration pose (90 deg + trim) and the PCA9685 register is read back.
//   pio test -e esp32dev -f hardware/test_servos

#include "../common/HwTestRig.h"

using namespace hw;

void setUp() { TEST_ASSERT_TRUE_MESSAGE(driver().begin(), "PCA9685 not responding - check wiring/address"); }
void tearDown() { driver().allOff(); }

void test_pca9685_present_and_configured() {
  TEST_ASSERT_TRUE(driver().present());
  TEST_ASSERT_TRUE(driver().verify());
  for (int ch = 0; ch < IServoDriver::kChannels; ++ch) assertChannelOff(ch);  // boots with everything off
}

void test_pulse_math_on_spare_channel() {
  // Channel 15 is not used by the robot, so the extremes can be checked safely.
  float frame[IServoDriver::kChannels];
  for (float& f : frame) f = NAN;
  for (float deg : {10.0f, 90.0f, 170.0f}) {
    frame[15] = deg;
    TEST_ASSERT_TRUE(driver().writeAll(frame));
    assertChannelAt(15, deg, 0.6f);
  }
}

void test_each_servo_individually() {
  EventLog log(&fakeClock);
  ServoBus bus(config(), driver(), log);
  const float centre = config().geometry.servoCentreDeg;
  const float sequence[] = {centre, centre - 20, centre + 20, centre};

  for (int s = 0; s < kServoCount; ++s) {
    Serial.printf("Servo %2d  %s %-5s  channel %2u : 90 -> 70 -> 110 -> 90\n", s, legName(legOfServo(s)),
                  jointName(jointOfServo(s)), bus.channel(s));
    bus.disableAll();
    bus.setEnabled(s, true);
    for (float target : sequence) {
      bus.setTarget(s, target);
      for (int i = 0; i < 40; ++i) {  // 0.8 s per step at the configured slew rate
        TEST_ASSERT_TRUE(bus.flush(0.02f));
        delay(20);
      }
      assertChannelAt(bus.channel(s), target);
      assertOutputsMatch(bus);  // and nothing else is powered
    }
  }
  bus.disableAll();
  bus.flush(0.02f);
}

void test_all_off_really_turns_everything_off() {
  float frame[IServoDriver::kChannels];
  for (float& f : frame) f = 90.0f;
  TEST_ASSERT_TRUE(driver().writeAll(frame));
  TEST_ASSERT_TRUE(driver().allOff());
  assertAllServosOff();
  TEST_ASSERT_TRUE(driver().begin());  // and the chip can be re-armed afterwards
}

void setup() {
  beginSuite("each servo");
  UNITY_BEGIN();
  RUN_TEST(test_pca9685_present_and_configured);
  RUN_TEST(test_pulse_math_on_spare_channel);
  RUN_TEST(test_each_servo_individually);
  RUN_TEST(test_all_off_really_turns_everything_off);
  UNITY_END();
}

void loop() {}
