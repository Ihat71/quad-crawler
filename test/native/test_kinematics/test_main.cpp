// Kinematics: IK/FK round trips, body transforms, servo mapping, reachability
// of every posture and animation with the real configuration.

#include <unity.h>

#include "quad_core.h"
#include "robot_config.h"

using namespace qspider;

static const RobotConfig kCfg = makeRobotConfig();

void setUp() {}
void tearDown() {}

static void assertVecNear(const Vec3& a, const Vec3& b, float tol) {
  TEST_ASSERT_FLOAT_WITHIN(tol, a.x, b.x);
  TEST_ASSERT_FLOAT_WITHIN(tol, a.y, b.y);
  TEST_ASSERT_FLOAT_WITHIN(tol, a.z, b.z);
}

void test_leg_ik_fk_round_trip() {
  LegKinematics leg(kCfg.geometry.coxaLength, kCfg.geometry.femurLength, kCfg.geometry.tibiaLength);
  const Vec3 targets[] = {{80, 0, -60}, {90, 20, -40}, {70, -25, -70}, {100, 0, -20}, {60, 10, -30}};
  for (const Vec3& t : targets) {
    JointAngles j;
    TEST_ASSERT_TRUE(leg.inverse(t, j));
    assertVecNear(t, leg.forward(j), 0.01f);
  }
}

void test_leg_ik_rejects_unreachable() {
  LegKinematics leg(27.5f, 55.0f, 77.5f);
  JointAngles j;
  TEST_ASSERT_FALSE(leg.inverse({300, 0, 0}, j));
  TEST_ASSERT_FALSE(leg.inverse({27.5f, 0, 0}, j));  // inside the folded radius
  TEST_ASSERT_FALSE(leg.inverse({NAN, 0, 0}, j));
}

void test_known_stand_angles() {
  // Foot 80 mm out, 60 mm down: femur ~18.6 deg up, knee ~71.7 deg.
  LegKinematics leg(27.5f, 55.0f, 77.5f);
  JointAngles j;
  TEST_ASSERT_TRUE(leg.inverse({80, 0, -60}, j));
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, j.coxa);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 18.6f, j.femur);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 71.7f, j.tibia);
}

void test_body_leg_transform_round_trip() {
  BodyKinematics kin(kCfg);
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Vec3 p{12, -7, -50};
    assertVecNear(p, kin.legToBody(leg, kin.bodyToLeg(leg, p)), 0.001f);
    // The neutral foot lies straight out along the mount direction.
    const Vec3 local = kin.bodyToLeg(leg, kin.neutralFoot(leg, 80, 60));
    assertVecNear(Vec3{80, 0, -60}, local, 0.01f);
  }
}

void test_servo_round_trip_all_legs() {
  BodyKinematics kin(kCfg);
  const FeetPositions stand = kin.neutralFeet(kCfg.posture.standReach, kCfg.posture.standHeight);
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Vec3 target = stand[leg] + Vec3{10, -5, 15};
    float s[kJointsPerLeg];
    TEST_ASSERT_TRUE(kin.footToServos(leg, target, s));
    assertVecNear(target, kin.servosToFoot(leg, s), 0.05f);
  }
}

void test_reference_pose_is_servo_centre() {
  // Femur horizontal + tibia perpendicular must map to 90 deg on every servo.
  BodyKinematics kin(kCfg);
  const float c = kCfg.geometry.servoCentreDeg;
  for (int leg = 0; leg < kLegCount; ++leg) {
    const float centre[kJointsPerLeg] = {c, c, c};
    const Vec3 foot = kin.servosToFoot(leg, centre);
    const Vec3 local = kin.bodyToLeg(leg, foot);
    assertVecNear(Vec3{kCfg.geometry.coxaLength + kCfg.geometry.femurLength, 0, -kCfg.geometry.tibiaLength}, local,
                  0.01f);
  }
}

void test_mirrored_legs_have_mirrored_servo_angles() {
  BodyKinematics kin(kCfg);
  const FeetPositions stand = kin.neutralFeet(kCfg.posture.standReach, kCfg.posture.standHeight);
  float fr[3], fl[3];
  TEST_ASSERT_TRUE(kin.footToServos(0, stand[0], fr));
  TEST_ASSERT_TRUE(kin.footToServos(1, stand[1], fl));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, fr[0], fl[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f - fr[1], fl[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f - fr[2], fl[2]);
}

void test_every_animation_keyframe_is_reachable() {
  BodyKinematics kin(kCfg);
  const AngleGuard guard(kCfg.servoLimits);
  const FeetPositions stand = kin.neutralFeet(kCfg.posture.standReach, kCfg.posture.standHeight);

  auto check = [&](const AnimationDef& a) {
    for (int f = 0; f < a.frameCount; ++f) {
      for (int leg = 0; leg < kLegCount; ++leg) {
        const Vec3 foot = applyBodyPose(stand[leg] + a.frames[f].foot[leg], a.frames[f].body);
        float s[3];
        char msg[64];
        snprintf(msg, sizeof(msg), "%s frame %d leg %s", a.name, f, legName(leg));
        TEST_ASSERT_TRUE_MESSAGE(kin.footToServos(leg, foot, s), msg);
        for (float v : s) {
          TEST_ASSERT_TRUE_MESSAGE(v >= guard.minDeg() && v <= guard.maxDeg(), msg);
        }
      }
    }
  };
  for (int d = 0; d < animations::danceCount(); ++d) check(animations::dance(d));
  check(animations::attack());
}

void test_body_pose_identity_and_translation() {
  const Vec3 foot{90, -90, -60};
  assertVecNear(foot, applyBodyPose(foot, BodyPose{}), 1e-4f);
  BodyPose up;
  up.offset = Vec3{0, 0, 10};
  assertVecNear(Vec3{90, -90, -70}, applyBodyPose(foot, up), 1e-4f);  // body up = feet lower
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_leg_ik_fk_round_trip);
  RUN_TEST(test_leg_ik_rejects_unreachable);
  RUN_TEST(test_known_stand_angles);
  RUN_TEST(test_body_leg_transform_round_trip);
  RUN_TEST(test_servo_round_trip_all_legs);
  RUN_TEST(test_reference_pose_is_servo_centre);
  RUN_TEST(test_mirrored_legs_have_mirrored_servo_angles);
  RUN_TEST(test_every_animation_keyframe_is_reachable);
  RUN_TEST(test_body_pose_identity_and_translation);
  return UNITY_END();
}
