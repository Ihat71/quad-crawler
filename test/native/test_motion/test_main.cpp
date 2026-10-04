// Gait engine and motion controller: walking, stopping, sit/stand, dances, IK leg.

#include <unity.h>

#include <cmath>

#include "quad_core.h"
#include "robot_config.h"

using namespace qspider;

static const RobotConfig kCfg = makeRobotConfig();

void setUp() {}
void tearDown() {}

static float maxStep(const FeetPositions& a, const FeetPositions& b) {
  float m = 0;
  for (int i = 0; i < kLegCount; ++i) m = std::fmax(m, (a[i] - b[i]).norm());
  return m;
}

void test_creep_gait_has_one_leg_in_swing() {
  BodyKinematics kin(kCfg);
  GaitEngine g;
  const FeetPositions neutral = kin.neutralFeet(kCfg.posture.standReach, kCfg.posture.standHeight);
  WalkCommand cmd;
  cmd.vx = 1;
  cmd.speed = 0.9f;
  FeetPositions out;
  for (int i = 0; i < 200; ++i) {
    g.update(0.02f, kCfg.motion.gaits[0], cmd, neutral, out);
    int swinging = 0;
    for (int l = 0; l < kLegCount; ++l) swinging += g.inSwing(l) ? 1 : 0;
    TEST_ASSERT_EQUAL_INT(1, swinging);
  }
}

void test_trot_gait_moves_diagonal_pairs() {
  BodyKinematics kin(kCfg);
  GaitEngine g;
  const FeetPositions neutral = kin.neutralFeet(kCfg.posture.standReach, kCfg.posture.standHeight);
  WalkCommand cmd;
  cmd.vx = 1;
  cmd.speed = 0.9f;
  FeetPositions out;
  for (int i = 0; i < 100; ++i) {
    g.update(0.02f, kCfg.motion.gaits[1], cmd, neutral, out);
    TEST_ASSERT_EQUAL(g.inSwing(0), g.inSwing(2));  // FR + RL
    TEST_ASSERT_EQUAL(g.inSwing(1), g.inSwing(3));  // FL + RR
    TEST_ASSERT_NOT_EQUAL(g.inSwing(0), g.inSwing(1));
  }
}

void test_stance_feet_move_backwards_when_walking_forward() {
  BodyKinematics kin(kCfg);
  GaitEngine g;
  const FeetPositions neutral = kin.neutralFeet(kCfg.posture.standReach, kCfg.posture.standHeight);
  WalkCommand cmd;
  cmd.vx = 1;
  cmd.speed = 0.5f;
  FeetPositions prev, out;
  bool prevSwing[kLegCount];
  g.update(0.02f, kCfg.motion.gaits[1], cmd, neutral, prev);
  for (int l = 0; l < kLegCount; ++l) prevSwing[l] = g.inSwing(l);
  for (int i = 0; i < 60; ++i) {
    g.update(0.02f, kCfg.motion.gaits[1], cmd, neutral, out);
    for (int l = 0; l < kLegCount; ++l) {
      if (!g.inSwing(l)) {
        if (!prevSwing[l]) TEST_ASSERT_TRUE(out[l].x < prev[l].x);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, neutral[l].z, out[l].z);  // stance foot on the ground
      }
      prevSwing[l] = g.inSwing(l);
    }
    prev = out;
  }
}

void test_zero_speed_freezes_gait() {
  BodyKinematics kin(kCfg);
  GaitEngine g;
  const FeetPositions neutral = kin.neutralFeet(kCfg.posture.standReach, kCfg.posture.standHeight);
  WalkCommand cmd;
  FeetPositions out;
  g.update(0.02f, kCfg.motion.gaits[0], cmd, neutral, out);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g.phase());
}

void test_sit_stand_transitions() {
  BodyKinematics kin(kCfg);
  MotionController m(kCfg, kin);
  m.resetTo(Posture::Sitting);
  m.stand();
  TEST_ASSERT_EQUAL(MotionState::Transition, m.state());
  for (int i = 0; i < 100 && m.busy(); ++i) m.update(0.02f);
  TEST_ASSERT_EQUAL(Posture::Standing, m.posture());
  TEST_ASSERT_TRUE(maxStep(m.feet(), m.standFeet()) < 0.01f);
  m.toggleSitStand();
  for (int i = 0; i < 100 && m.busy(); ++i) m.update(0.02f);
  TEST_ASSERT_EQUAL(Posture::Sitting, m.posture());
}

void test_walk_from_sitting_auto_stands_then_walks_then_settles() {
  BodyKinematics kin(kCfg);
  MotionController m(kCfg, kin);
  m.resetTo(Posture::Sitting);
  WalkCommand w;
  w.vx = 1;
  w.speed = 0.9f;
  FeetPositions prev = m.feet();
  bool walked = false;
  for (int i = 0; i < 200; ++i) {
    m.setWalk(w);
    m.update(0.02f);
    walked |= m.state() == MotionState::Walking;
    TEST_ASSERT_TRUE_MESSAGE(maxStep(prev, m.feet()) < 15.0f, "foot jump while starting to walk");
    prev = m.feet();
  }
  TEST_ASSERT_TRUE(walked);

  for (int i = 0; i < 200 && m.busy(); ++i) {
    m.setWalk(WalkCommand{});
    m.update(0.02f);
    TEST_ASSERT_TRUE_MESSAGE(maxStep(prev, m.feet()) < 15.0f, "foot jump while stopping");
    prev = m.feet();
  }
  TEST_ASSERT_EQUAL(MotionState::Idle, m.state());
  TEST_ASSERT_EQUAL(Posture::Standing, m.posture());
  TEST_ASSERT_TRUE(maxStep(m.feet(), m.standFeet()) < 0.01f);
}

void test_every_dance_plays_and_returns_to_stand() {
  BodyKinematics kin(kCfg);
  MotionController m(kCfg, kin);
  for (int d = 0; d < animations::danceCount(); ++d) {
    m.resetTo(Posture::Standing);
    m.playAnimation(animations::dance(d), false);
    TEST_ASSERT_EQUAL(MotionState::Animating, m.state());
    FeetPositions prev = m.feet();
    int ticks = 0;
    while (m.busy() && ticks < 1000) {
      m.update(0.02f);
      float s[3];
      for (int l = 0; l < kLegCount; ++l) TEST_ASSERT_TRUE(kin.footToServos(l, m.feet()[l], s));
      TEST_ASSERT_TRUE(maxStep(prev, m.feet()) < 12.0f);
      prev = m.feet();
      ++ticks;
    }
    TEST_ASSERT_FALSE(m.busy());
    TEST_ASSERT_TRUE(maxStep(m.feet(), m.standFeet()) < 0.5f);
  }
}

void test_looping_dance_runs_until_stopped() {
  BodyKinematics kin(kCfg);
  MotionController m(kCfg, kin);
  m.resetTo(Posture::Standing);
  m.playAnimation(animations::dance(0), true);
  for (int i = 0; i < 1000; ++i) m.update(0.02f);  // 20 s
  TEST_ASSERT_EQUAL(MotionState::Animating, m.state());
  m.stopAll();
  for (int i = 0; i < 100 && m.busy(); ++i) m.update(0.02f);
  TEST_ASSERT_EQUAL(Posture::Standing, m.posture());
}

void test_dance_from_sitting_stands_first() {
  BodyKinematics kin(kCfg);
  MotionController m(kCfg, kin);
  m.resetTo(Posture::Sitting);
  m.playAnimation(animations::attack(), false);
  TEST_ASSERT_EQUAL(MotionState::Transition, m.state());
  bool animated = false;
  for (int i = 0; i < 500 && m.busy(); ++i) {
    m.update(0.02f);
    animated |= m.state() == MotionState::Animating;
  }
  TEST_ASSERT_TRUE(animated);
}

void test_ik_leg_moves_only_selected_leg() {
  BodyKinematics kin(kCfg);
  MotionController m(kCfg, kin);
  m.resetTo(Posture::Standing);
  m.enterIk(kCfg.motion.ik.leg);
  for (int i = 0; i < 200 && m.state() != MotionState::IkControl; ++i) m.update(0.02f);
  TEST_ASSERT_EQUAL(MotionState::IkControl, m.state());

  const FeetPositions before = m.feet();
  for (int i = 0; i < 25; ++i) {
    TEST_ASSERT_TRUE(m.moveIk(Vec3{40, 0, 0}, 0.02f));
    m.update(0.02f);
  }
  const FeetPositions after = m.feet();
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, after[0].x - before[0].x);
  for (int l = 1; l < kLegCount; ++l) TEST_ASSERT_TRUE((after[l] - before[l]).norm() < 1e-3f);

  // Workspace limits are enforced.
  for (int i = 0; i < 500; ++i) m.moveIk(Vec3{0, 0, 200}, 0.02f);
  TEST_ASSERT_TRUE(m.ikOffset().z <= kCfg.motion.ik.rangeMax.z + 1e-3f);

  m.exitIk();
  for (int i = 0; i < 300 && m.busy(); ++i) m.update(0.02f);
  TEST_ASSERT_FALSE(m.ikActive());
  TEST_ASSERT_EQUAL(Posture::Standing, m.posture());
}

void test_ik_body_shifts_away_from_lifted_leg() {
  BodyKinematics kin(kCfg);
  MotionController m(kCfg, kin);
  m.resetTo(Posture::Standing);
  const FeetPositions stand = m.standFeet();
  m.enterIk(0);  // FR
  for (int i = 0; i < 200 && m.state() != MotionState::IkControl; ++i) m.update(0.02f);
  // Feet move toward FR in the body frame => body moved toward rear-left.
  TEST_ASSERT_TRUE(m.feet()[2].x > stand[2].x);
  TEST_ASSERT_TRUE(m.feet()[2].y < stand[2].y);
  TEST_ASSERT_TRUE(m.feet()[0].z > stand[0].z + 10.0f);  // FR foot lifted
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_creep_gait_has_one_leg_in_swing);
  RUN_TEST(test_trot_gait_moves_diagonal_pairs);
  RUN_TEST(test_stance_feet_move_backwards_when_walking_forward);
  RUN_TEST(test_zero_speed_freezes_gait);
  RUN_TEST(test_sit_stand_transitions);
  RUN_TEST(test_walk_from_sitting_auto_stands_then_walks_then_settles);
  RUN_TEST(test_every_dance_plays_and_returns_to_stand);
  RUN_TEST(test_looping_dance_runs_until_stopped);
  RUN_TEST(test_dance_from_sitting_stands_first);
  RUN_TEST(test_ik_leg_moves_only_selected_leg);
  RUN_TEST(test_ik_body_shifts_away_from_lifted_leg);
  return UNITY_END();
}
