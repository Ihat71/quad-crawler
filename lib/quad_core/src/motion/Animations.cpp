// Built-in poses, dances and the attack animation, expressed as keyframes
// relative to the standing pose. Add a new dance by adding a table here and
// listing it in kDances.

#include <cstring>

#include "common/MathUtil.h"
#include "motion/Animation.h"

namespace qspider {

Vec3 applyBodyPose(const Vec3& footGround, const BodyPose& pose) {
  Vec3 v = footGround - pose.offset;
  v = rotateZ(v, -deg2rad(pose.yawDeg));
  v = rotateY(v, -deg2rad(pose.pitchDeg));
  v = rotateX(v, -deg2rad(pose.rollDeg));
  return v;
}

namespace animations {
namespace {

constexpr int FR = 0, FL = 1, RL = 2, RR = 3;

Keyframe kf(uint16_t ms, float dx = 0, float dy = 0, float dz = 0, float roll = 0, float pitch = 0,
            float yaw = 0) {
  Keyframe k;
  k.body.offset = Vec3{dx, dy, dz};
  k.body.rollDeg = roll;
  k.body.pitchDeg = pitch;
  k.body.yawDeg = yaw;
  k.durationMs = ms;
  return k;
}

Keyframe foot(Keyframe k, int leg, float x, float y, float z) {
  k.foot[leg] = Vec3{x, y, z};
  return k;
}

// Left/right rocking.
const Keyframe kRock[] = {
    kf(400, 0, 10, 0, 10),  kf(600, 0, -10, 0, -10), kf(600, 0, 10, 0, 10),
    kf(600, 0, -10, 0, -10), kf(600, 0, 10, 0, 10),  kf(400),
};

// Body up and down.
const Keyframe kUpDown[] = {
    kf(400, 0, 0, 15), kf(600, 0, 0, -25), kf(600, 0, 0, 15), kf(600, 0, 0, -25), kf(400),
};

// Yaw twist with a little pitch.
const Keyframe kTwist[] = {
    kf(350, 0, 0, 0, 0, 5, 15),  kf(500, 0, 0, 0, 0, -5, -15), kf(500, 0, 0, 0, 0, 5, 15),
    kf(500, 0, 0, 0, 0, -5, -15), kf(350),
};

// Quick bouncing groove.
const Keyframe kBounce[] = {
    kf(250, 0, 0, 10, 6),  kf(250, 0, 0, -10, -6), kf(250, 0, 0, 10, -6),
    kf(250, 0, 0, -10, 6), kf(250, 0, 0, 10, 6),   kf(250, 0, 0, -10, -6),
    kf(300),
};

// Shift weight to the rear-left and wave the front-right foot.
const Keyframe kWave[] = {
    kf(400, -12, 12, 0),
    foot(kf(400, -12, 12, 0, 4), FR, 15, -15, 55),
    foot(kf(250, -12, 12, 0, 4), FR, 15, -35, 55),
    foot(kf(250, -12, 12, 0, 4), FR, 15, 5, 55),
    foot(kf(250, -12, 12, 0, 4), FR, 15, -35, 55),
    foot(kf(250, -12, 12, 0, 4), FR, 15, 5, 55),
    kf(400, -12, 12, 0),
    kf(300),
};

// Rear back, strike twice with each front leg (always on a stable tripod).
const Keyframe kAttack[] = {
    kf(300, -10, 0, -10, 0, -8),
    foot(kf(250, -15, 15, -8, 3, -6), FR, 10, -5, 50),
    foot(kf(120, -5, 15, -10, 3, 5), FR, 40, -10, 10),
    foot(kf(200, -15, 15, -8, 3, -6), FR, 10, -5, 50),
    foot(kf(120, -5, 15, -10, 3, 5), FR, 40, -10, 10),
    kf(300, -15, 0, -8),
    foot(kf(300, -15, -15, -8, -3, -6), FL, 10, 5, 50),
    foot(kf(120, -5, -15, -10, -3, 5), FL, 40, 10, 10),
    foot(kf(200, -15, -15, -8, -3, -6), FL, 10, 5, 50),
    foot(kf(120, -5, -15, -10, -3, 5), FL, 40, 10, 10),
    kf(350),
};

#define QUAD_ANIM(name, table) \
  AnimationDef { name, table, static_cast<uint8_t>(sizeof(table) / sizeof(table[0])) }

const AnimationDef kDances[] = {
    QUAD_ANIM("rock", kRock),     QUAD_ANIM("up_down", kUpDown), QUAD_ANIM("twist", kTwist),
    QUAD_ANIM("bounce", kBounce), QUAD_ANIM("wave", kWave),
};
const AnimationDef kAttackDef = QUAD_ANIM("attack", kAttack);

#undef QUAD_ANIM

// Unused-variable guard for leg constants not referenced above.
static_assert(RL == 2 && RR == 3, "leg order");

}  // namespace

int danceCount() { return static_cast<int>(sizeof(kDances) / sizeof(kDances[0])); }

const AnimationDef& dance(int index) {
  const int n = danceCount();
  index = ((index % n) + n) % n;
  return kDances[index];
}

const AnimationDef& attack() { return kAttackDef; }

const AnimationDef* findByName(const char* name) {
  if (!name) return nullptr;
  for (int i = 0; i < danceCount(); ++i)
    if (std::strcmp(kDances[i].name, name) == 0) return &kDances[i];
  if (std::strcmp(kAttackDef.name, name) == 0) return &kAttackDef;
  return nullptr;
}

}  // namespace animations
}  // namespace qspider
