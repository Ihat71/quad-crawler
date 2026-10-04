#pragma once

#include <cstdint>

#include "common/Types.h"
#include "kinematics/BodyKinematics.h"

namespace qspider {

// Body translation (mm) and rotation (deg) relative to the nominal standing body.
// roll > 0 lifts the left side, pitch > 0 lowers the nose, yaw > 0 turns left.
struct BodyPose {
  Vec3 offset;
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
};

// One animation pose: body pose plus per-foot offsets from the standing foot
// position (ground frame), reached `durationMs` after the previous keyframe.
struct Keyframe {
  BodyPose body;
  Vec3 foot[kLegCount];
  uint16_t durationMs = 0;
};

struct AnimationDef {
  const char* name;
  const Keyframe* frames;
  uint8_t frameCount;
};

// Foot positions in the body frame for a body pose applied to ground-fixed feet.
Vec3 applyBodyPose(const Vec3& footGround, const BodyPose& pose);

// Library of built-in animations (motion content, not configuration).
namespace animations {
int danceCount();
const AnimationDef& dance(int index);  // rock, up_down, twist, bounce, wave
const AnimationDef& attack();
const AnimationDef* findByName(const char* name);
}  // namespace animations

}  // namespace qspider
