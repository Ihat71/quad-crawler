#include "motion/Gait.h"

#include <cmath>

#include "common/MathUtil.h"

namespace qspider {

namespace {
// Fraction of the cycle before lift-off during which the body already shifts
// away from the leg that is about to swing.
constexpr float kSwayLead = 0.1f;
constexpr float kSwayTimeConstant = 0.12f;  // seconds

float swayWeight(float p, float duty) {
  const float start = duty - kSwayLead;
  if (p < start) return 0.0f;
  const float s = (p - start) / (1.0f - start);
  return 0.5f * (1.0f - std::cos(2.0f * kPi * s));
}
}  // namespace

void GaitEngine::reset() {
  phase_ = 0.0f;
  shift_ = Vec3{};
  for (bool& s : swing_) s = false;
}

void GaitEngine::update(float dt, const GaitConfig& gait, const WalkCommand& cmd,
                        const FeetPositions& neutral, FeetPositions& out) {
  phase_ += gait.maxCycleHz * clampf(cmd.speed, 0.0f, 1.0f) * dt;
  phase_ -= std::floor(phase_);

  // Foot displacement over one stance phase: translation plus rotation about the body centre.
  const Vec3 translation{cmd.vx * gait.maxStride, cmd.vy * gait.maxStride, 0.0f};
  const float theta = deg2rad(gait.maxTurnDeg) * clampf(cmd.turn, -1.0f, 1.0f);
  Vec3 stride[kLegCount];
  float scale = 1.0f;
  for (int i = 0; i < kLegCount; ++i) {
    Vec3 flat{neutral[i].x, neutral[i].y, 0.0f};
    stride[i] = translation + (rotateZ(flat, theta) - flat);
    const float len = stride[i].normXY();
    if (len > gait.maxStride && len > 0.0f) scale = std::fmin(scale, gait.maxStride / len);
  }

  Vec3 swayTarget;
  for (int i = 0; i < kLegCount; ++i) {
    const Vec3 d = stride[i] * scale;
    float p = phase_ + gait.phaseOffset[i];
    p -= std::floor(p);

    Vec3 foot = neutral[i];
    if (p < gait.dutyFactor) {
      const float u = p / gait.dutyFactor;  // 0 = touch-down, 1 = lift-off
      foot += d * (0.5f - u);
      swing_[i] = false;
    } else {
      const float w = (p - gait.dutyFactor) / (1.0f - gait.dutyFactor);
      foot += d * (smoothstep(w) - 0.5f);
      foot.z += gait.stepHeight * std::sin(kPi * w);
      swing_[i] = true;
    }
    out[i] = foot;

    if (gait.bodySwayGain > 0.0f) {
      const float wgt = swayWeight(p, gait.dutyFactor);
      swayTarget -= Vec3{neutral[i].x, neutral[i].y, 0.0f} * (gait.bodySwayGain * wgt);
    }
  }

  const float alpha = clampf(dt / (kSwayTimeConstant + dt), 0.0f, 1.0f);
  shift_ = lerp(shift_, swayTarget, alpha);
  for (int i = 0; i < kLegCount; ++i) out[i] -= shift_;
}

}  // namespace qspider
