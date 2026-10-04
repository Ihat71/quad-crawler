#include "motion/MotionController.h"

#include <cmath>

#include "common/MathUtil.h"
#include "servo/AngleGuard.h"

namespace qspider {

const char* toString(MotionState s) {
  switch (s) {
    case MotionState::Idle: return "IDLE";
    case MotionState::Transition: return "TRANSITION";
    case MotionState::Walking: return "WALKING";
    case MotionState::Animating: return "ANIMATING";
    case MotionState::IkControl: return "IK_CONTROL";
  }
  return "?";
}

namespace {
constexpr uint16_t kIkStepMs = 300;
constexpr float kSettledEpsilon = 0.02f;

BodyPose lerpPose(const BodyPose& a, const BodyPose& b, float t) {
  BodyPose p;
  p.offset = lerp(a.offset, b.offset, t);
  p.rollDeg = lerpf(a.rollDeg, b.rollDeg, t);
  p.pitchDeg = lerpf(a.pitchDeg, b.pitchDeg, t);
  p.yawDeg = lerpf(a.yawDeg, b.yawDeg, t);
  return p;
}

float maxDistance(const FeetPositions& a, const FeetPositions& b) {
  float m = 0.0f;
  for (int i = 0; i < kLegCount; ++i) m = std::fmax(m, (a[i] - b[i]).norm());
  return m;
}
}  // namespace

MotionController::MotionController(const RobotConfig& cfg, const BodyKinematics& kin)
    : cfg_(cfg), kin_(kin) {
  gaitIndex_ = cfg.motion.defaultGait;
  if (gaitIndex_ < 0 || gaitIndex_ >= cfg.motion.gaitCount) gaitIndex_ = 0;
  feet_ = sitFeet();
  from_ = to_ = feet_;
}

FeetPositions MotionController::standFeet() const {
  return kin_.neutralFeet(cfg_.posture.standReach, cfg_.posture.standHeight);
}

FeetPositions MotionController::sitFeet() const {
  return kin_.neutralFeet(cfg_.posture.sitReach, cfg_.posture.sitHeight);
}

void MotionController::resetTo(Posture p) {
  state_ = MotionState::Idle;
  posture_ = p == Posture::Standing ? Posture::Standing : Posture::Sitting;
  feet_ = posture_ == Posture::Standing ? standFeet() : sitFeet();
  walkTarget_ = walk_ = WalkCommand{};
  anim_ = pendingAnim_ = nullptr;
  ikLeg_ = -1;
  after_ = After::None;
}

void MotionController::syncFeet(const FeetPositions& feet) {
  resetTo(Posture::Sitting);
  feet_ = feet;
  posture_ = Posture::Custom;
}

void MotionController::sit() {
  walkTarget_ = walk_ = WalkCommand{};
  anim_ = pendingAnim_ = nullptr;
  ikLeg_ = -1;
  if (posture_ == Posture::Sitting && state_ == MotionState::Idle) return;
  startTransition(sitFeet(), cfg_.posture.transitionMs, Posture::Sitting, After::None);
}

void MotionController::stand() {
  if (posture_ == Posture::Standing && state_ == MotionState::Idle) return;
  if (state_ == MotionState::Transition && transitionResult_ == Posture::Standing && after_ == After::None) return;
  walkTarget_ = walk_ = WalkCommand{};
  anim_ = pendingAnim_ = nullptr;
  ikLeg_ = -1;
  startTransition(standFeet(), cfg_.posture.transitionMs, Posture::Standing, After::None);
}

void MotionController::toggleSitStand() {
  const Posture target = state_ == MotionState::Transition ? transitionResult_ : posture_;
  if (target == Posture::Sitting)
    stand();
  else
    sit();
}

void MotionController::setWalk(const WalkCommand& cmd) {
  walkTarget_ = cmd;
  walkTarget_.speed = clampf(cmd.speed, 0.0f, 1.0f);
  if (walkTarget_.isZero()) return;

  switch (state_) {
    case MotionState::Idle:
      if (posture_ == Posture::Standing) {
        state_ = MotionState::Walking;
        gaitEngine_.reset();
        walk_ = WalkCommand{};
      } else {
        // Auto-stand before walking.
        startTransition(standFeet(), cfg_.posture.transitionMs, Posture::Standing, After::Walk);
      }
      break;
    case MotionState::Transition:
      if (transitionResult_ == Posture::Standing && after_ == After::None) after_ = After::Walk;
      break;
    case MotionState::Walking:
      break;
    case MotionState::Animating:
    case MotionState::IkControl:
      walkTarget_ = WalkCommand{};  // sticks are ignored while dancing / in IK
      break;
  }
}

void MotionController::selectGait(int index) {
  if (cfg_.motion.gaitCount <= 0) return;
  index %= cfg_.motion.gaitCount;
  if (index < 0) index += cfg_.motion.gaitCount;
  gaitIndex_ = index;
}

void MotionController::playAnimation(const AnimationDef& anim, bool loop) {
  if (ikLeg_ >= 0) return;  // never dance on three legs
  pendingAnim_ = &anim;
  pendingLoop_ = loop;
  walkTarget_ = WalkCommand{};
  if (state_ == MotionState::Idle && posture_ == Posture::Standing) {
    startAnimation();
    return;
  }
  const uint16_t ms = state_ == MotionState::Walking     ? cfg_.motion.stopSettleMs
                      : state_ == MotionState::Animating ? cfg_.motion.animationReturnMs
                                                         : cfg_.posture.transitionMs;
  startTransition(standFeet(), ms, Posture::Standing, After::Animate);
}

void MotionController::stopAll() {
  walkTarget_ = WalkCommand{};
  pendingAnim_ = nullptr;
  switch (state_) {
    case MotionState::Walking:
      startTransition(standFeet(), cfg_.motion.stopSettleMs, Posture::Standing, After::None);
      break;
    case MotionState::Animating:
      startTransition(standFeet(), cfg_.motion.animationReturnMs, Posture::Standing, After::None);
      break;
    case MotionState::IkControl:
      exitIk();
      break;
    case MotionState::Transition:
      if (ikLeg_ >= 0) {
        ikLeg_ = -1;
        startTransition(standFeet(), cfg_.posture.transitionMs, Posture::Standing, After::None);
      } else {
        after_ = After::None;
      }
      break;
    case MotionState::Idle:
      if (posture_ == Posture::Custom) stand();
      break;
  }
}

FeetPositions MotionController::ikBaseFeet() const {
  FeetPositions f = standFeet();
  const Vec3 n = f[ikLeg_];
  const float len = n.normXY();
  const Vec3 shift = len > 0.0f ? Vec3{n.x / len, n.y / len, 0.0f} * cfg_.motion.ik.bodyShift : Vec3{};
  for (Vec3& p : f) p += shift;  // feet move toward the IK leg = body moves away from it
  return f;
}

void MotionController::enterIk(int leg) {
  if (ikLeg_ >= 0 || leg < 0 || leg >= kLegCount) return;
  walkTarget_ = walk_ = WalkCommand{};
  anim_ = pendingAnim_ = nullptr;
  ikLeg_ = leg;
  ikOffset_ = Vec3{};
  const FeetPositions base = ikBaseFeet();
  ikNeutral_ = base[leg] + Vec3{0.0f, 0.0f, cfg_.motion.ik.startLift};
  // Shift the body first (all feet on the ground), then lift the foot.
  startTransition(base, cfg_.posture.transitionMs, Posture::Custom, After::IkLift);
}

void MotionController::exitIk() {
  if (ikLeg_ < 0) return;
  if (state_ == MotionState::IkControl) {
    startTransition(ikBaseFeet(), kIkStepMs, Posture::Custom, After::IkUnshift);
  } else {
    ikLeg_ = -1;
    startTransition(standFeet(), cfg_.posture.transitionMs, Posture::Standing, After::None);
  }
}

bool MotionController::moveIk(const Vec3& velocity, float dt) {
  if (state_ != MotionState::IkControl) return false;
  const IkModeConfig& ik = cfg_.motion.ik;
  Vec3 c = ikOffset_ + velocity * dt;
  c.x = clampf(c.x, ik.rangeMin.x, ik.rangeMax.x);
  c.y = clampf(c.y, ik.rangeMin.y, ik.rangeMax.y);
  c.z = clampf(c.z, ik.rangeMin.z, ik.rangeMax.z);

  float servo[kJointsPerLeg];
  bool ok = kin_.footToServos(ikLeg_, ikNeutral_ + c, servo);
  if (ok) {
    const AngleGuard guard(cfg_.servoLimits);
    for (float s : servo) ok = ok && s >= guard.minDeg() && s <= guard.maxDeg();
  }
  if (!ok) {
    ++ikRejections_;
    return false;
  }
  ikOffset_ = c;
  return true;
}

void MotionController::resetIk() {
  if (state_ != MotionState::IkControl) return;
  ikOffset_ = Vec3{};
  FeetPositions to = ikBaseFeet();
  to[ikLeg_] = ikNeutral_;
  startTransition(to, kIkStepMs, Posture::Custom, After::IkControl);
}

void MotionController::startTransition(const FeetPositions& to, uint16_t ms, Posture result, After after) {
  if (state_ == MotionState::Animating) anim_ = nullptr;
  from_ = feet_;
  to_ = to;
  transitionMs_ = ms > 0 ? static_cast<float>(ms) : 1.0f;
  transitionElapsedMs_ = 0.0f;
  transitionResult_ = result;
  after_ = after;
  state_ = MotionState::Transition;
}

void MotionController::finishTransition() {
  feet_ = to_;
  posture_ = transitionResult_;
  state_ = MotionState::Idle;
  const After after = after_;
  after_ = After::None;

  switch (after) {
    case After::None:
      break;
    case After::Walk:
      if (!walkTarget_.isZero()) {
        state_ = MotionState::Walking;
        gaitEngine_.reset();
        walk_ = WalkCommand{};
      }
      break;
    case After::Animate:
      startAnimation();
      break;
    case After::IkLift: {
      FeetPositions to = ikBaseFeet();
      to[ikLeg_] = ikNeutral_;
      startTransition(to, kIkStepMs, Posture::Custom, After::IkControl);
      break;
    }
    case After::IkControl:
      state_ = MotionState::IkControl;
      break;
    case After::IkUnshift:
      ikLeg_ = -1;
      startTransition(standFeet(), cfg_.posture.transitionMs, Posture::Standing, After::None);
      break;
  }
}

void MotionController::startAnimation() {
  anim_ = pendingAnim_;
  animLoop_ = pendingLoop_;
  pendingAnim_ = nullptr;
  if (!anim_ || anim_->frameCount == 0) {
    anim_ = nullptr;
    return;
  }
  frame_ = 0;
  frameElapsedMs_ = 0.0f;
  poseFrom_ = BodyPose{};
  for (Vec3& o : offsetsFrom_) o = Vec3{};
  posture_ = Posture::Standing;
  state_ = MotionState::Animating;
}

FeetPositions MotionController::animationFeet(const BodyPose& pose, const Vec3 (&offsets)[kLegCount]) const {
  const FeetPositions stand = standFeet();
  FeetPositions f;
  for (int i = 0; i < kLegCount; ++i) f[i] = applyBodyPose(stand[i] + offsets[i], pose);
  return f;
}

void MotionController::updateAnimation(float dt) {
  frameElapsedMs_ += dt * 1000.0f;
  for (;;) {
    const Keyframe& k = anim_->frames[frame_];
    const float dur = k.durationMs > 0 ? static_cast<float>(k.durationMs) : 1.0f;
    if (frameElapsedMs_ < dur) break;
    frameElapsedMs_ -= dur;
    poseFrom_ = k.body;
    for (int i = 0; i < kLegCount; ++i) offsetsFrom_[i] = k.foot[i];
    if (++frame_ >= anim_->frameCount) {
      if (animLoop_) {
        frame_ = 0;
        continue;
      }
      feet_ = animationFeet(poseFrom_, offsetsFrom_);
      anim_ = nullptr;
      state_ = MotionState::Idle;
      if (maxDistance(feet_, standFeet()) > 0.5f)
        startTransition(standFeet(), cfg_.motion.animationReturnMs, Posture::Standing, After::None);
      return;
    }
  }

  const Keyframe& k = anim_->frames[frame_];
  const float dur = k.durationMs > 0 ? static_cast<float>(k.durationMs) : 1.0f;
  const float t = smoothstep(frameElapsedMs_ / dur);
  Vec3 offsets[kLegCount];
  for (int i = 0; i < kLegCount; ++i) offsets[i] = lerp(offsetsFrom_[i], k.foot[i], t);
  feet_ = animationFeet(lerpPose(poseFrom_, k.body, t), offsets);
}

void MotionController::updateWalking(float dt) {
  const float step = cfg_.motion.commandRampPerSec * dt;
  walk_.vx = approach(walk_.vx, walkTarget_.vx, step);
  walk_.vy = approach(walk_.vy, walkTarget_.vy, step);
  walk_.turn = approach(walk_.turn, walkTarget_.turn, step);
  walk_.speed = approach(walk_.speed, walkTarget_.speed, step);

  if (walkTarget_.isZero() && std::fabs(walk_.vx) < kSettledEpsilon && std::fabs(walk_.vy) < kSettledEpsilon &&
      std::fabs(walk_.turn) < kSettledEpsilon && walk_.speed < kSettledEpsilon) {
    walk_ = WalkCommand{};
    startTransition(standFeet(), cfg_.motion.stopSettleMs, Posture::Standing, After::None);
    return;
  }
  gaitEngine_.update(dt, gait(), walk_, standFeet(), feet_);
}

void MotionController::update(float dt) {
  switch (state_) {
    case MotionState::Idle:
      break;
    case MotionState::Transition:
      transitionElapsedMs_ += dt * 1000.0f;
      if (transitionElapsedMs_ >= transitionMs_) {
        finishTransition();
      } else {
        const float t = smoothstep(transitionElapsedMs_ / transitionMs_);
        for (int i = 0; i < kLegCount; ++i) feet_[i] = lerp(from_[i], to_[i], t);
      }
      break;
    case MotionState::Walking:
      updateWalking(dt);
      break;
    case MotionState::Animating:
      updateAnimation(dt);
      break;
    case MotionState::IkControl: {
      FeetPositions f = ikBaseFeet();
      f[ikLeg_] = ikNeutral_ + ikOffset_;
      feet_ = f;
      break;
    }
  }
}

}  // namespace qspider
