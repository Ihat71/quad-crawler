#pragma once

#include "common/Types.h"
#include "config/RobotConfig.h"
#include "kinematics/BodyKinematics.h"
#include "motion/Animation.h"
#include "motion/Gait.h"

namespace qspider {

enum class MotionState : uint8_t { Idle, Transition, Walking, Animating, IkControl };
const char* toString(MotionState s);

// Produces body-frame foot targets for every leg. Owns postures, walking,
// animations and single-leg IK; knows nothing about servos or hardware.
class MotionController {
 public:
  MotionController(const RobotConfig& cfg, const BodyKinematics& kin);

  // Instantly sets the feet (only used while outputs are disabled).
  void resetTo(Posture p);
  // Adopts externally driven feet (e.g. after calibration moved the servos directly).
  void syncFeet(const FeetPositions& feet);

  void sit();
  void stand();
  void toggleSitStand();

  // Continuous walking input; a zero command stops and settles the feet.
  void setWalk(const WalkCommand& cmd);
  void selectGait(int index);
  int gaitIndex() const { return gaitIndex_; }
  const GaitConfig& gait() const { return cfg_.motion.gaits[gaitIndex_]; }

  void playAnimation(const AnimationDef& anim, bool loop);
  // Stops walking/animation/IK and returns to the standing pose.
  void stopAll();

  void enterIk(int leg);
  void exitIk();
  // Moves the IK foot with a velocity (mm/s, body frame). Returns false if the
  // requested point is outside the workspace or unreachable.
  bool moveIk(const Vec3& velocity, float dt);
  void resetIk();

  void update(float dt);

  const FeetPositions& feet() const { return feet_; }
  MotionState state() const { return state_; }
  Posture posture() const { return posture_; }
  // Posture the robot is in or currently moving into.
  Posture targetPosture() const { return state_ == MotionState::Transition ? transitionResult_ : posture_; }
  bool busy() const { return state_ != MotionState::Idle; }
  const AnimationDef* animation() const { return state_ == MotionState::Animating ? anim_ : nullptr; }
  const AnimationDef* pendingAnimation() const { return pendingAnim_; }
  bool animationLooping() const { return animLoop_; }
  const WalkCommand& walkCommand() const { return walk_; }
  bool ikActive() const { return ikLeg_ >= 0; }
  int ikLeg() const { return ikLeg_; }
  Vec3 ikOffset() const { return ikOffset_; }
  Vec3 ikFoot() const { return ikNeutral_ + ikOffset_; }
  uint32_t ikRejections() const { return ikRejections_; }

  FeetPositions standFeet() const;
  FeetPositions sitFeet() const;

 private:
  enum class After : uint8_t { None, Walk, Animate, IkLift, IkControl, IkUnshift };

  void startTransition(const FeetPositions& to, uint16_t ms, Posture result, After after);
  void finishTransition();
  void startAnimation();
  void updateWalking(float dt);
  void updateAnimation(float dt);
  FeetPositions animationFeet(const BodyPose& pose, const Vec3 (&offsets)[kLegCount]) const;
  FeetPositions ikBaseFeet() const;

  const RobotConfig& cfg_;
  const BodyKinematics& kin_;

  MotionState state_ = MotionState::Idle;
  Posture posture_ = Posture::Sitting;
  FeetPositions feet_;

  // transition
  FeetPositions from_;
  FeetPositions to_;
  float transitionMs_ = 0.0f;
  float transitionElapsedMs_ = 0.0f;
  Posture transitionResult_ = Posture::Standing;
  After after_ = After::None;

  // walking
  GaitEngine gaitEngine_;
  int gaitIndex_ = 0;
  WalkCommand walkTarget_;
  WalkCommand walk_;

  // animation
  const AnimationDef* anim_ = nullptr;
  const AnimationDef* pendingAnim_ = nullptr;
  bool animLoop_ = false;
  bool pendingLoop_ = false;
  int frame_ = 0;
  float frameElapsedMs_ = 0.0f;
  BodyPose poseFrom_;
  Vec3 offsetsFrom_[kLegCount];

  // single-leg IK
  int ikLeg_ = -1;
  Vec3 ikNeutral_;
  Vec3 ikOffset_;
  uint32_t ikRejections_ = 0;
};

}  // namespace qspider
