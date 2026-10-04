#pragma once

#include "common/Types.h"
#include "config/RobotConfig.h"
#include "kinematics/BodyKinematics.h"

namespace qspider {

// Normalised walking command.
//   vx, vy : translation direction/share, |(vx,vy)| <= 1 (x forward, y left)
//   turn   : -1..1, positive = counter-clockwise (left)
//   speed  : 0..1 fraction of the gait's maximum cycle rate
struct WalkCommand {
  float vx = 0.0f;
  float vy = 0.0f;
  float turn = 0.0f;
  float speed = 0.0f;

  bool isZero() const { return speed <= 0.0f; }
};

// Phase-based periodic gait generator. Produces body-frame foot positions
// around the neutral stance for any GaitConfig (creep, trot, ...).
class GaitEngine {
 public:
  void reset();
  void update(float dt, const GaitConfig& gait, const WalkCommand& cmd, const FeetPositions& neutral,
              FeetPositions& out);

  float phase() const { return phase_; }
  bool inSwing(int leg) const { return swing_[leg]; }
  Vec3 bodyShift() const { return shift_; }

 private:
  float phase_ = 0.0f;
  bool swing_[kLegCount] = {};
  Vec3 shift_;
};

}  // namespace qspider
