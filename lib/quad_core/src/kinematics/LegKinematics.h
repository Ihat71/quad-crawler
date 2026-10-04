#pragma once

#include "common/Types.h"

namespace qspider {

// Analytic inverse/forward kinematics of a 3-DOF leg (coxa yaw, femur pitch,
// tibia pitch). Leg frame: origin on the coxa axis, x points outward along the
// leg's mounting direction, y is 90 deg counter-clockwise from x, z up.
class LegKinematics {
 public:
  LegKinematics(float coxa, float femur, float tibia);

  // Returns false if the point is out of reach (output left untouched).
  bool inverse(const Vec3& foot, JointAngles& out) const;
  Vec3 forward(const JointAngles& j) const;

  float maxReach() const { return femur_ + tibia_; }

 private:
  float coxa_;
  float femur_;
  float tibia_;
};

}  // namespace qspider
