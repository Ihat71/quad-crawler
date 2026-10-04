#pragma once

#include <array>

#include "common/Types.h"
#include "config/RobotConfig.h"
#include "kinematics/LegKinematics.h"

namespace qspider {

using FeetPositions = std::array<Vec3, kLegCount>;

// Body-level kinematics: converts foot positions in the body frame to nominal
// servo angles (before calibration trim and limits) and back.
class BodyKinematics {
 public:
  explicit BodyKinematics(const RobotConfig& cfg);

  Vec3 bodyToLeg(int leg, const Vec3& pBody) const;
  Vec3 legToBody(int leg, const Vec3& pLeg) const;

  // Neutral foot position for a posture (reach/height measured from the coxa axis).
  Vec3 neutralFoot(int leg, float reach, float height) const;
  FeetPositions neutralFeet(float reach, float height) const;

  float jointToServo(int joint, int8_t direction, float jointDeg) const;
  float servoToJoint(int joint, int8_t direction, float servoDeg) const;

  // Foot (body frame) -> 3 nominal servo angles. False if unreachable.
  bool footToServos(int leg, const Vec3& footBody, float (&servoDeg)[kJointsPerLeg],
                    JointAngles* joints = nullptr) const;
  // 3 nominal servo angles -> foot (body frame).
  Vec3 servosToFoot(int leg, const float (&servoDeg)[kJointsPerLeg]) const;

  const LegKinematics& leg() const { return leg_; }

 private:
  const RobotConfig& cfg_;
  LegKinematics leg_;
};

}  // namespace qspider
