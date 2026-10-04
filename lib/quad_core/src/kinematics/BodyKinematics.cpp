#include "kinematics/BodyKinematics.h"

#include "common/MathUtil.h"

namespace qspider {

BodyKinematics::BodyKinematics(const RobotConfig& cfg)
    : cfg_(cfg),
      leg_(cfg.geometry.coxaLength, cfg.geometry.femurLength, cfg.geometry.tibiaLength) {}

Vec3 BodyKinematics::bodyToLeg(int leg, const Vec3& pBody) const {
  const LegConfig& lc = cfg_.legs[leg];
  return rotateZ(pBody - lc.mountPosition, -deg2rad(lc.mountYawDeg));
}

Vec3 BodyKinematics::legToBody(int leg, const Vec3& pLeg) const {
  const LegConfig& lc = cfg_.legs[leg];
  return rotateZ(pLeg, deg2rad(lc.mountYawDeg)) + lc.mountPosition;
}

Vec3 BodyKinematics::neutralFoot(int leg, float reach, float height) const {
  return legToBody(leg, Vec3{reach, 0.0f, -height});
}

FeetPositions BodyKinematics::neutralFeet(float reach, float height) const {
  FeetPositions f;
  for (int i = 0; i < kLegCount; ++i) f[i] = neutralFoot(i, reach, height);
  return f;
}

static float jointRef(const GeometryConfig& g, int joint) {
  switch (joint) {
    case 0: return g.coxaRefDeg;
    case 1: return g.femurRefDeg;
    default: return g.tibiaRefDeg;
  }
}

float BodyKinematics::jointToServo(int joint, int8_t direction, float jointDeg) const {
  const GeometryConfig& g = cfg_.geometry;
  return g.servoCentreDeg + static_cast<float>(direction) * (jointDeg - jointRef(g, joint));
}

float BodyKinematics::servoToJoint(int joint, int8_t direction, float servoDeg) const {
  const GeometryConfig& g = cfg_.geometry;
  return jointRef(g, joint) + (servoDeg - g.servoCentreDeg) / static_cast<float>(direction);
}

bool BodyKinematics::footToServos(int leg, const Vec3& footBody, float (&servoDeg)[kJointsPerLeg],
                                  JointAngles* joints) const {
  JointAngles j;
  if (!leg_.inverse(bodyToLeg(leg, footBody), j)) return false;
  const LegConfig& lc = cfg_.legs[leg];
  servoDeg[0] = jointToServo(0, lc.servo[0].direction, j.coxa);
  servoDeg[1] = jointToServo(1, lc.servo[1].direction, j.femur);
  servoDeg[2] = jointToServo(2, lc.servo[2].direction, j.tibia);
  if (joints) *joints = j;
  return true;
}

Vec3 BodyKinematics::servosToFoot(int leg, const float (&servoDeg)[kJointsPerLeg]) const {
  const LegConfig& lc = cfg_.legs[leg];
  JointAngles j;
  j.coxa = servoToJoint(0, lc.servo[0].direction, servoDeg[0]);
  j.femur = servoToJoint(1, lc.servo[1].direction, servoDeg[1]);
  j.tibia = servoToJoint(2, lc.servo[2].direction, servoDeg[2]);
  return legToBody(leg, leg_.forward(j));
}

}  // namespace qspider
