#include "kinematics/LegKinematics.h"

#include <cmath>

#include "common/MathUtil.h"

namespace qspider {

LegKinematics::LegKinematics(float coxa, float femur, float tibia)
    : coxa_(coxa), femur_(femur), tibia_(tibia) {}

bool LegKinematics::inverse(const Vec3& foot, JointAngles& out) const {
  if (!isFinite(foot)) return false;

  const float coxaAngle = std::atan2(foot.y, foot.x);
  const float r = std::sqrt(foot.x * foot.x + foot.y * foot.y) - coxa_;
  const float d2 = r * r + foot.z * foot.z;
  const float d = std::sqrt(d2);

  // Keep a small margin from the singular fully-stretched/folded configurations.
  constexpr float kMargin = 0.5f;
  if (d > femur_ + tibia_ - kMargin || d < std::fabs(femur_ - tibia_) + kMargin) return false;

  const float cosFemur = (femur_ * femur_ + d2 - tibia_ * tibia_) / (2.0f * femur_ * d);
  const float cosKnee = (femur_ * femur_ + tibia_ * tibia_ - d2) / (2.0f * femur_ * tibia_);

  out.coxa = rad2deg(coxaAngle);
  out.femur = rad2deg(std::atan2(foot.z, r) + std::acos(clampf(cosFemur, -1.0f, 1.0f)));
  out.tibia = rad2deg(std::acos(clampf(cosKnee, -1.0f, 1.0f)));
  return true;
}

Vec3 LegKinematics::forward(const JointAngles& j) const {
  const float coxa = deg2rad(j.coxa);
  const float femur = deg2rad(j.femur);
  // Tibia direction relative to horizontal: femur direction rotated down by (180 - knee).
  const float tibiaDir = femur - (kPi - deg2rad(j.tibia));
  const float r = coxa_ + femur_ * std::cos(femur) + tibia_ * std::cos(tibiaDir);
  const float z = femur_ * std::sin(femur) + tibia_ * std::sin(tibiaDir);
  return {r * std::cos(coxa), r * std::sin(coxa), z};
}

}  // namespace qspider
