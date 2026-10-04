#pragma once

#include <cmath>

#include "common/Types.h"

namespace qspider {

constexpr float kPi = 3.14159265358979f;

inline float deg2rad(float d) { return d * (kPi / 180.0f); }
inline float rad2deg(float r) { return r * (180.0f / kPi); }

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

inline Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
  return {lerpf(a.x, b.x, t), lerpf(a.y, b.y, t), lerpf(a.z, b.z, t)};
}

// Ease-in/ease-out curve, t in [0,1].
inline float smoothstep(float t) {
  t = clampf(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

inline Vec3 rotateZ(const Vec3& v, float rad) {
  const float c = std::cos(rad), s = std::sin(rad);
  return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}
inline Vec3 rotateY(const Vec3& v, float rad) {
  const float c = std::cos(rad), s = std::sin(rad);
  return {c * v.x + s * v.z, v.y, -s * v.x + c * v.z};
}
inline Vec3 rotateX(const Vec3& v, float rad) {
  const float c = std::cos(rad), s = std::sin(rad);
  return {v.x, c * v.y - s * v.z, s * v.y + c * v.z};
}

// Moves `current` towards `target` by at most `maxStep`.
inline float approach(float current, float target, float maxStep) {
  if (target > current + maxStep) return current + maxStep;
  if (target < current - maxStep) return current - maxStep;
  return target;
}

inline bool isFinite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

}  // namespace qspider
