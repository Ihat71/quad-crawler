#pragma once
// Basic types shared by every module. No hardware or framework dependencies.

#include <cmath>
#include <cstdint>

namespace qspider {

constexpr int kLegCount = 4;
constexpr int kJointsPerLeg = 3;
constexpr int kServoCount = kLegCount * kJointsPerLeg;

// Leg order is fixed throughout the code base (index = enum value).
enum class LegId : uint8_t { FrontRight = 0, FrontLeft = 1, RearLeft = 2, RearRight = 3 };
enum class Joint : uint8_t { Coxa = 0, Femur = 1, Tibia = 2 };

constexpr int servoIndex(int leg, int joint) { return leg * kJointsPerLeg + joint; }
constexpr int legOfServo(int servo) { return servo / kJointsPerLeg; }
constexpr int jointOfServo(int servo) { return servo % kJointsPerLeg; }

const char* legName(int leg);      // "FR", "FL", "RL", "RR"
const char* jointName(int joint);  // "coxa", "femur", "tibia"

// Body frame: x forward, y left, z up. Units: millimetres.
struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  constexpr Vec3() = default;
  constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
  Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
  Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  float norm() const { return std::sqrt(x * x + y * y + z * z); }
  float normXY() const { return std::sqrt(x * x + y * y); }
};

// Joint angles in degrees.
//   coxa : yaw of the leg relative to its mounting direction (0 = straight out)
//   femur: elevation of the femur above horizontal (positive = up)
//   tibia: interior knee angle between femur and tibia (180 = straight leg)
struct JointAngles {
  float coxa = 0.0f;
  float femur = 0.0f;
  float tibia = 90.0f;
};

enum class Mode : uint8_t { Normal, IK, Dance, Calibration, EmergencyStop };

// Power/lifecycle state of the robot, independent of the operating mode.
enum class Lifecycle : uint8_t {
  Booting,
  SelfTest,
  WaitingForController,  // outputs disabled until a controller (or an "arm" command) is present
  Arming,                // outputs are enabled one leg at a time into the sit pose
  Active,
  Stopping,              // controlled sit-down before outputs are disabled
  Shutdown,              // outputs disabled, safe to power off
};

enum class Posture : uint8_t { Sitting, Standing, Custom };

const char* toString(Mode m);
const char* toString(Lifecycle l);
const char* toString(Posture p);

}  // namespace qspider
