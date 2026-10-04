#pragma once
// Shape of the robot configuration. This header only declares the structure;
// every value lives in config/robot_config.h at the project root so that tuning
// the robot never requires touching movement or safety logic.

#include <cstdint>

#include "common/Types.h"
#include "input/Buttons.h"

namespace qspider {

constexpr int kMaxGaits = 4;

// ---------------------------------------------------------------- hardware --

struct PcaConfig {
  uint8_t i2cAddress;
  int sdaPin;
  int sclPin;
  uint32_t i2cClockHz;
  float pwmFrequencyHz;
  float oscillatorHz;
  float pulseAtMinDegUs;  // pulse width for 0 deg
  float pulseAtMaxDegUs;  // pulse width for 180 deg
  int outputEnablePin;    // PCA9685 OE pin (active low), -1 if not wired
};

struct BatteryConfig {
  bool enabled;            // false when no voltage divider is wired
  int adcPin;
  float dividerRatio;      // Vbattery = Vadc * dividerRatio
  float calibrationScale;  // fine trim against a multimeter
  uint8_t samplesPerRead;
  uint32_t samplePeriodMs;
  float emaAlpha;          // smoothing factor (0..1], higher reacts faster
  float fullVoltage;
  float warnVoltage;
  float criticalVoltage;
  float overVoltage;
  float minStartVoltage;   // refuse to arm below this
  float hysteresisVoltage;
  float rippleMaxVoltage;  // max (max-min) inside the window before power is "irregular"
  uint32_t rippleWindowMs;
  uint32_t criticalHoldMs; // voltage must stay critical this long before stopping
  float plausibleMinVoltage;  // below this the sensor is assumed disconnected
  float plausibleMaxVoltage;
};

struct ControllerHwConfig {
  const char* macAddress;  // MAC the PS5 controller was paired to (see README)
  bool invertLeftY;
  bool invertRightY;
};

struct NetworkConfig {
  bool enabled;
  const char* hostname;  // mDNS name -> http://<hostname>.local
  const char* staSsid;   // join this network if non-empty ...
  const char* staPassword;
  uint32_t staConnectTimeoutMs;
  const char* apSsid;    // ... otherwise (or on failure) start an access point
  const char* apPassword;
  uint16_t httpPort;
};

// --------------------------------------------------------------- geometry --

struct ServoMapping {
  uint8_t channel;   // PCA9685 channel 0..15
  int8_t direction;  // +1 or -1, sign between joint angle and servo angle
};

struct LegConfig {
  Vec3 mountPosition;  // coxa axis position in the body frame (mm)
  float mountYawDeg;   // direction the leg points when the coxa is at 0 deg
  ServoMapping servo[kJointsPerLeg];  // coxa, femur, tibia
};

struct GeometryConfig {
  float coxaLength;
  float femurLength;
  float tibiaLength;
  // Joint angle that corresponds to the servo centre (servoCentreDeg).
  float coxaRefDeg;
  float femurRefDeg;
  float tibiaRefDeg;
  float servoCentreDeg;
};

// ------------------------------------------------------------------ servo --

struct ServoLimitConfig {
  float minDeg;               // hard software limits (10..170 by requirement)
  float maxDeg;
  bool allowLimitOverride;    // must be true to configure limits outside 10..170
  float maxSlewDegPerSec;     // SG90 is ~600 deg/s unloaded
  float maxTrimDeg;           // calibration trim range
};

// ----------------------------------------------------------------- motion --

struct PostureConfig {
  float standHeight;  // distance from coxa plane down to the feet (mm)
  float standReach;   // horizontal distance coxa axis -> foot (mm)
  float sitHeight;
  float sitReach;
  uint16_t transitionMs;
};

struct GaitConfig {
  const char* name;
  float dutyFactor;              // fraction of the cycle a foot is on the ground
  float phaseOffset[kLegCount];  // per-leg phase offset in [0,1)
  float maxCycleHz;              // 100% speed
  float stepHeight;              // mm
  float maxStride;               // mm, foot travel per stance phase
  float maxTurnDeg;              // body yaw per stance phase at full turn
  float bodySwayGain;            // shift body away from the swinging leg (0 = off)
};

struct IkModeConfig {
  int leg;                // leg controlled in IK mode (preset FR)
  float bodyShift;        // mm, body moves away from the lifted leg
  float startLift;        // mm, initial lift of the controlled foot
  float speedMmPerSec;    // joystick full-deflection foot speed
  Vec3 rangeMin;          // allowed offset from the neutral foot position
  Vec3 rangeMax;
};

struct MotionConfig {
  GaitConfig gaits[kMaxGaits];
  int gaitCount;
  int defaultGait;
  float maxSpeedFraction;     // joystick full deflection = this fraction of max speed
  float commandRampPerSec;    // how fast walk commands may change (1/s)
  uint16_t stopSettleMs;      // time to put all feet back to neutral after stopping
  uint16_t animationReturnMs; // time to return to stand when a dance is interrupted
  int defaultDance;
  IkModeConfig ik;
};

// ------------------------------------------------------------------ input --

struct ControllerMapping {
  Button modeToggle;
  Button emergencyStop;
  Button nextGaitOrDance;
  Button sitStand;
  Button resetDefaults;
  Button dance;
  Button attack;
  Button clearEmergencyStop;  // must be held for holdMs
  Button shutdown;            // must be held for holdMs
  Button calPrevServo;
  Button calNextServo;
  Button calSave;
  Button calResetTrim;
  Button calWiggle;
};

struct InputConfig {
  ControllerMapping map;
  float stickDeadzone;     // 0..1
  uint32_t holdMs;         // long-press threshold
  uint32_t staleTimeoutMs; // no packets for this long => controller lost
  float calNudgeStepDeg;
  uint32_t calNudgeRepeatMs;
};

// ----------------------------------------------------------------- safety --

struct SafetyConfig {
  uint32_t controllerLostShutdownMs;  // sit immediately, disable outputs after this
  uint32_t driverVerifyPeriodMs;      // PCA9685 register readback period
  uint8_t maxConsecutiveI2cErrors;
  float overrunFactor;                // tick period > factor * nominal counts as overrun
  uint32_t overrunWarnPerSecond;
};

struct TimingConfig {
  uint16_t controlPeriodMs;  // servo update period (SG90: 20 ms = 50 Hz)
  uint16_t armStaggerMs;     // delay between enabling legs (limits inrush current)
  uint16_t armSettleMs;
  uint16_t calibrationMoveMs;
  uint16_t telemetryPeriodMs;  // dashboard poll period
};

// ------------------------------------------------------------------- root --

struct RobotConfig {
  const char* robotName;
  PcaConfig pca;
  BatteryConfig battery;
  ControllerHwConfig controller;
  NetworkConfig network;
  GeometryConfig geometry;
  LegConfig legs[kLegCount];
  ServoLimitConfig servoLimits;
  PostureConfig posture;
  MotionConfig motion;
  InputConfig input;
  SafetyConfig safety;
  TimingConfig timing;
};

}  // namespace qspider
