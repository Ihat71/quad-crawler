#pragma once
// =============================================================================
//  QuadSpider robot configuration
//
//  Every tunable value of the robot lives here: wiring, geometry, servo limits,
//  gaits, controller mapping, safety thresholds and timing. Movement, safety
//  and driver code only read this structure, so the robot can be re-tuned or
//  re-wired without touching any logic.
//
//  Units: millimetres, degrees, milliseconds, volts unless stated otherwise.
//  Coordinate frame: x forward, y left, z up (origin = body centre).
// =============================================================================

#include "config/RobotConfig.h"

// WiFi credentials and the PS5 controller MAC live in config/secrets.h, which
// is git-ignored. Copy config/secrets.example.h to config/secrets.h to set them.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

namespace qspider {

inline RobotConfig makeRobotConfig() {
  RobotConfig c{};
  c.robotName = "QuadSpider";

  // --------------------------------------------------------------- PCA9685 --
  c.pca.i2cAddress = 0x40;
  c.pca.sdaPin = 21;
  c.pca.sclPin = 22;
  c.pca.i2cClockHz = 400000;
  c.pca.pwmFrequencyHz = 50.0f;        // SG90 expects a 20 ms frame
  c.pca.oscillatorHz = 25000000.0f;    // trim if the measured PWM frequency is off
  c.pca.pulseAtMinDegUs = 500.0f;      // SG90: ~500 us = 0 deg
  c.pca.pulseAtMaxDegUs = 2400.0f;     //       ~2400 us = 180 deg
  c.pca.outputEnablePin = -1;          // wire PCA9685 OE to a GPIO for a hardware kill (recommended)

  // ------------------------------------------- battery (optional divider) --
  // 2S Li-ion (7.4 V nominal). Suggested divider: 100k (top) / 33k (bottom)
  // -> ratio 4.03, 8.4 V reads as ~2.1 V on the ADC. Set enabled = true once wired.
  c.battery.enabled = false;
  c.battery.adcPin = 34;               // ADC1 pin (ADC2 is unusable while WiFi runs)
  c.battery.dividerRatio = 133.0f / 33.0f;
  c.battery.calibrationScale = 1.0f;   // adjust after comparing with a multimeter
  c.battery.samplesPerRead = 8;
  c.battery.samplePeriodMs = 100;
  c.battery.emaAlpha = 0.2f;
  c.battery.fullVoltage = 8.4f;
  c.battery.warnVoltage = 7.0f;
  c.battery.criticalVoltage = 6.6f;    // 3.3 V/cell under load
  c.battery.overVoltage = 8.8f;
  c.battery.minStartVoltage = 7.0f;
  c.battery.hysteresisVoltage = 0.15f;
  c.battery.rippleMaxVoltage = 1.2f;   // peak-to-peak within the window => "irregular power"
  c.battery.rippleWindowMs = 1000;
  c.battery.criticalHoldMs = 2000;
  c.battery.plausibleMinVoltage = 2.0f;   // below: divider disconnected / sensor fault
  c.battery.plausibleMaxVoltage = 12.0f;

  // ------------------------------------------------------- PS5 controller --
  c.controller.macAddress = QUAD_PS5_MAC;
  c.controller.invertLeftY = false;
  c.controller.invertRightY = false;

  // -------------------------------------------------------------- network --
  c.network.enabled = true;
  c.network.hostname = "quadspider";   // http://quadspider.local
  c.network.staSsid = QUAD_WIFI_SSID;  // empty -> access point only
  c.network.staPassword = QUAD_WIFI_PASSWORD;
  c.network.staConnectTimeoutMs = 10000;
  c.network.apSsid = "QuadSpider";
  c.network.apPassword = QUAD_AP_PASSWORD;  // >= 8 characters
  c.network.httpPort = 80;

  // ------------------------------------------------------------- geometry --
  // Measure your legs: coxa = coxa axis -> femur axis, femur = femur axis ->
  // tibia axis, tibia = tibia axis -> foot tip.
  c.geometry.coxaLength = 27.5f;
  c.geometry.femurLength = 55.0f;
  c.geometry.tibiaLength = 77.5f;
  // Joint angle at which the servo sits at servoCentreDeg (the calibration pose):
  // coxa straight out, femur horizontal, tibia at 90 deg to the femur.
  c.geometry.coxaRefDeg = 0.0f;
  c.geometry.femurRefDeg = 0.0f;
  c.geometry.tibiaRefDeg = 90.0f;
  c.geometry.servoCentreDeg = 90.0f;

  // Legs: coxa axis position, outward direction, and {channel, direction} for
  // coxa / femur / tibia. Flip a direction if that joint moves the wrong way
  // in the hardware leg test.
  //                    mount position           yaw      coxa      femur     tibia
  c.legs[0] = LegConfig{Vec3{35.0f, -35.0f, 0.0f}, -45.0f, {{0, 1}, {1, 1}, {2, 1}}};    // FR
  c.legs[1] = LegConfig{Vec3{35.0f, 35.0f, 0.0f}, 45.0f, {{3, 1}, {4, -1}, {5, -1}}};    // FL
  c.legs[2] = LegConfig{Vec3{-35.0f, 35.0f, 0.0f}, 135.0f, {{6, 1}, {7, -1}, {8, -1}}};  // RL
  c.legs[3] = LegConfig{Vec3{-35.0f, -35.0f, 0.0f}, -135.0f, {{9, 1}, {10, 1}, {11, 1}}};// RR

  // --------------------------------------------------------- servo limits --
  c.servoLimits.minDeg = 10.0f;        // requirement: never outside 10..170 ...
  c.servoLimits.maxDeg = 170.0f;
  c.servoLimits.allowLimitOverride = false;  // ... unless this is set to true
  c.servoLimits.maxSlewDegPerSec = 450.0f;
  c.servoLimits.maxTrimDeg = 25.0f;

  // ------------------------------------------------------------- postures --
  c.posture.standHeight = 60.0f;
  c.posture.standReach = 80.0f;
  c.posture.sitHeight = 20.0f;
  c.posture.sitReach = 90.0f;
  c.posture.transitionMs = 800;

  // ---------------------------------------------------------------- gaits --
  // Leg order for phaseOffset: FR, FL, RL, RR.
  // Creep: one leg in the air at a time (RL -> FL -> RR -> FR), statically stable.
  c.motion.gaits[0] = GaitConfig{"creep", 0.75f, {0.0f, 0.5f, 0.75f, 0.25f}, 1.0f, 25.0f, 40.0f, 12.0f, 0.12f};
  // Trot: diagonal pairs (FR+RL, FL+RR), faster but dynamic.
  c.motion.gaits[1] = GaitConfig{"trot", 0.5f, {0.0f, 0.5f, 0.0f, 0.5f}, 1.6f, 20.0f, 36.0f, 10.0f, 0.0f};
  c.motion.gaitCount = 2;
  c.motion.defaultGait = 0;
  c.motion.maxSpeedFraction = 0.9f;    // full stick = 90% of the maximum capable speed
  c.motion.commandRampPerSec = 2.5f;
  c.motion.stopSettleMs = 300;
  c.motion.animationReturnMs = 400;
  c.motion.defaultDance = 0;

  // ------------------------------------------------------------- IK mode --
  c.motion.ik.leg = static_cast<int>(LegId::FrontRight);
  c.motion.ik.bodyShift = 18.0f;
  c.motion.ik.startLift = 30.0f;
  c.motion.ik.speedMmPerSec = 40.0f;
  c.motion.ik.rangeMin = Vec3{-30.0f, -30.0f, -10.0f};
  c.motion.ik.rangeMax = Vec3{40.0f, 30.0f, 50.0f};

  // --------------------------------------------------- controller mapping --
  c.input.map.modeToggle = Button::Up;
  c.input.map.emergencyStop = Button::Left;
  c.input.map.nextGaitOrDance = Button::Right;   // gait in NORMAL, dance in DANCE
  c.input.map.sitStand = Button::Cross;
  c.input.map.resetDefaults = Button::Triangle;
  c.input.map.dance = Button::Circle;
  c.input.map.attack = Button::Square;
  c.input.map.clearEmergencyStop = Button::PlayStation;   // hold
  c.input.map.shutdown = Button::Options;        // hold
  c.input.map.calPrevServo = Button::L1;         // CALIBRATION mode only
  c.input.map.calNextServo = Button::R1;
  c.input.map.calSave = Button::Cross;
  c.input.map.calResetTrim = Button::Triangle;
  c.input.map.calWiggle = Button::Circle;
  c.input.stickDeadzone = 0.12f;
  c.input.holdMs = 2000;
  c.input.staleTimeoutMs = 1000;                 // 0 disables the report-freshness check
  c.input.calNudgeStepDeg = 0.5f;
  c.input.calNudgeRepeatMs = 150;

  // --------------------------------------------------------------- safety --
  c.safety.controllerLostShutdownMs = 60000;
  c.safety.driverVerifyPeriodMs = 1000;
  c.safety.maxConsecutiveI2cErrors = 3;
  c.safety.overrunFactor = 1.5f;
  c.safety.overrunWarnPerSecond = 5;

  // --------------------------------------------------------------- timing --
  c.timing.controlPeriodMs = 20;       // 50 Hz servo updates
  c.timing.armStaggerMs = 250;
  c.timing.armSettleMs = 400;
  c.timing.calibrationMoveMs = 1200;
  c.timing.telemetryPeriodMs = 250;

  return c;
}

}  // namespace qspider
