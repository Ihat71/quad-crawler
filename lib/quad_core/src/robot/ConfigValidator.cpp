#include "robot/ConfigValidator.h"

#include <cmath>
#include <cstdarg>

#include "common/MathUtil.h"
#include "servo/AngleGuard.h"

namespace qspider {

namespace {

// Checks that a body-frame foot position is reachable with every servo inside the guard envelope.
bool footOk(const BodyKinematics& kin, const AngleGuard& guard, int leg, const Vec3& foot) {
  float s[kJointsPerLeg];
  if (!kin.footToServos(leg, foot, s)) return false;
  for (float a : s)
    if (a < guard.minDeg() || a > guard.maxDeg()) return false;
  return true;
}

struct Reporter {
  EventLog& log;
  ConfigReport& report;

  void error(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list args;
    va_start(args, fmt);
    ++report.errors;
    log.vlog(LogLevel::Error, LogCategory::System, fmt, args);
    va_end(args);
  }
  void warn(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list args;
    va_start(args, fmt);
    ++report.warnings;
    log.vlog(LogLevel::Warn, LogCategory::System, fmt, args);
    va_end(args);
  }
};

}  // namespace

ConfigReport validateConfig(const RobotConfig& cfg, const BodyKinematics& kin, EventLog& log) {
  ConfigReport r;
  Reporter rep{log, r};

  // Servo limits (requirement: 10..170 unless explicitly overridden).
  const AngleGuard guard(cfg.servoLimits);
  if (guard.configAdjusted())
    rep.error("Servo limits %.0f..%.0f violate 10..170 without allowLimitOverride; using %.0f..%.0f",
          cfg.servoLimits.minDeg, cfg.servoLimits.maxDeg, guard.minDeg(), guard.maxDeg());

  // Channel map.
  uint32_t used = 0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int j = 0; j < kJointsPerLeg; ++j) {
      const ServoMapping& m = cfg.legs[leg].servo[j];
      if (m.channel > 15) {
        rep.error("%s %s: channel %u out of range", legName(leg), jointName(j), m.channel);
        continue;
      }
      if (used & (1u << m.channel)) rep.error("%s %s: channel %u used twice", legName(leg), jointName(j), m.channel);
      used |= 1u << m.channel;
      if (m.direction != 1 && m.direction != -1)
        rep.error("%s %s: direction must be +1 or -1", legName(leg), jointName(j));
    }
  }

  const GeometryConfig& g = cfg.geometry;
  if (!(g.coxaLength > 0 && g.femurLength > 0 && g.tibiaLength > 0)) rep.error("Leg segment lengths must be > 0");

  const MotionConfig& mc = cfg.motion;
  if (mc.gaitCount < 1 || mc.gaitCount > kMaxGaits) rep.error("gaitCount must be 1..%d", kMaxGaits);
  if (mc.defaultGait < 0 || mc.defaultGait >= mc.gaitCount) rep.error("defaultGait out of range");
  if (!(mc.maxSpeedFraction > 0.0f && mc.maxSpeedFraction <= 1.0f)) rep.error("maxSpeedFraction must be in (0,1]");
  if (mc.ik.leg < 0 || mc.ik.leg >= kLegCount) rep.error("IK leg out of range");

  if (cfg.timing.controlPeriodMs < 5 || cfg.timing.controlPeriodMs > 50)
    rep.error("controlPeriodMs must be 5..50 (SG90 refresh is 20 ms)");

  if (cfg.battery.enabled) {
    const BatteryConfig& b = cfg.battery;
    if (!(b.criticalVoltage < b.warnVoltage && b.warnVoltage < b.fullVoltage && b.fullVoltage < b.overVoltage))
      rep.error("Battery thresholds must satisfy critical < warn < full < over");
    if (b.dividerRatio <= 0) rep.error("Battery divider ratio must be > 0");
  }

  if (r.errors > 0) return r;  // geometry checks below need a sane config

  // Postures.
  const FeetPositions stand = kin.neutralFeet(cfg.posture.standReach, cfg.posture.standHeight);
  const FeetPositions sit = kin.neutralFeet(cfg.posture.sitReach, cfg.posture.sitHeight);
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!footOk(kin, guard, leg, stand[leg])) rep.error("Stand pose unreachable/out of limits for %s", legName(leg));
    if (!footOk(kin, guard, leg, sit[leg])) rep.error("Sit pose unreachable/out of limits for %s", legName(leg));
  }

  // Gait envelope: stride extremes in 8 directions, lifted and on the ground, plus turning.
  for (int gi = 0; gi < mc.gaitCount; ++gi) {
    const GaitConfig& gc = mc.gaits[gi];
    if (!(gc.dutyFactor > 0.0f && gc.dutyFactor < 1.0f) || gc.maxCycleHz <= 0.0f || gc.maxStride <= 0.0f) {
      rep.error("Gait '%s': invalid duty/cycle/stride", gc.name);
      continue;
    }
    bool ok = true;
    for (int leg = 0; leg < kLegCount && ok; ++leg) {
      for (int k = 0; k < 8 && ok; ++k) {
        const float a = k * kPi / 4.0f;
        const Vec3 d{std::cos(a) * gc.maxStride * 0.5f, std::sin(a) * gc.maxStride * 0.5f, 0.0f};
        ok = footOk(kin, guard, leg, stand[leg] + d) && footOk(kin, guard, leg, stand[leg] + d + Vec3{0, 0, gc.stepHeight});
      }
    }
    if (!ok) rep.warn("Gait '%s': stride/step height partly outside the reachable envelope", gc.name);
  }

  return r;
}

}  // namespace qspider
