#include "robot/Robot.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "common/MathUtil.h"
#include "motion/Animation.h"
#include "robot/ConfigValidator.h"

namespace qspider {

namespace {

// Wrap-around safe "now >= t".
bool reached(uint32_t now, uint32_t t) { return static_cast<int32_t>(now - t) >= 0; }

const char* directionLabel(float vx, float vy) {
  static const char* const kLabels[8] = {"forward",  "forward-left",   "left",  "backward-left",
                                         "backward", "backward-right", "right", "forward-right"};
  const float deg = rad2deg(std::atan2(vy, vx));
  int sector = static_cast<int>(std::lround(deg / 45.0f));
  sector = ((sector % 8) + 8) % 8;
  return kLabels[sector];
}

void faultList(uint32_t mask, char* buf, size_t size) {
  size_t used = 0;
  buf[0] = '\0';
  for (int i = 0; i < kFaultCount && used < size; ++i) {
    if (!(mask & (1u << i))) continue;
    const int n = std::snprintf(buf + used, size - used, "%s%s", used ? "," : "", toString(static_cast<Fault>(i)));
    if (n < 0) break;
    used += static_cast<size_t>(n);
  }
}

}  // namespace

Robot::Robot(const RobotConfig& cfg, IServoDriver& driver, IGamepad& gamepad, IPowerSensor& power,
             ICalibrationStore& calStore, EventLog& log)
    : cfg_(cfg),
      driver_(driver),
      gamepad_(gamepad),
      power_(power),
      calStore_(calStore),
      log_(log),
      kin_(cfg),
      motion_(cfg, kin_),
      bus_(cfg, driver, log),
      battery_(cfg.battery),
      input_(cfg.input) {
  danceIndex_ = cfg.motion.defaultDance;
}

// ------------------------------------------------------------------ boot --

void Robot::begin(uint32_t nowMs) {
  lastTickMs_ = nowMs;
  lifecycle_ = Lifecycle::Booting;
  log_.log(LogLevel::Info, LogCategory::System, "%s booting", cfg_.robotName);

  lifecycle_ = Lifecycle::SelfTest;
  const ConfigReport report = validateConfig(cfg_, kin_, log_);
  if (report.errors > 0)
    setFault(Fault::ConfigInvalid, true);
  else
    log_.log(LogLevel::Info, LogCategory::System, "Config OK (%d warning(s))", report.warnings);

  CalibrationData cal;
  if (calStore_.load(cal)) {
    for (int s = 0; s < kServoCount; ++s) bus_.setTrim(s, cal.trimDeg[s]);
    log_.log(LogLevel::Info, LogCategory::Calibration, "Calibration loaded");
  } else {
    log_.log(LogLevel::Warn, LogCategory::Calibration, "No stored calibration - trims are 0, run CALIBRATION mode");
  }

  if (driver_.begin()) {
    log_.log(LogLevel::Info, LogCategory::System, "Servo driver OK, all outputs disabled");
  } else {
    setFault(Fault::ServoDriverInit, true);
  }

  if (power_.available()) {
    float v = 0.0f;
    for (int i = 0; i < 4; ++i) {
      if (power_.readVoltage(v)) battery_.addSample(v, nowMs);
    }
    lastBatterySampleMs_ = nowMs;
    log_.log(LogLevel::Info, LogCategory::Safety, "Battery %.2f V (%d%%)", battery_.voltage(), battery_.percent());
  } else {
    log_.log(LogLevel::Info, LogCategory::Safety, "Battery monitoring disabled (no voltage divider configured)");
  }

  lifecycle_ = Lifecycle::WaitingForController;
  if (activeFaults_ & kCriticalFaultMask) {
    emergencyStop("self-test failed");
  } else {
    log_.log(LogLevel::Info, LogCategory::System, "Self-test passed - waiting for controller (press PS)");
  }
  publishSnapshot(nowMs, InputFrame{});
}

// ------------------------------------------------------------------ tick --

void Robot::tick(uint32_t nowMs) {
  float dt = firstTick_ ? cfg_.timing.controlPeriodMs / 1000.0f : (nowMs - lastTickMs_) / 1000.0f;
  dt = clampf(dt, 0.0f, 0.1f);
  firstTick_ = false;
  lastTickMs_ = nowMs;
  ++ticks_;

  const InputFrame in = input_.update(gamepad_.read(), nowMs);
  logButtons(in);

  // The emergency stop button is evaluated before anything else.
  if (in.isPressed(cfg_.input.map.emergencyStop)) emergencyStop("controller emergency stop button");

  drainInbox(nowMs);
  superviseController(in, nowMs);
  superviseBattery(nowMs);
  handleInput(in, nowMs, dt);
  updateLifecycle(nowMs);
  updateOutputs(nowMs, dt);
  publishSnapshot(nowMs, in);
}

void Robot::reportTiming(uint32_t periodUs, uint32_t execUs) {
  constexpr float kAlpha = 0.05f;
  periodAvgUs_ = periodAvgUs_ == 0.0f ? periodUs : periodAvgUs_ + kAlpha * (periodUs - periodAvgUs_);
  execAvgUs_ = execAvgUs_ == 0.0f ? execUs : execAvgUs_ + kAlpha * (execUs - execAvgUs_);
  if (execUs > execMaxUs_) execMaxUs_ = execUs;

  const float nominalUs = cfg_.timing.controlPeriodMs * 1000.0f;
  if (periodUs > nominalUs * cfg_.safety.overrunFactor || execUs > nominalUs) {
    ++overruns_;
    ++overrunsInWindow_;
    lastOverrunMs_ = lastTickMs_;
  }
  if (reached(lastTickMs_, overrunWindowStartMs_ + 1000)) {
    if (overrunsInWindow_ > cfg_.safety.overrunWarnPerSecond && !hasFault(Fault::LoopOverrun)) {
      setFault(Fault::LoopOverrun, true);
      log_.log(LogLevel::Warn, LogCategory::System, "Control loop overran %u times in 1 s",
               static_cast<unsigned>(overrunsInWindow_));
    }
    overrunsInWindow_ = 0;
    overrunWindowStartMs_ = lastTickMs_;
    execMaxUs_ = execUs;  // max is reported per window
  }
  if (hasFault(Fault::LoopOverrun) && reached(lastTickMs_, lastOverrunMs_ + 5000)) setFault(Fault::LoopOverrun, false);
}

// -------------------------------------------------------------- commands --

bool Robot::submit(const Command& cmd) {
  std::lock_guard<std::mutex> lock(inboxMutex_);
  if (inboxCount_ >= kInboxSize) {
    if (cmd.type != CommandType::EmergencyStop) return false;
    inbox_[(inboxHead_ + kInboxSize - 1) % kInboxSize] = cmd;  // an E-stop is never dropped
    return true;
  }
  inbox_[(inboxHead_ + inboxCount_) % kInboxSize] = cmd;
  ++inboxCount_;
  return true;
}

void Robot::drainInbox(uint32_t now) {
  Command local[kInboxSize];
  int n = 0;
  {
    std::lock_guard<std::mutex> lock(inboxMutex_);
    for (; n < inboxCount_; ++n) local[n] = inbox_[(inboxHead_ + n) % kInboxSize];
    inboxHead_ = 0;
    inboxCount_ = 0;
  }
  for (int i = 0; i < n; ++i) execute(local[i], now);
}

void Robot::execute(const Command& c, uint32_t now) {
  if (c.source != CommandSource::Controller)
    log_.log(LogLevel::Info, LogCategory::Input, "Command '%s' from %s", toString(c.type), toString(c.source));

  // Commands that are valid in any state.
  switch (c.type) {
    case CommandType::EmergencyStop: {
      char reason[40];
      std::snprintf(reason, sizeof(reason), "%s request", toString(c.source));
      emergencyStop(reason);
      return;
    }
    case CommandType::ClearEmergencyStop: clearEmergencyStop(c.source, now); return;
    case CommandType::Arm: arm(c.source, now); return;
    case CommandType::Shutdown: beginShutdown(StopReason::User, toString(c.source)); return;
    case CommandType::SetMode:
      if (c.i >= 0 && c.i <= static_cast<int32_t>(Mode::EmergencyStop)) setMode(static_cast<Mode>(c.i), c.source);
      return;
    case CommandType::CalSave: calSave(); return;
    case CommandType::SetLogLevel:
      log_.setMinLevel(static_cast<LogLevel>(clampf(static_cast<float>(c.i), 0.0f, 4.0f)));
      return;
    default: break;
  }

  if (mode_ == Mode::EmergencyStop || lifecycle_ != Lifecycle::Active) {
    log_.log(LogLevel::Warn, LogCategory::Input, "'%s' ignored (%s / %s)", toString(c.type), toString(lifecycle_),
             toString(mode_));
    return;
  }

  const bool posturesAllowed = mode_ == Mode::Normal || mode_ == Mode::Dance;
  const bool calOnly = c.type == CommandType::CalSelect || c.type == CommandType::CalSetTrim ||
                       c.type == CommandType::CalNudge || c.type == CommandType::CalResetTrim;
  if (calOnly && mode_ != Mode::Calibration) {
    log_.log(LogLevel::Warn, LogCategory::Calibration, "'%s' only works in CALIBRATION mode", toString(c.type));
    return;
  }

  switch (c.type) {
    case CommandType::Sit:
    case CommandType::Stand:
    case CommandType::ToggleSitStand:
      if (!posturesAllowed) {
        log_.log(LogLevel::Warn, LogCategory::Motion, "Posture commands need NORMAL or DANCE mode");
        return;
      }
      if (c.type == CommandType::Sit) motion_.sit();
      else if (c.type == CommandType::Stand) motion_.stand();
      else motion_.toggleSitStand();
      log_.movement(mode_, "%s", motion_.targetPosture() == Posture::Sitting ? "SIT" : "STAND");
      break;
    case CommandType::Dance:
      if (posturesAllowed) playDance(c.i < 0 ? danceIndex_ : c.i, c.f > 0.0f, c.source);
      break;
    case CommandType::StopMotion:
      if (mode_ == Mode::IK) {
        motion_.resetIk();
      } else if (mode_ != Mode::Calibration) {
        motion_.stopAll();
        log_.movement(mode_, "STOP");
      }
      break;
    case CommandType::Attack:
      if (posturesAllowed) attack(c.source);
      break;
    case CommandType::NextGait: nextGait(c.source); break;
    case CommandType::SetGait:
      if (motion_.state() == MotionState::Walking) motion_.stopAll();
      motion_.selectGait(c.i);
      log_.log(LogLevel::Info, LogCategory::Motion, "Gait: %s", motion_.gait().name);
      break;
    case CommandType::NextDance: {
      danceIndex_ = (danceIndex_ + 1) % animations::danceCount();
      log_.log(LogLevel::Info, LogCategory::Motion, "Dance selected: %s", animations::dance(danceIndex_).name);
      if (motion_.animation() && motion_.animationLooping()) playDance(danceIndex_, true, c.source);
      break;
    }
    case CommandType::ResetDefaults: resetDefaults(c.source); break;
    case CommandType::CalSelect: calSelect(c.i); break;
    case CommandType::CalSetTrim:
      if (c.i >= 0 && c.i < kServoCount) calSetTrim(c.i, c.f);
      break;
    case CommandType::CalNudge: {
      const int s = c.i < 0 ? calSelected_ : c.i;
      if (s >= 0 && s < kServoCount) calSetTrim(s, bus_.trim(s) + c.f);
      break;
    }
    case CommandType::CalResetTrim:
      if (c.i < 0) {
        for (int s = 0; s < kServoCount; ++s) bus_.setTrim(s, 0.0f);
        calDirty_ = true;
        log_.log(LogLevel::Info, LogCategory::Calibration, "All trims reset to 0 (not saved)");
      } else if (c.i < kServoCount) {
        calSetTrim(c.i, 0.0f);
      }
      break;
    default: break;
  }
}

// ----------------------------------------------------------------- input --

void Robot::logButtons(const InputFrame& in) {
  if (!(in.pressed | in.longPressed)) return;
  for (int i = 0; i < kButtonCount; ++i) {
    const Button b = static_cast<Button>(i);
    if (in.pressed & buttonBit(b)) log_.button(b, ButtonAction::Press, mode_);
    if (in.longPressed & buttonBit(b)) log_.button(b, ButtonAction::Hold, mode_);
  }
}

void Robot::handleInput(const InputFrame& in, uint32_t now, float dt) {
  if (!in.connected) return;
  const ControllerMapping& m = cfg_.input.map;

  if (in.isLongPress(m.clearEmergencyStop)) {
    if (mode_ == Mode::EmergencyStop)
      clearEmergencyStop(CommandSource::Controller, now);
    else if (lifecycle_ == Lifecycle::Shutdown)
      arm(CommandSource::Controller, now);
  }
  if (mode_ == Mode::EmergencyStop || lifecycle_ != Lifecycle::Active) return;

  if (in.isLongPress(m.shutdown)) {
    beginShutdown(StopReason::User, "controller");
    return;
  }
  if (in.isPressed(m.modeToggle)) cycleMode(CommandSource::Controller);

  switch (mode_) {
    case Mode::Normal: handleNormal(in, dt); break;
    case Mode::IK: handleIk(in, dt); break;
    case Mode::Dance: handleDance(in); break;
    case Mode::Calibration: handleCalibration(in, now); break;
    case Mode::EmergencyStop: break;
  }
}

void Robot::handleNormal(const InputFrame& in, float /*dt*/) {
  const ControllerMapping& m = cfg_.input.map;
  if (in.isPressed(m.sitStand)) {
    motion_.toggleSitStand();
    log_.movement(mode_, "%s", motion_.targetPosture() == Posture::Sitting ? "SIT" : "STAND");
  }
  if (in.isPressed(m.resetDefaults)) resetDefaults(CommandSource::Controller);
  if (in.isPressed(m.nextGaitOrDance)) nextGait(CommandSource::Controller);
  if (in.isPressed(m.dance)) playDance(danceIndex_, false, CommandSource::Controller);
  if (in.isPressed(m.attack)) attack(CommandSource::Controller);

  const WalkCommand w = sticksToWalk(in);
  motion_.setWalk(w);
  if (!motion_.animation() && !motion_.pendingAnimation()) logWalk(w);
}

void Robot::handleIk(const InputFrame& in, float dt) {
  const ControllerMapping& m = cfg_.input.map;
  if (in.isPressed(m.resetDefaults)) {
    motion_.resetIk();
    log_.movement(mode_, "IK %s foot reset", legName(cfg_.motion.ik.leg));
  }
  if (motion_.state() != MotionState::IkControl) return;

  // Left stick: forward/back + left/right, right stick vertical: up/down.
  const Vec3 v = Vec3{in.ly, -in.lx, in.ry} * cfg_.motion.ik.speedMmPerSec;
  const bool moving = v.norm() > 0.0f;
  if (moving && !motion_.moveIk(v, dt) && reached(lastTickMs_, lastIkRejectLogMs_ + 1000)) {
    lastIkRejectLogMs_ = lastTickMs_;
    log_.log(LogLevel::Warn, LogCategory::Motion, "IK target outside the reachable workspace");
  }
  if (ikMoving_ && !moving) {
    const Vec3 o = motion_.ikOffset();
    log_.movement(mode_, "IK %s foot offset (%.0f, %.0f, %.0f) mm", legName(motion_.ikLeg()), o.x, o.y, o.z);
  }
  ikMoving_ = moving;
}

void Robot::handleDance(const InputFrame& in) {
  const ControllerMapping& m = cfg_.input.map;
  if (in.isPressed(m.dance)) {
    if (motion_.animation() && motion_.animationLooping()) {
      motion_.stopAll();
      log_.movement(mode_, "DANCE stop");
    } else {
      playDance(danceIndex_, true, CommandSource::Controller);
    }
  }
  if (in.isPressed(m.nextGaitOrDance)) {
    Command c;
    c.type = CommandType::NextDance;
    c.source = CommandSource::Controller;
    execute(c, lastTickMs_);
  }
  if (in.isPressed(m.sitStand)) {
    motion_.toggleSitStand();
    log_.movement(mode_, "%s", motion_.targetPosture() == Posture::Sitting ? "SIT" : "STAND");
  }
  if (in.isPressed(m.resetDefaults)) resetDefaults(CommandSource::Controller);
  if (in.isPressed(m.attack)) attack(CommandSource::Controller);
}

void Robot::handleCalibration(const InputFrame& in, uint32_t now) {
  const ControllerMapping& m = cfg_.input.map;
  if (in.isPressed(m.calNextServo)) calSelect(calSelected_ + 1);
  if (in.isPressed(m.calPrevServo)) calSelect(calSelected_ - 1);
  if (in.isPressed(m.calSave)) calSave();
  if (in.isPressed(m.calResetTrim)) calSetTrim(calSelected_, 0.0f);
  if (in.isPressed(m.calWiggle)) wiggleElapsedMs_ = 0.0f;

  if (std::fabs(in.ly) > 0.5f) {
    if (reached(now, calNudgeNextMs_)) {
      const float step = in.ly > 0.0f ? cfg_.input.calNudgeStepDeg : -cfg_.input.calNudgeStepDeg;
      calSetTrim(calSelected_, bus_.trim(calSelected_) + step);
      calNudgeNextMs_ = now + cfg_.input.calNudgeRepeatMs;
    }
  } else {
    calNudgeNextMs_ = now;
  }
}

WalkCommand Robot::sticksToWalk(const InputFrame& in) const {
  WalkCommand w;
  const float dominant = std::fmax(in.leftMagnitude, std::fabs(in.rx));
  if (dominant <= 0.0f) return w;  // both sticks in the deadzone: stop
  w.speed = cfg_.motion.maxSpeedFraction * dominant;
  w.vx = in.ly / dominant;    // stick forward = walk forward
  w.vy = -in.lx / dominant;   // stick right   = walk right (-y)
  w.turn = -in.rx / dominant; // stick right   = turn clockwise
  return w;
}

void Robot::logWalk(const WalkCommand& w) {
  char desc[sizeof(lastWalkDesc_)];
  if (w.isZero()) {
    if (lastWalkDesc_[0] == '\0' || std::strcmp(lastWalkDesc_, "STOP") == 0) return;
    std::snprintf(desc, sizeof(desc), "STOP");
  } else {
    const bool translating = std::sqrt(w.vx * w.vx + w.vy * w.vy) > 0.2f;
    const bool turning = std::fabs(w.turn) > 0.2f;
    const int pct = static_cast<int>(std::lround(w.speed * 10.0f)) * 10;
    std::snprintf(desc, sizeof(desc), "WALK %s%s%s %d%% %s", translating ? directionLabel(w.vx, w.vy) : "",
                  translating && turning ? " + " : "", turning ? (w.turn > 0 ? "turn-left" : "turn-right") : "",
                  pct, motion_.gait().name);
  }
  if (std::strcmp(desc, lastWalkDesc_) == 0) return;
  std::snprintf(lastWalkDesc_, sizeof(lastWalkDesc_), "%s", desc);
  log_.movement(mode_, "%s", desc);
}

// -------------------------------------------------------- motion helpers --

void Robot::playDance(int index, bool loop, CommandSource src) {
  const int n = animations::danceCount();
  danceIndex_ = ((index % n) + n) % n;
  const AnimationDef& a = animations::dance(danceIndex_);
  motion_.playAnimation(a, loop);
  log_.log(LogLevel::Info, LogCategory::Motion, "Dance '%s'%s (%s)", a.name, loop ? " (loop)" : "", toString(src));
  log_.movement(mode_, "DANCE %s%s", a.name, loop ? " loop" : "");
  lastWalkDesc_[0] = '\0';
}

void Robot::attack(CommandSource src) {
  motion_.playAnimation(animations::attack(), false);
  log_.log(LogLevel::Info, LogCategory::Motion, "Attack animation (%s)", toString(src));
  log_.movement(mode_, "ATTACK");
  lastWalkDesc_[0] = '\0';
}

void Robot::resetDefaults(CommandSource src) {
  if (mode_ == Mode::IK) {
    motion_.resetIk();
    return;
  }
  motion_.stopAll();
  motion_.selectGait(cfg_.motion.defaultGait);
  danceIndex_ = cfg_.motion.defaultDance;
  motion_.stand();
  log_.log(LogLevel::Info, LogCategory::Motion, "Movement reset to defaults (%s)", toString(src));
  log_.movement(mode_, "RESET (gait %s, stand)", motion_.gait().name);
  lastWalkDesc_[0] = '\0';
}

void Robot::nextGait(CommandSource src) {
  // Phase offsets differ between gaits, so settle the feet before switching.
  if (motion_.state() == MotionState::Walking) motion_.stopAll();
  motion_.selectGait(motion_.gaitIndex() + 1);
  log_.log(LogLevel::Info, LogCategory::Motion, "Gait: %s (%s)", motion_.gait().name, toString(src));
  log_.movement(mode_, "GAIT %s", motion_.gait().name);
}

// ----------------------------------------------------------------- modes --

void Robot::cycleMode(CommandSource src) {
  switch (mode_) {
    case Mode::Normal: setMode(Mode::IK, src); break;
    case Mode::IK: setMode(Mode::Dance, src); break;
    case Mode::Dance: setMode(Mode::Calibration, src); break;
    case Mode::Calibration: setMode(Mode::Normal, src); break;
    case Mode::EmergencyStop: break;
  }
}

void Robot::setMode(Mode m, CommandSource src) {
  if (m == mode_) return;
  if (m == Mode::EmergencyStop) {
    emergencyStop("mode request");
    return;
  }
  if (mode_ == Mode::EmergencyStop) {
    log_.log(LogLevel::Warn, LogCategory::Mode, "Mode change refused: clear EMERGENCY_STOP first (hold PS)");
    return;
  }
  if (lifecycle_ != Lifecycle::Active) {
    log_.log(LogLevel::Warn, LogCategory::Mode, "Mode change refused while %s", toString(lifecycle_));
    return;
  }

  const Mode old = mode_;
  switch (old) {
    case Mode::Normal:
      motion_.setWalk(WalkCommand{});
      if (motion_.state() == MotionState::Walking || motion_.state() == MotionState::Animating) motion_.stopAll();
      break;
    case Mode::IK:
      motion_.exitIk();
      ikMoving_ = false;
      break;
    case Mode::Dance: motion_.stopAll(); break;
    case Mode::Calibration: exitCalibration(false); break;
    case Mode::EmergencyStop: break;
  }

  mode_ = m;
  if (m == Mode::IK) motion_.enterIk(cfg_.motion.ik.leg);
  if (m == Mode::Calibration) enterCalibration();

  lastWalkDesc_[0] = '\0';
  log_.log(LogLevel::Info, LogCategory::Mode, "Mode %s -> %s (%s)", toString(old), toString(m), toString(src));
  log_.movement(mode_, "MODE %s", toString(m));
}

void Robot::emergencyStop(const char* reason) {
  // Always (re-)assert outputs off, even if already stopped.
  bus_.disableAll();
  driver_.allOff();
  if (mode_ == Mode::EmergencyStop) return;

  motion_.resetTo(Posture::Sitting);
  mode_ = Mode::EmergencyStop;
  if (lifecycle_ == Lifecycle::Arming || lifecycle_ == Lifecycle::Stopping) lifecycle_ = Lifecycle::Active;
  std::snprintf(estopReason_, sizeof(estopReason_), "%s", reason);
  log_.log(LogLevel::Critical, LogCategory::Safety, "EMERGENCY STOP: %s (servos limp, hold PS 2 s to clear)", reason);
  log_.movement(mode_, "EMERGENCY STOP (%s)", reason);
  lastWalkDesc_[0] = '\0';
}

void Robot::clearEmergencyStop(CommandSource src, uint32_t now) {
  if (mode_ != Mode::EmergencyStop) {
    log_.log(LogLevel::Info, LogCategory::Safety, "Clear ignored: not in EMERGENCY_STOP");
    return;
  }
  log_.log(LogLevel::Info, LogCategory::Safety, "Emergency stop clear requested (%s)", toString(src));

  // The servo driver may have browned out: re-initialise and re-check it.
  if (!driver_.begin()) {
    setFault(Fault::ServoDriverInit, true);
    log_.log(LogLevel::Error, LogCategory::Safety, "Cannot clear: servo driver not responding");
    return;
  }
  setFault(Fault::ServoDriverInit, false);
  setFault(Fault::ServoDriverComm, false);
  setFault(Fault::ServoDriverReset, false);
  setFault(Fault::ServoReadback, false);
  consecutiveI2cErrors_ = 0;

  if (!canArm(true)) return;

  mode_ = Mode::Normal;
  estopReason_[0] = '\0';
  log_.log(LogLevel::Info, LogCategory::Mode, "EMERGENCY_STOP cleared -> NORMAL");
  log_.movement(mode_, "E-STOP cleared");
  startArming(now);
}

bool Robot::canArm(bool verbose) {
  const uint32_t critical = activeFaults_ & kCriticalFaultMask;
  if (critical) {
    if (verbose) {
      char list[96];
      faultList(critical, list, sizeof(list));
      log_.log(LogLevel::Error, LogCategory::Safety, "Cannot arm: critical fault(s) active: %s", list);
    }
    return false;
  }
  if (power_.available() && battery_.hasData() && battery_.voltage() < cfg_.battery.minStartVoltage) {
    if (verbose)
      log_.log(LogLevel::Error, LogCategory::Safety, "Cannot arm: battery %.2f V below start minimum %.2f V",
               battery_.voltage(), cfg_.battery.minStartVoltage);
    return false;
  }
  return true;
}

void Robot::arm(CommandSource src, uint32_t now) {
  if (mode_ == Mode::EmergencyStop) {
    log_.log(LogLevel::Warn, LogCategory::Safety, "Arm refused: clear the emergency stop first");
    return;
  }
  if (lifecycle_ != Lifecycle::WaitingForController && lifecycle_ != Lifecycle::Shutdown) {
    log_.log(LogLevel::Info, LogCategory::System, "Arm ignored while %s", toString(lifecycle_));
    return;
  }
  if (lifecycle_ == Lifecycle::Shutdown && !driver_.begin()) {
    setFault(Fault::ServoDriverInit, true);
    emergencyStop("servo driver re-init failed");
    return;
  }
  log_.log(LogLevel::Info, LogCategory::System, "Arm requested (%s)", toString(src));
  if (canArm(true)) startArming(now);
}

void Robot::startArming(uint32_t now) {
  lifecycle_ = Lifecycle::Arming;
  motion_.resetTo(Posture::Sitting);
  bus_.disableAll();
  computeServoTargets(now);
  armStep_ = 0;
  armNextMs_ = now;
  log_.log(LogLevel::Info, LogCategory::System, "Arming: enabling servos leg by leg into the sit pose");
}

void Robot::beginShutdown(StopReason reason, const char* why) {
  if (lifecycle_ == Lifecycle::Shutdown || lifecycle_ == Lifecycle::Stopping) return;

  const bool outputsLive =
      mode_ != Mode::EmergencyStop && (lifecycle_ == Lifecycle::Active || lifecycle_ == Lifecycle::Arming);
  if (!outputsLive || lifecycle_ == Lifecycle::Arming) {
    // Nothing is holding the robot up: switch off directly.
    bus_.disableAll();
    driver_.allOff();
    driver_.sleep();
    lifecycle_ = Lifecycle::Shutdown;
    log_.log(LogLevel::Info, LogCategory::System, "Shutdown (%s): outputs off, safe to power off", why);
    return;
  }

  if (mode_ == Mode::Calibration) {
    exitCalibration(true);
  } else {
    motion_.sit();
  }
  if (mode_ != Mode::Normal) {
    log_.log(LogLevel::Info, LogCategory::Mode, "Mode %s -> NORMAL (shutdown)", toString(mode_));
    mode_ = Mode::Normal;
  }
  stopReason_ = reason;
  stopStartMs_ = lastTickMs_;
  lifecycle_ = Lifecycle::Stopping;
  log_.log(reason == StopReason::User ? LogLevel::Info : LogLevel::Warn, LogCategory::System,
           "Shutting down (%s): sitting before disabling outputs", why);
  log_.movement(mode_, "SIT (shutdown: %s)", why);
}

void Robot::updateLifecycle(uint32_t now) {
  switch (lifecycle_) {
    case Lifecycle::WaitingForController:
      if (mode_ != Mode::EmergencyStop && controllerConnected_) {
        const bool verbose = reached(now, lastArmRefusalMs_);
        if (canArm(verbose)) {
          startArming(now);
        } else if (verbose) {
          lastArmRefusalMs_ = now + 5000;
        }
      }
      break;

    case Lifecycle::Arming:
      if (armStep_ < kLegCount) {
        if (reached(now, armNextMs_)) {
          bus_.setLegEnabled(armStep_, true);
          log_.log(LogLevel::Debug, LogCategory::System, "Leg %s enabled", legName(armStep_));
          ++armStep_;
          armNextMs_ = now + (armStep_ < kLegCount ? cfg_.timing.armStaggerMs : cfg_.timing.armSettleMs);
        }
      } else if (reached(now, armNextMs_)) {
        lifecycle_ = Lifecycle::Active;
        log_.log(LogLevel::Info, LogCategory::System, "Armed: sitting in %s mode", toString(mode_));
        log_.movement(mode_, "SIT (armed)");
      }
      break;

    case Lifecycle::Stopping: {
      const bool seated = !motion_.busy() && motion_.posture() == Posture::Sitting;
      if (seated || reached(now, stopStartMs_ + 4000)) {
        bus_.disableAll();
        driver_.allOff();
        if (stopReason_ == StopReason::BatteryCritical) {
          emergencyStop("battery critical");
        } else {
          driver_.sleep();
          lifecycle_ = Lifecycle::Shutdown;
          log_.log(LogLevel::Info, LogCategory::System,
                   "Shutdown complete: outputs off, safe to power off (hold PS or 'arm' to wake)");
          log_.movement(mode_, "SHUTDOWN");
        }
      }
      break;
    }

    default: break;
  }
}

// ----------------------------------------------------------- calibration --

void Robot::enterCalibration() {
  motion_.syncFeet(motion_.feet());
  for (int s = 0; s < kServoCount; ++s) {
    calFrom_[s] = bus_.target(s);
    calNominal_[s] = calFrom_[s];
  }
  calElapsedMs_ = 0.0f;
  wiggleElapsedMs_ = -1.0f;
  log_.log(LogLevel::Info, LogCategory::Calibration,
           "Calibration: joints move to %.0f deg + trim. L1/R1 select, stick up/down trim, X save",
           cfg_.geometry.servoCentreDeg);
  calSelect(calSelected_);
}

void Robot::exitCalibration(bool sitAfterwards) {
  FeetPositions feet;
  for (int leg = 0; leg < kLegCount; ++leg) {
    const float s[kJointsPerLeg] = {calNominal_[servoIndex(leg, 0)], calNominal_[servoIndex(leg, 1)],
                                    calNominal_[servoIndex(leg, 2)]};
    feet[leg] = kin_.servosToFoot(leg, s);
  }
  motion_.syncFeet(feet);
  if (sitAfterwards)
    motion_.sit();
  else
    motion_.stand();
  wiggleElapsedMs_ = -1.0f;
  if (calDirty_)
    log_.log(LogLevel::Warn, LogCategory::Calibration, "Calibration has unsaved changes (X or 'cal save')");
}

void Robot::calSelect(int servo) {
  calSelected_ = ((servo % kServoCount) + kServoCount) % kServoCount;
  wiggleElapsedMs_ = 0.0f;
  log_.log(LogLevel::Info, LogCategory::Calibration, "Selected servo %d (%s %s, ch %u) trim %.1f", calSelected_,
           legName(legOfServo(calSelected_)), jointName(jointOfServo(calSelected_)), bus_.channel(calSelected_),
           bus_.trim(calSelected_));
}

void Robot::calSetTrim(int servo, float trim) {
  bus_.setTrim(servo, trim);
  calDirty_ = true;
  log_.log(LogLevel::Info, LogCategory::Calibration, "Servo %d trim %.1f deg", servo, bus_.trim(servo));
}

void Robot::calSave() {
  CalibrationData d;
  for (int s = 0; s < kServoCount; ++s) d.trimDeg[s] = bus_.trim(s);
  if (calStore_.save(d)) {
    calDirty_ = false;
    log_.log(LogLevel::Info, LogCategory::Calibration, "Calibration saved");
  } else {
    log_.log(LogLevel::Error, LogCategory::Calibration, "Calibration save failed");
  }
}

void Robot::computeCalibrationTargets(float dt) {
  calElapsedMs_ += dt * 1000.0f;
  const float moveMs = cfg_.timing.calibrationMoveMs > 0 ? cfg_.timing.calibrationMoveMs : 1.0f;
  const float t = smoothstep(calElapsedMs_ / moveMs);
  float wiggle = 0.0f;
  if (wiggleElapsedMs_ >= 0.0f) {
    wiggleElapsedMs_ += dt * 1000.0f;
    const float w = wiggleElapsedMs_ / 600.0f;
    if (w >= 1.0f)
      wiggleElapsedMs_ = -1.0f;
    else
      wiggle = 12.0f * std::sin(2.0f * kPi * w);
  }
  for (int s = 0; s < kServoCount; ++s) {
    calNominal_[s] = lerpf(calFrom_[s], cfg_.geometry.servoCentreDeg, t);
    bus_.setTarget(s, calNominal_[s] + (s == calSelected_ ? wiggle : 0.0f));
  }
}

// ----------------------------------------------------------- supervision --

void Robot::setFault(Fault f, bool active) {
  const uint32_t b = faultBit(f);
  const bool was = (activeFaults_ & b) != 0;
  if (active == was) return;
  if (active) {
    activeFaults_ |= b;
    seenFaults_ |= b;
    log_.log(isCritical(f) ? LogLevel::Critical : LogLevel::Warn, LogCategory::Safety, "Fault raised: %s",
             toString(f));
  } else {
    activeFaults_ &= ~b;
    log_.log(LogLevel::Info, LogCategory::Safety, "Fault cleared: %s", toString(f));
  }
}

void Robot::raiseCritical(Fault f, const char* reason) {
  setFault(f, true);
  emergencyStop(reason);
}

void Robot::superviseController(const InputFrame& in, uint32_t now) {
  if (in.connected && !controllerConnected_) {
    controllerConnected_ = true;
    setFault(Fault::ControllerLost, false);
    log_.log(LogLevel::Info, LogCategory::Input, "Controller connected");
  } else if (!in.connected && controllerConnected_) {
    controllerConnected_ = false;
    controllerLostMs_ = now;
    setFault(Fault::ControllerLost, true);
    log_.log(LogLevel::Warn, LogCategory::Input, "Controller lost: stopping and sitting (outputs off in %u s)",
             static_cast<unsigned>(cfg_.safety.controllerLostShutdownMs / 1000));
    if (lifecycle_ == Lifecycle::Active && mode_ != Mode::EmergencyStop) {
      if (mode_ == Mode::Calibration) {
        exitCalibration(true);
      } else {
        motion_.sit();
      }
      if (mode_ != Mode::Normal) {
        log_.log(LogLevel::Info, LogCategory::Mode, "Mode %s -> NORMAL (controller lost)", toString(mode_));
        mode_ = Mode::Normal;
      }
      log_.movement(mode_, "SIT (controller lost)");
      lastWalkDesc_[0] = '\0';
    }
  }

  if (!controllerConnected_ && hasFault(Fault::ControllerLost) && lifecycle_ == Lifecycle::Active &&
      mode_ != Mode::EmergencyStop && reached(now, controllerLostMs_ + cfg_.safety.controllerLostShutdownMs)) {
    beginShutdown(StopReason::ControllerLost, "controller lost timeout");
  }
}

void Robot::superviseBattery(uint32_t now) {
  if (!power_.available()) return;
  if (!reached(now, lastBatterySampleMs_ + cfg_.battery.samplePeriodMs)) return;
  lastBatterySampleMs_ = now;

  float v = 0.0f;
  if (power_.readVoltage(v))
    battery_.addSample(v, now);
  else
    battery_.markReadError(now);

  const BatteryStatus st = battery_.status();
  if (st == lastBatteryStatus_) return;
  log_.log(st == BatteryStatus::Ok ? LogLevel::Info : LogLevel::Warn, LogCategory::Safety,
           "Battery %s -> %s (%.2f V, ripple %.2f V)", toString(lastBatteryStatus_), toString(st), battery_.voltage(),
           battery_.ripple());
  lastBatteryStatus_ = st;

  setFault(Fault::BatteryLow, st == BatteryStatus::Low);
  setFault(Fault::BatterySensor, st == BatteryStatus::SensorFault);
  setFault(Fault::BatteryCritical, st == BatteryStatus::Critical);
  setFault(Fault::BatteryOvervoltage, st == BatteryStatus::Overvoltage);
  setFault(Fault::BatteryUnstable, st == BatteryStatus::Unstable);

  switch (st) {
    case BatteryStatus::Critical:
      // Controlled stop: sit down first, then go limp (EMERGENCY_STOP).
      if (lifecycle_ == Lifecycle::Active && mode_ != Mode::EmergencyStop)
        beginShutdown(StopReason::BatteryCritical, "battery critical");
      else if (lifecycle_ == Lifecycle::Arming)
        emergencyStop("battery critical");
      break;
    case BatteryStatus::Overvoltage: emergencyStop("battery over-voltage"); break;
    case BatteryStatus::Unstable: emergencyStop("irregular battery power"); break;
    default: break;
  }
}

void Robot::superviseDriver(uint32_t now, bool flushOk) {
  if (!flushOk) {
    if (++consecutiveI2cErrors_ >= cfg_.safety.maxConsecutiveI2cErrors) {
      consecutiveI2cErrors_ = cfg_.safety.maxConsecutiveI2cErrors;
      raiseCritical(Fault::ServoDriverComm, "servo driver I2C failure");
    }
  } else {
    consecutiveI2cErrors_ = 0;
  }

  if (!bus_.anyEnabled() || !reached(now, lastVerifyMs_ + cfg_.safety.driverVerifyPeriodMs)) return;
  lastVerifyMs_ = now;

  if (!driver_.verify()) {
    raiseCritical(Fault::ServoDriverReset, "servo driver lost its configuration (brown-out?)");
    return;
  }
  // Round-robin readback of one enabled channel per period.
  for (int k = 0; k < kServoCount; ++k) {
    const int s = (readbackServo_ + k) % kServoCount;
    if (!bus_.enabled(s)) continue;
    readbackServo_ = (s + 1) % kServoCount;
    float deg = 0.0f;
    bool off = false;
    if (!driver_.readChannel(bus_.channel(s), deg, off)) {
      raiseCritical(Fault::ServoDriverComm, "servo driver readback failed");
    } else if (off || std::fabs(deg - bus_.output(s)) > 1.5f) {
      char msg[56];
      std::snprintf(msg, sizeof(msg), "servo %d readback %.1f != %.1f deg", s, off ? -1.0f : deg, bus_.output(s));
      raiseCritical(Fault::ServoReadback, msg);
    }
    break;
  }
}

// --------------------------------------------------------------- outputs --

bool Robot::outputsAllowed() const {
  return mode_ != Mode::EmergencyStop && (lifecycle_ == Lifecycle::Arming || lifecycle_ == Lifecycle::Active ||
                                          lifecycle_ == Lifecycle::Stopping);
}

void Robot::computeServoTargets(uint32_t now) {
  const FeetPositions& feet = motion_.feet();
  bool failed = false;
  for (int leg = 0; leg < kLegCount; ++leg) {
    float s[kJointsPerLeg];
    if (!kin_.footToServos(leg, feet[leg], s)) {
      failed = true;  // keep this leg's previous targets
      continue;
    }
    for (int j = 0; j < kJointsPerLeg; ++j) bus_.setTarget(servoIndex(leg, j), s[j]);
  }
  if (failed) {
    ++ikFailures_;
    lastKinFailMs_ = now;
    if (!hasFault(Fault::Kinematics)) {
      setFault(Fault::Kinematics, true);
      log_.log(LogLevel::Warn, LogCategory::Motion, "Unreachable foot target - holding last joint angles");
    }
  } else if (hasFault(Fault::Kinematics) && reached(now, lastKinFailMs_ + 2000)) {
    setFault(Fault::Kinematics, false);
  }
}

void Robot::updateOutputs(uint32_t now, float dt) {
  if (outputsAllowed()) {
    if (mode_ == Mode::Calibration && lifecycle_ == Lifecycle::Active) {
      computeCalibrationTargets(dt);
    } else {
      motion_.update(dt);
      computeServoTargets(now);
    }
  } else {
    bus_.disableAll();
  }
  superviseDriver(now, bus_.flush(dt));
}

void Robot::publishSnapshot(uint32_t now, const InputFrame& in) {
  RobotSnapshot s;
  s.timeMs = now;
  s.lifecycle = lifecycle_;
  s.mode = mode_;
  s.posture = motion_.posture();
  s.motion = motion_.state();
  s.gaitIndex = motion_.gaitIndex();
  s.gaitName = motion_.gait().name;
  s.danceIndex = danceIndex_;
  s.danceName = animations::dance(danceIndex_).name;
  s.animationName = motion_.animation() ? motion_.animation()->name : nullptr;
  s.animationLoop = motion_.animationLooping();
  s.walk = motion_.walkCommand();
  s.feet = motion_.feet();
  for (int i = 0; i < kServoCount; ++i) {
    s.servoDeg[i] = bus_.output(i);
    s.servoTarget[i] = bus_.target(i);
    s.trim[i] = bus_.trim(i);
  }
  s.outputsEnabled = bus_.anyEnabled();
  s.calSelected = calSelected_;
  s.calDirty = calDirty_;
  s.ikLeg = motion_.ikActive() ? motion_.ikLeg() : -1;
  s.ikOffset = motion_.ikOffset();
  s.activeFaults = activeFaults_;
  s.seenFaults = seenFaults_;
  std::memcpy(s.estopReason, estopReason_, sizeof(s.estopReason));
  s.batteryEnabled = power_.available();
  s.batteryStatus = power_.available() ? battery_.status() : BatteryStatus::Unavailable;
  s.batteryVoltage = battery_.voltage();
  s.batteryRipple = battery_.ripple();
  s.batteryPercent = battery_.percent();
  s.controllerConnected = in.connected;
  s.ticks = ticks_;
  s.overruns = overruns_;
  s.periodAvgMs = periodAvgUs_ / 1000.0f;
  s.execAvgUs = execAvgUs_;
  s.execMaxUs = execMaxUs_;
  s.i2cErrors = driver_.errorCount();
  s.clampEvents = bus_.clampEvents();
  s.ikFailures = ikFailures_ + motion_.ikRejections();
  s.warnings = log_.warnings();
  s.errors = log_.errors();

  std::lock_guard<std::mutex> lock(snapshotMutex_);
  snapshot_ = s;
}

RobotSnapshot Robot::snapshot() const {
  std::lock_guard<std::mutex> lock(snapshotMutex_);
  return snapshot_;
}

}  // namespace qspider
