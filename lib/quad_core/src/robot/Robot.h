#pragma once

#include <cstdint>
#include <mutex>

#include "common/Types.h"
#include "config/RobotConfig.h"
#include "hal/ICalibrationStore.h"
#include "hal/IGamepad.h"
#include "hal/IPowerSensor.h"
#include "hal/IServoDriver.h"
#include "input/InputMapper.h"
#include "kinematics/BodyKinematics.h"
#include "motion/MotionController.h"
#include "robot/Command.h"
#include "safety/BatteryMonitor.h"
#include "safety/Faults.h"
#include "servo/ServoBus.h"
#include "telemetry/EventLog.h"

namespace qspider {

// Everything the dashboard / serial status needs, copied once per tick.
struct RobotSnapshot {
  uint32_t timeMs = 0;
  Lifecycle lifecycle = Lifecycle::Booting;
  Mode mode = Mode::Normal;
  Posture posture = Posture::Sitting;
  MotionState motion = MotionState::Idle;
  int gaitIndex = 0;
  const char* gaitName = "";
  int danceIndex = 0;
  const char* danceName = "";
  const char* animationName = nullptr;  // currently playing, if any
  bool animationLoop = false;
  WalkCommand walk;
  FeetPositions feet;
  float servoDeg[kServoCount] = {};     // NaN when the output is off
  float servoTarget[kServoCount] = {};  // nominal angle before trim
  float trim[kServoCount] = {};
  bool outputsEnabled = false;
  int calSelected = 0;
  bool calDirty = false;
  int ikLeg = -1;
  Vec3 ikOffset;
  uint32_t activeFaults = 0;
  uint32_t seenFaults = 0;
  char estopReason[56] = {};
  bool batteryEnabled = false;
  BatteryStatus batteryStatus = BatteryStatus::Unavailable;
  float batteryVoltage = 0.0f;
  float batteryRipple = 0.0f;
  int batteryPercent = -1;
  bool controllerConnected = false;
  uint32_t ticks = 0;
  uint32_t overruns = 0;
  float periodAvgMs = 0.0f;
  float execAvgUs = 0.0f;
  uint32_t execMaxUs = 0;
  uint32_t i2cErrors = 0;
  uint32_t clampEvents = 0;
  uint32_t ikFailures = 0;
  uint32_t warnings = 0;
  uint32_t errors = 0;
};

// Composition of all robot logic. The platform layer calls begin() once and
// tick() at a fixed rate from the control task; other tasks interact only
// through submit() and snapshot(), which are thread-safe.
class Robot {
 public:
  Robot(const RobotConfig& cfg, IServoDriver& driver, IGamepad& gamepad, IPowerSensor& power,
        ICalibrationStore& calStore, EventLog& log);

  void begin(uint32_t nowMs);
  void tick(uint32_t nowMs);

  bool submit(const Command& cmd);  // thread-safe, executed on the next tick
  RobotSnapshot snapshot() const;   // thread-safe
  void reportTiming(uint32_t periodUs, uint32_t execUs);  // from the control task

  // Control-thread accessors (tests, diagnostics).
  Mode mode() const { return mode_; }
  Lifecycle lifecycle() const { return lifecycle_; }
  bool hasFault(Fault f) const { return (activeFaults_ & faultBit(f)) != 0; }
  uint32_t activeFaults() const { return activeFaults_; }
  const MotionController& motion() const { return motion_; }
  const ServoBus& servos() const { return bus_; }
  const BodyKinematics& kinematics() const { return kin_; }
  int selectedDance() const { return danceIndex_; }
  int calSelected() const { return calSelected_; }

 private:
  enum class StopReason : uint8_t { User, ControllerLost, BatteryCritical };

  // command handling
  void drainInbox(uint32_t now);
  void execute(const Command& c, uint32_t now);
  void handleInput(const InputFrame& in, uint32_t now, float dt);
  void handleNormal(const InputFrame& in, float dt);
  void handleIk(const InputFrame& in, float dt);
  void handleDance(const InputFrame& in);
  void handleCalibration(const InputFrame& in, uint32_t now);
  void logButtons(const InputFrame& in);

  // modes & lifecycle
  void setMode(Mode m, CommandSource src);
  void cycleMode(CommandSource src);
  void emergencyStop(const char* reason);
  void clearEmergencyStop(CommandSource src, uint32_t now);
  bool canArm(bool verbose);
  void arm(CommandSource src, uint32_t now);
  void startArming(uint32_t now);
  void beginShutdown(StopReason reason, const char* why);
  void updateLifecycle(uint32_t now);

  // motion helpers
  void playDance(int index, bool loop, CommandSource src);
  void attack(CommandSource src);
  void resetDefaults(CommandSource src);
  void nextGait(CommandSource src);
  WalkCommand sticksToWalk(const InputFrame& in) const;
  void logWalk(const WalkCommand& w);

  // calibration
  void enterCalibration();
  void exitCalibration(bool sitAfterwards);
  void calSelect(int servo);
  void calSetTrim(int servo, float trim);
  void calSave();

  // supervision
  void superviseController(const InputFrame& in, uint32_t now);
  void superviseBattery(uint32_t now);
  void superviseDriver(uint32_t now, bool flushOk);
  void setFault(Fault f, bool active);
  void raiseCritical(Fault f, const char* reason);

  // outputs
  bool outputsAllowed() const;
  void computeServoTargets(uint32_t now);
  void computeCalibrationTargets(float dt);
  void updateOutputs(uint32_t now, float dt);
  void publishSnapshot(uint32_t now, const InputFrame& in);

  const RobotConfig& cfg_;
  IServoDriver& driver_;
  IGamepad& gamepad_;
  IPowerSensor& power_;
  ICalibrationStore& calStore_;
  EventLog& log_;

  BodyKinematics kin_;
  MotionController motion_;
  ServoBus bus_;
  BatteryMonitor battery_;
  InputMapper input_;

  Lifecycle lifecycle_ = Lifecycle::Booting;
  Mode mode_ = Mode::Normal;
  uint32_t activeFaults_ = 0;
  uint32_t seenFaults_ = 0;
  char estopReason_[56] = {};

  // lifecycle sequencing
  int armStep_ = 0;
  uint32_t armNextMs_ = 0;
  StopReason stopReason_ = StopReason::User;
  uint32_t stopStartMs_ = 0;
  uint32_t lastArmRefusalMs_ = 0;

  // controller supervision
  bool controllerConnected_ = false;
  uint32_t controllerLostMs_ = 0;

  // driver supervision
  uint8_t consecutiveI2cErrors_ = 0;
  uint32_t lastVerifyMs_ = 0;
  int readbackServo_ = 0;

  // battery
  uint32_t lastBatterySampleMs_ = 0;
  BatteryStatus lastBatteryStatus_ = BatteryStatus::Unavailable;

  // motion / modes
  int danceIndex_ = 0;
  char lastWalkDesc_[48] = {};
  bool ikMoving_ = false;
  uint32_t ikFailures_ = 0;
  uint32_t lastKinFailMs_ = 0;
  uint32_t lastIkRejectLogMs_ = 0;

  // calibration
  int calSelected_ = 0;
  bool calDirty_ = false;
  float calFrom_[kServoCount] = {};
  float calNominal_[kServoCount] = {};
  float calElapsedMs_ = 0.0f;
  float wiggleElapsedMs_ = -1.0f;  // < 0 = no wiggle running
  uint32_t calNudgeNextMs_ = 0;

  // timing
  uint32_t lastTickMs_ = 0;
  bool firstTick_ = true;
  uint32_t ticks_ = 0;
  uint32_t overruns_ = 0;
  uint32_t overrunWindowStartMs_ = 0;
  uint32_t overrunsInWindow_ = 0;
  uint32_t lastOverrunMs_ = 0;
  float periodAvgUs_ = 0.0f;
  float execAvgUs_ = 0.0f;
  uint32_t execMaxUs_ = 0;

  // cross-thread
  mutable std::mutex inboxMutex_;
  static constexpr int kInboxSize = 16;
  Command inbox_[kInboxSize];
  int inboxHead_ = 0;
  int inboxCount_ = 0;

  mutable std::mutex snapshotMutex_;
  RobotSnapshot snapshot_;
};

}  // namespace qspider
