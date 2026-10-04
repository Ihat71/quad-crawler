#pragma once

#include <cstdint>

#include "DashboardServer.h"
#include "Ps5Gamepad.h"
#include "SerialConsole.h"
#include "config/RobotConfig.h"
#include "robot/Robot.h"

namespace qspider {

// FreeRTOS wiring:
//   control task  - core 1, high priority, fixed period (vTaskDelayUntil),
//                   watchdog-supervised: input -> safety -> motion -> servos.
//   service task  - core 0 (with WiFi/Bluetooth), low priority: serial log +
//                   console, dashboard, controller lightbar/rumble feedback.
// The tasks only share the Robot's thread-safe submit()/snapshot() and the
// lock-protected EventLog, so logging/dashboard load cannot delay servo updates.
class RobotRuntime {
 public:
  RobotRuntime(Robot& robot, const RobotConfig& cfg, SerialConsole& console, DashboardServer& dashboard,
               Ps5Gamepad& gamepad);
  void start();

 private:
  static void controlTaskEntry(void* self);
  static void serviceTaskEntry(void* self);
  void controlLoop();
  void serviceLoop();
  void updateControllerFeedback(uint32_t nowMs);

  Robot& robot_;
  const RobotConfig& cfg_;
  SerialConsole& console_;
  DashboardServer& dashboard_;
  Ps5Gamepad& gamepad_;

  int lastColour_ = -1;
  bool lastConnected_ = false;
  bool lastEstop_ = false;
};

}  // namespace qspider
