#pragma once

#include <Arduino.h>

#include "config/RobotConfig.h"
#include "robot/Robot.h"
#include "telemetry/EventLog.h"
#include "telemetry/TelemetryJson.h"

namespace qspider {

// USB serial sink for the event / movement / button logs plus a line-based
// command console (same language as the dashboard, type "help").
// Runs in the low-priority service task, never in the control loop.
class SerialConsole {
 public:
  SerialConsole(Robot& robot, EventLog& log, const RobotConfig& cfg, Stream& io);
  void poll();

 private:
  void printNewEntries();
  void handleLine(const char* line);
  void printStatus();

  Robot& robot_;
  EventLog& log_;
  const RobotConfig& cfg_;
  Stream& io_;
  char line_[96] = {};
  size_t len_ = 0;
  TelemetryCursor cursor_;
};

}  // namespace qspider
