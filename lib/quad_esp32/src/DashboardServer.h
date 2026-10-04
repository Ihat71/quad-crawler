#pragma once

#include <WebServer.h>

#include "config/RobotConfig.h"
#include "robot/Robot.h"
#include "telemetry/EventLog.h"

namespace qspider {

// WiFi + HTTP dashboard. Joins the configured network (non-blocking) and falls
// back to its own access point. The page polls GET /api/state and sends text
// commands to POST /api/cmd. Runs in the low-priority service task on core 0.
class DashboardServer {
 public:
  DashboardServer(Robot& robot, EventLog& log, const RobotConfig& cfg, const char* firmware);

  void begin();
  void poll();
  const char* ip() const { return ip_; }

 private:
  enum class NetState : uint8_t { Off, Connecting, Station, AccessPoint };

  void startAccessPoint();
  void startHttp();
  void handleIndex();
  void handleState();
  void handleCommand();

  Robot& robot_;
  EventLog& log_;
  const RobotConfig& cfg_;
  const char* firmware_;
  WebServer server_;
  NetState state_ = NetState::Off;
  uint32_t connectStartMs_ = 0;
  bool httpStarted_ = false;
  char ip_[16] = "0.0.0.0";
};

}  // namespace qspider
