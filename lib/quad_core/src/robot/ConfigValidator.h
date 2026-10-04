#pragma once

#include "config/RobotConfig.h"
#include "kinematics/BodyKinematics.h"
#include "telemetry/EventLog.h"

namespace qspider {

struct ConfigReport {
  int errors = 0;    // the robot must not move with this configuration
  int warnings = 0;  // works, but something will be clamped or degraded
};

// Static sanity checks run during the boot self-test (and by the unit tests):
// channel map, geometry, gait parameters, battery thresholds and whether every
// posture / gait extreme is reachable inside the servo limits.
ConfigReport validateConfig(const RobotConfig& cfg, const BodyKinematics& kin, EventLog& log);

}  // namespace qspider
