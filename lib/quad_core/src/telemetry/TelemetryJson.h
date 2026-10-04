#pragma once

#include <cstdint>
#include <string>

#include "robot/Robot.h"
#include "telemetry/EventLog.h"

namespace qspider {

// Platform information added by the runtime (heap, network ...).
struct PlatformInfo {
  uint32_t uptimeMs = 0;
  uint32_t freeHeap = 0;
  uint32_t minFreeHeap = 0;
  int wifiRssi = 0;
  const char* ip = "";
  const char* firmware = "";
};

struct TelemetryCursor {
  uint32_t logSeq = 0;
  uint32_t moveSeq = 0;
  uint32_t buttonSeq = 0;
};

// Serialises the snapshot plus any log / movement / button entries newer than
// the cursor (so a polling client only receives what it has not seen yet).
void writeTelemetryJson(std::string& out, const RobotConfig& cfg, const RobotSnapshot& s, const EventLog& log,
                        const TelemetryCursor& since, const PlatformInfo& platform, size_t maxEntries = 40);

}  // namespace qspider
