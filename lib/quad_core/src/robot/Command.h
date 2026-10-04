#pragma once

#include <cstdint>

namespace qspider {

enum class CommandType : uint8_t {
  EmergencyStop,
  ClearEmergencyStop,
  Arm,        // enable outputs (from WAITING_FOR_CONTROLLER or SHUTDOWN)
  Shutdown,   // controlled sit-down, then outputs off
  SetMode,    // i = Mode
  Sit,
  Stand,
  ToggleSitStand,
  Dance,      // i = dance index, -1 = currently selected; f > 0 = loop
  StopMotion,
  Attack,
  NextGait,
  SetGait,    // i = gait index
  NextDance,
  ResetDefaults,
  CalSelect,  // i = servo
  CalSetTrim, // i = servo, f = trim deg
  CalNudge,   // i = servo (-1 = selected), f = delta deg
  CalSave,
  CalResetTrim,  // i = servo, -1 = all
  SetLogLevel,   // i = LogLevel
};

enum class CommandSource : uint8_t { Controller, Dashboard, Serial, Internal, Test };

struct Command {
  CommandType type = CommandType::StopMotion;
  CommandSource source = CommandSource::Internal;
  int32_t i = 0;
  float f = 0.0f;
};

const char* toString(CommandType t);
const char* toString(CommandSource s);

}  // namespace qspider
