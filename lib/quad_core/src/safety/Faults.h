#pragma once

#include <cstdint>

namespace qspider {

// Fault flags. Critical faults force EMERGENCY_STOP; warnings are reported only.
enum class Fault : uint8_t {
  ServoDriverInit = 0,   // PCA9685 not found at boot / re-init failed
  ServoDriverComm,       // repeated I2C write failures
  ServoDriverReset,      // chip lost its configuration (servo rail brown-out?)
  ServoReadback,         // a channel does not hold the value we wrote
  BatteryCritical,       // sustained under-voltage
  BatteryOvervoltage,
  BatteryUnstable,       // voltage swings beyond the ripple limit ("irregular power")
  ConfigInvalid,
  BatteryLow,            // warning
  BatterySensor,         // warning: implausible reading, protection degraded
  Kinematics,            // warning: unreachable target / NaN, last pose held
  ControllerLost,        // warning: robot sits down, shuts down after a timeout
  LoopOverrun,           // warning: control loop missed its deadline
  Count
};

constexpr int kFaultCount = static_cast<int>(Fault::Count);
constexpr uint32_t faultBit(Fault f) { return 1u << static_cast<uint8_t>(f); }

const char* toString(Fault f);
bool isCritical(Fault f);

constexpr uint32_t kCriticalFaultMask =
    faultBit(Fault::ServoDriverInit) | faultBit(Fault::ServoDriverComm) | faultBit(Fault::ServoDriverReset) |
    faultBit(Fault::ServoReadback) | faultBit(Fault::BatteryCritical) | faultBit(Fault::BatteryOvervoltage) |
    faultBit(Fault::BatteryUnstable) | faultBit(Fault::ConfigInvalid);

}  // namespace qspider
