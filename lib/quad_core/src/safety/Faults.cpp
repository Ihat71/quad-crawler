#include "safety/Faults.h"

namespace qspider {

const char* toString(Fault f) {
  switch (f) {
    case Fault::ServoDriverInit: return "SERVO_DRIVER_INIT";
    case Fault::ServoDriverComm: return "SERVO_DRIVER_COMM";
    case Fault::ServoDriverReset: return "SERVO_DRIVER_RESET";
    case Fault::ServoReadback: return "SERVO_READBACK";
    case Fault::BatteryCritical: return "BATTERY_CRITICAL";
    case Fault::BatteryOvervoltage: return "BATTERY_OVERVOLTAGE";
    case Fault::BatteryUnstable: return "BATTERY_UNSTABLE";
    case Fault::ConfigInvalid: return "CONFIG_INVALID";
    case Fault::BatteryLow: return "BATTERY_LOW";
    case Fault::BatterySensor: return "BATTERY_SENSOR";
    case Fault::Kinematics: return "KINEMATICS";
    case Fault::ControllerLost: return "CONTROLLER_LOST";
    case Fault::LoopOverrun: return "LOOP_OVERRUN";
    case Fault::Count: break;
  }
  return "?";
}

bool isCritical(Fault f) { return (kCriticalFaultMask & faultBit(f)) != 0; }

}  // namespace qspider
