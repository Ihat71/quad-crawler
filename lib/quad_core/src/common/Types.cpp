#include "common/Types.h"

namespace qspider {

const char* legName(int leg) {
  static const char* const kNames[kLegCount] = {"FR", "FL", "RL", "RR"};
  return (leg >= 0 && leg < kLegCount) ? kNames[leg] : "??";
}

const char* jointName(int joint) {
  static const char* const kNames[kJointsPerLeg] = {"coxa", "femur", "tibia"};
  return (joint >= 0 && joint < kJointsPerLeg) ? kNames[joint] : "??";
}

const char* toString(Mode m) {
  switch (m) {
    case Mode::Normal: return "NORMAL";
    case Mode::IK: return "IK";
    case Mode::Dance: return "DANCE";
    case Mode::Calibration: return "CALIBRATION";
    case Mode::EmergencyStop: return "EMERGENCY_STOP";
  }
  return "?";
}

const char* toString(Lifecycle l) {
  switch (l) {
    case Lifecycle::Booting: return "BOOTING";
    case Lifecycle::SelfTest: return "SELF_TEST";
    case Lifecycle::WaitingForController: return "WAITING_FOR_CONTROLLER";
    case Lifecycle::Arming: return "ARMING";
    case Lifecycle::Active: return "ACTIVE";
    case Lifecycle::Stopping: return "STOPPING";
    case Lifecycle::Shutdown: return "SHUTDOWN";
  }
  return "?";
}

const char* toString(Posture p) {
  switch (p) {
    case Posture::Sitting: return "SITTING";
    case Posture::Standing: return "STANDING";
    case Posture::Custom: return "CUSTOM";
  }
  return "?";
}

}  // namespace qspider
