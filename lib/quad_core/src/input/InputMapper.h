#pragma once

#include <cstdint>

#include "config/RobotConfig.h"
#include "hal/IGamepad.h"

namespace qspider {

// One control tick worth of processed controller input.
struct InputFrame {
  bool connected = false;  // connected AND receiving fresh reports
  uint32_t held = 0;
  uint32_t pressed = 0;      // went down this tick
  uint32_t released = 0;     // went up this tick
  uint32_t longPressed = 0;  // held for InputConfig::holdMs (fires once per press)
  float lx = 0, ly = 0, rx = 0, ry = 0;  // after deadzone, -1..1
  float leftMagnitude = 0;
  float rightMagnitude = 0;

  bool isPressed(Button b) const { return (pressed & buttonBit(b)) != 0; }
  bool isHeld(Button b) const { return (held & buttonBit(b)) != 0; }
  bool isLongPress(Button b) const { return (longPressed & buttonBit(b)) != 0; }
};

// Turns raw gamepad state into edges, long presses and deadzone-filtered sticks.
class InputMapper {
 public:
  explicit InputMapper(const InputConfig& cfg);
  InputFrame update(const GamepadState& gp, uint32_t nowMs);

 private:
  void applyDeadzone(float x, float y, float& ox, float& oy, float& mag) const;

  const InputConfig& cfg_;
  uint32_t prev_ = 0;
  uint32_t holdFired_ = 0;
  uint32_t pressStartMs_[kButtonCount] = {};
};

}  // namespace qspider
