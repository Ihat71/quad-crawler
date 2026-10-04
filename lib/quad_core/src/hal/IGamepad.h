#pragma once

#include <cstdint>

#include "input/Buttons.h"

namespace qspider {

struct GamepadState {
  bool connected = false;
  uint32_t buttons = 0;        // buttonBit(Button::X) set while pressed
  float lx = 0, ly = 0;        // -1..1, ly > 0 = stick pushed forward
  float rx = 0, ry = 0;
  uint32_t lastPacketMs = 0;   // time of the latest report from the controller
};

class IGamepad {
 public:
  virtual ~IGamepad() = default;
  virtual GamepadState read() = 0;
  // Optional feedback; implementations may ignore it.
  virtual void setLightbar(uint8_t /*r*/, uint8_t /*g*/, uint8_t /*b*/) {}
  virtual void rumble(uint8_t /*strength*/, uint16_t /*ms*/) {}
};

}  // namespace qspider
