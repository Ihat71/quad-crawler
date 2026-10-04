#pragma once

#include <cstdint>

namespace qspider {

// Logical gamepad buttons. Values are bit positions in GamepadState::buttons.
enum class Button : uint8_t {
  Up = 0,
  Down,
  Left,
  Right,
  Cross,
  Circle,
  Square,
  Triangle,
  L1,
  R1,
  L2,
  R2,
  L3,
  R3,
  Create,
  Options,
  PlayStation,
  Touchpad,
  Mute,
  Count
};

constexpr int kButtonCount = static_cast<int>(Button::Count);

constexpr uint32_t buttonBit(Button b) { return 1u << static_cast<uint8_t>(b); }

const char* toString(Button b);

}  // namespace qspider
