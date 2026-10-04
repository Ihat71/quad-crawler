#include "input/InputMapper.h"

#include <cmath>

#include "common/MathUtil.h"

namespace qspider {

InputMapper::InputMapper(const InputConfig& cfg) : cfg_(cfg) {}

void InputMapper::applyDeadzone(float x, float y, float& ox, float& oy, float& mag) const {
  const float m = std::sqrt(x * x + y * y);
  const float dz = clampf(cfg_.stickDeadzone, 0.0f, 0.95f);
  if (m <= dz || !std::isfinite(m)) {
    ox = oy = mag = 0.0f;
    return;
  }
  mag = (std::fmin(m, 1.0f) - dz) / (1.0f - dz);
  ox = x / m * mag;
  oy = y / m * mag;
}

InputFrame InputMapper::update(const GamepadState& gp, uint32_t nowMs) {
  InputFrame f;
  const int32_t age = static_cast<int32_t>(nowMs - gp.lastPacketMs);
  f.connected = gp.connected && (cfg_.staleTimeoutMs == 0 || age <= static_cast<int32_t>(cfg_.staleTimeoutMs));

  const uint32_t buttons = f.connected ? gp.buttons : 0u;
  f.held = buttons;
  f.pressed = buttons & ~prev_;
  f.released = prev_ & ~buttons;

  for (int i = 0; i < kButtonCount; ++i) {
    const uint32_t b = 1u << i;
    if (f.pressed & b) {
      pressStartMs_[i] = nowMs;
      holdFired_ &= ~b;
    }
    if ((buttons & b) && !(holdFired_ & b) && nowMs - pressStartMs_[i] >= cfg_.holdMs) {
      f.longPressed |= b;
      holdFired_ |= b;
    }
    if (!(buttons & b)) holdFired_ &= ~b;
  }
  prev_ = buttons;

  if (f.connected) {
    applyDeadzone(gp.lx, gp.ly, f.lx, f.ly, f.leftMagnitude);
    applyDeadzone(gp.rx, gp.ry, f.rx, f.ry, f.rightMagnitude);
  }
  return f;
}

}  // namespace qspider
