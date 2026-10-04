#include "Ps5Gamepad.h"

#include <Arduino.h>
#include <ps5Controller.h>

namespace qspider {

std::atomic<uint32_t> Ps5Gamepad::lastPacketMs_{0};

void Ps5Gamepad::onPacket() { lastPacketMs_.store(millis()); }

bool Ps5Gamepad::begin() {
  ps5.attach(&Ps5Gamepad::onPacket);
  return ps5.begin(cfg_.macAddress);
}

GamepadState Ps5Gamepad::read() {
  GamepadState s;
  s.connected = ps5.isConnected();
  if (!s.connected) return s;
  s.lastPacketMs = lastPacketMs_.load();

  uint32_t b = 0;
  auto set = [&b](bool pressed, Button btn) {
    if (pressed) b |= buttonBit(btn);
  };
  // Diagonal D-pad positions count as both directions so that a slightly
  // diagonal press of D-pad left still triggers the emergency stop.
  set(ps5.Up() || ps5.UpLeft() || ps5.UpRight(), Button::Up);
  set(ps5.Down() || ps5.DownLeft() || ps5.DownRight(), Button::Down);
  set(ps5.Left() || ps5.UpLeft() || ps5.DownLeft(), Button::Left);
  set(ps5.Right() || ps5.UpRight() || ps5.DownRight(), Button::Right);
  set(ps5.Cross(), Button::Cross);
  set(ps5.Circle(), Button::Circle);
  set(ps5.Square(), Button::Square);
  set(ps5.Triangle(), Button::Triangle);
  set(ps5.L1(), Button::L1);
  set(ps5.R1(), Button::R1);
  set(ps5.L2(), Button::L2);
  set(ps5.R2(), Button::R2);
  set(ps5.L3(), Button::L3);
  set(ps5.R3(), Button::R3);
  set(ps5.Share(), Button::Create);
  set(ps5.Options(), Button::Options);
  set(ps5.PSButton(), Button::PlayStation);
  set(ps5.Touchpad(), Button::Touchpad);
  s.buttons = b;

  auto axis = [](int8_t v) {
    const float f = v / 127.0f;
    return f > 1.0f ? 1.0f : (f < -1.0f ? -1.0f : f);
  };
  s.lx = axis(ps5.LStickX());
  s.ly = axis(ps5.LStickY()) * (cfg_.invertLeftY ? -1.0f : 1.0f);  // library reports up as positive
  s.rx = axis(ps5.RStickX());
  s.ry = axis(ps5.RStickY()) * (cfg_.invertRightY ? -1.0f : 1.0f);
  return s;
}

void Ps5Gamepad::setLightbar(uint8_t r, uint8_t g, uint8_t b) {
  if (!ps5.isConnected()) return;
  ps5.setLed(r, g, b);
  ps5.sendToController();
}

void Ps5Gamepad::rumble(uint8_t strength, uint16_t ms) {
  if (!ps5.isConnected()) return;
  ps5.setRumble(strength, strength);
  ps5.sendToController();
  rumbleOffMs_ = millis() + ms;
  rumbling_ = true;
}

void Ps5Gamepad::service(uint32_t nowMs) {
  if (rumbling_ && static_cast<int32_t>(nowMs - rumbleOffMs_) >= 0) {
    rumbling_ = false;
    if (!ps5.isConnected()) return;
    ps5.setRumble(0, 0);
    ps5.sendToController();
  }
}

}  // namespace qspider
