#pragma once

#include <atomic>
#include <cstdint>

#include "config/RobotConfig.h"
#include "hal/IGamepad.h"

namespace qspider {

// DualSense (PS5) controller over Bluetooth Classic via the ps5-esp32 library.
// read() is called from the control task; lightbar/rumble from the service task.
class Ps5Gamepad : public IGamepad {
 public:
  explicit Ps5Gamepad(const ControllerHwConfig& cfg) : cfg_(cfg) {}

  bool begin();
  GamepadState read() override;
  void setLightbar(uint8_t r, uint8_t g, uint8_t b) override;
  void rumble(uint8_t strength, uint16_t ms) override;
  void service(uint32_t nowMs);  // ends rumble pulses

 private:
  static void onPacket();
  static std::atomic<uint32_t> lastPacketMs_;

  const ControllerHwConfig& cfg_;
  uint32_t rumbleOffMs_ = 0;
  bool rumbling_ = false;
};

}  // namespace qspider
