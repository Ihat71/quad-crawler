#pragma once
// Hardware abstraction for the PWM servo driver (PCA9685 on the real robot).

#include <cstdint>

namespace qspider {

class IServoDriver {
 public:
  static constexpr int kChannels = 16;

  virtual ~IServoDriver() = default;

  // Initialise the chip with every output OFF. Safe to call again to recover.
  virtual bool begin() = 0;

  // Writes all 16 channels in one burst. NaN angle = output fully off (limp).
  virtual bool writeAll(const float (&angleDeg)[kChannels]) = 0;

  // Forces every output off (used for emergency stop).
  virtual bool allOff() = 0;

  // Reads back chip configuration; false if the chip stopped responding or was
  // reset (e.g. a brown-out on the servo rail).
  virtual bool verify() = 0;

  // Reads back one channel. `off` is true when the output is disabled.
  virtual bool readChannel(int channel, float& angleDeg, bool& off) = 0;

  // Puts the chip in low-power sleep (outputs stop). begin() wakes it up.
  virtual bool sleep() = 0;

  virtual uint32_t errorCount() const = 0;
};

}  // namespace qspider
