#pragma once
// Header-only fakes of the hardware interfaces. Used by the native unit tests
// and, for inputs that cannot be produced on demand (controller presses, low
// battery), by the hardware test firmware as well.

#include <cmath>
#include <cstdint>

#include "hal/ICalibrationStore.h"
#include "hal/IGamepad.h"
#include "hal/IPowerSensor.h"
#include "hal/IServoDriver.h"

namespace qspider {
namespace testing {

class FakeServoDriver : public IServoDriver {
 public:
  bool present = true;      // false = chip not responding
  bool failWrites = false;  // simulate I2C write errors
  bool configLost = false;  // simulate a brown-out reset
  bool sleeping = false;
  int beginCalls = 0;
  int allOffCalls = 0;
  uint32_t writes = 0;
  uint32_t errors = 0;
  float channels[kChannels];

  FakeServoDriver() {
    for (float& c : channels) c = NAN;
  }

  bool begin() override {
    ++beginCalls;
    if (!present) { ++errors; return false; }
    configLost = false;
    sleeping = false;
    for (float& c : channels) c = NAN;
    return true;
  }
  bool writeAll(const float (&angleDeg)[kChannels]) override {
    ++writes;
    if (!present || failWrites) { ++errors; return false; }
    for (int i = 0; i < kChannels; ++i) channels[i] = angleDeg[i];
    return true;
  }
  bool allOff() override {
    ++allOffCalls;
    for (float& c : channels) c = NAN;
    return present && !failWrites;
  }
  bool verify() override { return present && !configLost; }
  bool readChannel(int ch, float& deg, bool& off) override {
    if (!present) return false;
    off = std::isnan(channels[ch]);
    deg = off ? 0.0f : channels[ch];
    return true;
  }
  bool sleep() override {
    sleeping = true;
    return present;
  }
  uint32_t errorCount() const override { return errors; }

  bool anyOn() const {
    for (float c : channels)
      if (!std::isnan(c)) return true;
    return false;
  }
  int onCount() const {
    int n = 0;
    for (float c : channels)
      if (!std::isnan(c)) ++n;
    return n;
  }
};

class FakeGamepad : public IGamepad {
 public:
  GamepadState state;
  uint32_t* clock = nullptr;  // when set, lastPacketMs tracks this clock
  uint8_t led[3] = {};

  GamepadState read() override {
    if (clock && state.connected) state.lastPacketMs = *clock;
    return state;
  }
  void setLightbar(uint8_t r, uint8_t g, uint8_t b) override {
    led[0] = r;
    led[1] = g;
    led[2] = b;
  }

  void connect() { state.connected = true; }
  void disconnect() { state = GamepadState{}; }
  void press(Button b) { state.buttons |= buttonBit(b); }
  void release(Button b) { state.buttons &= ~buttonBit(b); }
  void releaseAll() { state.buttons = 0; }
  void sticks(float lx, float ly, float rx = 0.0f, float ry = 0.0f) {
    state.lx = lx;
    state.ly = ly;
    state.rx = rx;
    state.ry = ry;
  }
};

class FakePowerSensor : public IPowerSensor {
 public:
  bool fitted = true;
  bool failReads = false;
  float volts = 8.0f;

  bool available() const override { return fitted; }
  bool readVoltage(float& v) override {
    if (failReads) return false;
    v = volts;
    return true;
  }
};

class FakeCalibrationStore : public ICalibrationStore {
 public:
  bool hasData = false;
  bool failSave = false;
  int saves = 0;
  CalibrationData data;

  bool load(CalibrationData& out) override {
    if (!hasData) return false;
    out = data;
    return true;
  }
  bool save(const CalibrationData& d) override {
    if (failSave) return false;
    data = d;
    hasData = true;
    ++saves;
    return true;
  }
};

}  // namespace testing
}  // namespace qspider
