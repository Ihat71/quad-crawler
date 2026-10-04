#pragma once
// Builds a complete Robot around fake (or real) hardware with a controllable
// clock, so whole-robot scenarios can be scripted in tests:
//   h.boot(); h.connectController(); h.runMs(2000); h.tap(Button::Left); ...

#include <cstring>

#include "Fakes.h"
#include "robot/Robot.h"
#include "telemetry/EventLog.h"

namespace qspider {
namespace testing {

inline uint32_t& fakeNow() {
  static uint32_t t = 1000;
  return t;
}
inline uint32_t fakeClock() { return fakeNow(); }

class RobotHarness {
 public:
  using SleepFn = void (*)(uint32_t ms);

  explicit RobotHarness(const RobotConfig& c, IServoDriver* realDriver = nullptr, SleepFn sleep = nullptr)
      : cfg(c),
        driver(realDriver ? *realDriver : fake),
        log(&fakeClock),
        robot(cfg, driver, pad, power, store, log),
        sleep_(sleep) {
    pad.clock = &fakeNow();
    power.fitted = cfg.battery.enabled;
    log.setMinLevel(LogLevel::Debug);
  }

  RobotConfig cfg;
  FakeServoDriver fake;
  IServoDriver& driver;
  FakeGamepad pad;
  FakePowerSensor power;
  FakeCalibrationStore store;
  EventLog log;
  Robot robot;

  uint32_t now() const { return fakeNow(); }

  void boot() { robot.begin(fakeNow()); }

  void tick() {
    fakeNow() += cfg.timing.controlPeriodMs;
    robot.tick(fakeNow());
    if (sleep_) sleep_(cfg.timing.controlPeriodMs);
  }

  void runMs(uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += cfg.timing.controlPeriodMs) tick();
  }

  // Runs until the predicate holds or the timeout expires. Returns the predicate result.
  template <typename Pred>
  bool runUntil(Pred pred, uint32_t timeoutMs) {
    for (uint32_t t = 0; t < timeoutMs; t += cfg.timing.controlPeriodMs) {
      if (pred()) return true;
      tick();
    }
    return pred();
  }

  void connectController() { pad.connect(); }

  // Boot, connect the controller and wait until the robot is armed and sitting.
  bool bootAndArm() {
    boot();
    connectController();
    return runUntil([&] { return robot.lifecycle() == Lifecycle::Active; }, 5000);
  }

  void tap(Button b) {
    pad.press(b);
    tick();
    pad.release(b);
    tick();
  }

  void hold(Button b, uint32_t ms) {
    pad.press(b);
    runMs(ms);
    pad.release(b);
    tick();
  }

  void submit(CommandType type, int32_t i = 0, float f = 0.0f) {
    Command c;
    c.type = type;
    c.source = CommandSource::Test;
    c.i = i;
    c.f = f;
    robot.submit(c);
    tick();
  }

  bool waitIdle(uint32_t timeoutMs = 5000) {
    return runUntil([&] { return !robot.motion().busy(); }, timeoutMs);
  }

  // True if any log entry (since boot) contains the text.
  bool logContains(const char* text) const {
    LogEntry chunk[8];
    uint32_t after = 0;
    for (;;) {
      const size_t n = log.events().since(after, chunk, 8);
      if (n == 0) return false;
      for (size_t i = 0; i < n; ++i)
        if (std::strstr(chunk[i].text, text)) return true;
      after = chunk[n - 1].seq;
    }
  }

  bool movementContains(const char* text) const {
    MovementEntry chunk[8];
    uint32_t after = 0;
    for (;;) {
      const size_t n = log.movements().since(after, chunk, 8);
      if (n == 0) return false;
      for (size_t i = 0; i < n; ++i)
        if (std::strstr(chunk[i].text, text)) return true;
      after = chunk[n - 1].seq;
    }
  }

 private:
  SleepFn sleep_;
};

}  // namespace testing
}  // namespace qspider
