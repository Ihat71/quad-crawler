#pragma once

#include <atomic>
#include <cstdarg>
#include <cstdint>

#include "common/Types.h"
#include "input/Buttons.h"
#include "telemetry/SeqRing.h"

namespace qspider {

enum class LogLevel : uint8_t { Debug, Info, Warn, Error, Critical };
enum class LogCategory : uint8_t { System, Mode, Motion, Input, Safety, Calibration, Network, Test };
enum class ButtonAction : uint8_t { Press, Release, Hold };

const char* toString(LogLevel l);
const char* toString(LogCategory c);
const char* toString(ButtonAction a);

struct LogEntry {
  uint32_t seq;
  uint32_t timeMs;
  LogLevel level;
  LogCategory category;
  char text[84];
};

struct MovementEntry {
  uint32_t seq;
  uint32_t timeMs;
  Mode mode;
  char text[52];
};

struct ButtonEntry {
  uint32_t seq;
  uint32_t timeMs;
  Button button;
  ButtonAction action;
  Mode mode;
};

// Central, lock-protected, allocation-free log. Writers (control loop) only
// format into a ring slot; readers (serial sink, dashboard) pull by sequence
// number from a low-priority task, so logging never blocks on I/O.
class EventLog {
 public:
  using ClockFn = uint32_t (*)();

  static constexpr size_t kEventCapacity = 96;
  static constexpr size_t kMovementCapacity = 48;
  static constexpr size_t kButtonCapacity = 64;

  explicit EventLog(ClockFn clock);

  void log(LogLevel level, LogCategory cat, const char* fmt, ...)
      __attribute__((format(printf, 4, 5)));
  void vlog(LogLevel level, LogCategory cat, const char* fmt, va_list args);
  void movement(Mode mode, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
  void button(Button b, ButtonAction action, Mode mode);

  void setMinLevel(LogLevel l) { minLevel_.store(static_cast<uint8_t>(l)); }
  LogLevel minLevel() const { return static_cast<LogLevel>(minLevel_.load()); }

  uint32_t warnings() const { return warnings_.load(); }
  uint32_t errors() const { return errors_.load(); }
  uint32_t now() const { return clock_(); }

  const SeqRing<LogEntry, kEventCapacity>& events() const { return events_; }
  const SeqRing<MovementEntry, kMovementCapacity>& movements() const { return movements_; }
  const SeqRing<ButtonEntry, kButtonCapacity>& buttons() const { return buttons_; }

 private:
  ClockFn clock_;
  std::atomic<uint8_t> minLevel_{static_cast<uint8_t>(LogLevel::Info)};
  std::atomic<uint32_t> warnings_{0};
  std::atomic<uint32_t> errors_{0};
  SeqRing<LogEntry, kEventCapacity> events_;
  SeqRing<MovementEntry, kMovementCapacity> movements_;
  SeqRing<ButtonEntry, kButtonCapacity> buttons_;
};

}  // namespace qspider
