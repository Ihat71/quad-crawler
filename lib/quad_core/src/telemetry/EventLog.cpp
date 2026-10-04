#include "telemetry/EventLog.h"

#include <cstdio>

namespace qspider {

const char* toString(LogLevel l) {
  switch (l) {
    case LogLevel::Debug: return "DEBUG";
    case LogLevel::Info: return "INFO";
    case LogLevel::Warn: return "WARN";
    case LogLevel::Error: return "ERROR";
    case LogLevel::Critical: return "CRIT";
  }
  return "?";
}

const char* toString(LogCategory c) {
  switch (c) {
    case LogCategory::System: return "SYSTEM";
    case LogCategory::Mode: return "MODE";
    case LogCategory::Motion: return "MOTION";
    case LogCategory::Input: return "INPUT";
    case LogCategory::Safety: return "SAFETY";
    case LogCategory::Calibration: return "CAL";
    case LogCategory::Network: return "NET";
    case LogCategory::Test: return "TEST";
  }
  return "?";
}

const char* toString(ButtonAction a) {
  switch (a) {
    case ButtonAction::Press: return "press";
    case ButtonAction::Release: return "release";
    case ButtonAction::Hold: return "hold";
  }
  return "?";
}

const char* toString(Button b) {
  switch (b) {
    case Button::Up: return "DPAD_UP";
    case Button::Down: return "DPAD_DOWN";
    case Button::Left: return "DPAD_LEFT";
    case Button::Right: return "DPAD_RIGHT";
    case Button::Cross: return "CROSS";
    case Button::Circle: return "CIRCLE";
    case Button::Square: return "SQUARE";
    case Button::Triangle: return "TRIANGLE";
    case Button::L1: return "L1";
    case Button::R1: return "R1";
    case Button::L2: return "L2";
    case Button::R2: return "R2";
    case Button::L3: return "L3";
    case Button::R3: return "R3";
    case Button::Create: return "CREATE";
    case Button::Options: return "OPTIONS";
    case Button::PlayStation: return "PS";
    case Button::Touchpad: return "TOUCHPAD";
    case Button::Mute: return "MUTE";
    case Button::Count: break;
  }
  return "?";
}

EventLog::EventLog(ClockFn clock) : clock_(clock) {}

void EventLog::log(LogLevel level, LogCategory cat, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vlog(level, cat, fmt, args);
  va_end(args);
}

void EventLog::vlog(LogLevel level, LogCategory cat, const char* fmt, va_list args) {
  if (level >= LogLevel::Error) {
    errors_.fetch_add(1);
  } else if (level == LogLevel::Warn) {
    warnings_.fetch_add(1);
  }
  if (static_cast<uint8_t>(level) < minLevel_.load()) return;

  LogEntry e{};
  e.timeMs = clock_();
  e.level = level;
  e.category = cat;
  vsnprintf(e.text, sizeof(e.text), fmt, args);
  events_.push(e);
}

void EventLog::movement(Mode mode, const char* fmt, ...) {
  MovementEntry e{};
  e.timeMs = clock_();
  e.mode = mode;
  va_list args;
  va_start(args, fmt);
  vsnprintf(e.text, sizeof(e.text), fmt, args);
  va_end(args);
  movements_.push(e);
}

void EventLog::button(Button b, ButtonAction action, Mode mode) {
  ButtonEntry e{};
  e.timeMs = clock_();
  e.button = b;
  e.action = action;
  e.mode = mode;
  buttons_.push(e);
}

}  // namespace qspider
