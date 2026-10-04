#include "robot/CommandParser.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "common/Types.h"
#include "motion/Animation.h"
#include "telemetry/EventLog.h"

namespace qspider {

const char* toString(CommandType t) {
  switch (t) {
    case CommandType::EmergencyStop: return "estop";
    case CommandType::ClearEmergencyStop: return "clear";
    case CommandType::Arm: return "arm";
    case CommandType::Shutdown: return "shutdown";
    case CommandType::SetMode: return "mode";
    case CommandType::Sit: return "sit";
    case CommandType::Stand: return "stand";
    case CommandType::ToggleSitStand: return "toggle";
    case CommandType::Dance: return "dance";
    case CommandType::StopMotion: return "stop";
    case CommandType::Attack: return "attack";
    case CommandType::NextGait: return "gait next";
    case CommandType::SetGait: return "gait";
    case CommandType::NextDance: return "dance next";
    case CommandType::ResetDefaults: return "reset";
    case CommandType::CalSelect: return "cal select";
    case CommandType::CalSetTrim: return "cal trim";
    case CommandType::CalNudge: return "cal nudge";
    case CommandType::CalSave: return "cal save";
    case CommandType::CalResetTrim: return "cal reset";
    case CommandType::SetLogLevel: return "log";
  }
  return "?";
}

const char* toString(CommandSource s) {
  switch (s) {
    case CommandSource::Controller: return "controller";
    case CommandSource::Dashboard: return "dashboard";
    case CommandSource::Serial: return "serial";
    case CommandSource::Internal: return "internal";
    case CommandSource::Test: return "test";
  }
  return "?";
}

const char* commandHelp() {
  return "Commands:\n"
         "  estop | clear | arm | shutdown\n"
         "  mode <normal|ik|dance|cal>\n"
         "  sit | stand | toggle | stop | reset | attack\n"
         "  dance [next|<name>|<index>] [loop]   names: rock up_down twist bounce wave\n"
         "  gait <next|index>\n"
         "  cal select <servo> | cal trim <servo> <deg> | cal nudge <servo> <deg>\n"
         "  cal save | cal reset [<servo>|all]\n"
         "  log <debug|info|warn|error>\n"
         "  status | help\n";
}

namespace {

constexpr int kMaxTokens = 5;

int tokenize(char* buf, char* tokens[kMaxTokens]) {
  int n = 0;
  char* p = buf;
  while (*p && n < kMaxTokens) {
    while (*p && std::isspace(static_cast<unsigned char>(*p))) ++p;
    if (!*p) break;
    tokens[n++] = p;
    while (*p && !std::isspace(static_cast<unsigned char>(*p))) {
      *p = static_cast<char>(std::tolower(static_cast<unsigned char>(*p)));
      ++p;
    }
    if (*p) *p++ = '\0';
  }
  return n;
}

bool toInt(const char* s, int32_t& out) {
  if (!s) return false;
  char* end = nullptr;
  const long v = std::strtol(s, &end, 10);
  if (end == s || *end) return false;
  out = static_cast<int32_t>(v);
  return true;
}

bool toFloat(const char* s, float& out) {
  if (!s) return false;
  char* end = nullptr;
  const float v = std::strtof(s, &end);
  if (end == s || *end) return false;
  out = v;
  return true;
}

bool eq(const char* a, const char* b) { return a && std::strcmp(a, b) == 0; }

}  // namespace

ParseResult parseCommand(const char* line, Command& out, char* error, size_t errorSize) {
  auto fail = [&](const char* msg) {
    if (error && errorSize) std::snprintf(error, errorSize, "%s", msg);
    return ParseResult::Error;
  };

  char buf[96];
  std::snprintf(buf, sizeof(buf), "%s", line ? line : "");
  char* t[kMaxTokens] = {};
  const int n = tokenize(buf, t);
  if (n == 0) return ParseResult::Empty;

  out = Command{};
  const char* c = t[0];

  if (eq(c, "help") || eq(c, "?")) return ParseResult::Help;
  if (eq(c, "status")) return ParseResult::Status;

  if (eq(c, "estop") || eq(c, "e")) { out.type = CommandType::EmergencyStop; return ParseResult::Ok; }
  if (eq(c, "clear")) { out.type = CommandType::ClearEmergencyStop; return ParseResult::Ok; }
  if (eq(c, "arm") || eq(c, "wake")) { out.type = CommandType::Arm; return ParseResult::Ok; }
  if (eq(c, "shutdown")) { out.type = CommandType::Shutdown; return ParseResult::Ok; }
  if (eq(c, "sit")) { out.type = CommandType::Sit; return ParseResult::Ok; }
  if (eq(c, "stand")) { out.type = CommandType::Stand; return ParseResult::Ok; }
  if (eq(c, "toggle")) { out.type = CommandType::ToggleSitStand; return ParseResult::Ok; }
  if (eq(c, "stop")) { out.type = CommandType::StopMotion; return ParseResult::Ok; }
  if (eq(c, "attack")) { out.type = CommandType::Attack; return ParseResult::Ok; }
  if (eq(c, "reset")) { out.type = CommandType::ResetDefaults; return ParseResult::Ok; }

  if (eq(c, "mode")) {
    out.type = CommandType::SetMode;
    if (eq(t[1], "normal")) out.i = static_cast<int32_t>(Mode::Normal);
    else if (eq(t[1], "ik")) out.i = static_cast<int32_t>(Mode::IK);
    else if (eq(t[1], "dance")) out.i = static_cast<int32_t>(Mode::Dance);
    else if (eq(t[1], "cal") || eq(t[1], "calibration")) out.i = static_cast<int32_t>(Mode::Calibration);
    else return fail("usage: mode <normal|ik|dance|cal>");
    return ParseResult::Ok;
  }

  if (eq(c, "dance")) {
    out.type = CommandType::Dance;
    out.i = -1;
    int arg = 1;
    if (eq(t[1], "next")) {
      out.type = CommandType::NextDance;
      return ParseResult::Ok;
    }
    if (n > 1 && !eq(t[1], "loop")) {
      int32_t idx;
      if (toInt(t[1], idx)) {
        out.i = idx;
      } else {
        const AnimationDef* a = animations::findByName(t[1]);
        bool found = false;
        for (int d = 0; a && d < animations::danceCount(); ++d) {
          if (&animations::dance(d) == a) { out.i = d; found = true; }
        }
        if (!found) return fail("unknown dance");
      }
      arg = 2;
    }
    out.f = eq(t[arg], "loop") ? 1.0f : 0.0f;
    return ParseResult::Ok;
  }

  if (eq(c, "gait")) {
    if (eq(t[1], "next")) { out.type = CommandType::NextGait; return ParseResult::Ok; }
    out.type = CommandType::SetGait;
    if (!toInt(t[1], out.i)) return fail("usage: gait <next|index>");
    return ParseResult::Ok;
  }

  if (eq(c, "cal")) {
    const char* sub = t[1];
    if (eq(sub, "save")) { out.type = CommandType::CalSave; return ParseResult::Ok; }
    if (eq(sub, "reset")) {
      out.type = CommandType::CalResetTrim;
      out.i = -1;
      if (n > 2 && !eq(t[2], "all") && !toInt(t[2], out.i)) return fail("usage: cal reset [servo|all]");
      return ParseResult::Ok;
    }
    if (eq(sub, "select")) {
      out.type = CommandType::CalSelect;
      if (!toInt(t[2], out.i)) return fail("usage: cal select <servo>");
      return ParseResult::Ok;
    }
    if (eq(sub, "trim") || eq(sub, "nudge")) {
      out.type = eq(sub, "trim") ? CommandType::CalSetTrim : CommandType::CalNudge;
      if (!toInt(t[2], out.i) || !toFloat(t[3], out.f)) return fail("usage: cal trim|nudge <servo> <deg>");
      return ParseResult::Ok;
    }
    return fail("usage: cal <select|trim|nudge|save|reset>");
  }

  if (eq(c, "log")) {
    out.type = CommandType::SetLogLevel;
    if (eq(t[1], "debug")) out.i = static_cast<int32_t>(LogLevel::Debug);
    else if (eq(t[1], "info")) out.i = static_cast<int32_t>(LogLevel::Info);
    else if (eq(t[1], "warn")) out.i = static_cast<int32_t>(LogLevel::Warn);
    else if (eq(t[1], "error")) out.i = static_cast<int32_t>(LogLevel::Error);
    else return fail("usage: log <debug|info|warn|error>");
    return ParseResult::Ok;
  }

  return fail("unknown command (try 'help')");
}

}  // namespace qspider
