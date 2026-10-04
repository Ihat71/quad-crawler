// Observability and configuration: event log ring buffers, telemetry JSON,
// the shared command language and validation of the shipped configuration.

#include <unity.h>

#include <cctype>
#include <cstring>
#include <string>

#include "RobotHarness.h"
#include "quad_core.h"
#include "robot_config.h"

using namespace qspider;
using namespace qspider::testing;

void setUp() {}
void tearDown() {}

// Minimal JSON validator (recursive descent) so the dashboard payload is
// guaranteed to parse.
namespace json {
const char* p;
void ws() { while (*p && std::isspace(static_cast<unsigned char>(*p))) ++p; }
bool value();
bool str() {
  if (*p != '"') return false;
  for (++p; *p && *p != '"'; ++p)
    if (*p == '\\' && !*++p) return false;
  if (*p != '"') return false;
  ++p;
  return true;
}
bool number() {
  const char* s = p;
  if (*p == '-') ++p;
  while (std::isdigit(static_cast<unsigned char>(*p)) || *p == '.' || *p == 'e' || *p == 'E' || *p == '+' || *p == '-') ++p;
  return p > s;
}
bool literal(const char* l) {
  const size_t n = std::strlen(l);
  if (std::strncmp(p, l, n) != 0) return false;
  p += n;
  return true;
}
bool container(char open, char close, bool object) {
  if (*p != open) return false;
  ++p;
  ws();
  if (*p == close) { ++p; return true; }
  for (;;) {
    ws();
    if (object) {
      if (!str()) return false;
      ws();
      if (*p++ != ':') return false;
    }
    ws();
    if (!value()) return false;
    ws();
    if (*p == ',') { ++p; continue; }
    if (*p == close) { ++p; return true; }
    return false;
  }
}
bool value() {
  ws();
  switch (*p) {
    case '{': return container('{', '}', true);
    case '[': return container('[', ']', false);
    case '"': return str();
    case 't': return literal("true");
    case 'f': return literal("false");
    case 'n': return literal("null");
    default: return number();
  }
}
bool valid(const std::string& s) {
  p = s.c_str();
  if (!value()) return false;
  ws();
  return *p == '\0';
}
}  // namespace json

void test_shipped_config_is_valid_without_warnings() {
  const RobotConfig cfg = makeRobotConfig();
  BodyKinematics kin(cfg);
  EventLog log(&fakeClock);
  const ConfigReport r = validateConfig(cfg, kin, log);
  TEST_ASSERT_EQUAL_INT(0, r.errors);
  TEST_ASSERT_EQUAL_INT(0, r.warnings);
}

void test_config_validator_catches_duplicate_channels() {
  RobotConfig cfg = makeRobotConfig();
  cfg.legs[1].servo[0].channel = cfg.legs[0].servo[0].channel;
  BodyKinematics kin(cfg);
  EventLog log(&fakeClock);
  TEST_ASSERT_TRUE(validateConfig(cfg, kin, log).errors > 0);
}

void test_config_validator_flags_limits_outside_10_170() {
  RobotConfig cfg = makeRobotConfig();
  cfg.servoLimits.minDeg = 0;
  BodyKinematics kin(cfg);
  EventLog log(&fakeClock);
  TEST_ASSERT_TRUE(validateConfig(cfg, kin, log).errors > 0);
}

void test_event_log_sequence_and_wraparound() {
  EventLog log(&fakeClock);
  for (int i = 0; i < 200; ++i) log.log(LogLevel::Info, LogCategory::System, "entry %d", i);
  LogEntry e[8];
  const size_t n = log.events().since(0, e, 8);
  TEST_ASSERT_EQUAL_UINT(8, n);
  // Oldest retained entry is (200 - capacity).
  TEST_ASSERT_EQUAL_UINT32(200 - EventLog::kEventCapacity + 1, e[0].seq);
  TEST_ASSERT_EQUAL_UINT(0, log.events().since(200, e, 8));
  TEST_ASSERT_EQUAL_UINT(1, log.events().since(199, e, 8));
  TEST_ASSERT_EQUAL_STRING("entry 199", e[0].text);
}

void test_event_log_level_filter_and_counters() {
  EventLog log(&fakeClock);
  log.setMinLevel(LogLevel::Warn);
  log.log(LogLevel::Info, LogCategory::System, "hidden");
  log.log(LogLevel::Warn, LogCategory::Safety, "shown");
  log.log(LogLevel::Error, LogCategory::Safety, "error");
  TEST_ASSERT_EQUAL_UINT32(2, log.events().lastSeq());
  TEST_ASSERT_EQUAL_UINT32(1, log.warnings());
  TEST_ASSERT_EQUAL_UINT32(1, log.errors());
}

void test_telemetry_json_is_valid_and_incremental() {
  RobotHarness h(makeRobotConfig());
  TEST_ASSERT_TRUE(h.bootAndArm());
  h.tap(Button::Cross);
  h.runMs(500);

  std::string out;
  PlatformInfo info;
  info.ip = "192.168.4.1";
  writeTelemetryJson(out, h.cfg, h.robot.snapshot(), h.log, TelemetryCursor{}, info);
  TEST_ASSERT_TRUE_MESSAGE(json::valid(out), out.c_str());
  TEST_ASSERT_NOT_NULL(std::strstr(out.c_str(), "\"mode\":\"NORMAL\""));
  TEST_ASSERT_NOT_NULL(std::strstr(out.c_str(), "\"btn\":\"CROSS\""));

  TelemetryCursor all;
  all.logSeq = h.log.events().lastSeq();
  all.moveSeq = h.log.movements().lastSeq();
  all.buttonSeq = h.log.buttons().lastSeq();
  out.clear();
  writeTelemetryJson(out, h.cfg, h.robot.snapshot(), h.log, all, info);
  TEST_ASSERT_TRUE(json::valid(out));
  TEST_ASSERT_NOT_NULL(std::strstr(out.c_str(), "\"logs\":[]"));
  TEST_ASSERT_NOT_NULL(std::strstr(out.c_str(), "\"buttons\":[]"));
}

void test_json_escapes_strings() {
  std::string out;
  JsonWriter w(out);
  w.beginObject().str("k", "a\"b\\c\n").real("nan", NAN).endObject();
  TEST_ASSERT_EQUAL_STRING("{\"k\":\"a\\\"b\\\\c\\n\",\"nan\":null}", out.c_str());
  TEST_ASSERT_TRUE(json::valid(out));
}

void test_command_parser() {
  Command c;
  char err[64];
  TEST_ASSERT_EQUAL(ParseResult::Ok, parseCommand("estop", c, err, sizeof(err)));
  TEST_ASSERT_EQUAL(CommandType::EmergencyStop, c.type);

  TEST_ASSERT_EQUAL(ParseResult::Ok, parseCommand("  MODE Dance ", c, err, sizeof(err)));
  TEST_ASSERT_EQUAL(CommandType::SetMode, c.type);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Mode::Dance), c.i);

  TEST_ASSERT_EQUAL(ParseResult::Ok, parseCommand("cal trim 4 -2.5", c, err, sizeof(err)));
  TEST_ASSERT_EQUAL(CommandType::CalSetTrim, c.type);
  TEST_ASSERT_EQUAL_INT(4, c.i);
  TEST_ASSERT_EQUAL_FLOAT(-2.5f, c.f);

  TEST_ASSERT_EQUAL(ParseResult::Ok, parseCommand("dance wave loop", c, err, sizeof(err)));
  TEST_ASSERT_EQUAL(CommandType::Dance, c.type);
  TEST_ASSERT_EQUAL_STRING("wave", animations::dance(c.i).name);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, c.f);

  TEST_ASSERT_EQUAL(ParseResult::Ok, parseCommand("cal reset all", c, err, sizeof(err)));
  TEST_ASSERT_EQUAL_INT(-1, c.i);

  TEST_ASSERT_EQUAL(ParseResult::Help, parseCommand("help", c, err, sizeof(err)));
  TEST_ASSERT_EQUAL(ParseResult::Empty, parseCommand("   ", c, err, sizeof(err)));
  TEST_ASSERT_EQUAL(ParseResult::Error, parseCommand("fly", c, err, sizeof(err)));
  TEST_ASSERT_EQUAL(ParseResult::Error, parseCommand("cal trim x", c, err, sizeof(err)));
  TEST_ASSERT_EQUAL(ParseResult::Error, parseCommand("dance moonwalk", c, err, sizeof(err)));
}

void test_mode_changes_are_logged_as_state_changes() {
  RobotHarness h(makeRobotConfig());
  TEST_ASSERT_TRUE(h.bootAndArm());
  h.tap(Button::Up);
  TEST_ASSERT_TRUE(h.logContains("Mode NORMAL -> IK"));
  TEST_ASSERT_TRUE(h.movementContains("MODE IK"));
  TEST_ASSERT_TRUE(h.logContains("Armed"));
  TEST_ASSERT_TRUE(h.logContains("Self-test passed"));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_shipped_config_is_valid_without_warnings);
  RUN_TEST(test_config_validator_catches_duplicate_channels);
  RUN_TEST(test_config_validator_flags_limits_outside_10_170);
  RUN_TEST(test_event_log_sequence_and_wraparound);
  RUN_TEST(test_event_log_level_filter_and_counters);
  RUN_TEST(test_telemetry_json_is_valid_and_incremental);
  RUN_TEST(test_json_escapes_strings);
  RUN_TEST(test_command_parser);
  RUN_TEST(test_mode_changes_are_logged_as_state_changes);
  return UNITY_END();
}
