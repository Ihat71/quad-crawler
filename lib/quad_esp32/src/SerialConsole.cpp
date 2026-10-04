#include "SerialConsole.h"

#include "robot/CommandParser.h"

namespace qspider {

SerialConsole::SerialConsole(Robot& robot, EventLog& log, const RobotConfig& cfg, Stream& io)
    : robot_(robot), log_(log), cfg_(cfg), io_(io) {}

void SerialConsole::poll() {
  printNewEntries();
  while (io_.available() > 0) {
    const int c = io_.read();
    if (c == '\r' || c == '\n') {
      if (len_ > 0) {
        line_[len_] = '\0';
        io_.printf("> %s\n", line_);
        handleLine(line_);
        len_ = 0;
      }
    } else if (len_ + 1 < sizeof(line_)) {
      line_[len_++] = static_cast<char>(c);
    }
  }
}

void SerialConsole::printNewEntries() {
  // Interleaving by time is not needed for a terminal; each stream is printed in order.
  LogEntry logs[8];
  size_t n;
  while ((n = log_.events().since(cursor_.logSeq, logs, 8)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      const LogEntry& e = logs[i];
      io_.printf("[%9.3f] %-5s %-6s %s\n", e.timeMs / 1000.0f, toString(e.level), toString(e.category), e.text);
      cursor_.logSeq = e.seq;
    }
  }
  MovementEntry moves[8];
  while ((n = log_.movements().since(cursor_.moveSeq, moves, 8)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      io_.printf("[%9.3f] MOVE  %-6.6s %s\n", moves[i].timeMs / 1000.0f, toString(moves[i].mode), moves[i].text);
      cursor_.moveSeq = moves[i].seq;
    }
  }
  ButtonEntry buttons[8];
  while ((n = log_.buttons().since(cursor_.buttonSeq, buttons, 8)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (log_.minLevel() <= LogLevel::Debug || buttons[i].action == ButtonAction::Hold)
        io_.printf("[%9.3f] BTN   %-6.6s %s %s\n", buttons[i].timeMs / 1000.0f, toString(buttons[i].mode),
                   toString(buttons[i].button), toString(buttons[i].action));
      cursor_.buttonSeq = buttons[i].seq;
    }
  }
}

void SerialConsole::handleLine(const char* line) {
  Command c;
  char error[64] = {};
  switch (parseCommand(line, c, error, sizeof(error))) {
    case ParseResult::Ok:
      c.source = CommandSource::Serial;
      if (!robot_.submit(c)) io_.println("command queue full");
      break;
    case ParseResult::Help: io_.print(commandHelp()); break;
    case ParseResult::Status: printStatus(); break;
    case ParseResult::Empty: break;
    case ParseResult::Error: io_.printf("error: %s\n", error); break;
  }
}

void SerialConsole::printStatus() {
  const RobotSnapshot s = robot_.snapshot();
  io_.printf("%s | %s | %s | %s | gait %s | dance %s\n", toString(s.lifecycle), toString(s.mode), toString(s.posture),
             toString(s.motion), s.gaitName, s.danceName);
  io_.printf("controller %s | battery %s %.2f V (%d%%) | outputs %s\n", s.controllerConnected ? "connected" : "lost",
             toString(s.batteryStatus), s.batteryVoltage, s.batteryPercent, s.outputsEnabled ? "on" : "off");
  io_.printf("loop %.2f ms avg, exec %.0f us avg / %u us max, overruns %u | i2c errors %u | clamps %u\n",
             s.periodAvgMs, s.execAvgUs, static_cast<unsigned>(s.execMaxUs), static_cast<unsigned>(s.overruns),
             static_cast<unsigned>(s.i2cErrors), static_cast<unsigned>(s.clampEvents));
  io_.print("faults:");
  if (!s.activeFaults) io_.print(" none");
  for (int i = 0; i < kFaultCount; ++i)
    if (s.activeFaults & (1u << i)) io_.printf(" %s", toString(static_cast<Fault>(i)));
  if (s.estopReason[0]) io_.printf(" | e-stop: %s", s.estopReason);
  io_.println();
  for (int i = 0; i < kServoCount; ++i) {
    io_.printf("  servo %2d %s %-5s ch%2u  out %6.1f  trim %+5.1f\n", i, legName(legOfServo(i)),
               jointName(jointOfServo(i)), cfg_.legs[legOfServo(i)].servo[jointOfServo(i)].channel, s.servoDeg[i],
               s.trim[i]);
  }
}

}  // namespace qspider
