#pragma once

#include <cstddef>

#include "robot/Command.h"

namespace qspider {

enum class ParseResult : uint8_t { Ok, Help, Status, Empty, Error };

// Text command language shared by the serial console and the dashboard, e.g.
// "estop", "mode dance", "cal trim 4 -2.5", "dance rock loop".
ParseResult parseCommand(const char* line, Command& out, char* error, size_t errorSize);

// Human readable list of commands.
const char* commandHelp();

}  // namespace qspider
