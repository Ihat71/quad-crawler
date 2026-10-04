#include "telemetry/JsonWriter.h"

#include <cmath>
#include <cstdio>

namespace qspider {

void JsonWriter::prefix(const char* key) {
  if (depth_ > 0) {
    if (!first_[depth_]) out_ += ',';
    first_[depth_] = false;
  }
  if (key) {
    quoted(key);
    out_ += ':';
  }
}

void JsonWriter::quoted(const char* s) {
  out_ += '"';
  for (const char* p = s; *p; ++p) {
    const char c = *p;
    switch (c) {
      case '"': out_ += "\\\""; break;
      case '\\': out_ += "\\\\"; break;
      case '\n': out_ += "\\n"; break;
      case '\r': out_ += "\\r"; break;
      case '\t': out_ += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out_ += buf;
        } else {
          out_ += c;
        }
    }
  }
  out_ += '"';
}

JsonWriter& JsonWriter::beginObject(const char* key) {
  prefix(key);
  out_ += '{';
  if (depth_ + 1 < kMaxDepth) first_[++depth_] = true;
  return *this;
}

JsonWriter& JsonWriter::endObject() {
  out_ += '}';
  if (depth_ > 0) --depth_;
  return *this;
}

JsonWriter& JsonWriter::beginArray(const char* key) {
  prefix(key);
  out_ += '[';
  if (depth_ + 1 < kMaxDepth) first_[++depth_] = true;
  return *this;
}

JsonWriter& JsonWriter::endArray() {
  out_ += ']';
  if (depth_ > 0) --depth_;
  return *this;
}

JsonWriter& JsonWriter::str(const char* key, const char* value) {
  prefix(key);
  if (value)
    quoted(value);
  else
    out_ += "null";
  return *this;
}

JsonWriter& JsonWriter::boolean(const char* key, bool value) {
  prefix(key);
  out_ += value ? "true" : "false";
  return *this;
}

JsonWriter& JsonWriter::num(const char* key, long long value) {
  prefix(key);
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%lld", value);
  out_ += buf;
  return *this;
}

JsonWriter& JsonWriter::real(const char* key, double value, int decimals) {
  prefix(key);
  if (!std::isfinite(value)) {
    out_ += "null";
    return *this;
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
  out_ += buf;
  return *this;
}

JsonWriter& JsonWriter::null(const char* key) {
  prefix(key);
  out_ += "null";
  return *this;
}

}  // namespace qspider
