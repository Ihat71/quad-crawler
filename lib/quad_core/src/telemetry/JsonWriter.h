#pragma once

#include <cstdint>
#include <string>

namespace qspider {

// Minimal streaming JSON writer (no allocations beyond the output string).
// Pass key = nullptr when writing array elements.
class JsonWriter {
 public:
  explicit JsonWriter(std::string& out) : out_(out) {}

  JsonWriter& beginObject(const char* key = nullptr);
  JsonWriter& endObject();
  JsonWriter& beginArray(const char* key = nullptr);
  JsonWriter& endArray();

  JsonWriter& str(const char* key, const char* value);  // nullptr -> null
  JsonWriter& boolean(const char* key, bool value);
  JsonWriter& num(const char* key, long long value);
  JsonWriter& real(const char* key, double value, int decimals = 2);  // NaN/inf -> null
  JsonWriter& null(const char* key);

 private:
  void prefix(const char* key);
  void quoted(const char* s);

  static constexpr int kMaxDepth = 16;
  std::string& out_;
  bool first_[kMaxDepth] = {true};
  int depth_ = 0;
};

}  // namespace qspider
