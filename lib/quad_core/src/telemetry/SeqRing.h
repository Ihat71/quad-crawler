#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>

namespace qspider {

// Fixed-size, thread-safe ring buffer. Every pushed item gets a monotonically
// increasing sequence number so readers can ask for "everything after N"
// without consuming entries (several readers: serial, dashboard, tests).
// T must have a `uint32_t seq` member.
template <typename T, size_t N>
class SeqRing {
 public:
  uint32_t push(T item) {
    std::lock_guard<std::mutex> lock(mutex_);
    item.seq = ++lastSeq_;
    buf_[lastSeq_ % N] = item;
    return lastSeq_;
  }

  // Copies up to `max` entries with seq > `after`, oldest first.
  size_t since(uint32_t after, T* out, size_t max) const {
    std::lock_guard<std::mutex> lock(mutex_);
    uint32_t first = after + 1;
    const uint32_t oldest = lastSeq_ >= N ? lastSeq_ - N + 1 : 1;
    if (first < oldest) first = oldest;
    size_t n = 0;
    for (uint32_t s = first; s <= lastSeq_ && n < max; ++s) out[n++] = buf_[s % N];
    return n;
  }

  uint32_t lastSeq() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lastSeq_;
  }

  static constexpr size_t capacity() { return N; }

 private:
  mutable std::mutex mutex_;
  T buf_[N] = {};
  uint32_t lastSeq_ = 0;
};

}  // namespace qspider
