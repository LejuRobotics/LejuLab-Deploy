#pragma once

#include <cassert>
#include <vector>

#include "leju-rl-controller/rl/rl_controller_types.h"

namespace leju {

/// 定容环形缓冲（语义对齐 HEFT TensorRingBuffer：push 先减 head，take(0)=最新）
class ObsRingBuffer {
 public:
  ObsRingBuffer() = default;

  void init(int capacity, int dim) {
    assert(capacity > 0 && dim > 0);
    capacity_ = capacity;
    dim_ = dim;
    head_ = 0;
    slots_.assign(capacity_, array_t::Zero(dim_));
  }

  void reset(const array_t& value) {
    assert(value.size() == dim_);
    head_ = 0;
    for (auto& slot : slots_) {
      slot = value;
    }
  }

  void resetZero() { reset(array_t::Zero(dim_)); }

  void push(const array_t& value) {
    assert(value.size() == dim_);
    head_ = (head_ - 1 + capacity_) % capacity_;
    slots_[head_] = value;
  }

  const array_t& take(int offset) const {
    assert(offset >= 0 && offset < capacity_);
    return slots_[(head_ + offset) % capacity_];
  }

  array_t takeFlattened(const std::vector<int>& offsets) const {
    array_t out(static_cast<int>(offsets.size()) * dim_);
    for (size_t i = 0; i < offsets.size(); ++i) {
      out.segment(static_cast<int>(i) * dim_, dim_) = take(offsets[i]);
    }
    return out;
  }

  int capacity() const { return capacity_; }
  int dim() const { return dim_; }
  bool initialized() const { return capacity_ > 0; }

 private:
  int capacity_ = 0;
  int dim_ = 0;
  int head_ = 0;
  std::vector<array_t> slots_;
};

}  // namespace leju
