#pragma once

#include <cstddef>
#include <cstdint>

namespace remus::blade {

enum class Slot : uint8_t { Left = 0, Right = 1 };

struct SlotAssignment {
  uint32_t sourceIdentityHash = 0;
  bool configured = false;
};

class SlotRegistry {
 public:
  bool assign(Slot slot, uint32_t sourceIdentityHash) {
    if (sourceIdentityHash == 0) return false;
    const size_t target = index(slot);
    const size_t other = target == 0 ? 1 : 0;
    if (assignments_[other].configured &&
        assignments_[other].sourceIdentityHash == sourceIdentityHash) {
      return false;
    }
    if (assignments_[target].configured &&
        assignments_[target].sourceIdentityHash == sourceIdentityHash) {
      return true;
    }
    assignments_[target] = {sourceIdentityHash, true};
    ++revision_;
    return true;
  }

  void clear(Slot slot) {
    assignments_[index(slot)] = {};
    ++revision_;
  }

  const SlotAssignment& get(Slot slot) const { return assignments_[index(slot)]; }
  uint32_t revision() const { return revision_; }

  void restore(uint32_t left, uint32_t right, uint32_t revision) {
    assignments_[0] = {left, left != 0};
    const bool rightValid = right != 0 && right != left;
    assignments_[1] = {rightValid ? right : 0, rightValid};
    revision_ = revision;
  }

 private:
  static constexpr size_t index(Slot slot) {
    return slot == Slot::Left ? 0 : 1;
  }

  SlotAssignment assignments_[2]{};
  uint32_t revision_ = 0;
};

}  // namespace remus::blade
