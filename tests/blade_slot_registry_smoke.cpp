#include <cassert>

#include "remus/core/BladeSlotRegistry.hpp"

int main() {
  remus::blade::SlotRegistry registry;
  assert(registry.assign(remus::blade::Slot::Left, 0x11223344));
  const uint32_t assignedRevision = registry.revision();
  assert(registry.assign(remus::blade::Slot::Left, 0x11223344));
  assert(registry.revision() == assignedRevision);
  assert(!registry.assign(remus::blade::Slot::Right, 0x11223344));
  assert(registry.assign(remus::blade::Slot::Right, 0x55667788));
  assert(registry.get(remus::blade::Slot::Left).sourceIdentityHash == 0x11223344);
  assert(registry.get(remus::blade::Slot::Right).sourceIdentityHash == 0x55667788);
  registry.clear(remus::blade::Slot::Left);
  assert(!registry.get(remus::blade::Slot::Left).configured);
  registry.restore(0xAABBCCDD, 0xAABBCCDD, 7);
  assert(registry.get(remus::blade::Slot::Left).configured);
  assert(!registry.get(remus::blade::Slot::Right).configured);
  assert(registry.get(remus::blade::Slot::Right).sourceIdentityHash == 0);
  assert(registry.revision() == 7);
}
