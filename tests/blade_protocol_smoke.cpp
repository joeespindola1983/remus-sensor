#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>

#include "remus/core/BladeProtocol.hpp"

int main() {
  namespace protocol = remus::blade::protocol;
  std::array<protocol::RawImuFrame, 2> samples{};
  samples[0] = {41, 123456789ULL, -1, 2, -3, 4, -5, 6, 0};
  samples[1] = {42, 123461789ULL, 7, -8, 9, -10, 11, -12, 0};

  std::array<uint8_t, protocol::kMaxBatchSize> bytes{};
  const size_t length = protocol::encodeImuBatch(
      bytes.data(), bytes.size(), 7, samples.data(), samples.size());
  assert(length == protocol::kBatchHeaderSize + 30 + protocol::kBatchCrcSize);
  assert(bytes[0] == 1);
  assert(bytes[1] == static_cast<uint8_t>(protocol::MessageType::ImuBatch));
  assert(protocol::readU32(bytes.data() + 4) == 7);
  assert(protocol::readU32(bytes.data() + 8) == 41);
  assert(bytes[22] == 2);
  const uint32_t expectedCrc = protocol::crc32(bytes.data(), length - 4);
  assert(protocol::readU32(bytes.data() + length - 4) == expectedCrc);

  std::array<uint8_t, 32> fragment{};
  const size_t fragmentLength = protocol::encodeFragment(
      fragment.data(), fragment.size(), 7, 0, 2, bytes.data(), 20);
  assert(fragmentLength == protocol::kFragmentHeaderSize + 20);
  assert(fragment[1] == static_cast<uint8_t>(protocol::MessageType::Fragment));
  assert(fragment[7] == 2);
  assert(fragment[8] == 20);

  std::cout << "blade_protocol_smoke OK\n";
}
