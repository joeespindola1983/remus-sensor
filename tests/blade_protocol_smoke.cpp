#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>

#include "remus/core/BladeProtocol.hpp"

int main() {
  uint8_t u64Bytes[8]{};
  remus::blade::protocol::writeU64(u64Bytes, 0x0123456789ABCDEFULL);
  assert(remus::blade::protocol::readU64(u64Bytes) == 0x0123456789ABCDEFULL);
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

  std::array<uint8_t, protocol::kMaxRelayedPacketSize> relayed{};
  const size_t relayedLength = protocol::encodeRelayedPacket(
      relayed.data(), relayed.size(), 0x11223344, 9876,
      bytes.data(), length);
  assert(relayedLength == protocol::kRelayHeaderSize + length + protocol::kRelayCrcSize);
  assert(relayed[1] == static_cast<uint8_t>(protocol::MessageType::RelayedPacket));
  assert(protocol::readU32(relayed.data() + 2) == 0x11223344);
  assert(protocol::readU32(relayed.data() + 6) == 9876);
  assert(protocol::readU16(relayed.data() + 10) == length);
  assert(protocol::readU32(relayed.data() + relayedLength - 4) ==
         protocol::crc32(relayed.data(), relayedLength - 4));

  std::cout << "blade_protocol_smoke OK\n";
}
