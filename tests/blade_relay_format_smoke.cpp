#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "remus/core/BladeRelayFormat.hpp"

int main() {
  namespace relay = remus::blade::relay;
  const std::array<uint8_t, 6> address{{0x10, 0x20, 0x30, 0x40, 0x50, 0x60}};
  const auto header = relay::makeHeader(1234, 0xAABBCCDD, address.data(), 0x11223344);
  assert(std::memcmp(header.magic, "RBR1", 4) == 0);
  assert(header.version == 1);
  assert(header.header_size == sizeof(relay::FileHeader));
  assert(header.computer_session_id == 0xAABBCCDD);
  assert(std::memcmp(header.source_address, address.data(), address.size()) == 0);

  const std::array<uint8_t, 7> packet{{1, 1, 0, 23, 7, 8, 9}};
  std::array<uint8_t, 64> encoded{};
  const size_t size = relay::encodePacketRecord(
      encoded.data(), encoded.size(), 5678, packet.data(), packet.size());
  assert(size == relay::packetRecordSize(packet.size()));
  assert(relay::validatePacketRecord(encoded.data(), size));

  encoded[sizeof(relay::PacketRecordHeader) + 2] ^= 0x80;
  assert(!relay::validatePacketRecord(encoded.data(), size));
  assert(relay::encodePacketRecord(
      encoded.data(), 4, 0, packet.data(), packet.size()) == 0);

  std::cout << "blade_relay_format_smoke OK\n";
}
