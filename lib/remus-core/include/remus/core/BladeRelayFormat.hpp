#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "remus/core/BladeProtocol.hpp"

namespace remus::blade::relay {

inline constexpr uint8_t kVersion = 1;
inline constexpr uint8_t kPacketRecordType = 0x01;
inline constexpr size_t kRecordCrcSize = 4;
inline constexpr size_t kMaxPacketSize = protocol::kMaxBatchSize;

#pragma pack(push, 1)
struct FileHeader {
  char magic[4];                 // "RBR1" — Remus Blade Relay v1.
  uint8_t version;
  uint8_t header_size;
  uint8_t blade_protocol_version;
  uint8_t flags;
  uint32_t computer_started_at_ms;
  uint32_t computer_session_id;
  uint8_t source_address[6];     // BLE address bytes as reported by the stack.
  uint32_t source_identity_hash; // Stable identity when device info is available.
  uint8_t reserved[38];
};

struct PacketRecordHeader {
  uint8_t type;                  // kPacketRecordType.
  uint8_t flags;                 // bit 0: notification, remaining bits reserved.
  uint16_t payload_length;
  uint32_t received_at_ms;       // Remus Computer monotonic receive time.
};
#pragma pack(pop)

static_assert(sizeof(FileHeader) == 64, "RBR1 header contract changed");
static_assert(sizeof(PacketRecordHeader) == 8, "RBR1 packet header contract changed");

inline FileHeader makeHeader(uint32_t startedAtMs, uint32_t sessionId,
                             const uint8_t sourceAddress[6],
                             uint32_t sourceIdentityHash) {
  FileHeader header{};
  std::memcpy(header.magic, "RBR1", 4);
  header.version = kVersion;
  header.header_size = static_cast<uint8_t>(sizeof(FileHeader));
  header.blade_protocol_version = protocol::kVersion;
  header.computer_started_at_ms = startedAtMs;
  header.computer_session_id = sessionId;
  if (sourceAddress) std::memcpy(header.source_address, sourceAddress, 6);
  header.source_identity_hash = sourceIdentityHash;
  return header;
}

inline size_t packetRecordSize(size_t payloadLength) {
  return sizeof(PacketRecordHeader) + payloadLength + kRecordCrcSize;
}

inline size_t encodePacketRecord(uint8_t* out, size_t capacity,
                                 uint32_t receivedAtMs,
                                 const uint8_t* payload, size_t payloadLength,
                                 uint8_t flags = 0x01) {
  if (!out || !payload || payloadLength == 0 || payloadLength > kMaxPacketSize) return 0;
  const size_t required = packetRecordSize(payloadLength);
  if (capacity < required) return 0;

  PacketRecordHeader header{};
  header.type = kPacketRecordType;
  header.flags = flags;
  header.payload_length = static_cast<uint16_t>(payloadLength);
  header.received_at_ms = receivedAtMs;
  std::memcpy(out, &header, sizeof(header));
  std::memcpy(out + sizeof(header), payload, payloadLength);
  protocol::writeU32(out + sizeof(header) + payloadLength,
                     protocol::crc32(out, sizeof(header) + payloadLength));
  return required;
}

inline bool validatePacketRecord(const uint8_t* record, size_t length) {
  if (!record || length < packetRecordSize(1)) return false;
  PacketRecordHeader header{};
  std::memcpy(&header, record, sizeof(header));
  if (header.type != kPacketRecordType || header.payload_length == 0 ||
      header.payload_length > kMaxPacketSize ||
      length != packetRecordSize(header.payload_length)) return false;
  const size_t crcOffset = sizeof(header) + header.payload_length;
  return protocol::readU32(record + crcOffset) == protocol::crc32(record, crcOffset);
}

}  // namespace remus::blade::relay
