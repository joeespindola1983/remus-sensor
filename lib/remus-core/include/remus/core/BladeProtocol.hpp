#pragma once

#include <cstddef>
#include <cstdint>
#include <climits>

namespace remus::blade::protocol {

inline constexpr uint8_t kVersion = 1;
inline constexpr uint8_t kDeviceFamilyBlade = 2;
inline constexpr uint16_t kNominalSamplePeriodUs = 5000;
inline constexpr size_t kAxesBytesPerSample = 12;
inline constexpr size_t kTimingBytesPerSample = 3;  // signed jitter us + status.
inline constexpr size_t kBytesPerSample = kAxesBytesPerSample + kTimingBytesPerSample;
inline constexpr size_t kBatchHeaderSize = 23;
inline constexpr size_t kBatchCrcSize = 4;
inline constexpr size_t kMaxSamplesPerBatch = 10;
inline constexpr size_t kMaxBatchSize =
    kBatchHeaderSize + (kBytesPerSample * kMaxSamplesPerBatch) + kBatchCrcSize;
inline constexpr size_t kFragmentHeaderSize = 9;
inline constexpr size_t kRelayHeaderSize = 12;
inline constexpr size_t kRelayCrcSize = 4;
inline constexpr size_t kMaxRelayedPacketSize =
    kRelayHeaderSize + kMaxBatchSize + kRelayCrcSize;

enum class MessageType : uint8_t {
  ImuBatch = 0x01,
  Status = 0x02,
  ControlAck = 0x03,
  ClockSyncResponse = 0x04,
  Fragment = 0x11,
  RelayedPacket = 0x21,
};

enum class ControlCommand : uint8_t {
  StartStream = 0x01,
  StopStream = 0x02,
};

enum class ControlStatus : uint8_t {
  Accepted = 0x00,
  InvalidVersion = 0x01,
  InvalidCommand = 0x02,
  ImuUnavailable = 0x03,
};

struct RawImuFrame {
  uint32_t sampleSequence = 0;
  uint64_t nativeTimestampUs = 0;
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  uint8_t status = 0;
};

inline void writeU16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
}

inline void writeU32(uint8_t* out, uint32_t value) {
  for (size_t i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (i * 8));
}

inline void writeU64(uint8_t* out, uint64_t value) {
  for (size_t i = 0; i < 8; ++i) out[i] = static_cast<uint8_t>(value >> (i * 8));
}

inline uint32_t readU32(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) |
         (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

inline uint64_t readU64(const uint8_t* in) {
  uint64_t value = 0;
  for (size_t i = 0; i < 8; ++i) value |= static_cast<uint64_t>(in[i]) << (i * 8);
  return value;
}

inline uint16_t readU16(const uint8_t* in) {
  return static_cast<uint16_t>(in[0]) |
         (static_cast<uint16_t>(in[1]) << 8);
}

inline uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0U - (crc & 1U)));
    }
  }
  return ~crc;
}

inline size_t encodeImuBatch(uint8_t* out, size_t capacity,
                             uint32_t batchSequence,
                             const RawImuFrame* samples, size_t sampleCount) {
  if (!out || !samples || sampleCount == 0 || sampleCount > kMaxSamplesPerBatch) return 0;
  const size_t required = kBatchHeaderSize + sampleCount * kBytesPerSample + kBatchCrcSize;
  if (capacity < required) return 0;

  out[0] = kVersion;
  out[1] = static_cast<uint8_t>(MessageType::ImuBatch);
  out[2] = 0;
  out[3] = static_cast<uint8_t>(kBatchHeaderSize);
  writeU32(out + 4, batchSequence);
  writeU32(out + 8, samples[0].sampleSequence);
  writeU64(out + 12, samples[0].nativeTimestampUs);
  writeU16(out + 20, kNominalSamplePeriodUs);
  out[22] = static_cast<uint8_t>(sampleCount);

  size_t offset = kBatchHeaderSize;
  for (size_t i = 0; i < sampleCount; ++i) {
    const int16_t axes[] = {
      samples[i].ax, samples[i].ay, samples[i].az,
      samples[i].gx, samples[i].gy, samples[i].gz,
    };
    for (int16_t axis : axes) {
      writeU16(out + offset, static_cast<uint16_t>(axis));
      offset += 2;
    }
    const int64_t expectedTimestamp = static_cast<int64_t>(samples[0].nativeTimestampUs) +
        static_cast<int64_t>(i * kNominalSamplePeriodUs);
    int64_t jitterUs = static_cast<int64_t>(samples[i].nativeTimestampUs) - expectedTimestamp;
    if (jitterUs < INT16_MIN) jitterUs = INT16_MIN;
    if (jitterUs > INT16_MAX) jitterUs = INT16_MAX;
    writeU16(out + offset, static_cast<uint16_t>(static_cast<int16_t>(jitterUs)));
    offset += 2;
    out[offset++] = samples[i].status;
  }
  writeU32(out + offset, crc32(out, offset));
  return required;
}

inline size_t encodeFragment(uint8_t* out, size_t capacity,
                             uint32_t batchSequence, uint8_t fragmentIndex,
                             uint8_t fragmentCount, const uint8_t* payload,
                             size_t payloadLength) {
  if (!out || !payload || fragmentCount == 0 || fragmentIndex >= fragmentCount ||
      payloadLength > 255 || capacity < kFragmentHeaderSize + payloadLength) return 0;
  out[0] = kVersion;
  out[1] = static_cast<uint8_t>(MessageType::Fragment);
  writeU32(out + 2, batchSequence);
  out[6] = fragmentIndex;
  out[7] = fragmentCount;
  out[8] = static_cast<uint8_t>(payloadLength);
  for (size_t i = 0; i < payloadLength; ++i) out[kFragmentHeaderSize + i] = payload[i];
  return kFragmentHeaderSize + payloadLength;
}

inline size_t encodeRelayedPacket(uint8_t* out, size_t capacity,
                                 uint32_t sourceIdentityHash,
                                 uint32_t computerReceivedAtMs,
                                 const uint8_t* payload,
                                 size_t payloadLength) {
  if (!out || !payload || payloadLength == 0 || payloadLength > kMaxBatchSize) return 0;
  const size_t required = kRelayHeaderSize + payloadLength + kRelayCrcSize;
  if (capacity < required) return 0;
  out[0] = kVersion;
  out[1] = static_cast<uint8_t>(MessageType::RelayedPacket);
  writeU32(out + 2, sourceIdentityHash);
  writeU32(out + 6, computerReceivedAtMs);
  writeU16(out + 10, static_cast<uint16_t>(payloadLength));
  for (size_t i = 0; i < payloadLength; ++i) out[kRelayHeaderSize + i] = payload[i];
  writeU32(out + kRelayHeaderSize + payloadLength,
           crc32(out, kRelayHeaderSize + payloadLength));
  return required;
}

}  // namespace remus::blade::protocol
