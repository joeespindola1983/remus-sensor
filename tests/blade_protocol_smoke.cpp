#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include "remus/core/BladeProtocol.hpp"

// Standalone encoder for testing Device Info v2 (replicates BladeApp/RemusApp schemas)
size_t encodeDeviceInfoV2(uint8_t* out, size_t capacity, uint8_t family,
                          uint8_t model, uint8_t hardware, uint16_t capabilities,
                          uint16_t imuRateHz, uint8_t accelRange, uint8_t gyroRange,
                          uint8_t dlpfSetting, const std::string& serial,
                          const std::string& firmware) {
  const size_t serialLength = std::min<size_t>(serial.length(), 31);
  const size_t firmwareLength = firmware.length();
  const size_t required = 12 + serialLength + 1 + firmwareLength;
  if (!out || capacity < required) return 0;
  out[0] = 2; // version
  out[1] = family;
  out[2] = model;
  out[3] = hardware;
  remus::blade::protocol::writeU16(out + 4, capabilities);
  remus::blade::protocol::writeU16(out + 6, imuRateHz);
  out[8] = accelRange;
  out[9] = gyroRange;
  out[10] = dlpfSetting;
  out[11] = static_cast<uint8_t>(serialLength);
  std::memcpy(out + 12, serial.c_str(), serialLength);
  out[12 + serialLength] = static_cast<uint8_t>(firmwareLength);
  std::memcpy(out + 13 + serialLength, firmware.c_str(), firmwareLength);
  return required;
}

void testDeviceInfoV2() {
  std::array<uint8_t, 128> out{};
  
  // 1. Blade Device
  size_t len = encodeDeviceInfoV2(out.data(), out.size(),
                                  2, 1, 1, 0x0003, 200, 2, 3, 3,
                                  "RB-D-001122334455", "1.4.0");
  assert(len == 12 + 17 + 1 + 5);
  assert(out[0] == 2);
  assert(out[1] == 2);
  assert(out[2] == 1);
  assert(out[3] == 1);
  assert(remus::blade::protocol::readU16(out.data() + 4) == 0x0003);
  assert(remus::blade::protocol::readU16(out.data() + 6) == 200);
  assert(out[8] == 2);
  assert(out[9] == 3);
  assert(out[10] == 3);
  assert(out[11] == 17);
  assert(std::string(reinterpret_cast<const char*>(out.data() + 12), 17) == "RB-D-001122334455");
  assert(out[29] == 5);
  assert(std::string(reinterpret_cast<const char*>(out.data() + 30), 5) == "1.4.0");

  // 2. Computer Device
  len = encodeDeviceInfoV2(out.data(), out.size(),
                           1, 1, 1, 0x001F, 200, 1, 1, 3,
                           "RC-D-AABBCCDDEEFF", "1.4.0");
  assert(len == 12 + 17 + 1 + 5);
  assert(out[0] == 2);
  assert(out[1] == 1);
  assert(out[2] == 1);
  assert(out[3] == 1);
  assert(remus::blade::protocol::readU16(out.data() + 4) == 0x001F);
  assert(remus::blade::protocol::readU16(out.data() + 6) == 200);
  assert(out[8] == 1);
  assert(out[9] == 1);
  assert(out[10] == 3);
  assert(out[11] == 17);
  assert(std::string(reinterpret_cast<const char*>(out.data() + 12), 17) == "RC-D-AABBCCDDEEFF");
  assert(out[29] == 5);
  assert(std::string(reinterpret_cast<const char*>(out.data() + 30), 5) == "1.4.0");
}

// Standalone encoder for testing Device Info v3
size_t encodeDeviceInfoV3(uint8_t* out, size_t capacity, uint8_t family,
                          uint8_t model, uint8_t hardware, uint16_t capabilities,
                          uint16_t imuRateHz, uint8_t accelRange, uint8_t gyroRange,
                          uint8_t dlpfSetting, uint32_t deviceBootId,
                          const std::string& serial, const std::string& firmware) {
  const size_t serialLength = std::min<size_t>(serial.length(), 31);
  const size_t firmwareLength = firmware.length();
  const size_t required = 16 + serialLength + 1 + firmwareLength;
  if (!out || capacity < required) return 0;
  out[0] = 3; // version 3
  out[1] = family;
  out[2] = model;
  out[3] = hardware;
  remus::blade::protocol::writeU16(out + 4, capabilities);
  remus::blade::protocol::writeU16(out + 6, imuRateHz);
  out[8] = accelRange;
  out[9] = gyroRange;
  out[10] = dlpfSetting;
  remus::blade::protocol::writeU32(out + 11, deviceBootId);
  out[15] = static_cast<uint8_t>(serialLength);
  std::memcpy(out + 16, serial.c_str(), serialLength);
  out[16 + serialLength] = static_cast<uint8_t>(firmwareLength);
  std::memcpy(out + 17 + serialLength, firmware.c_str(), firmwareLength);
  return required;
}

void testDeviceInfoV3() {
  std::array<uint8_t, 128> out{};
  size_t len = encodeDeviceInfoV3(out.data(), out.size(),
                                  2, 1, 1, 0x0003, 200, 2, 3, 3, 0xA1B2C3D4,
                                  "RB-D-001122334455", "1.4.0");
  assert(len == 16 + 17 + 1 + 5);
  assert(out[0] == 3);
  assert(out[1] == 2);
  assert(remus::blade::protocol::readU32(out.data() + 11) == 0xA1B2C3D4);
  assert(out[15] == 17);
  assert(std::string(reinterpret_cast<const char*>(out.data() + 16), 17) == "RB-D-001122334455");
}

void testClockSyncCodec() {
  // Request
  std::array<uint8_t, 14> req{};
  req[0] = 1;
  req[1] = 0x04;
  remus::blade::protocol::writeU32(req.data() + 2, 0x12345678);
  remus::blade::protocol::writeU64(req.data() + 6, 0xAABBCCDDEEFF1122ULL);
  
  assert(req[0] == 1);
  assert(req[1] == 0x04);
  assert(remus::blade::protocol::readU32(req.data() + 2) == 0x12345678);
  assert(remus::blade::protocol::readU64(req.data() + 6) == 0xAABBCCDDEEFF1122ULL);

  // Response V1 (30 bytes legacy)
  std::array<uint8_t, 30> resp{};
  resp[0] = 1;
  resp[1] = 0x04;
  remus::blade::protocol::writeU32(resp.data() + 2, 0x12345678);
  remus::blade::protocol::writeU64(resp.data() + 6, 0xAABBCCDDEEFF1122ULL);
  remus::blade::protocol::writeU64(resp.data() + 14, 0x1122334455667788ULL);
  remus::blade::protocol::writeU64(resp.data() + 22, 0x99AABBCCDDEEFF00ULL);

  assert(resp[0] == 1);
  assert(resp[1] == 0x04);
  assert(remus::blade::protocol::readU32(resp.data() + 2) == 0x12345678);
  assert(remus::blade::protocol::readU64(resp.data() + 6) == 0xAABBCCDDEEFF1122ULL);
  assert(remus::blade::protocol::readU64(resp.data() + 14) == 0x1122334455667788ULL);
  assert(remus::blade::protocol::readU64(resp.data() + 22) == 0x99AABBCCDDEEFF00ULL);

  // Response V2 (34 bytes with deviceBootId)
  std::array<uint8_t, 34> respV2{};
  size_t v2Len = remus::blade::protocol::encodeClockSyncResponse(
      respV2.data(), respV2.size(), 0x12345678, 0xAABBCCDDEEFF1122ULL,
      0x1122334455667788ULL, 0x99AABBCCDDEEFF00ULL, 0xA1B2C3D4);
  assert(v2Len == 34);
  assert(respV2[0] == 1);
  assert(respV2[1] == 0x04);
  assert(remus::blade::protocol::readU32(respV2.data() + 2) == 0x12345678);
  assert(remus::blade::protocol::readU64(respV2.data() + 6) == 0xAABBCCDDEEFF1122ULL);
  assert(remus::blade::protocol::readU64(respV2.data() + 14) == 0x1122334455667788ULL);
  assert(remus::blade::protocol::readU64(respV2.data() + 22) == 0x99AABBCCDDEEFF00ULL);
  assert(remus::blade::protocol::readU32(respV2.data() + 30) == 0xA1B2C3D4);
}

// MPU Range conversion references
enum class MockAccelRange : uint8_t { G8 = 8, G16 = 16 };
enum class MockGyroRange : uint16_t { Dps500 = 500, Dps1000 = 1000, Dps2000 = 2000 };

uint8_t getAccelRegisterValue(MockAccelRange range) {
  return range == MockAccelRange::G16 ? 0x18 : 0x10;
}

float getAccelScaleFactor(MockAccelRange range) {
  return range == MockAccelRange::G16 ? 2048.0f : 4096.0f;
}

uint8_t getGyroRegisterValue(MockGyroRange range) {
  return range == MockGyroRange::Dps2000 ? 0x18 :
         range == MockGyroRange::Dps1000 ? 0x10 : 0x08;
}

float getGyroScaleFactor(MockGyroRange range) {
  return range == MockGyroRange::Dps2000 ? 16.4f :
         range == MockGyroRange::Dps1000 ? 32.8f : 65.5f;
}

void testMpuRange() {
  assert(getAccelRegisterValue(MockAccelRange::G16) == 0x18);
  assert(getAccelScaleFactor(MockAccelRange::G16) == 2048.0f);

  assert(getAccelRegisterValue(MockAccelRange::G8) == 0x10);
  assert(getAccelScaleFactor(MockAccelRange::G8) == 4096.0f);

  assert(getGyroRegisterValue(MockGyroRange::Dps2000) == 0x18);
  assert(getGyroScaleFactor(MockGyroRange::Dps2000) == 16.4f);

  assert(getGyroRegisterValue(MockGyroRange::Dps1000) == 0x10);
  assert(getGyroScaleFactor(MockGyroRange::Dps1000) == 32.8f);

  assert(getGyroRegisterValue(MockGyroRange::Dps500) == 0x08);
  assert(getGyroScaleFactor(MockGyroRange::Dps500) == 65.5f);
}

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

  testDeviceInfoV2();
  testDeviceInfoV3();
  testClockSyncCodec();
  testMpuRange();

  std::cout << "blade_protocol_smoke OK\n";
}
