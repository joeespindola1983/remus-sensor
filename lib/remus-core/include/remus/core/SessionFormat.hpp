#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace remus::session {

// RBP1 is intentionally kept byte-compatible with the existing ESP32 firmware.
// Prototype 1 and Prototype 2 can therefore generate files handled by the same
// importer/analyser.
#pragma pack(push, 1)
struct FileHeader {
  char magic[4];          // "RBP1"
  uint8_t version;        // 1
  uint8_t imu_fs_accel;   // 1 = +/-8g (4096 LSB/g)
  uint8_t imu_fs_gyro;    // 1 = +/-500 dps (65.5 LSB/dps)
  uint8_t imu_rate_hz;    // 200
  uint32_t started_at_ms; // monotonic milliseconds
  uint32_t session_id;    // random token
  uint8_t padding[16];
};

struct ImuRecord {
  uint8_t type;           // 0x01
  uint32_t timestamp_ms;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

struct GpsRecord {
  uint8_t type;           // 0x02
  uint32_t timestamp_ms;
  int32_t lat_e7;
  int32_t lon_e7;
  uint16_t speed_cm_s;
  uint8_t sats_in_use;
  uint8_t max_snr;
  uint16_t accuracy_cm;
};

// RBP2 preserves the RBP1 IMU/SPM records and adds receiver-native GNSS
// evidence. Position and velocity must come from the same GPS time-of-week.
struct GpsV2Record {
  uint8_t type;           // 0x04
  uint32_t timestamp_ms;  // ESP32 monotonic clock
  uint32_t gps_itow_ms;   // receiver GPS time-of-week
  int32_t lat_e7;
  int32_t lon_e7;
  uint32_t ground_speed_cm_s;
  uint32_t speed_accuracy_cm_s;
  int32_t course_deg_e5;
  uint32_t course_accuracy_deg_e5;
  uint32_t horizontal_accuracy_mm;
  uint8_t sats_in_use;
  uint8_t max_snr;
  uint8_t fix_type;
  uint8_t flags;          // bit 0: receiver fix valid
};

struct SpmRecord {
  uint8_t type;           // 0x03
  uint32_t timestamp_ms;
  uint16_t spm_x10;
};
#pragma pack(pop)

static_assert(sizeof(FileHeader) == 32, "RBP1 header contract changed");
static_assert(sizeof(ImuRecord) == 17, "RBP1 IMU record contract changed");
static_assert(sizeof(GpsRecord) == 19, "RBP1 GPS record contract changed");
static_assert(sizeof(GpsV2Record) == 41, "RBP2 GPS record contract changed");
static_assert(sizeof(SpmRecord) == 7, "RBP1 SPM record contract changed");

inline FileHeader makeHeader(uint32_t startedAtMs, uint32_t sessionId, uint16_t imuRateHz) {
  FileHeader header{};
  std::memcpy(header.magic, "RBP1", 4);
  header.version = 1;
  header.imu_fs_accel = 1;
  header.imu_fs_gyro = 1;
  header.imu_rate_hz = static_cast<uint8_t>(imuRateHz > 255 ? 255 : imuRateHz);
  header.started_at_ms = startedAtMs;
  header.session_id = sessionId;
  return header;
}

inline FileHeader makeHeaderV2(uint32_t startedAtMs, uint32_t sessionId, uint16_t imuRateHz) {
  FileHeader header = makeHeader(startedAtMs, sessionId, imuRateHz);
  std::memcpy(header.magic, "RBP2", 4);
  header.version = 2;
  return header;
}

}  // namespace remus::session
