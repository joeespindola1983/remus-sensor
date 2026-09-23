#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "remus/drivers/LinuxSessionStorage.hpp"

int main() {
  const auto dir = std::filesystem::temp_directory_path() / "remus_session_smoke";
  std::filesystem::remove_all(dir);
  remus::drivers::LinuxSessionStorage storage(dir.string());
  assert(storage.begin());
  assert(storage.startSession(1234, 200, 2));

  remus::session::ImuRecord imu{}; imu.type = 0x01; imu.timestamp_ms = 1235;
  remus::session::GpsRecord gps{}; gps.type = 0x02; gps.timestamp_ms = 1240;
  remus::session::SpmRecord spm{}; spm.type = 0x03; spm.timestamp_ms = 1300; spm.spm_x10 = 245;
  assert(storage.writeImu(imu));
  assert(storage.writeGps(gps));
  assert(storage.writeSpm(spm));
  const std::string path = storage.currentPath();
  storage.stopSession();

  assert(std::filesystem::file_size(path) == 32 + 17 + 19 + 7);
  {
    std::ifstream in(path, std::ios::binary);
    remus::session::FileHeader header{};
    in.read(reinterpret_cast<char*>(&header), sizeof(header));
    assert(std::string(header.magic, header.magic + 4) == "RBP1");
    assert(header.imu_rate_hz == 200);
    assert(header.padding[0] == 2);
    assert(header.padding[2] == 1);
    assert(header.padding[3] == 0x13);
  }

  // Test Blade / No-GPS session
  assert(storage.startSession(2000, 200, 2, false));
  assert(storage.writeImu(imu));
  assert(storage.writeSpm(spm));
  const std::string noGpsPath = storage.currentPath();
  storage.stopSession();

  assert(std::filesystem::file_size(noGpsPath) == 32 + 17 + 7);
  {
    std::ifstream in(noGpsPath, std::ios::binary);
    remus::session::FileHeader headerNoGps{};
    in.read(reinterpret_cast<char*>(&headerNoGps), sizeof(headerNoGps));
    assert(std::string(headerNoGps.magic, headerNoGps.magic + 4) == "RBP1");
    assert(headerNoGps.imu_rate_hz == 200);
    assert(headerNoGps.padding[0] == 2);
    assert(headerNoGps.padding[2] == 0); // No GPS
    assert(headerNoGps.padding[3] == 0x11); // Storage + display without GPS
  }

  std::filesystem::remove_all(dir);
  std::cout << "session_storage_smoke OK\n";
}
