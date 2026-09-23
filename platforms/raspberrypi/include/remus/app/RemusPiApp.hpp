#pragma once

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "remus/core/LiveSpmEstimator.hpp"
#include "remus/drivers/LinuxMpu6050.hpp"
#include "remus/drivers/LinuxNeo6m.hpp"
#include "remus/drivers/LinuxSessionStorage.hpp"
#include "remus/drivers/LinuxSsd1306.hpp"
#include "remus/profiles/Prototype2.hpp"

namespace remus::app {

struct PiAppOptions {
  std::string i2cDevice;
  std::string gpsDevice;
  std::string sessionDir;
  bool autoStart = false;
  bool oledEnabled = true;
  bool gpsEnabled = true;
  bool rawNmea = false;
};

class RemusPiApp {
public:
  explicit RemusPiApp(PiAppOptions options);
  ~RemusPiApp();

  bool begin();
  int run(const volatile sig_atomic_t* externalStop = nullptr);
  void requestStop();

  bool startWorkout();
  void stopWorkout();

private:
  struct SpmInputSample {
    uint32_t timestampMs = 0;
    float axMps2 = 0;
    float ayMps2 = 0;
    float azMps2 = 0;
    uint32_t generation = 0;
  };

  uint32_t millisSinceBoot() const;
  void imuLoop();
  void spmLoop();
  void processGps();
  void processConsole();
  void printStatus();
  void updateDisplay();
  int gpsBars() const;
  void clearSpmQueue();

  PiAppOptions options_;
  const profiles::RaspberryPiProfile& profile_ = profiles::Prototype2;
  std::chrono::steady_clock::time_point bootTime_;

  drivers::LinuxMpu6050 imu_;
  drivers::LinuxNeo6m gps_;
  drivers::LinuxSessionStorage storage_;
  std::unique_ptr<drivers::LinuxSsd1306> display_;

  std::atomic<bool> running_{false};
  std::atomic<bool> workoutActive_{false};
  std::atomic<float> liveSpm_{0.0f};
  std::atomic<uint32_t> generation_{0};

  std::thread imuThread_;
  std::thread spmThread_;

  mutable std::mutex telemetryMutex_;
  hal::ImuSample latestImu_{};
  std::atomic<uint32_t> imuReadFailures_{0};
  std::atomic<uint32_t> imuGapCount_{0};
  std::atomic<uint32_t> maxImuGapMs_{0};
  std::atomic<uint32_t> spmQueueDrops_{0};

  std::mutex spmQueueMutex_;
  std::condition_variable spmQueueCv_;
  std::deque<SpmInputSample> spmQueue_;
  static constexpr size_t kSpmQueueMax = 128;

  live::LiveSpmEstimator spmEstimator_;
  std::chrono::steady_clock::time_point lastFlush_{};
  std::chrono::steady_clock::time_point lastStatus_{};
  std::chrono::steady_clock::time_point lastDisplay_{};
  std::chrono::steady_clock::time_point lastHardwareRetry_{};

  void initIpc();
  void closeIpc();
  void pollIpc();
  void broadcastIpc(const std::string& msg);
  void handleIpcCommand(const std::string& cmd, int clientFd);

  std::string ipcSocketPath_;
  int ipcServerFd_ = -1;
  std::vector<int> ipcClientFds_;
};

}  // namespace remus::app
