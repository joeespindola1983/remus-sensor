#include "remus/app/RemusPiApp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

#include "remus/core/SessionFormat.hpp"
#include "remus/core/SplitCalculator.hpp"

namespace remus::app {
namespace {
constexpr uint8_t kSpmDownsampleFactor = 8; // 200 Hz -> 25 Hz preview
constexpr double kGravity = 9.80665;
}

RemusPiApp::RemusPiApp(PiAppOptions options)
    : options_(std::move(options)),
      bootTime_(std::chrono::steady_clock::now()),
      imu_(options_.i2cDevice, profile_.imuAddress, profile_.imuRateHz),
      gps_(options_.gpsDevice, profile_.gpsBaud),
      storage_(options_.sessionDir) {
  if (options_.oledEnabled && profile_.hasOled) {
    display_ = std::make_unique<drivers::LinuxSsd1306>(options_.i2cDevice, profile_.oledAddress);
  }
}

RemusPiApp::~RemusPiApp() {
  requestStop();
  if (imuThread_.joinable()) imuThread_.join();
  if (spmThread_.joinable()) spmThread_.join();
  storage_.stopSession();
  closeIpc();
}

uint32_t RemusPiApp::millisSinceBoot() const {
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - bootTime_).count();
  return static_cast<uint32_t>(elapsed & 0xFFFFFFFFu);
}

bool RemusPiApp::begin() {
  std::cout << "\n=========================================================\n"
            << " REMUS - MULTI-PLATFORM TELEMETRY\n"
            << " Profile: " << profile_.displayName << " (" << profile_.code << ")\n"
            << "=========================================================\n"
            << " I2C : " << options_.i2cDevice << "\n"
            << " IMU : MPU-6050 @ 0x" << std::hex << int(profile_.imuAddress) << std::dec
            << " / " << profile_.imuRateHz << " Hz\n"
            << " GPS : " << (options_.gpsEnabled ? (options_.gpsDevice + " / " + std::to_string(profile_.gpsBaud) + " baud") : "disabled") << "\n"
            << " DATA: " << options_.sessionDir << "\n"
            << " OLED: " << (display_ ? "enabled" : "disabled") << "\n"
            << "=========================================================\n";

  if (!storage_.begin()) return false;
  if (!imu_.begin()) {
    std::cerr << "[IMU] Warning: MPU-6050 not detected at " << options_.i2cDevice
              << "; continuing and retrying dynamically.\n";
  }
  if (options_.gpsEnabled) {
    if (!gps_.begin()) {
      std::cerr << "[GPS] Continuing without GPS; IMU/session capture remains available.\n";
    }
  } else {
    std::cout << "[GPS] Disabled (--no-gps). Running in IMU-only blade capture mode.\n";
  }
  if (display_ && !display_->begin()) {
    std::cerr << "[OLED] Continuing without display.\n";
    display_.reset();
  }

  initIpc();
  running_.store(true);
  lastFlush_ = lastStatus_ = lastDisplay_ = lastHardwareRetry_ = std::chrono::steady_clock::now();
  imuThread_ = std::thread(&RemusPiApp::imuLoop, this);
  spmThread_ = std::thread(&RemusPiApp::spmLoop, this);

  if (options_.autoStart && !startWorkout()) return false;
  return true;
}

void RemusPiApp::requestStop() {
  running_.store(false);
  workoutActive_.store(false);
  spmQueueCv_.notify_all();
}

void RemusPiApp::clearSpmQueue() {
  std::lock_guard<std::mutex> lock(spmQueueMutex_);
  spmQueue_.clear();
}

bool RemusPiApp::startWorkout() {
  if (workoutActive_.load()) return true;
  const uint32_t newGeneration = generation_.fetch_add(1) + 1;
  (void)newGeneration;
  clearSpmQueue();
  liveSpm_.store(0.0f);
  if (!storage_.startSession(millisSinceBoot(), profile_.imuRateHz, 2, options_.gpsEnabled)) return false;
  workoutActive_.store(true);
  std::cout << "[SESSION] START\n";
  return true;
}

void RemusPiApp::stopWorkout() {
  if (!workoutActive_.exchange(false)) return;
  generation_.fetch_add(1);
  clearSpmQueue();
  liveSpm_.store(0.0f);
  storage_.stopSession();
  std::cout << "[SESSION] STOP\n";
}

void RemusPiApp::imuLoop() {
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::microseconds(1000000 / std::max<uint16_t>(1, profile_.imuRateHz));
  auto next = clock::now();
  auto previousSample = next;

  uint8_t count = 0;
  float sx = 0, sy = 0, sz = 0;
  uint32_t accumulatorGeneration = 0;

  while (running_.load()) {
    next += period;
    std::this_thread::sleep_until(next);
    const auto now = clock::now();
    const auto dtMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - previousSample).count();
    previousSample = now;
    if (dtMs > 6) {
      imuGapCount_.fetch_add(1);
      uint32_t old = maxImuGapMs_.load();
      while (static_cast<uint32_t>(dtMs) > old && !maxImuGapMs_.compare_exchange_weak(old, static_cast<uint32_t>(dtMs))) {}
    }

    hal::ImuSample sample{};
    if (!imu_.read(sample)) {
      imuReadFailures_.fetch_add(1);
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(telemetryMutex_);
      latestImu_ = sample;
    }

    if (!workoutActive_.load()) {
      count = 0;
      sx = sy = sz = 0;
      continue;
    }

    const uint32_t ts = millisSinceBoot();
    session::ImuRecord record{};
    record.type = 0x01;
    record.timestamp_ms = ts;
    record.ax = sample.rawAx; record.ay = sample.rawAy; record.az = sample.rawAz;
    record.gx = sample.rawGx; record.gy = sample.rawGy; record.gz = sample.rawGz;
    storage_.writeImu(record);

    const uint32_t currentGeneration = generation_.load();
    if (accumulatorGeneration != currentGeneration) {
      accumulatorGeneration = currentGeneration;
      count = 0;
      sx = sy = sz = 0;
    }

    sx += static_cast<float>(sample.accelX * kGravity);
    sy += static_cast<float>(sample.accelY * kGravity);
    sz += static_cast<float>(sample.accelZ * kGravity);
    ++count;

    if (count >= kSpmDownsampleFactor) {
      const float inv = 1.0f / count;
      SpmInputSample input{ts, sx * inv, sy * inv, sz * inv, currentGeneration};
      {
        std::lock_guard<std::mutex> lock(spmQueueMutex_);
        if (spmQueue_.size() >= kSpmQueueMax) {
          spmQueue_.pop_front();
          spmQueueDrops_.fetch_add(1);
        }
        spmQueue_.push_back(input);
      }
      spmQueueCv_.notify_one();
      count = 0;
      sx = sy = sz = 0;
    }
  }
}

void RemusPiApp::spmLoop() {
  uint32_t estimatorGeneration = 0;
  spmEstimator_.setPreferredAxis(-1);

  while (running_.load()) {
    SpmInputSample sample{};
    {
      std::unique_lock<std::mutex> lock(spmQueueMutex_);
      spmQueueCv_.wait_for(lock, std::chrono::milliseconds(100), [&] {
        return !running_.load() || !spmQueue_.empty();
      });
      if (!running_.load()) break;
      if (spmQueue_.empty()) continue;
      sample = spmQueue_.front();
      spmQueue_.pop_front();
    }

    if (!workoutActive_.load() || sample.generation != generation_.load()) continue;
    if (sample.generation != estimatorGeneration) {
      spmEstimator_.reset();
      spmEstimator_.setPreferredAxis(-1);
      estimatorGeneration = sample.generation;
    }

    const auto result = spmEstimator_.push(
        sample.timestampMs / 1000.0,
        sample.axMps2,
        sample.ayMps2,
        sample.azMps2);
    if (!result.updated) continue;
    if (!workoutActive_.load() || sample.generation != generation_.load()) continue;

    if (result.available) {
      const float spm = static_cast<float>(result.stroke_rate_spm);
      liveSpm_.store(spm);
      if (!result.held) {
        session::SpmRecord rec{};
        rec.type = 0x03;
        rec.timestamp_ms = millisSinceBoot();
        rec.spm_x10 = static_cast<uint16_t>(std::clamp(spm * 10.0f, 0.0f, 65535.0f));
        storage_.writeSpm(rec);
      }
    } else if (result.progress >= 1.0) {
      liveSpm_.store(0.0f);
    }
  }
}

void RemusPiApp::processGps() {
  if (!gps_.healthy()) return;
  gps_.poll(options_.rawNmea, workoutActive_.load());
  if (!workoutActive_.load() || !gps_.fixValid() || !gps_.locationUpdated()) return;

  session::GpsRecord rec{};
  rec.type = 0x02;
  rec.timestamp_ms = millisSinceBoot();
  rec.lat_e7 = static_cast<int32_t>(gps_.latitude() * 1e7);
  rec.lon_e7 = static_cast<int32_t>(gps_.longitude() * 1e7);
  rec.speed_cm_s = static_cast<uint16_t>(std::clamp(gps_.speedKmph() * 27.7778f, 0.0f, 65535.0f));
  rec.sats_in_use = gps_.satellitesInUse();
  rec.max_snr = static_cast<uint8_t>(std::clamp(gps_.maxSnr(), 0, 255));
  rec.accuracy_cm = static_cast<uint16_t>(std::clamp(gps_.hdop() * 250.0f, 0.0f, 65535.0f));
  storage_.writeGps(rec);
}

int RemusPiApp::gpsBars() const {
  if (!gps_.healthy()) return 0;
  const int sats = gps_.satellitesInUse();
  const int snr = gps_.maxSnr();
  if (!gps_.fixValid()) return (gps_.satellitesInView() >= 3 || snr >= 20) ? 1 : 0;
  if (sats >= 7 && snr >= 30) return 3;
  if (sats >= 4) return 2;
  return 1;
}

void RemusPiApp::updateDisplay() {
  if (!display_) return;
  const double speedMps = (options_.gpsEnabled && gps_.fixValid()) ? gps_.speedKmph() / 3.6 : 0.0;
  const double split = options_.gpsEnabled ? metrics::splitSeconds500m(speedMps) : -1.0;
  display_->render(liveSpm_.load(), split, options_.gpsEnabled ? gpsBars() : 0, workoutActive_.load());
}

void RemusPiApp::printStatus() {
  hal::ImuSample sample{};
  {
    std::lock_guard<std::mutex> lock(telemetryMutex_);
    sample = latestImu_;
  }
  const double speedMps = (options_.gpsEnabled && gps_.fixValid()) ? gps_.speedKmph() / 3.6 : 0.0;
  const double split = options_.gpsEnabled ? metrics::splitSeconds500m(speedMps) : -1.0;
  std::cout << "[REMUS] "
            << (workoutActive_.load() ? "REC " : "IDLE ")
            << "IMU[" << std::fixed << std::setprecision(2)
            << sample.accelX << ',' << sample.accelY << ',' << sample.accelZ << "] "
            << "SPM=" << std::setprecision(1) << liveSpm_.load() << ' ';
  if (split > 0) {
    const int sec = static_cast<int>(std::lround(split));
    std::cout << "500=" << sec / 60 << ':' << std::setw(2) << std::setfill('0') << sec % 60 << std::setfill(' ') << ' ';
  } else {
    std::cout << "500=--:-- ";
  }
  if (options_.gpsEnabled) {
    std::cout << "GPS=" << (gps_.fixValid() ? "LOCK" : "SEARCH")
              << " sats=" << int(gps_.satellitesInUse()) << '/' << gps_.satellitesInView()
              << " snr=" << gps_.maxSnr() << ' ';
  } else {
    std::cout << "GPS=OFF ";
  }
  std::cout << "records=" << storage_.recordsWritten()
            << " gaps=" << imuGapCount_.load()
            << " maxGap=" << maxImuGapMs_.load() << "ms"
            << " imuErr=" << imuReadFailures_.load()
            << " spmDrop=" << spmQueueDrops_.load()
            << '\n';

  char bleBuf[256];
  char satsStr[32] = "0/0:0:0m";
  char latStr[20] = "";
  char lonStr[20] = "";
  char spdStr[16] = "";

  if (options_.gpsEnabled && gps_.fixValid()) {
    std::snprintf(latStr, sizeof(latStr), "%.6f", gps_.latitude());
    std::snprintf(lonStr, sizeof(lonStr), "%.6f", gps_.longitude());
    std::snprintf(spdStr, sizeof(spdStr), "%.2f", gps_.speedKmph());
    std::snprintf(satsStr, sizeof(satsStr), "%d/%d:%d:%.1fm",
                  int(gps_.satellitesInUse()), gps_.satellitesInView(),
                  gps_.maxSnr(), gps_.hdop() * 2.5f);
  }

  std::snprintf(bleBuf, sizeof(bleBuf),
                "%u,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%s,%s,%s,%s,%u,%u,%.1f,%d,%d,%u,%u,0,%u\n",
                millisSinceBoot(),
                sample.accelX, sample.accelY, sample.accelZ,
                sample.gyroX, sample.gyroY, sample.gyroZ,
                latStr, lonStr, spdStr, satsStr,
                storage_.recordsWritten(),
                options_.gpsEnabled ? static_cast<uint32_t>(gps_.charsProcessed()) : 0,
                liveSpm_.load(),
                imu_.healthy() ? 1 : 0,
                storage_.healthy() ? 1 : 0,
                imuGapCount_.load(),
                maxImuGapMs_.load(),
                imuReadFailures_.load());

  broadcastIpc(bleBuf);
}

void RemusPiApp::processConsole() {
  pollfd pfd{STDIN_FILENO, POLLIN, 0};
  if (::poll(&pfd, 1, 0) <= 0 || !(pfd.revents & POLLIN)) return;
  std::string cmd;
  if (!std::getline(std::cin, cmd)) return;
  std::transform(cmd.begin(), cmd.end(), cmd.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (cmd == "start" || cmd == "s") startWorkout();
  else if (cmd == "stop") stopWorkout();
  else if (cmd == "status") printStatus();
  else if (cmd == "nmea") options_.rawNmea = !options_.rawNmea;
  else if (cmd == "quit" || cmd == "q" || cmd == "exit") requestStop();
  else if (!cmd.empty()) std::cout << "Commands: start | stop | status | nmea | quit\n";
}

int RemusPiApp::run(const volatile sig_atomic_t* externalStop) {
  while (running_.load()) {
    if (externalStop && *externalStop) requestStop();
    if (options_.gpsEnabled) processGps();
    processConsole();
    pollIpc();
    const auto now = std::chrono::steady_clock::now();
    if (now - lastDisplay_ >= std::chrono::milliseconds(250)) {
      lastDisplay_ = now;
      updateDisplay();
    }
    if (now - lastStatus_ >= std::chrono::seconds(1)) {
      lastStatus_ = now;
      printStatus();
    }
    if (workoutActive_.load() && now - lastFlush_ >= std::chrono::seconds(60)) {
      lastFlush_ = now;
      storage_.flush();
    }
    if (now - lastHardwareRetry_ >= std::chrono::seconds(5)) {
      lastHardwareRetry_ = now;
      if (options_.gpsEnabled && !gps_.healthy()) gps_.begin();
      if (!imu_.healthy()) imu_.begin();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  stopWorkout();
  spmQueueCv_.notify_all();
  if (imuThread_.joinable()) imuThread_.join();
  if (spmThread_.joinable()) spmThread_.join();
  return 0;
}

void RemusPiApp::initIpc() {
  ipcSocketPath_ = "/run/remus/remus.sock";
  std::error_code ec;
  std::filesystem::create_directories("/run/remus", ec);
  if (ec) {
    ipcSocketPath_ = "/tmp/remus.sock";
  }

  ipcServerFd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (ipcServerFd_ < 0) {
    std::cerr << "[IPC] Failed to create unix socket: " << std::strerror(errno) << '\n';
    return;
  }

  int flags = ::fcntl(ipcServerFd_, F_GETFL, 0);
  ::fcntl(ipcServerFd_, F_SETFL, flags | O_NONBLOCK);

  ::unlink(ipcSocketPath_.c_str());

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, ipcSocketPath_.c_str(), sizeof(addr.sun_path) - 1);

  if (::bind(ipcServerFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    // If binding /run/remus failed due to permissions, fallback to /tmp
    ipcSocketPath_ = "/tmp/remus.sock";
    ::unlink(ipcSocketPath_.c_str());
    std::strncpy(addr.sun_path, ipcSocketPath_.c_str(), sizeof(addr.sun_path) - 1);
    if (::bind(ipcServerFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      std::cerr << "[IPC] Failed to bind socket: " << std::strerror(errno) << '\n';
      ::close(ipcServerFd_);
      ipcServerFd_ = -1;
      return;
    }
  }

  ::chmod(ipcSocketPath_.c_str(), 0777);

  if (::listen(ipcServerFd_, 5) < 0) {
    std::cerr << "[IPC] Failed to listen on " << ipcSocketPath_ << '\n';
    ::close(ipcServerFd_);
    ipcServerFd_ = -1;
    return;
  }

  std::cout << "[IPC] OK Unix socket listening at " << ipcSocketPath_ << '\n';
}

void RemusPiApp::closeIpc() {
  for (int fd : ipcClientFds_) {
    if (fd >= 0) ::close(fd);
  }
  ipcClientFds_.clear();
  if (ipcServerFd_ >= 0) {
    ::close(ipcServerFd_);
    ipcServerFd_ = -1;
    ::unlink(ipcSocketPath_.c_str());
  }
}

void RemusPiApp::pollIpc() {
  if (ipcServerFd_ < 0) return;

  while (true) {
    int clientFd = ::accept(ipcServerFd_, nullptr, nullptr);
    if (clientFd < 0) break;
    int flags = ::fcntl(clientFd, F_GETFL, 0);
    ::fcntl(clientFd, F_SETFL, flags | O_NONBLOCK);
    ipcClientFds_.push_back(clientFd);
  }

  std::vector<int> activeClients;
  char buf[256];
  for (int fd : ipcClientFds_) {
    ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      std::string cmd(buf);
      handleIpcCommand(cmd, fd);
      activeClients.push_back(fd);
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      activeClients.push_back(fd);
    } else {
      ::close(fd);
    }
  }
  ipcClientFds_ = std::move(activeClients);
}

void RemusPiApp::handleIpcCommand(const std::string& rawCmd, int clientFd) {
  std::string cmd = rawCmd;
  while (!cmd.empty() && (cmd.back() == '\r' || cmd.back() == '\n' || cmd.back() == ' ')) cmd.pop_back();
  std::string lowerCmd = cmd;
  std::transform(lowerCmd.begin(), lowerCmd.end(), lowerCmd.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (lowerCmd == "start") {
    startWorkout();
    const char* resp = "OK:STARTED\n";
    (void)::write(clientFd, resp, std::strlen(resp));
  } else if (lowerCmd == "stop") {
    stopWorkout();
    const char* resp = "OK:STOPPED\n";
    (void)::write(clientFd, resp, std::strlen(resp));
  } else if (lowerCmd == "status") {
    std::string resp = workoutActive_.load() ? "STATUS:RECORDING\n" : "STATUS:IDLE\n";
    (void)::write(clientFd, resp.c_str(), resp.size());
  }
}

void RemusPiApp::broadcastIpc(const std::string& msg) {
  std::vector<int> active;
  for (int fd : ipcClientFds_) {
    ssize_t written = ::write(fd, msg.c_str(), msg.size());
    if (written > 0 || (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
      active.push_back(fd);
    } else {
      ::close(fd);
    }
  }
  ipcClientFds_ = std::move(active);
}

}  // namespace remus::app
