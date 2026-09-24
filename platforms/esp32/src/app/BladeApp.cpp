#include <Arduino.h>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <algorithm>
#include <array>
#include <string>

#include "remus/ActiveProfile.hpp"
#include "remus/app/BladeApp.hpp"
#include "remus/core/BladeProtocol.hpp"
#include "remus/drivers/Mpu6050Imu.hpp"

namespace {

namespace protocol = remus::blade::protocol;

constexpr char kFirmwareVersion[] = "1.0.0";
constexpr char kServiceUuid[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char kDeviceInfoUuid[] = "beb5483f-36e1-4688-b7f5-ea07361b26a8";
constexpr char kControlUuid[] = "beb54840-36e1-4688-b7f5-ea07361b26a8";
constexpr char kImuStreamUuid[] = "beb54841-36e1-4688-b7f5-ea07361b26a8";
constexpr char kStatusUuid[] = "beb54842-36e1-4688-b7f5-ea07361b26a8";
constexpr char kClockSyncUuid[] = "beb54843-36e1-4688-b7f5-ea07361b26a8";
constexpr UBaseType_t kSampleQueueLength = 400;  // Two seconds at 200 Hz.

remus::drivers::Mpu6050Imu imuDevice(
    Wire, remus::hardware.imuPins.sda, remus::hardware.imuPins.scl,
    remus::hardware.imuRateHz);

Preferences preferences;
QueueHandle_t sampleQueue = nullptr;
TaskHandle_t samplerTaskHandle = nullptr;
TaskHandle_t bleTaskHandle = nullptr;

BLEServer* server = nullptr;
BLECharacteristic* controlCharacteristic = nullptr;
BLECharacteristic* streamCharacteristic = nullptr;
BLECharacteristic* statusCharacteristic = nullptr;
BLECharacteristic* clockSyncCharacteristic = nullptr;

volatile bool bleConnected = false;
volatile bool streamRequested = false;
volatile bool imuHealthy = false;
volatile uint32_t nextSampleSequence = 0;
volatile uint32_t nextBatchSequence = 0;
volatile uint32_t imuReadFailureCount = 0;
volatile uint32_t queueDropCount = 0;
volatile uint32_t notificationErrorCount = 0;

String deviceSerial;
char deviceName[32] = "REMUS-BLD-DEV";

uint32_t atomicRead(const volatile uint32_t* value) {
  return __atomic_load_n(value, __ATOMIC_RELAXED);
}

void atomicIncrement(volatile uint32_t* value) {
  __atomic_fetch_add(value, 1U, __ATOMIC_RELAXED);
}

void buildIdentity() {
  preferences.begin("remus_blade", false);
  deviceSerial = preferences.getString("serial", "");
  if (deviceSerial.isEmpty()) {
    const uint64_t efuse = ESP.getEfuseMac();
    char serial[24];
    snprintf(serial, sizeof(serial), "RB-D-%012llX",
             static_cast<unsigned long long>(efuse & 0xFFFFFFFFFFFFULL));
    deviceSerial = serial;
    preferences.putString("serial", deviceSerial);
  }
  preferences.end();

  const uint32_t shortHash = protocol::crc32(
      reinterpret_cast<const uint8_t*>(deviceSerial.c_str()), deviceSerial.length());
  snprintf(deviceName, sizeof(deviceName), "REMUS-BLD-%04lX",
           static_cast<unsigned long>(shortHash & 0xFFFFU));
}

size_t encodeDeviceInfo(uint8_t* out, size_t capacity) {
  const size_t serialLength = std::min<size_t>(deviceSerial.length(), 31);
  const size_t firmwareLength = sizeof(kFirmwareVersion) - 1;
  const size_t required = 11 + serialLength + 1 + firmwareLength;
  if (!out || capacity < required) return 0;

  out[0] = protocol::kVersion;
  out[1] = protocol::kDeviceFamilyBlade;
  out[2] = 1;  // Development Blade model revision.
  out[3] = 1;  // Development hardware revision.
  protocol::writeU16(out + 4, 0x0003);  // RAW IMU + live streaming.
  protocol::writeU16(out + 6, remus::hardware.imuRateHz);
  out[8] = 1;  // +/-8 g configuration identifier.
  out[9] = 1;  // +/-500 dps configuration identifier.
  out[10] = static_cast<uint8_t>(serialLength);
  memcpy(out + 11, deviceSerial.c_str(), serialLength);
  out[11 + serialLength] = static_cast<uint8_t>(firmwareLength);
  memcpy(out + 12 + serialLength, kFirmwareVersion, firmwareLength);
  return required;
}

void notifyControlAck(uint32_t requestId, protocol::ControlCommand command,
                      protocol::ControlStatus status) {
  if (!bleConnected || !controlCharacteristic) return;
  uint8_t response[9]{};
  response[0] = protocol::kVersion;
  response[1] = static_cast<uint8_t>(protocol::MessageType::ControlAck);
  response[2] = static_cast<uint8_t>(command);
  response[3] = static_cast<uint8_t>(status);
  protocol::writeU32(response + 4, requestId);
  response[8] = streamRequested ? 1 : 0;
  controlCharacteristic->setValue(response, sizeof(response));
  controlCharacteristic->notify();
}

class BladeServerCallbacks final : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer*) override {
    bleConnected = true;
    Serial.println("[BLADE] 📲 BLE Client connected!");
  }

  void onDisconnect(BLEServer* disconnectedServer) override {
    bleConnected = false;
    streamRequested = false;
    if (sampleQueue) xQueueReset(sampleQueue);
    disconnectedServer->startAdvertising();
    Serial.println("[BLADE] 📴 BLE Client disconnected. Advertising...");
  }
};

class ControlCallbacks final : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    const std::string value = characteristic->getValue();
    if (value.size() < 6) {
      notifyControlAck(0, protocol::ControlCommand::StopStream,
                       protocol::ControlStatus::InvalidCommand);
      return;
    }

    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(value.data());
    const uint32_t requestId = protocol::readU32(bytes + 2);
    if (bytes[0] != protocol::kVersion) {
      notifyControlAck(requestId, static_cast<protocol::ControlCommand>(bytes[1]),
                       protocol::ControlStatus::InvalidVersion);
      return;
    }

    const auto command = static_cast<protocol::ControlCommand>(bytes[1]);
    switch (command) {
      case protocol::ControlCommand::StartStream:
        if (!imuHealthy) {
          notifyControlAck(requestId, command, protocol::ControlStatus::ImuUnavailable);
          return;
        }
        if (sampleQueue) xQueueReset(sampleQueue);
        streamRequested = true;
        Serial.println("[BLADE] ▶️ IMU 200 Hz stream started");
        notifyControlAck(requestId, command, protocol::ControlStatus::Accepted);
        return;
      case protocol::ControlCommand::StopStream:
        streamRequested = false;
        if (sampleQueue) xQueueReset(sampleQueue);
        Serial.println("[BLADE] ⏹️ IMU stream stopped");
        notifyControlAck(requestId, command, protocol::ControlStatus::Accepted);
        return;
      default:
        notifyControlAck(requestId, command, protocol::ControlStatus::InvalidCommand);
        return;
    }
  }
};

class StreamCallbacks final : public BLECharacteristicCallbacks {
 public:
  void onStatus(BLECharacteristic*, Status status, uint32_t) override {
    if (status != SUCCESS_NOTIFY) atomicIncrement(&notificationErrorCount);
  }
};

class ClockSyncCallbacks final : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    const int64_t receivedAtUs = esp_timer_get_time();
    const std::string value = characteristic->getValue();
    if (value.size() < 14 || static_cast<uint8_t>(value[0]) != protocol::kVersion) return;
    const uint8_t* request = reinterpret_cast<const uint8_t*>(value.data());
    uint8_t response[30]{};
    response[0] = protocol::kVersion;
    response[1] = static_cast<uint8_t>(protocol::MessageType::ClockSyncResponse);
    memcpy(response + 2, request + 2, 12);  // request ID + phone t1.
    protocol::writeU64(response + 14, static_cast<uint64_t>(receivedAtUs));
    protocol::writeU64(response + 22, static_cast<uint64_t>(esp_timer_get_time()));
    characteristic->setValue(response, sizeof(response));
    characteristic->notify();
  }
};

void setupBle() {
  BLEDevice::init(deviceName);
  BLEDevice::setMTU(185);
  server = BLEDevice::createServer();
  server->setCallbacks(new BladeServerCallbacks());
  BLEService* service = server->createService(kServiceUuid);

  BLECharacteristic* info = service->createCharacteristic(
      kDeviceInfoUuid, BLECharacteristic::PROPERTY_READ);
  uint8_t deviceInfo[80]{};
  const size_t deviceInfoLength = encodeDeviceInfo(deviceInfo, sizeof(deviceInfo));
  info->setValue(deviceInfo, deviceInfoLength);

  controlCharacteristic = service->createCharacteristic(
      kControlUuid, BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_WRITE_NR |
                        BLECharacteristic::PROPERTY_NOTIFY);
  controlCharacteristic->addDescriptor(new BLE2902());
  controlCharacteristic->setCallbacks(new ControlCallbacks());

  streamCharacteristic = service->createCharacteristic(
      kImuStreamUuid, BLECharacteristic::PROPERTY_NOTIFY);
  streamCharacteristic->addDescriptor(new BLE2902());
  streamCharacteristic->setCallbacks(new StreamCallbacks());

  statusCharacteristic = service->createCharacteristic(
      kStatusUuid, BLECharacteristic::PROPERTY_READ |
                       BLECharacteristic::PROPERTY_NOTIFY);
  statusCharacteristic->addDescriptor(new BLE2902());

  clockSyncCharacteristic = service->createCharacteristic(
      kClockSyncUuid, BLECharacteristic::PROPERTY_WRITE |
                          BLECharacteristic::PROPERTY_NOTIFY);
  clockSyncCharacteristic->addDescriptor(new BLE2902());
  clockSyncCharacteristic->setCallbacks(new ClockSyncCallbacks());

  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(kServiceUuid);
  advertising->setScanResponse(true);

  BLEAdvertisementData advertisementData;
  advertisementData.setCompleteServices(BLEUUID(kServiceUuid));
  std::string manufacturerData;
  manufacturerData.push_back(static_cast<char>(protocol::kVersion));
  manufacturerData.push_back(static_cast<char>(protocol::kDeviceFamilyBlade));
  const uint32_t shortHash = protocol::crc32(
      reinterpret_cast<const uint8_t*>(deviceSerial.c_str()), deviceSerial.length());
  manufacturerData.push_back(static_cast<char>(shortHash & 0xFF));
  manufacturerData.push_back(static_cast<char>((shortHash >> 8) & 0xFF));
  advertisementData.setManufacturerData(manufacturerData);
  advertising->setAdvertisementData(advertisementData);

  BLEAdvertisementData scanResponseData;
  scanResponseData.setName(deviceName);
  advertising->setScanResponseData(scanResponseData);
  advertising->start();
}

void samplerTask(void*) {
  const TickType_t interval = pdMS_TO_TICKS(1000 / remus::hardware.imuRateHz);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, interval);
    if (!streamRequested || !bleConnected || !imuHealthy) continue;

    remus::hal::ImuSample sample{};
    const uint32_t sequence = __atomic_fetch_add(&nextSampleSequence, 1U, __ATOMIC_RELAXED);
    const uint64_t timestampUs = static_cast<uint64_t>(esp_timer_get_time());
    if (!imuDevice.read(sample)) {
      imuHealthy = false;
      atomicIncrement(&imuReadFailureCount);
      continue;
    }

    protocol::RawImuFrame frame{};
    frame.sampleSequence = sequence;
    frame.nativeTimestampUs = timestampUs;
    frame.ax = sample.rawAx;
    frame.ay = sample.rawAy;
    frame.az = sample.rawAz;
    frame.gx = sample.rawGx;
    frame.gy = sample.rawGy;
    frame.gz = sample.rawGz;
    if (xQueueSend(sampleQueue, &frame, 0) != pdTRUE) atomicIncrement(&queueDropCount);
  }
}

void sendBatch(const uint8_t* batch, size_t batchLength, uint32_t batchSequence) {
  if (!bleConnected || !streamCharacteristic || !batch || batchLength == 0) return;
  const size_t mtuPayload = BLEDevice::getMTU() > 3 ? BLEDevice::getMTU() - 3 : 20;
  if (batchLength <= mtuPayload) {
    streamCharacteristic->setValue(const_cast<uint8_t*>(batch), batchLength);
    streamCharacteristic->notify();
    return;
  }

  if (mtuPayload <= protocol::kFragmentHeaderSize) {
    atomicIncrement(&notificationErrorCount);
    return;
  }
  const size_t fragmentPayload = mtuPayload - protocol::kFragmentHeaderSize;
  const size_t fragmentCountSize = (batchLength + fragmentPayload - 1) / fragmentPayload;
  if (fragmentCountSize > 255) {
    atomicIncrement(&notificationErrorCount);
    return;
  }
  const uint8_t fragmentCount = static_cast<uint8_t>(fragmentCountSize);
  std::array<uint8_t, 256> frame{};
  for (uint8_t index = 0; index < fragmentCount && bleConnected; ++index) {
    const size_t offset = static_cast<size_t>(index) * fragmentPayload;
    const size_t length = std::min(fragmentPayload, batchLength - offset);
    const size_t encoded = protocol::encodeFragment(
        frame.data(), frame.size(), batchSequence, index, fragmentCount,
        batch + offset, length);
    streamCharacteristic->setValue(frame.data(), encoded);
    streamCharacteristic->notify();
    taskYIELD();
  }
}

void bleStreamingTask(void*) {
  std::array<protocol::RawImuFrame, protocol::kMaxSamplesPerBatch> samples{};
  std::array<uint8_t, protocol::kMaxBatchSize> encoded{};
  for (;;) {
    if (!streamRequested || !bleConnected) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    size_t count = 0;
    if (xQueueReceive(sampleQueue, &samples[count], pdMS_TO_TICKS(100)) == pdTRUE) {
      ++count;
    }
    while (count < samples.size() &&
           xQueueReceive(sampleQueue, &samples[count], pdMS_TO_TICKS(6)) == pdTRUE) {
      ++count;
    }
    if (count == 0) continue;

    const uint32_t batchSequence = __atomic_fetch_add(
        &nextBatchSequence, 1U, __ATOMIC_RELAXED);
    const size_t length = protocol::encodeImuBatch(
        encoded.data(), encoded.size(), batchSequence, samples.data(), count);
    sendBatch(encoded.data(), length, batchSequence);
  }
}

void publishStatus() {
  uint8_t status[26]{};
  status[0] = protocol::kVersion;
  status[1] = static_cast<uint8_t>(protocol::MessageType::Status);
  status[2] = streamRequested ? 2 : (bleConnected ? 1 : 0);
  status[3] = (imuHealthy ? 0x01 : 0x00) | (bleConnected ? 0x02 : 0x00);
  protocol::writeU32(status + 4, atomicRead(&nextSampleSequence));
  protocol::writeU32(status + 8, atomicRead(&nextBatchSequence));
  protocol::writeU32(status + 12, atomicRead(&imuReadFailureCount));
  protocol::writeU32(status + 16, atomicRead(&queueDropCount));
  protocol::writeU32(status + 20, atomicRead(&notificationErrorCount));
  protocol::writeU16(status + 24, BLEDevice::getMTU());
  if (statusCharacteristic) {
    statusCharacteristic->setValue(status, sizeof(status));
    if (bleConnected) statusCharacteristic->notify();
  }
}

}  // namespace

void remus::app::BladeApp::begin() {
  Serial.setTxBufferSize(4096);
  Serial.setRxBufferSize(1024);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(800);

  Serial.println("\n=========================================================");
  Serial.println("         REMUS BLADE — SENSOR DE PÁ / IMU BLE            ");
  Serial.printf("         Firmware %s | Protocol Major %u\n", kFirmwareVersion, protocol::kVersion);
  Serial.println("=========================================================");

  buildIdentity();
  sampleQueue = xQueueCreate(kSampleQueueLength, sizeof(protocol::RawImuFrame));
  imuHealthy = sampleQueue && imuDevice.begin();
  setupBle();

  if (sampleQueue) {
    xTaskCreate(samplerTask, "bladeSampler", 4096, nullptr, 5, &samplerTaskHandle);
    xTaskCreate(bleStreamingTask, "bladeBleTx", 4096, nullptr, 2, &bleTaskHandle);
  }
  Serial.printf("[BLADE] %s serial=%s SDA=%d SCL=%d IMU=%s firmware=%s\n",
                deviceName, deviceSerial.c_str(), remus::hardware.imuPins.sda,
                remus::hardware.imuPins.scl, imuHealthy ? "OK" : "ERROR",
                kFirmwareVersion);
  Serial.println("=========================================================\n");
  Serial.flush();
}

void remus::app::BladeApp::tick() {
  static uint32_t lastStatusMs = 0;
  static uint32_t lastSerialHeartbeatMs = 0;
  const uint32_t now = millis();

  // If user presses Enter or sends characters via Serial, reply with device info
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    Serial.printf("[BLADE] %s serial=%s SDA=%d SCL=%d IMU=%s BLE=%s samples=%lu drops=%lu firmware=%s\n",
                  deviceName, deviceSerial.c_str(), remus::hardware.imuPins.sda,
                  remus::hardware.imuPins.scl, imuHealthy ? "OK" : "ERROR",
                  bleConnected ? (streamRequested ? "STREAMING" : "CONNECTED") : "ADVERTISING",
                  static_cast<unsigned long>(atomicRead(&nextSampleSequence)),
                  static_cast<unsigned long>(atomicRead(&queueDropCount)),
                  kFirmwareVersion);
    Serial.flush();
  }

  if (now - lastStatusMs >= 1000) {
    lastStatusMs = now;
    if (!imuHealthy && !streamRequested) imuHealthy = imuDevice.begin();
    publishStatus();
  }

  // Periodic heartbeat so monitor is never silent
  if (now - lastSerialHeartbeatMs >= 3000) {
    lastSerialHeartbeatMs = now;
    Serial.printf("[BLADE] %s | IMU=%s | BLE=%s | samples=%lu | drops=%lu\n",
                  deviceName,
                  imuHealthy ? "OK" : "ERROR",
                  bleConnected ? (streamRequested ? "STREAMING" : "CONNECTED") : "ADV",
                  static_cast<unsigned long>(atomicRead(&nextSampleSequence)),
                  static_cast<unsigned long>(atomicRead(&queueDropCount)));
    Serial.flush();
  }

  delay(5);
}
