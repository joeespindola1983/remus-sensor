#include <Arduino.h>

#ifndef ARDUINO_USB_CDC_ON_BOOT
#define ARDUINO_USB_CDC_ON_BOOT 0
#endif
#ifndef ARDUINO_USB_MODE
#define ARDUINO_USB_MODE 0
#endif
#if !ARDUINO_USB_CDC_ON_BOOT || !ARDUINO_USB_MODE
#error "REMUS USB recovery requires -DARDUINO_USB_CDC_ON_BOOT=1 and -DARDUINO_USB_MODE=1 in platformio.ini"
#endif

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLERemoteCharacteristic.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <string>

#include "remus/ActiveProfile.hpp"
#include "remus/app/RemusApp.hpp"
#include "remus/core/BladeProtocol.hpp"
#include "remus/core/BladeRelayFormat.hpp"
#include "remus/core/BladeSlotRegistry.hpp"
#include "remus/core/DualBladeOrientation.hpp"
#include "remus/core/LiveSpmEstimator.hpp"
#include "remus/core/SessionFormat.hpp"
#include "remus/drivers/Gmt024Display.hpp"
#include "remus/drivers/Mpu6050Imu.hpp"
#include "remus/drivers/Neo6mGps.hpp"
#include "remus/drivers/SdCardStorage.hpp"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define IMU_STREAM_CHARACTERISTIC_UUID "beb54841-36e1-4688-b7f5-ea07361b26a8"
#define BLADE_CONTROL_CHARACTERISTIC_UUID "beb54840-36e1-4688-b7f5-ea07361b26a8"
#define BLADE_RELAY_CHARACTERISTIC_UUID "beb54844-36e1-4688-b7f5-ea07361b26a8"
#define BLADE_CLOCK_SYNC_CHARACTERISTIC_UUID "beb54843-36e1-4688-b7f5-ea07361b26a8"
#define REMUS_FIRMWARE_VERSION "1.3.0"

// Binary session records live in remus-core and are shared with Prototype 2.
using RemusFileHeader = remus::session::FileHeader;
using RemusImuRecord = remus::session::ImuRecord;
using RemusGpsRecord = remus::session::GpsRecord;
using RemusGpsV2Record = remus::session::GpsV2Record;
using RemusSpmRecord = remus::session::SpmRecord;

remus::drivers::Mpu6050Imu imuDevice(
  Wire, remus::hardware.imuPins.sda, remus::hardware.imuPins.scl, remus::hardware.imuRateHz);
remus::drivers::Neo6mGps gpsDevice(
  Serial1, remus::hardware.gpsPins.rx, remus::hardware.gpsPins.tx);
remus::drivers::SdCardStorage storageDevice(
  remus::hardware.sdPins.sck, remus::hardware.sdPins.miso,
  remus::hardware.sdPins.mosi, remus::hardware.sdPins.cs);
remus::drivers::Gmt024Display displayDevice(
  remus::hardware.displayPins.sck, remus::hardware.displayPins.mosi,
  remus::hardware.displayPins.dc, remus::hardware.displayPins.rst,
  remus::hardware.displayPins.cs);

remus::live::LiveSpmEstimator spmEstimator;
float liveSpm = 0.0;

// Display-only workout statistics. SPM uses half-stroke bins and pace uses
// whole seconds per 500 m. Histograms give an exact running median with fixed
// memory and no growing allocation during long sessions.
constexpr uint16_t DISPLAY_MAX_SPM_X2 = 160;       // 80.0 SPM
constexpr uint16_t DISPLAY_MAX_PACE_SECONDS = 1800; // 30:00 /500 m
uint32_t displaySpmHistogram[DISPLAY_MAX_SPM_X2 + 1]{};
uint32_t displayPaceHistogram[DISPLAY_MAX_PACE_SECONDS + 1]{};
uint32_t displaySpmSamples = 0;
uint32_t displayPaceSamples = 0;
unsigned long lastDisplayStatisticsSampleMs = 0;

Preferences prefs;
File logFile;
File transferFile;
File bladeRelayFiles[2];

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLECharacteristic* pImuStreamCharacteristic = NULL;
BLECharacteristic* pBladeRelayCharacteristic = NULL;
volatile bool bleConnected = false;
volatile bool bladeRosterDirty = true;
bool oldBleConnected = false;
char remusDeviceName[24] = "REMUS-ESP32";

volatile bool imuOk = false;
bool sdOk = false;
bool displayOk = false;
bool showRawNmea = false;

inline bool isGpsFixValid() {
  return remus::hardware.hasGps && gpsDevice.fixValid();
}

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t rawAcX = 0, rawAcY = 0, rawAcZ = 0;
int16_t rawGyX = 0, rawGyY = 0, rawGyZ = 0;

volatile unsigned long lastImuLoop = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastFlush = 0;
int fixCount = 0;
// Acquisition and durable persistence are deliberately separate facts.
// recordsQueued is incremented only after a complete record enters the RAM
// buffer. recordsPersisted is incremented only after all bytes of that record
// have been accepted by the MicroSD File implementation.
volatile unsigned long recordsQueued = 0;
volatile unsigned long recordsPersisted = 0;
volatile unsigned long bytesPersisted = 0;
volatile unsigned long storageWriteFailureCount = 0;
volatile bool recordingStorageFault = false;
volatile bool isWorkoutActive = false;
volatile bool bladeCalibrationRequested = false;
volatile unsigned long bladeCalibrationRequestedAtMs = 0;
volatile unsigned long imuGapCount = 0;
volatile unsigned long maxImuGapMs = 0;
volatile unsigned long imuReadFailureCount = 0;
volatile unsigned long gpsEpochsReceived = 0;
volatile unsigned long gpsValidEpochs = 0;
volatile unsigned long gpsInvalidEpochs = 0;
volatile unsigned long gpsItowGapCount = 0;
volatile unsigned long gpsMaxItowGapMs = 0;
uint32_t previousGpsItowMs = UINT32_MAX;
bool gpsSnapshotDirty = false;
unsigned long lastGpsNvsPersist = 0;
constexpr unsigned long GPS_NVS_IDLE_INTERVAL_MS = 60000;

// Estado de Transferência de Arquivo via BLE (GET)
volatile bool isTransferActive = false;
uint32_t transferTotalBytes = 0;
uint32_t transferOffset = 0;
uint32_t transferCrcState = 0xFFFFFFFFUL;
unsigned long lastTransferChunk = 0;

// Estado síncrono da última tentativa de notification usada pelo download.
// BLECharacteristic::notify() retorna void, mas a implementação Arduino-ESP32
// chama onStatus() antes de retornar, reportando SUCCESS_NOTIFY ou ERROR_GATT.
enum class TransferNotifyState : uint8_t { Idle, Pending, Success, Failed };
volatile TransferNotifyState transferNotifyState = TransferNotifyState::Idle;
volatile uint32_t transferNotifyStatusCode = 0;
volatile unsigned long transferNotifyErrors = 0;
volatile unsigned long transferNotifyRetries = 0;
unsigned long transferPacketCount = 0;
int transferLastLoggedPercent = -10;
uint16_t transferPeerMtu = 23;
size_t transferMaxPayload = 13;

// Estado de recuperação de arquivos do MicroSD via USB Serial.
// Este modo é ativado somente durante LIST/GET e nunca apaga ou altera arquivos.
volatile bool usbRecoveryTransferActive = false;
String usbRecoveryCommandBuffer;
// Recuperação USB prioriza integridade sobre throughput. Cada bloco vira uma
// linha ASCII hexadecimal e o próximo bloco só é enviado após ACK do host.
constexpr size_t USB_RECOVERY_CHUNK_SIZE = 192;
constexpr unsigned long USB_RECOVERY_WRITE_STALL_TIMEOUT_MS = 5000;
constexpr unsigned long USB_RECOVERY_ACK_TIMEOUT_MS = 10000;

// Nome único do arquivo de sessão no SD (ex: /remus_sensor_A1B2C3D4.bin)
char currentSessionFileName[64] = "/remus_session.bin";
char currentTransferFileName[64] = "";
char currentBladeRelayFileNames[2][64]{};
uint32_t currentSessionId = 0;
uint32_t currentSessionStartedAtMs = 0;

// Fila circular em RAM (16 KB = ~4,8s de IMU a 200 Hz). Ela absorve a
// latência normal do MicroSD sem alterar o contrato binário RBP1.
static RingbufHandle_t s_recordingRingBuf = NULL;
constexpr size_t RECORDING_RING_BUFFER_SIZE = 16384;
constexpr unsigned long IMU_GAP_THRESHOLD_MS = 6;
volatile unsigned long ringBufferOverflowCount = 0;
TaskHandle_t imuTaskHandle = NULL;
volatile bool isConfiguringMpu = false;
SemaphoreHandle_t recordingStateMutex = NULL;
size_t persistedRecordBytesRemaining = 0;

// The live app copy is an independent sink. Radio pressure may create an
// explicit sequence gap, but it must never block the acquisition task or the
// durable MicroSD path.
QueueHandle_t pcLiveImuQueue = NULL;
TaskHandle_t pcLiveImuTaskHandle = NULL;
constexpr UBaseType_t PC_LIVE_IMU_QUEUE_LENGTH = 400;  // Two seconds at 200 Hz.
volatile uint32_t pcLiveSampleSequence = 0;
volatile uint32_t pcLiveBatchSequence = 0;
volatile unsigned long pcLiveQueueDropCount = 0;

// The Computer is the primary BLE central for up to two Blades. The slot is a
// user-declared role and is never inferred from discovery order. Original Blade
// notifications share one tagged queue but keep independent clients, clocks,
// counters and source-specific RBR1 sidecars.
struct BladeRelayPacket {
  uint8_t channelIndex;
  uint32_t sourceIdentityHash;
  uint32_t receivedAtMs;
  uint16_t length;
  uint8_t payload[remus::blade::relay::kMaxPacketSize];
};
QueueHandle_t bladeRelayQueue = NULL;
TaskHandle_t bladeRelayClientTaskHandle = NULL;
constexpr UBaseType_t BLADE_RELAY_QUEUE_LENGTH = 32;
constexpr size_t BLADE_CHANNEL_COUNT = 2;
struct BladeChannel {
  BLEClient* client = NULL;
  BLERemoteCharacteristic* controlCharacteristic = NULL;
  BLERemoteCharacteristic* streamCharacteristic = NULL;
  BLERemoteCharacteristic* clockSyncCharacteristic = NULL;
  volatile bool connected = false;
  volatile bool streamCommanded = false;
  bool provisionallyDiscovered = false;
  uint8_t sourceAddress[6]{};
  uint32_t sourceIdentityHash = 0;
  uint32_t controlRequestId = 0;
  uint32_t clockSyncRequestId = 0;
  unsigned long lastClockSyncMs = 0;
  volatile unsigned long notificationsReceived = 0;
  volatile unsigned long packetsPersisted = 0;
  volatile unsigned long bytesPersisted = 0;
  volatile unsigned long queueDropCount = 0;
  volatile unsigned long writeFailureCount = 0;
  volatile unsigned long liveDropCount = 0;
  volatile bool storageFault = false;
  volatile uint32_t liveSequence = 0;
  remus::orientation::ClockMapping clockMapping{};
};
BladeChannel bladeChannels[BLADE_CHANNEL_COUNT]{};
remus::blade::SlotRegistry bladeSlotRegistry;
remus::orientation::DualBladeOrientation dualBladeOrientation;
portMUX_TYPE bladeClockMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE bladeSlotMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t bladeOrientationMutex = NULL;

remus::blade::SlotAssignment bladeAssignment(remus::blade::Slot slot) {
  portENTER_CRITICAL(&bladeSlotMux);
  const auto assignment = bladeSlotRegistry.get(slot);
  portEXIT_CRITICAL(&bladeSlotMux);
  return assignment;
}

uint32_t bladeSlotRevision() {
  portENTER_CRITICAL(&bladeSlotMux);
  const uint32_t revision = bladeSlotRegistry.revision();
  portEXIT_CRITICAL(&bladeSlotMux);
  return revision;
}

// Live SPM is only a low-priority preview for the app. The 200 Hz IMU task
// never runs the estimator. It only contributes a cheap 8-sample average
// (~25 Hz) to this queue; the expensive autocorrelation runs elsewhere.
struct LiveSpmInputSample {
  uint32_t timestamp_ms;
  float ax_mps2;
  float ay_mps2;
  float az_mps2;
  uint32_t generation;
};
QueueHandle_t liveSpmSampleQueue = NULL;
TaskHandle_t liveSpmTaskHandle = NULL;
constexpr uint8_t LIVE_SPM_DOWNSAMPLE_FACTOR = 8; // 200 Hz / 8 = 25 Hz
constexpr UBaseType_t LIVE_SPM_QUEUE_LENGTH = 128; // >5 s of preview samples
// -1 = auto (mounting-agnostic), 0 = X, 1 = Y, 2 = Z.
// Keep AUTO for the product. During controlled tests you can set the physical
// bow/stern axis here; it is only a soft preference, never a hard requirement.
constexpr int8_t LIVE_SPM_PREFERRED_AXIS = -1;
volatile uint32_t liveSpmGeneration = 0;
volatile unsigned long liveSpmQueueDropCount = 0;
// Preview live is expendable. It is disabled whenever no workout is active
// and explicitly silenced during file transfer so BLE download has priority.
volatile bool liveSpmPreviewEnabled = false;

enum class ControlCommandType : uint8_t { Start, Stop, Get };
struct ControlCommand {
  ControlCommandType type;
  char path[64];
};
QueueHandle_t controlCommandQueue = NULL;

portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

void startWorkoutRecording();
void stopWorkoutRecording();
void persistLatestGpsFix();
String findLatestSessionFileOnSd();
void startFileTransfer(const String& targetFile = "");
void processFileTransfer();
void imuSamplingTask(void* pvParameters);
void liveSpmProcessingTask(void* pvParameters);
void pcLiveImuStreamingTask(void* pvParameters);
void bladeRelayClientTask(void* pvParameters);
void processBladeRelayStorage(size_t maxRecords = 8);
void closeBladeRelayFile();
void notifyBladeRelayPacket(const BladeRelayPacket& packet);
void notifyBladeRoster(bool force = false);
void notifyBladeSlotState(const char* status);
void processControlCommands();
void setupIMU();
void setupSD();
void setupGPS();
void setupDisplay();
void updateDisplay(bool force = false);
void printHardwareProfile();
void printSavedReport();
void processUsbRecoveryInput();
void handleUsbRecoveryCommand(const String& command);
void listSdFilesOverUsb();
void sendSdFileOverUsb(const String& path, uint32_t requestedOffset);

void incrementCounter(volatile unsigned long* counter, unsigned long amount = 1) {
  __atomic_fetch_add(counter, amount, __ATOMIC_RELAXED);
}

unsigned long readCounter(const volatile unsigned long* counter) {
  return __atomic_load_n(counter, __ATOMIC_RELAXED);
}

void updateMaximum(volatile unsigned long* target, unsigned long candidate) {
  unsigned long current = readCounter(target);
  while (candidate > current &&
         !__atomic_compare_exchange_n(target, &current, candidate, false,
                                      __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {}
}

size_t recordingRecordSize(uint8_t type) {
  switch (type) {
    case 0x01: return sizeof(RemusImuRecord);
    case 0x03: return sizeof(RemusSpmRecord);
    case 0x04: return sizeof(RemusGpsV2Record);
    default: return 0;
  }
}

bool accountPersistedRecordBytes(const uint8_t* data, size_t length) {
  size_t offset = 0;
  while (offset < length) {
    if (persistedRecordBytesRemaining == 0) {
      persistedRecordBytesRemaining = recordingRecordSize(data[offset]);
      if (persistedRecordBytesRemaining == 0) return false;
    }
    const size_t available = length - offset;
    const size_t consumed = std::min(available, persistedRecordBytesRemaining);
    offset += consumed;
    persistedRecordBytesRemaining -= consumed;
    if (persistedRecordBytesRemaining == 0) incrementCounter(&recordsPersisted);
  }
  return true;
}

bool writeRecordingBytes(const uint8_t* data, size_t length) {
  size_t offset = 0;
  while (offset < length) {
    const size_t accepted = logFile.write(data + offset, length - offset);
    if (accepted == 0 || accepted > length - offset) {
      incrementCounter(&storageWriteFailureCount);
      __atomic_store_n(&recordingStorageFault, true, __ATOMIC_RELAXED);
      return false;
    }
    if (!accountPersistedRecordBytes(data + offset, accepted)) {
      incrementCounter(&storageWriteFailureCount);
      __atomic_store_n(&recordingStorageFault, true, __ATOMIC_RELAXED);
      return false;
    }
    incrementCounter(&bytesPersisted, accepted);
    offset += accepted;
  }
  return true;
}

void abortRecordingAfterStorageFailure(size_t attemptedBytes) {
  if (logFile) {
    logFile.flush();
    logFile.close();
  }
  sdOk = false;
  updateDisplay(true);
  Serial.printf(
    "[SD] ❌ FALHA DE PERSISTÊNCIA LOCAL após %lu bytes/%lu registros duráveis; bloco=%u bytes; falhas=%lu. Stream ao app continua ativo.\n",
    readCounter(&bytesPersisted), readCounter(&recordsPersisted),
    static_cast<unsigned int>(attemptedBytes), readCounter(&storageWriteFailureCount));
}

bool writeBladeRelayBytes(size_t channelIndex, const uint8_t* data, size_t length) {
  if (channelIndex >= BLADE_CHANNEL_COUNT) return false;
  BladeChannel& channel = bladeChannels[channelIndex];
  size_t offset = 0;
  while (offset < length) {
    const size_t accepted = bladeRelayFiles[channelIndex].write(data + offset, length - offset);
    if (accepted == 0 || accepted > length - offset) {
      incrementCounter(&channel.writeFailureCount);
      __atomic_store_n(&channel.storageFault, true, __ATOMIC_RELAXED);
      return false;
    }
    incrementCounter(&channel.bytesPersisted, accepted);
    offset += accepted;
  }
  return true;
}

bool openBladeRelayFileIfNeeded(size_t channelIndex) {
  if (channelIndex >= BLADE_CHANNEL_COUNT) return false;
  BladeChannel& channel = bladeChannels[channelIndex];
  if (bladeRelayFiles[channelIndex]) return true;
  if (!sdOk || currentSessionId == 0 ||
      channel.sourceIdentityHash == 0 ||
      __atomic_load_n(&channel.storageFault, __ATOMIC_RELAXED)) return false;
  bladeRelayFiles[channelIndex] = SD.open(currentBladeRelayFileNames[channelIndex], FILE_WRITE);
  if (!bladeRelayFiles[channelIndex]) {
    incrementCounter(&channel.writeFailureCount);
    __atomic_store_n(&channel.storageFault, true, __ATOMIC_RELAXED);
    Serial.printf("[BLADE RELAY] ❌ Não foi possível criar %s.\n",
                  currentBladeRelayFileNames[channelIndex]);
    return false;
  }
  const auto header = remus::blade::relay::makeHeader(
    currentSessionStartedAtMs, currentSessionId, channel.sourceAddress,
    channel.sourceIdentityHash);
  if (!writeBladeRelayBytes(channelIndex, reinterpret_cast<const uint8_t*>(&header), sizeof(header))) {
    bladeRelayFiles[channelIndex].close();
    return false;
  }
  bladeRelayFiles[channelIndex].flush();
  Serial.printf("[BLADE RELAY] 🔴 Backup do Blade iniciado: %s (source=%08lX).\n",
    currentBladeRelayFileNames[channelIndex],
    static_cast<unsigned long>(channel.sourceIdentityHash));
  return true;
}

void processBladeOrientation(const BladeRelayPacket& packet) {
  namespace protocol = remus::blade::protocol;
  if (packet.channelIndex >= BLADE_CHANNEL_COUNT ||
      packet.length < protocol::kBatchHeaderSize + protocol::kBatchCrcSize ||
      packet.payload[0] != protocol::kVersion ||
      packet.payload[1] != static_cast<uint8_t>(protocol::MessageType::ImuBatch)) return;
  const uint8_t sampleCount = packet.payload[22];
  const size_t required = protocol::kBatchHeaderSize +
    static_cast<size_t>(sampleCount) * protocol::kBytesPerSample + protocol::kBatchCrcSize;
  if (sampleCount == 0 || sampleCount > protocol::kMaxSamplesPerBatch ||
      required != packet.length ||
      protocol::readU32(packet.payload + required - protocol::kBatchCrcSize) !=
        protocol::crc32(packet.payload, required - protocol::kBatchCrcSize)) return;

  const remus::blade::Slot slot = packet.channelIndex == 0
    ? remus::blade::Slot::Left : remus::blade::Slot::Right;
  const auto assignment = bladeAssignment(slot);
  if (!assignment.configured || assignment.sourceIdentityHash != packet.sourceIdentityHash) return;
  if (!bladeOrientationMutex ||
      xSemaphoreTake(bladeOrientationMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
  const auto side = packet.channelIndex == 0
    ? remus::orientation::BladeSide::Left : remus::orientation::BladeSide::Right;
  remus::orientation::ClockMapping clockMapping{};
  portENTER_CRITICAL(&bladeClockMux);
  clockMapping = bladeChannels[packet.channelIndex].clockMapping;
  portEXIT_CRITICAL(&bladeClockMux);
  dualBladeOrientation.updateClockMapping(side, clockMapping);
  const uint64_t baseTimestampUs = protocol::readU64(packet.payload + 12);
  const uint16_t samplePeriodUs = protocol::readU16(packet.payload + 20);
  size_t offset = protocol::kBatchHeaderSize;
  constexpr float kDegreesToRadians = 0.01745329251994329577f;
  for (uint8_t index = 0; index < sampleCount; ++index) {
    auto readI16 = [&](size_t at) {
      return static_cast<int16_t>(protocol::readU16(packet.payload + at));
    };
    remus::orientation::Sample sample{};
    sample.accelerationG = {
      readI16(offset) / 4096.0f,
      readI16(offset + 2) / 4096.0f,
      readI16(offset + 4) / 4096.0f,
    };
    sample.rotationRateRadiansPerSecond = {
      readI16(offset + 6) / 65.5f * kDegreesToRadians,
      readI16(offset + 8) / 65.5f * kDegreesToRadians,
      readI16(offset + 10) / 65.5f * kDegreesToRadians,
    };
    const int16_t jitterUs = readI16(offset + 12);
    const int64_t timestampUs = static_cast<int64_t>(baseTimestampUs) +
      static_cast<int64_t>(index) * samplePeriodUs + jitterUs;
    if (timestampUs <= 0) {
      offset += protocol::kBytesPerSample;
      continue;
    }
    sample.nativeTimestampUs = static_cast<uint64_t>(timestampUs);
    dualBladeOrientation.push(side, packet.sourceIdentityHash, sample);
    offset += protocol::kBytesPerSample;
  }
  if (__atomic_load_n(&bladeCalibrationRequested, __ATOMIC_RELAXED)) {
    const auto snapshot = dualBladeOrientation.snapshot();
    const bool leftReady = snapshot.left.orientation.orientationQuality >=
      remus::orientation::Quality::Degraded;
    const bool rightReady = snapshot.right.orientation.orientationQuality >=
      remus::orientation::Quality::Degraded;
    if (leftReady && rightReady &&
        dualBladeOrientation.calibrate(remus::orientation::BladeSide::Left) &&
        dualBladeOrientation.calibrate(remus::orientation::BladeSide::Right)) {
      __atomic_store_n(&bladeCalibrationRequested, false, __ATOMIC_RELAXED);
      notifyBladeSlotState("CALIBRATED");
    }
  }
  xSemaphoreGive(bladeOrientationMutex);
}

void processBladeRelayStorage(size_t maxRecords) {
  if (!bladeRelayQueue) return;
  std::array<uint8_t,
    sizeof(remus::blade::relay::PacketRecordHeader) +
    remus::blade::relay::kMaxPacketSize +
    remus::blade::relay::kRecordCrcSize> encoded{};
  BladeRelayPacket packet{};
  for (size_t index = 0; index < maxRecords; ++index) {
    if (xQueueReceive(bladeRelayQueue, &packet, 0) != pdTRUE) break;
    processBladeOrientation(packet);
    if (!isWorkoutActive) continue;
    notifyBladeRelayPacket(packet);
    if (packet.channelIndex >= BLADE_CHANNEL_COUNT) continue;
    BladeChannel& channel = bladeChannels[packet.channelIndex];
    if (__atomic_load_n(&channel.storageFault, __ATOMIC_RELAXED)) continue;
    if (!openBladeRelayFileIfNeeded(packet.channelIndex)) continue;
    const size_t recordSize = remus::blade::relay::encodePacketRecord(
      encoded.data(), encoded.size(), packet.receivedAtMs,
      packet.payload, packet.length);
    if (recordSize == 0 ||
        !writeBladeRelayBytes(packet.channelIndex, encoded.data(), recordSize)) {
      bladeRelayFiles[packet.channelIndex].flush();
      bladeRelayFiles[packet.channelIndex].close();
      Serial.printf("[BLADE RELAY] ❌ Falha no sidecar após %lu pacotes; RBP2 principal continua.\n",
        readCounter(&channel.packetsPersisted));
      continue;
    }
    incrementCounter(&channel.packetsPersisted);
  }
}

void closeBladeRelayFile() {
  processBladeRelayStorage(BLADE_RELAY_QUEUE_LENGTH);
  for (size_t index = 0; index < BLADE_CHANNEL_COUNT; ++index) {
    BladeChannel& channel = bladeChannels[index];
    if (bladeRelayFiles[index]) {
      bladeRelayFiles[index].flush();
      bladeRelayFiles[index].close();
    }
    if (readCounter(&channel.notificationsReceived) == 0) continue;
    Serial.printf("[BLADE RELAY] %s '%s': recebidos=%lu persistidos=%lu drops=%lu bytes=%lu falhas=%lu.\n",
      __atomic_load_n(&channel.storageFault, __ATOMIC_RELAXED) ? "⚠️ INTERROMPIDO" : "⏹️ FINALIZADO",
      currentBladeRelayFileNames[index],
      readCounter(&channel.notificationsReceived),
      readCounter(&channel.packetsPersisted),
      readCounter(&channel.queueDropCount),
      readCounter(&channel.bytesPersisted),
      readCounter(&channel.writeFailureCount));
  }
}

bool queueControlCommand(ControlCommandType type, const String& path = "") {
  if (!controlCommandQueue) return false;
  ControlCommand command{};
  command.type = type;
  if (path.length() > 0) {
    snprintf(command.path, sizeof(command.path), "%s", path.c_str());
  }
  return xQueueSend(controlCommandQueue, &command, 0) == pdTRUE;
}

void persistLatestGpsFix() {
  if (!gpsSnapshotDirty || !isGpsFixValid()) return;

  prefs.begin("remus_gps", false);
  prefs.putBool("had_fix", true);
  prefs.putInt("fix_count", fixCount);
  prefs.putDouble("lat", gpsDevice.latitude());
  prefs.putDouble("lng", gpsDevice.longitude());
  prefs.putFloat("speed", gpsDevice.speedKmph());

  if (gpsDevice.timeValid()) {
    char timeBuf[16];
    sprintf(timeBuf, "%02d:%02d:%02d", gpsDevice.hour(), gpsDevice.minute(), gpsDevice.second());
    prefs.putString("time", String(timeBuf));
  }
  if (gpsDevice.dateValid()) {
    char dateBuf[16];
    sprintf(dateBuf, "%02d/%02d/%04d", gpsDevice.day(), gpsDevice.month(), gpsDevice.year());
    prefs.putString("date", String(dateBuf));
  }
  prefs.end();

  gpsSnapshotDirty = false;
  lastGpsNvsPersist = millis();
}

class RemusBLEServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleConnected = true;
    bladeRosterDirty = true;
    if (!usbRecoveryTransferActive) Serial.println("[BLE] 📲 Cliente conectado com sucesso!");
  };

  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    isTransferActive = false;
    if (transferFile) {
      transferFile.close();
    }
    if (!usbRecoveryTransferActive) Serial.println("[BLE] 📴 Cliente desconectado. Reiniciando anúncio...");
  }
};

int bladeChannelIndexForClient(BLEClient* client) {
  for (size_t index = 0; index < BLADE_CHANNEL_COUNT; ++index) {
    if (bladeChannels[index].client == client) return static_cast<int>(index);
  }
  return -1;
}

int bladeChannelIndexForCharacteristic(BLERemoteCharacteristic* characteristic) {
  for (size_t index = 0; index < BLADE_CHANNEL_COUNT; ++index) {
    if (bladeChannels[index].streamCharacteristic == characteristic ||
        bladeChannels[index].clockSyncCharacteristic == characteristic) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

void onBladeStreamNotification(BLERemoteCharacteristic* characteristic, uint8_t* data,
                               size_t length, bool) {
  if (!data || length == 0 || length > remus::blade::relay::kMaxPacketSize ||
      (!isWorkoutActive &&
       !__atomic_load_n(&bladeCalibrationRequested, __ATOMIC_RELAXED)) ||
      !bladeRelayQueue) return;
  const int channelIndex = bladeChannelIndexForCharacteristic(characteristic);
  if (channelIndex < 0) return;
  BladeChannel& channel = bladeChannels[channelIndex];
  BladeRelayPacket packet{};
  packet.channelIndex = static_cast<uint8_t>(channelIndex);
  packet.sourceIdentityHash = channel.sourceIdentityHash;
  packet.receivedAtMs = millis();
  packet.length = static_cast<uint16_t>(length);
  std::memcpy(packet.payload, data, length);
  incrementCounter(&channel.notificationsReceived);
  if (xQueueSend(bladeRelayQueue, &packet, 0) != pdTRUE) {
    incrementCounter(&channel.queueDropCount);
  }
}

void onBladeClockSyncNotification(BLERemoteCharacteristic* characteristic, uint8_t* data,
                                  size_t length, bool) {
  namespace protocol = remus::blade::protocol;
  if (!data || length < 30 || data[0] != protocol::kVersion ||
      data[1] != static_cast<uint8_t>(protocol::MessageType::ClockSyncResponse)) return;
  const int channelIndex = bladeChannelIndexForCharacteristic(characteristic);
  if (channelIndex < 0) return;
  BladeChannel& channel = bladeChannels[channelIndex];
  const uint64_t computerReceiveUs = static_cast<uint64_t>(esp_timer_get_time());
  const uint64_t computerSendUs = protocol::readU64(data + 6);
  const uint64_t bladeReceiveUs = protocol::readU64(data + 14);
  const uint64_t bladeSendUs = protocol::readU64(data + 22);
  if (computerReceiveUs < computerSendUs || bladeSendUs < bladeReceiveUs) return;
  const int64_t roundTripUs = static_cast<int64_t>(computerReceiveUs - computerSendUs) -
                              static_cast<int64_t>(bladeSendUs - bladeReceiveUs);
  if (roundTripUs < 0 || roundTripUs > 200000) return;
  const int64_t offsetUs = (
      static_cast<int64_t>(computerSendUs) - static_cast<int64_t>(bladeReceiveUs) +
      static_cast<int64_t>(computerReceiveUs) - static_cast<int64_t>(bladeSendUs)) / 2;
  portENTER_CRITICAL(&bladeClockMux);
  channel.clockMapping.scale = 1.0;
  channel.clockMapping.offsetMicroseconds = offsetUs;
  channel.clockMapping.maximumErrorMicroseconds = static_cast<uint64_t>(roundTripUs / 2);
  channel.clockMapping.qualified = true;
  portEXIT_CRITICAL(&bladeClockMux);
}

class BladeClientCallbacks final : public BLEClientCallbacks {
 public:
  void onConnect(BLEClient* client) override {
    const int index = bladeChannelIndexForClient(client);
    if (index < 0) return;
    __atomic_store_n(&bladeChannels[index].connected, true, __ATOMIC_RELAXED);
    Serial.printf("[BLADE RELAY] ✅ Blade %c conectado ao Remus Computer.\n",
                  index == 0 ? 'L' : 'R');
  }

  void onDisconnect(BLEClient* client) override {
    const int index = bladeChannelIndexForClient(client);
    if (index < 0) return;
    BladeChannel& channel = bladeChannels[index];
    channel.controlCharacteristic = NULL;
    channel.streamCharacteristic = NULL;
    channel.clockSyncCharacteristic = NULL;
    channel.clockMapping = {};
    __atomic_store_n(&channel.connected, false, __ATOMIC_RELAXED);
    __atomic_store_n(&channel.streamCommanded, false, __ATOMIC_RELAXED);
    if (bladeOrientationMutex && xSemaphoreTake(bladeOrientationMutex, 0) == pdTRUE) {
      dualBladeOrientation.reset(index == 0 ? remus::orientation::BladeSide::Left
                                            : remus::orientation::BladeSide::Right);
      xSemaphoreGive(bladeOrientationMutex);
    }
    Serial.printf("[BLADE RELAY] 📴 Blade %c desconectado; nova busca será tentada.\n",
                  index == 0 ? 'L' : 'R');
  }
};

bool isBladeAdvertisement(BLEAdvertisedDevice& device) {
  if (device.haveManufacturerData()) {
    const std::string manufacturer = device.getManufacturerData();
    if (manufacturer.size() >= 2 &&
        static_cast<uint8_t>(manufacturer[0]) == remus::blade::protocol::kVersion &&
        static_cast<uint8_t>(manufacturer[1]) == remus::blade::protocol::kDeviceFamilyBlade) {
      return true;
    }
  }
  return device.haveName() && device.getName().rfind("REMUS-BLD-", 0) == 0;
}

bool sendBladeClockSyncRequest(size_t channelIndex) {
  if (channelIndex >= BLADE_CHANNEL_COUNT) return false;
  BladeChannel& channel = bladeChannels[channelIndex];
  if (!channel.clockSyncCharacteristic || !channel.client || !channel.client->isConnected()) return false;
  uint8_t request[14]{};
  request[0] = remus::blade::protocol::kVersion;
  request[1] = static_cast<uint8_t>(remus::blade::protocol::MessageType::ClockSyncResponse);
  remus::blade::protocol::writeU32(request + 2, ++channel.clockSyncRequestId);
  remus::blade::protocol::writeU64(
    request + 6, static_cast<uint64_t>(esp_timer_get_time()));
  channel.clockSyncCharacteristic->writeValue(request, sizeof(request), true);
  channel.lastClockSyncMs = millis();
  return true;
}

uint32_t advertisedBladeIdentity(BLEAdvertisedDevice& device) {
  if (device.haveManufacturerData()) {
    const std::string manufacturer = device.getManufacturerData();
    if (manufacturer.size() >= 6) {
      return static_cast<uint8_t>(manufacturer[2]) |
        (static_cast<uint32_t>(static_cast<uint8_t>(manufacturer[3])) << 8) |
        (static_cast<uint32_t>(static_cast<uint8_t>(manufacturer[4])) << 16) |
        (static_cast<uint32_t>(static_cast<uint8_t>(manufacturer[5])) << 24);
    }
    if (manufacturer.size() >= 4) {
      return static_cast<uint8_t>(manufacturer[2]) |
        (static_cast<uint32_t>(static_cast<uint8_t>(manufacturer[3])) << 8);
    }
  }
  BLEAddress address = device.getAddress();
  return remus::blade::protocol::crc32(
    reinterpret_cast<const uint8_t*>(address.getNative()), 6);
}

void captureBladeIdentity(size_t channelIndex, BLEAdvertisedDevice& device) {
  BladeChannel& channel = bladeChannels[channelIndex];
  BLEAddress address = device.getAddress();
  std::memcpy(channel.sourceAddress, address.getNative(), sizeof(channel.sourceAddress));
  channel.sourceIdentityHash = advertisedBladeIdentity(device);
}

bool sendBladeStreamCommand(size_t channelIndex, bool start) {
  if (channelIndex >= BLADE_CHANNEL_COUNT) return false;
  BladeChannel& channel = bladeChannels[channelIndex];
  if (!channel.controlCharacteristic || !channel.client || !channel.client->isConnected()) return false;
  uint8_t request[6]{};
  request[0] = remus::blade::protocol::kVersion;
  request[1] = static_cast<uint8_t>(start
    ? remus::blade::protocol::ControlCommand::StartStream
    : remus::blade::protocol::ControlCommand::StopStream);
  remus::blade::protocol::writeU32(request + 2, ++channel.controlRequestId);
  channel.controlCharacteristic->writeValue(request, sizeof(request), true);
  __atomic_store_n(&channel.streamCommanded, start, __ATOMIC_RELAXED);
  Serial.printf("[BLADE RELAY] %s solicitado para %c (request=%lu).\n",
    start ? "START" : "STOP", channelIndex == 0 ? 'L' : 'R',
    static_cast<unsigned long>(channel.controlRequestId));
  return true;
}

bool connectBlade(size_t channelIndex, BLEAdvertisedDevice& device) {
  if (channelIndex >= BLADE_CHANNEL_COUNT) return false;
  BladeChannel& channel = bladeChannels[channelIndex];
  if (!channel.client) {
    channel.client = BLEDevice::createClient();
    if (!channel.client) return false;
    channel.client->setClientCallbacks(new BladeClientCallbacks());
  }
  if (!channel.client->connect(&device)) return false;
  channel.client->setMTU(185);
  BLERemoteService* service = channel.client->getService(SERVICE_UUID);
  if (!service) {
    channel.client->disconnect();
    return false;
  }
  channel.controlCharacteristic = service->getCharacteristic(BLADE_CONTROL_CHARACTERISTIC_UUID);
  channel.streamCharacteristic = service->getCharacteristic(IMU_STREAM_CHARACTERISTIC_UUID);
  channel.clockSyncCharacteristic = service->getCharacteristic(BLADE_CLOCK_SYNC_CHARACTERISTIC_UUID);
  if (!channel.controlCharacteristic || !channel.streamCharacteristic ||
      !channel.controlCharacteristic->canWrite() || !channel.streamCharacteristic->canNotify()) {
    channel.client->disconnect();
    return false;
  }
  captureBladeIdentity(channelIndex, device);
  channel.streamCharacteristic->registerForNotify(onBladeStreamNotification);
  if (channel.clockSyncCharacteristic && channel.clockSyncCharacteristic->canWrite() &&
      channel.clockSyncCharacteristic->canNotify()) {
    channel.clockSyncCharacteristic->registerForNotify(onBladeClockSyncNotification);
    sendBladeClockSyncRequest(channelIndex);
  }
  const auto side = channelIndex == 0 ? remus::orientation::BladeSide::Left
                                      : remus::orientation::BladeSide::Right;
  if (bladeOrientationMutex &&
      xSemaphoreTake(bladeOrientationMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    dualBladeOrientation.configure(side, channel.sourceIdentityHash, channel.clockMapping);
    xSemaphoreGive(bladeOrientationMutex);
  }
  __atomic_store_n(&channel.connected, true, __ATOMIC_RELAXED);
  return true;
}

int selectBladeChannel(uint32_t identityHash) {
  for (size_t index = 0; index < BLADE_CHANNEL_COUNT; ++index) {
    const auto assignment = bladeAssignment(
      index == 0 ? remus::blade::Slot::Left : remus::blade::Slot::Right);
    if (assignment.configured && assignment.sourceIdentityHash == identityHash) {
      return static_cast<int>(index);
    }
  }
  for (size_t index = 0; index < BLADE_CHANNEL_COUNT; ++index) {
    const auto assignment = bladeAssignment(
      index == 0 ? remus::blade::Slot::Left : remus::blade::Slot::Right);
    BladeChannel& channel = bladeChannels[index];
    if (!assignment.configured && (!channel.client || !channel.client->isConnected()) &&
        channel.sourceIdentityHash == 0) {
      channel.provisionallyDiscovered = true;
      return static_cast<int>(index);
    }
  }
  return -1;
}

void bladeRelayClientTask(void*) {
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(48);
  for (;;) {
    if (__atomic_load_n(&bladeCalibrationRequested, __ATOMIC_RELAXED) &&
        millis() - __atomic_load_n(&bladeCalibrationRequestedAtMs, __ATOMIC_RELAXED) >= 15000) {
      __atomic_store_n(&bladeCalibrationRequested, false, __ATOMIC_RELAXED);
      notifyBladeSlotState("CALIBRATION_TIMEOUT");
    }
    bool allConnected = true;
    for (size_t index = 0; index < BLADE_CHANNEL_COUNT; ++index) {
      BladeChannel& channel = bladeChannels[index];
      if (channel.client && channel.client->isConnected()) {
        const bool shouldStream = isWorkoutActive ||
          __atomic_load_n(&bladeCalibrationRequested, __ATOMIC_RELAXED);
        if (shouldStream != __atomic_load_n(&channel.streamCommanded, __ATOMIC_RELAXED)) {
          sendBladeStreamCommand(index, shouldStream);
        }
        if (millis() - channel.lastClockSyncMs >= 10000) sendBladeClockSyncRequest(index);
      } else {
        allConnected = false;
      }
    }
    if (allConnected) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // Discovery is deliberately low duty-cycle. During a workout it retries,
    // but does not monopolize the radio used by the app-facing live stream.
    BLEScanResults results = scan->start(2, false);
    bool connectedAny = false;
    for (int index = 0; index < results.getCount(); ++index) {
      BLEAdvertisedDevice device = results.getDevice(index);
      if (!isBladeAdvertisement(device)) continue;
      const uint32_t identityHash = advertisedBladeIdentity(device);
      bool alreadyConnected = false;
      for (const BladeChannel& channel : bladeChannels) {
        if (channel.client && channel.client->isConnected() &&
            channel.sourceIdentityHash == identityHash) alreadyConnected = true;
      }
      if (alreadyConnected) continue;
      const int channelIndex = selectBladeChannel(identityHash);
      if (channelIndex < 0) continue;
      scan->stop();
      connectedAny = connectBlade(static_cast<size_t>(channelIndex), device) || connectedAny;
      if (connectedAny) break;  // reconnect sequentially to protect the single radio.
    }
    scan->clearResults();
    if (!bleConnected && pServer) pServer->startAdvertising();
    if (connectedAny && isWorkoutActive) {
      for (size_t index = 0; index < BLADE_CHANNEL_COUNT; ++index) {
        if (bladeChannels[index].client && bladeChannels[index].client->isConnected()) {
          sendBladeStreamCommand(index, true);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(connectedAny ? 200 : (isWorkoutActive ? 10000 : 3000)));
  }
}

void handleGpsAiding(const String& cmd) {
  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);
  int thirdComma = cmd.indexOf(',', secondComma + 1);
  if (firstComma > 0 && secondComma > 0) {
    String latStr = cmd.substring(firstComma + 1, secondComma);
    String lonStr = (thirdComma > 0) ? cmd.substring(secondComma + 1, thirdComma) : cmd.substring(secondComma + 1);
    float lat = latStr.toFloat();
    float lon = lonStr.toFloat();
    if (!usbRecoveryTransferActive) Serial.printf("[GPS AID] Posicao aproximada recebida do iPhone: Lat=%.6f, Lon=%.6f\n", lat, lon);

    if (remus::hardware.hasGps && !isGpsFixValid()) {
      gpsDevice.aidPosition(lat, lon);
    }
  }
}

void persistBladeSlots() {
  const auto left = bladeAssignment(remus::blade::Slot::Left);
  const auto right = bladeAssignment(remus::blade::Slot::Right);
  const uint32_t revision = bladeSlotRevision();
  prefs.begin("remus_slots", false);
  prefs.putUInt("left", left.sourceIdentityHash);
  prefs.putUInt("right", right.sourceIdentityHash);
  prefs.putUInt("revision", revision);
  prefs.end();
}

void loadBladeSlots() {
  prefs.begin("remus_slots", true);
  const uint32_t left = prefs.getUInt("left", 0);
  const uint32_t right = prefs.getUInt("right", 0);
  const uint32_t revision = prefs.getUInt("revision", 0);
  prefs.end();
  portENTER_CRITICAL(&bladeSlotMux);
  bladeSlotRegistry.restore(left, right, revision);
  portEXIT_CRITICAL(&bladeSlotMux);
  Serial.printf("[BLADE SLOT] Restaurado rev=%lu left=%08lX right=%08lX.\n",
                static_cast<unsigned long>(revision), static_cast<unsigned long>(left),
                static_cast<unsigned long>(right));
}

void notifyBladeSlotState(const char* status) {
  if (!bleConnected || !pCharacteristic) return;
  const auto left = bladeAssignment(remus::blade::Slot::Left);
  const auto right = bladeAssignment(remus::blade::Slot::Right);
  char response[96];
  snprintf(response, sizeof(response), "BLADE_SLOTS,%s,%lu,%08lX,%08lX",
           status, static_cast<unsigned long>(bladeSlotRevision()),
           static_cast<unsigned long>(left.sourceIdentityHash),
           static_cast<unsigned long>(right.sourceIdentityHash));
  pCharacteristic->setValue(response);
  pCharacteristic->notify();
}

void notifyBladeRoster(bool force) {
  if (!bleConnected || !pCharacteristic) return;
  const uint32_t leftHash = bladeChannels[0].sourceIdentityHash;
  const uint32_t rightHash = bladeChannels[1].sourceIdentityHash;
  const auto leftAssignment = bladeAssignment(remus::blade::Slot::Left);
  const auto rightAssignment = bladeAssignment(remus::blade::Slot::Right);
  const uint32_t revision = bladeSlotRevision();
  const bool leftConnected = __atomic_load_n(&bladeChannels[0].connected, __ATOMIC_RELAXED);
  const bool rightConnected = __atomic_load_n(&bladeChannels[1].connected, __ATOMIC_RELAXED);
  const uint32_t signature = leftHash ^ (rightHash * 16777619UL) ^
    (leftConnected ? 0x40000000UL : 0) ^ (rightConnected ? 0x80000000UL : 0) ^
    revision;
  static uint32_t lastSignature = 0;
  if (!force && !bladeRosterDirty && signature == lastSignature) return;
  lastSignature = signature;
  bladeRosterDirty = false;
  char response[112];
  snprintf(response, sizeof(response),
    "BLADE_ROSTER,%lu,%08lX:%d:%c,%08lX:%d:%c",
    static_cast<unsigned long>(revision),
    static_cast<unsigned long>(leftHash), leftConnected ? 1 : 0,
    leftAssignment.configured ? 'L' : 'U',
    static_cast<unsigned long>(rightHash), rightConnected ? 1 : 0,
    rightAssignment.configured ? 'R' : 'U');
  pCharacteristic->setValue(response);
  pCharacteristic->notify();
}

bool configureBladeSlot(remus::blade::Slot slot, uint32_t identityHash) {
  portENTER_CRITICAL(&bladeSlotMux);
  const bool assigned = bladeSlotRegistry.assign(slot, identityHash);
  portEXIT_CRITICAL(&bladeSlotMux);
  if (!assigned) return false;
  persistBladeSlots();
  for (size_t index = 0; index < BLADE_CHANNEL_COUNT; ++index) {
    const auto assignment = bladeAssignment(
      index == 0 ? remus::blade::Slot::Left : remus::blade::Slot::Right);
    BladeChannel& channel = bladeChannels[index];
    if (channel.client && channel.client->isConnected() &&
        channel.sourceIdentityHash != assignment.sourceIdentityHash) {
      channel.client->disconnect();
    }
    if (!channel.client || !channel.client->isConnected()) {
      channel.sourceIdentityHash = 0;
      channel.provisionallyDiscovered = false;
    }
  }
  return true;
}

void handleBladeSlotCommand(const String& command) {
  if (command.equalsIgnoreCase("BLADE_SLOTS?")) {
    notifyBladeSlotState("STATE");
    return;
  }
  const int firstComma = command.indexOf(',');
  const int secondComma = command.indexOf(',', firstComma + 1);
  if (firstComma < 0 || secondComma < 0) {
    notifyBladeSlotState("INVALID");
    return;
  }
  String sideText = command.substring(firstComma + 1, secondComma);
  String identityText = command.substring(secondComma + 1);
  sideText.trim();
  identityText.trim();
  const bool left = sideText.equalsIgnoreCase("L") || sideText.equalsIgnoreCase("LEFT");
  const bool right = sideText.equalsIgnoreCase("R") || sideText.equalsIgnoreCase("RIGHT");
  char* end = nullptr;
  const uint32_t identityHash = static_cast<uint32_t>(strtoul(identityText.c_str(), &end, 16));
  if ((!left && !right) || identityHash == 0 || !end || *end != '\0') {
    notifyBladeSlotState("INVALID");
    return;
  }
  const bool accepted = configureBladeSlot(
    left ? remus::blade::Slot::Left : remus::blade::Slot::Right, identityHash);
  notifyBladeSlotState(accepted ? "ACCEPTED" : "CONFLICT");
}

class RemusCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String rxValue = pChar->getValue().c_str();
    rxValue.trim();
    if (rxValue.length() > 0) {
      if (!usbRecoveryTransferActive) Serial.printf("[BLE RX] 📥 Comando recebido: %s\n", rxValue.c_str());
      if (rxValue.startsWith("AID,")) {
        handleGpsAiding(rxValue);
      } else if (rxValue.startsWith("BLADE_SLOT,") ||
                 rxValue.equalsIgnoreCase("BLADE_SLOTS?")) {
        handleBladeSlotCommand(rxValue);
      } else if (rxValue.equalsIgnoreCase("BLADE_CALIBRATE")) {
        __atomic_store_n(&bladeCalibrationRequestedAtMs, millis(), __ATOMIC_RELAXED);
        __atomic_store_n(&bladeCalibrationRequested, true, __ATOMIC_RELAXED);
        notifyBladeSlotState("CALIBRATION_PENDING");
      } else if (rxValue.equalsIgnoreCase("START") || rxValue.startsWith("START")) {
        queueControlCommand(ControlCommandType::Start);
      } else if (rxValue.equalsIgnoreCase("STOP") || rxValue.startsWith("STOP")) {
        queueControlCommand(ControlCommandType::Stop);
      } else if (rxValue.equalsIgnoreCase("GET")) {
        queueControlCommand(ControlCommandType::Get);
      } else if (rxValue.startsWith("GET,") || rxValue.startsWith("GET ")) {
        String reqPath = rxValue.substring(4);
        reqPath.trim();
        queueControlCommand(ControlCommandType::Get, reqPath);
      }
    }
  }

  void onStatus(BLECharacteristic* pChar, BLECharacteristicCallbacks::Status status, uint32_t code) override {
    // Durante download não confiamos no simples retorno de notify() (void).
    // O stack informa aqui se conseguiu ou não enfileirar a notification.
    if (transferNotifyState != TransferNotifyState::Pending) return;

    transferNotifyStatusCode = code;
    if (status == BLECharacteristicCallbacks::Status::SUCCESS_NOTIFY) {
      transferNotifyState = TransferNotifyState::Success;
    } else if (status == BLECharacteristicCallbacks::Status::ERROR_GATT ||
               status == BLECharacteristicCallbacks::Status::ERROR_NO_CLIENT ||
               status == BLECharacteristicCallbacks::Status::ERROR_NOTIFY_DISABLED) {
      transferNotifyState = TransferNotifyState::Failed;
    }
  }
};

void setupBLE() {
  uint64_t chipid = ESP.getEfuseMac();
  snprintf(remusDeviceName, sizeof(remusDeviceName), "REMUS-%s-%04X", remus::hardware.code, (uint16_t)(chipid & 0xFFFF));

  Serial.printf("[BLE] Inicializando BLE como '%s'...\n", remusDeviceName);
  BLEDevice::init(remusDeviceName);
  BLEDevice::setMTU(512);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new RemusBLEServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_WRITE_NR |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new RemusCharacteristicCallbacks());

  pImuStreamCharacteristic = pService->createCharacteristic(
                      IMU_STREAM_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pImuStreamCharacteristic->addDescriptor(new BLE2902());

  pBladeRelayCharacteristic = pService->createCharacteristic(
                      BLADE_RELAY_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pBladeRelayCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.printf("[BLE] ✅ Anúncio BLE ativo. Pronto para parear com o app (%s).\n", remusDeviceName);
}

void printHardwareProfile() {
  Serial.println("\n=========================================================");
  Serial.printf(" REMUS HARDWARE PROFILE: %s (%s)\n", remus::hardware.displayName, remus::hardware.code);
  Serial.println("=========================================================");
  Serial.printf(" IMU: %s | SDA GPIO %d | SCL GPIO %d | %u Hz\n",
                imuDevice.name(), remus::hardware.imuPins.sda,
                remus::hardware.imuPins.scl, remus::hardware.imuRateHz);
  if (remus::hardware.hasGps) {
    Serial.printf(" GPS: %s | RX GPIO %d | TX GPIO %d\n",
                  gpsDevice.name(), remus::hardware.gpsPins.rx, remus::hardware.gpsPins.tx);
  } else Serial.println(" GPS: DISABLED");
  if (remus::hardware.hasStorage) {
    Serial.printf(" SD : %s | MISO %d | MOSI %d | SCK %d | CS %d\n",
                  storageDevice.name(), remus::hardware.sdPins.miso,
                  remus::hardware.sdPins.mosi, remus::hardware.sdPins.sck,
                  remus::hardware.sdPins.cs);
  } else Serial.println(" SD : DISABLED");
  if (remus::hardware.hasDisplay) {
    Serial.printf(" TFT: %s | SCK %d | MOSI %d | DC %d | RST %d | CS %d | software SPI\n",
                  displayDevice.name(), remus::hardware.displayPins.sck,
                  remus::hardware.displayPins.mosi, remus::hardware.displayPins.dc,
                  remus::hardware.displayPins.rst, remus::hardware.displayPins.cs);
  } else Serial.println(" TFT: DISABLED");
  Serial.printf(" BLE: %s | USB recovery: %s\n",
                remus::hardware.hasBle ? "ENABLED" : "DISABLED",
                remus::hardware.hasUsbRecovery ? "ENABLED" : "DISABLED");
  Serial.println("=========================================================\n");
}

void setupIMU() {
  isConfiguringMpu = true;
  imuOk = imuDevice.begin();
  isConfiguringMpu = false;
}

void setupGPS() {
  if (remus::hardware.hasGps) gpsDevice.begin();
}

void setupSD() {
  if (!remus::hardware.hasStorage) { sdOk = false; return; }
  sdOk = storageDevice.begin();
}

void setupDisplay() {
  if (!remus::hardware.hasDisplay) { displayOk = false; return; }
  displayOk = displayDevice.begin();
}

uint16_t histogramMedian(const uint32_t* histogram, uint16_t maxValue,
                         uint32_t sampleCount) {
  if (sampleCount == 0) return 0;
  const uint32_t target = (sampleCount + 1) / 2;
  uint32_t cumulative = 0;
  for (uint16_t value = 0; value <= maxValue; ++value) {
    cumulative += histogram[value];
    if (cumulative >= target) return value;
  }
  return 0;
}

void resetDisplayStatistics() {
  memset(displaySpmHistogram, 0, sizeof(displaySpmHistogram));
  memset(displayPaceHistogram, 0, sizeof(displayPaceHistogram));
  displaySpmSamples = 0;
  displayPaceSamples = 0;
  lastDisplayStatisticsSampleMs = 0;
}

void sampleDisplayStatistics(float spm, bool gpsFix, float speedKmph) {
  if (!isWorkoutActive) return;
  const unsigned long now = millis();
  if (lastDisplayStatisticsSampleMs != 0 &&
      now - lastDisplayStatisticsSampleMs < 900) return;
  lastDisplayStatisticsSampleMs = now;

  if (spm > 0.0f) {
    const uint16_t spmX2 = static_cast<uint16_t>(roundf(spm * 2.0f));
    if (spmX2 <= DISPLAY_MAX_SPM_X2) {
      ++displaySpmHistogram[spmX2];
      ++displaySpmSamples;
    }
  }
  if (gpsFix && speedKmph >= 1.0f) {
    const uint16_t pace = static_cast<uint16_t>(roundf(1800.0f / speedKmph));
    if (pace > 0 && pace <= DISPLAY_MAX_PACE_SECONDS) {
      ++displayPaceHistogram[pace];
      ++displayPaceSamples;
    }
  }
}

void updateDisplay(bool force) {
  if (!displayOk) return;

  float telemetrySpm;
  portENTER_CRITICAL(&telemetryMux);
  telemetrySpm = liveSpm;
  portEXIT_CRITICAL(&telemetryMux);

  remus::drivers::DisplayTelemetry telemetry{};
  telemetry.recording = isWorkoutActive;
  telemetry.transferring = isTransferActive;
  telemetry.bleConnected = bleConnected;
  telemetry.imuHealthy = imuOk;
  telemetry.sdHealthy = sdOk;
  telemetry.gpsFix = isGpsFixValid();
  telemetry.gpsAccuracyEstimateAvailable = telemetry.gpsFix &&
    (gpsDevice.enhancedNavigationAvailable() || gpsDevice.hdopValidRecent());
  const float speedKmph = telemetry.gpsFix ? gpsDevice.speedKmph() : 0.0f;
  sampleDisplayStatistics(telemetrySpm, telemetry.gpsFix, speedKmph);
  telemetry.strokeRateSpm = telemetrySpm > 0.0f
    ? roundf(telemetrySpm * 2.0f) / 2.0f : 0.0f;
  telemetry.medianStrokeRateSpm = displaySpmSamples > 0
    ? histogramMedian(displaySpmHistogram, DISPLAY_MAX_SPM_X2, displaySpmSamples) / 2.0f
    : 0.0f;
  telemetry.paceSecondsPer500m = speedKmph >= 1.0f
    ? static_cast<uint16_t>(roundf(1800.0f / speedKmph)) : 0;
  telemetry.medianPaceSecondsPer500m = histogramMedian(
    displayPaceHistogram, DISPLAY_MAX_PACE_SECONDS, displayPaceSamples);
  telemetry.gpsAccuracyEstimateMeters = !telemetry.gpsAccuracyEstimateAvailable ? 0.0f
    : gpsDevice.enhancedNavigationAvailable()
      ? gpsDevice.horizontalAccuracyMm() / 1000.0f
      : gpsDevice.hdop() * 2.5f;
  telemetry.satellitesInUse = telemetry.gpsFix ? gpsDevice.satellitesInUse() : 0;
  telemetry.satellitesInView = static_cast<uint8_t>(
    constrain(gpsDevice.satellitesInView(), 0, 255));
  telemetry.recordsWritten = readCounter(&recordsPersisted);
  remus::orientation::DualBladeSnapshot bladeSnapshot{};
  if (bladeOrientationMutex && xSemaphoreTake(bladeOrientationMutex, 0) == pdTRUE) {
    bladeSnapshot = dualBladeOrientation.snapshot();
    xSemaphoreGive(bladeOrientationMutex);
  }
  telemetry.bladeAlignmentAvailable = bladeSnapshot.alignment.available;
  telemetry.relativeEquipmentAlignmentDegrees =
    bladeSnapshot.alignment.relativeEquipmentAlignmentDegrees;
  telemetry.orientationQuality = static_cast<uint8_t>(
    bladeSnapshot.alignment.orientationQuality);
  displayDevice.render(telemetry, force);
}

void startWorkoutRecording() {
  if (!sdOk) {
    Serial.println("[SD] 🔄 MicroSD offline. Tentando reinicializar antes de gravar...");
    setupSD();
  }
  if (!sdOk) {
    Serial.println("[SD] ⚠️ Não é possível iniciar gravação: MicroSD offline.");
    return;
  }
  if (isWorkoutActive) {
    Serial.println("[SD] ⚠️ Gravação do workout já está ativa.");
    return;
  }

  if (!s_recordingRingBuf || !recordingStateMutex) {
    Serial.println("[BUF] ❌ Infraestrutura de gravação indisponível.");
    return;
  }

  // START/STOP usam este mutex apenas para proteger o estado da gravação e
  // impedir que a fila seja drenada no meio de um enqueue. O estimador de SPM
  // roda em outra task e nunca executa enquanto este mutex está retido.
  xSemaphoreTake(recordingStateMutex, portMAX_DELAY);
  isWorkoutActive = false;

  // Descarta dados remanescentes no RingBuffer antes de iniciar nova sessão
  if (s_recordingRingBuf) {
    size_t rxSize = 0;
    while (true) {
      uint8_t* pData = (uint8_t*)xRingbufferReceiveUpTo(s_recordingRingBuf, &rxSize, 0, 4096);
      if (pData != NULL) {
        vRingbufferReturnItem(s_recordingRingBuf, pData);
      } else {
        break;
      }
    }
  }

  // Gera nome único para o novo arquivo (ex: /remus_sensor_1A2B3C4D.bin) sem apagar dados anteriores
  closeBladeRelayFile();
  currentSessionId = esp_random();
  currentSessionStartedAtMs = millis();
  snprintf(currentSessionFileName, sizeof(currentSessionFileName), "/remus_sensor_%08X.bin", currentSessionId);
  for (size_t index = 0; index < BLADE_CHANNEL_COUNT; ++index) {
    snprintf(currentBladeRelayFileNames[index], sizeof(currentBladeRelayFileNames[index]),
      "/remus_blade_%08X_%c_%08lX.rbr", currentSessionId,
      index == 0 ? 'L' : 'R',
      static_cast<unsigned long>(bladeChannels[index].sourceIdentityHash));
  }

  logFile = SD.open(currentSessionFileName, FILE_WRITE);
  if (!logFile) {
    Serial.printf("[SD] ❌ Erro ao criar %s para gravação!\n", currentSessionFileName);
    isWorkoutActive = false;
    xSemaphoreGive(recordingStateMutex);
    return;
  }

  // RBP2 keeps the RBP1 IMU/SPM layout and adds coherent receiver-native GNSS.
  RemusFileHeader header = remus::session::makeHeaderV2(
    currentSessionStartedAtMs, currentSessionId, 200);
  header.padding[0] = static_cast<uint8_t>(remus::hardware.id);
  header.padding[1] = static_cast<uint8_t>(remus::hardware.imuModel);
  header.padding[2] = static_cast<uint8_t>(remus::hardware.gpsModel);
  header.padding[3] = remus::esp32::hw::capabilityByte(remus::hardware);

  __atomic_store_n(&recordsQueued, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&recordsPersisted, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&bytesPersisted, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&storageWriteFailureCount, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&recordingStorageFault, false, __ATOMIC_RELAXED);
  __atomic_store_n(&pcLiveSampleSequence, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&pcLiveBatchSequence, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&pcLiveQueueDropCount, 0UL, __ATOMIC_RELAXED);
  for (BladeChannel& channel : bladeChannels) {
    __atomic_store_n(&channel.notificationsReceived, 0UL, __ATOMIC_RELAXED);
    __atomic_store_n(&channel.packetsPersisted, 0UL, __ATOMIC_RELAXED);
    __atomic_store_n(&channel.bytesPersisted, 0UL, __ATOMIC_RELAXED);
    __atomic_store_n(&channel.queueDropCount, 0UL, __ATOMIC_RELAXED);
    __atomic_store_n(&channel.writeFailureCount, 0UL, __ATOMIC_RELAXED);
    __atomic_store_n(&channel.liveDropCount, 0UL, __ATOMIC_RELAXED);
    __atomic_store_n(&channel.storageFault, false, __ATOMIC_RELAXED);
    __atomic_store_n(&channel.liveSequence, 0U, __ATOMIC_RELAXED);
  }
  persistedRecordBytesRemaining = 0;
  if (pcLiveImuQueue) xQueueReset(pcLiveImuQueue);
  if (bladeRelayQueue) xQueueReset(bladeRelayQueue);

  const size_t headerBytes = logFile.write((const uint8_t*)&header, sizeof(header));
  if (headerBytes != sizeof(header)) {
    incrementCounter(&storageWriteFailureCount);
    __atomic_store_n(&recordingStorageFault, true, __ATOMIC_RELAXED);
    logFile.close();
    sdOk = false;
    Serial.printf("[SD] ❌ Cabeçalho incompleto: %u/%u bytes. Somente o stream ao app continuará.\n",
      static_cast<unsigned int>(headerBytes), static_cast<unsigned int>(sizeof(header)));
  } else {
    logFile.flush();
  }

  __atomic_store_n(&imuGapCount, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&maxImuGapMs, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&imuReadFailureCount, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&ringBufferOverflowCount, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&liveSpmQueueDropCount, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&gpsEpochsReceived, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&gpsValidEpochs, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&gpsInvalidEpochs, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&gpsItowGapCount, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&gpsMaxItowGapMs, 0UL, __ATOMIC_RELAXED);
  previousGpsItowMs = UINT32_MAX;

  // Nova geração de preview. Primeiro desabilita/invalida qualquer trabalho
  // antigo; a task de SPM fará reset ao receber a primeira amostra nova.
  __atomic_store_n(&liveSpmPreviewEnabled, false, __ATOMIC_RELAXED);
  __atomic_add_fetch(&liveSpmGeneration, 1U, __ATOMIC_RELAXED);
  if (liveSpmSampleQueue) xQueueReset(liveSpmSampleQueue);

  portENTER_CRITICAL(&telemetryMux);
  liveSpm = 0.0f;
  lastImuLoop = 0;
  portEXIT_CRITICAL(&telemetryMux);
  resetDisplayStatistics();
  lastFlush = millis();
  isWorkoutActive = true;
  __atomic_store_n(&liveSpmPreviewEnabled, true, __ATOMIC_RELAXED);
  xSemaphoreGive(recordingStateMutex);
  updateDisplay();
  if (__atomic_load_n(&recordingStorageFault, __ATOMIC_RELAXED)) {
    Serial.println("[BLE] 🔴 Workout iniciado em modo degradado: IMU bruto somente no app.");
  } else {
    Serial.printf("[SD] 🔴 Gravação do Workout INICIADA em binário: %s\n", currentSessionFileName);
  }
}

void stopWorkoutRecording() {
  // Live SPM nunca deve sobreviver ao fim da sessão. Invalidamos a geração
  // antes de qualquer retorno para que um cálculo antigo não continue usando CPU
  // nem publique resultado durante/apos a transferencia do arquivo.
  __atomic_store_n(&liveSpmPreviewEnabled, false, __ATOMIC_RELAXED);
  __atomic_add_fetch(&liveSpmGeneration, 1U, __ATOMIC_RELAXED);
  if (liveSpmSampleQueue) xQueueReset(liveSpmSampleQueue);

  portENTER_CRITICAL(&telemetryMux);
  liveSpm = 0.0f;
  portEXIT_CRITICAL(&telemetryMux);

  if (!isWorkoutActive) {
    Serial.println("[SD] ⚠️ Nenhuma gravação de workout ativa para parar.");
    return;
  }

  // Espera qualquer amostra em voo terminar e impede novos enqueues. Todos os
  // comandos de arquivo chegam aqui pela loop(), portanto logFile tem um único
  // proprietário durante escrita, flush e fechamento.
  xSemaphoreTake(recordingStateMutex, portMAX_DELAY);
  isWorkoutActive = false;
  xSemaphoreGive(recordingStateMutex);

  if (logFile) {
    // Drena todos os registros remanescentes do RingBuffer para o cartão
    if (s_recordingRingBuf) {
      size_t rxSize = 0;
      while (true) {
        uint8_t* pData = (uint8_t*)xRingbufferReceiveUpTo(s_recordingRingBuf, &rxSize, 0, 4096);
        if (pData != NULL && rxSize > 0) {
          const bool persisted = writeRecordingBytes(pData, rxSize);
          vRingbufferReturnItem(s_recordingRingBuf, pData);
          if (!persisted) {
            abortRecordingAfterStorageFailure(rxSize);
            break;
          }
        } else {
          break;
        }
      }
    }
    logFile.flush();
    logFile.close();
  }
  closeBladeRelayFile();

  // Persist only the latest GPS recovery snapshot after the time-sensitive
  // recording path has stopped. Repeated NVS commits during capture caused
  // long IMU gaps and invalidated the 15-second cadence windows.
  persistLatestGpsFix();
  updateDisplay();
  const bool complete = !__atomic_load_n(&recordingStorageFault, __ATOMIC_RELAXED) &&
    persistedRecordBytesRemaining == 0;
  Serial.printf("[SD] %s Arquivo '%s' preservado. Fila=%lu, persistidos=%lu, bytes=%lu, overflows=%lu, falhas=%lu\n",
    complete ? "⏹️ Gravação FINALIZADA!" : "⚠️ Gravação INTERROMPIDA!",
    currentSessionFileName, readCounter(&recordsQueued), readCounter(&recordsPersisted),
    readCounter(&bytesPersisted), readCounter(&ringBufferOverflowCount),
    readCounter(&storageWriteFailureCount));
}

String findLatestSessionFileOnSd() {
  if (!sdOk) return "";
  File root = SD.open("/");
  if (!root || !root.isDirectory()) return "";

  String latestName = "";
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      String name = file.name();
      if (!name.startsWith("/")) {
        name = "/" + name;
      }
      if (name.endsWith(".bin") && file.size() > 0) {
        latestName = name;
      }
    }
    file = root.openNextFile();
  }
  root.close();
  return latestName;
}

// Envia uma notification do protocolo de arquivo e só considera o pacote
// aceito quando o próprio stack GATT reporta SUCCESS_NOTIFY. Isso não transforma
// notification em ACK de aplicação, mas impede o firmware de avançar o offset
// quando esp_ble_gatts_send_indicate() rejeita o pacote por congestionamento.
bool sendTransferNotification(const uint8_t* data, size_t len, uint8_t maxAttempts = 20) {
  if (!pCharacteristic || !bleConnected) return false;

  for (uint8_t attempt = 0; attempt < maxAttempts; ++attempt) {
    transferNotifyStatusCode = 0;
    transferNotifyState = TransferNotifyState::Pending;

    // Arduino-ESP32 2.0.0 exposes setValue(uint8_t*, size_t), not const uint8_t*.
    // BLECharacteristic copies the bytes into its internal BLEValue, so this cast
    // does not allow the library to mutate our source buffer.
    pCharacteristic->setValue(const_cast<uint8_t*>(data), len);
    pCharacteristic->notify();

    const TransferNotifyState state = transferNotifyState;
    transferNotifyState = TransferNotifyState::Idle;

    // Arduino-ESP32 chama onStatus() de forma síncrona para notification.
    // Se uma versão antiga do core não o fizer, Pending é tratado como sucesso
    // para manter compatibilidade em vez de travar toda a transferência.
    if (state == TransferNotifyState::Success || state == TransferNotifyState::Pending) {
      return true;
    }

    incrementCounter(&transferNotifyErrors);
    if (attempt + 1 < maxAttempts) {
      incrementCounter(&transferNotifyRetries);
      // Backoff curto: libera a fila HCI/GATT antes de repetir EXATAMENTE o
      // mesmo offset. O app escreve por offset, portanto duplicatas são seguras.
      delay(2 + (attempt < 8 ? attempt : 8));
    }
  }

  return false;
}

bool sendTransferText(const char* text, uint8_t maxAttempts = 20) {
  return sendTransferNotification(reinterpret_cast<const uint8_t*>(text), strlen(text), maxAttempts);
}

void configureTransferMtu() {
  transferPeerMtu = 23;
  if (pServer && bleConnected) {
    const uint16_t connId = pServer->getConnId();
    const uint16_t peerMtu = pServer->getPeerMTU(connId);
    if (peerMtu >= 23 && peerMtu <= 512) transferPeerMtu = peerMtu;
  }

  // ATT notification comporta MTU-3 bytes no total. Nosso protocolo usa 7 B
  // de cabeçalho, então payload máximo = MTU - 10. Limite local: MTU 512.
  transferMaxPayload = transferPeerMtu > 10 ? (size_t)transferPeerMtu - 10U : 13U;
  if (transferMaxPayload < 13) transferMaxPayload = 13;
  if (transferMaxPayload > 502) transferMaxPayload = 502;
}

void startFileTransfer(const String& targetFile) {
  if (!bleConnected) return;

  if (isWorkoutActive || logFile) {
    stopWorkoutRecording();
  } else {
    // GET pode chegar depois de um STOP separado. Garanta que nenhum backlog
    // de preview continue competindo com a transferência BLE.
    __atomic_store_n(&liveSpmPreviewEnabled, false, __ATOMIC_RELAXED);
    __atomic_add_fetch(&liveSpmGeneration, 1U, __ATOMIC_RELAXED);
    if (liveSpmSampleQueue) xQueueReset(liveSpmSampleQueue);
  }

  String resolvedPath = targetFile;
  resolvedPath.trim();
  if (resolvedPath.length() > 0 && !resolvedPath.startsWith("/")) {
    resolvedPath = "/" + resolvedPath;
  }

  if (resolvedPath.length() == 0 || !SD.exists(resolvedPath)) {
    if (strlen(currentSessionFileName) > 0 && SD.exists(currentSessionFileName)) {
      resolvedPath = String(currentSessionFileName);
    } else {
      resolvedPath = findLatestSessionFileOnSd();
    }
  }

  if (!sdOk || resolvedPath.length() == 0 || !SD.exists(resolvedPath)) {
    Serial.printf("[TRANSFER] ❌ Arquivo de sessão não encontrado no MicroSD (buscou: '%s', atual: '%s').\n",
      resolvedPath.c_str(), currentSessionFileName);
    sendTransferText("FILE_ERR:NOT_FOUND", 8);
    return;
  }

  transferFile = SD.open(resolvedPath, FILE_READ);
  if (!transferFile) {
    Serial.printf("[TRANSFER] ❌ Erro ao abrir '%s' para leitura.\n", resolvedPath.c_str());
    sendTransferText("FILE_ERR:OPEN_FAILED", 8);
    return;
  }

  transferTotalBytes = transferFile.size();
  transferOffset = 0;
  transferCrcState = 0xFFFFFFFFUL;
  transferPacketCount = 0;
  transferLastLoggedPercent = -10;
  __atomic_store_n(&transferNotifyErrors, 0UL, __ATOMIC_RELAXED);
  __atomic_store_n(&transferNotifyRetries, 0UL, __ATOMIC_RELAXED);
  configureTransferMtu();
  isTransferActive = true;
  lastTransferChunk = millis();
  snprintf(currentTransferFileName, sizeof(currentTransferFileName), "%s", resolvedPath.c_str());
  updateDisplay();

  Serial.printf("[TRANSFER] 🚀 '%s': %u bytes, MTU=%u, payload=%u B, records=%lu\n",
    resolvedPath.c_str(), transferTotalBytes, transferPeerMtu,
    (unsigned int)transferMaxPayload, readCounter(&recordsPersisted));

  char startBuf[112];
  snprintf(startBuf, sizeof(startBuf), "FILE_START:%s:%u:%lu",
    resolvedPath.c_str(), transferTotalBytes, readCounter(&recordsPersisted));

  if (!sendTransferText(startBuf, 20)) {
    Serial.printf("[TRANSFER] ❌ FILE_START não pôde ser enfileirado no BLE (code=%lu).\n",
      (unsigned long)transferNotifyStatusCode);
    transferFile.close();
    isTransferActive = false;
    updateDisplay();
    return;
  }

  // Dá uma pequena vantagem ao cliente para alocar Buffer(totalBytes) antes
  // do primeiro chunk binário, sem consumir uma fração relevante dos 30 s.
  delay(3);
}

void processFileTransfer() {
  if (!isTransferActive || !transferFile || !bleConnected) return;

  // Buffer máximo para MTU local 512: ATT=509 B; header=7 B; payload=502 B.
  static uint8_t chunkBuf[7 + 502];
  chunkBuf[0] = 0x20;

  if (transferOffset >= transferTotalBytes) {
    transferFile.close();

    // FILE_END também passa pela mesma validação de enqueue. O app só resolve
    // o download depois desta mensagem, então não encerramos silenciosamente.
    char endBuf[112];
    const uint32_t completedCrc = ~transferCrcState;
    snprintf(endBuf, sizeof(endBuf), "FILE_END:%s:%u:%08lX", currentTransferFileName,
      transferOffset, (unsigned long)completedCrc);
    const bool endOk = sendTransferText(endBuf, 30);

    Serial.printf("[TRANSFER] %s Total=%u B, packets=%lu, notifyErrors=%lu, retries=%lu\n",
      endOk ? "✅ Download concluído." : "❌ Dados enviados, mas FILE_END falhou.",
      transferOffset, transferPacketCount,
      readCounter(&transferNotifyErrors), readCounter(&transferNotifyRetries));

    isTransferActive = false;
    updateDisplay();
    return;
  }

  const uint32_t currentOffset = transferOffset;
  const size_t remaining = (size_t)(transferTotalBytes - currentOffset);
  const size_t bytesToRead = min(transferMaxPayload, remaining);

  // Mantém a posição do arquivo explicitamente vinculada ao offset de protocolo.
  // Se um notify falhar, o offset NÃO muda e na próxima chamada o mesmo trecho
  // será lido e reenviado.
  if ((uint32_t)transferFile.position() != currentOffset) {
    if (!transferFile.seek(currentOffset)) {
      Serial.printf("[TRANSFER] ❌ SEEK_FAILED em offset %u.\n", currentOffset);
      sendTransferText("FILE_ERR:SEEK_FAILED", 8);
      transferFile.close();
      isTransferActive = false;
      updateDisplay();
      return;
    }
  }

  const int bytesRead = transferFile.read(chunkBuf + 7, bytesToRead);
  if (bytesRead <= 0) {
    Serial.printf("[TRANSFER] ❌ READ_FAILED em offset %u/%u.\n", currentOffset, transferTotalBytes);
    sendTransferText("FILE_ERR:READ_FAILED", 8);
    transferFile.close();
    isTransferActive = false;
    updateDisplay();
    return;
  }

  memcpy(chunkBuf + 1, &currentOffset, sizeof(currentOffset));
  const uint16_t chunkLen = (uint16_t)bytesRead;
  memcpy(chunkBuf + 5, &chunkLen, sizeof(chunkLen));

  if (!sendTransferNotification(chunkBuf, 7U + (size_t)bytesRead, 20)) {
    // Fundamental: não avançar offset e voltar o ponteiro do SD.
    transferFile.seek(currentOffset);
    // Não abortamos imediatamente: o próximo loop tenta novamente depois de
    // um backoff maior. Se o telefone desconectar, o callback encerra tudo.
    delay(8);
    return;
  }

  for (int i = 0; i < bytesRead; ++i) {
    transferCrcState ^= chunkBuf[7 + i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      transferCrcState = (transferCrcState >> 1) ^
        (0xEDB88320UL & (0UL - (transferCrcState & 1UL)));
    }
  }

  transferOffset = currentOffset + (uint32_t)bytesRead;
  transferPacketCount++;
  lastTransferChunk = millis();

  // Não inundar a fila GATT. Com MTU ~185, uma pausa a cada 4 pacotes custa
  // ~6 s em 4 MB; com MTU maior custa ainda menos, mantendo o GET dentro da
  // janela atual de 30 s e reduzindo fortemente congestionamento.
  if ((transferPacketCount & 0x03UL) == 0) delay(1);

  // Log leve a cada ~10% para diagnóstico, sem imprimir por pacote.
  const int percent = transferTotalBytes > 0
    ? (int)((uint64_t)transferOffset * 100ULL / transferTotalBytes) : 100;
  if (percent >= transferLastLoggedPercent + 10 || percent == 100) {
    transferLastLoggedPercent = percent;
    Serial.printf("[TRANSFER] %d%% (%u/%u B), packets=%lu, errors=%lu, retries=%lu\n",
      percent, transferOffset, transferTotalBytes, transferPacketCount,
      readCounter(&transferNotifyErrors), readCounter(&transferNotifyRetries));
  }
}


// =========================================================================
// RECUPERAÇÃO DO MICROSD VIA USB SERIAL — protocolo confiável v7
// =========================================================================
// O protocolo de arquivo é propositalmente textual durante a recuperação.
// Isso evita que bytes binários e mensagens CDC sejam confundidos caso haja
// backpressure / short write no USB Serial/JTAG.
//
// Host -> "USB_LIST\n"
// ESP  -> USB_LIST_BEGIN
//         USB_FILE\t<size>\t<path>
//         ...
//         USB_LIST_END
//
// Host -> "USB_GET\t<path>\t<offset>\n"
// ESP  -> USB_DATA\t<totalSize>\t<offset>\t<path>\n
//
// Para cada bloco:
// ESP  -> USB_CHUNK\t<offset>\t<len>\t<crc32hex>\t<HEX_PAYLOAD>\n
// Host -> USB_ACK\t<nextOffset>\n
//      ou USB_NACK\t<offset>\n para retransmitir o mesmo bloco.
//
// Final:
// ESP  -> USB_END\t<totalSize>\n
// Host -> USB_ACK_END\n
//
// O .part no host contém apenas blocos já validados por CRC e confirmados.

uint32_t remusCrc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
  }
  return ~crc;
}

bool usbWriteAll(const uint8_t* data, size_t len) {
  size_t sent = 0;
  unsigned long lastProgress = millis();

  while (sent < len) {
    size_t written = Serial.write(data + sent, len - sent);
    if (written > 0) {
      sent += written;
      lastProgress = millis();
      continue;
    }

    if (millis() - lastProgress > USB_RECOVERY_WRITE_STALL_TIMEOUT_MS) {
      return false;
    }
    delay(1);
  }
  return true;
}

bool usbWriteLine(const String& line) {
  if (line.length() > 0) {
    if (!usbWriteAll((const uint8_t*)line.c_str(), line.length())) return false;
  }
  const uint8_t newline = '\n';
  if (!usbWriteAll(&newline, 1)) return false;
  Serial.flush();
  return true;
}

void usbProtocolError(const char* code, const String& detail = "") {
  String line = "USB_ERR\t";
  line += code;
  if (detail.length() > 0) {
    line += "\t";
    line += detail;
  }
  usbWriteLine(line);
}

bool usbRecoveryCanAccessSd() {
  if (isWorkoutActive || logFile) {
    usbProtocolError("BUSY_RECORDING", "Pare o workout antes de acessar o SD");
    return false;
  }
  if (isTransferActive || transferFile) {
    usbProtocolError("BUSY_BLE_TRANSFER", "Transferencia BLE em andamento");
    return false;
  }
  if (!sdOk) {
    setupSD();
    delay(50);
    if (!sdOk) {
      usbProtocolError("SD_OFFLINE", "MicroSD nao esta montado apos tentativa de remount");
      return false;
    }
  }
  return true;
}

void listSdFilesOverUsb() {
  usbRecoveryTransferActive = true;
  // Durante uma operação interativa USB podemos bloquear por alguns ms sem
  // comprometer a captura autônoma, que já foi impedida por usbRecoveryCanAccessSd.
  Serial.setTxTimeoutMs(1000);

  if (!usbRecoveryCanAccessSd()) {
    Serial.setTxTimeoutMs(0);
    usbRecoveryTransferActive = false;
    return;
  }

  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    usbProtocolError("OPEN_ROOT_FAILED");
    Serial.setTxTimeoutMs(0);
    usbRecoveryTransferActive = false;
    return;
  }

  usbWriteLine("USB_LIST_BEGIN");

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      String name = file.name();
      if (!name.startsWith("/")) name = "/" + name;

      String line = "USB_FILE\t";
      line += String((uint32_t)file.size());
      line += "\t";
      line += name;
      if (!usbWriteLine(line)) {
        file.close();
        root.close();
        Serial.setTxTimeoutMs(0);
        usbRecoveryTransferActive = false;
        return;
      }
    }
    file.close();
    file = root.openNextFile();
  }

  root.close();
  usbWriteLine("USB_LIST_END");
  Serial.setTxTimeoutMs(0);
  usbRecoveryTransferActive = false;
}

enum class UsbChunkAckResult : uint8_t {
  Ack,
  Retry,
  Timeout
};

UsbChunkAckResult waitForUsbChunkAck(uint32_t chunkOffset, uint32_t expectedNextOffset) {
  unsigned long started = millis();
  String line;
  line.reserve(64);

  while (millis() - started < USB_RECOVERY_ACK_TIMEOUT_MS) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') continue;

      if (c == '\n') {
        line.trim();

        if (line.startsWith("USB_ACK\t")) {
          uint32_t ackOffset = (uint32_t)strtoul(line.substring(8).c_str(), nullptr, 10);
          if (ackOffset == expectedNextOffset) return UsbChunkAckResult::Ack;
        } else if (line.startsWith("USB_NACK\t")) {
          uint32_t nackOffset = (uint32_t)strtoul(line.substring(9).c_str(), nullptr, 10);
          if (nackOffset == chunkOffset) return UsbChunkAckResult::Retry;
        }

        line = "";
        continue;
      }

      if (line.length() < 96) {
        line += c;
      } else {
        line = "";
      }
    }
    delay(1);
  }

  return UsbChunkAckResult::Timeout;
}

bool waitForUsbEndAck() {
  unsigned long started = millis();
  String line;
  line.reserve(32);

  while (millis() - started < USB_RECOVERY_ACK_TIMEOUT_MS) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line.trim();
        if (line == "USB_ACK_END") return true;
        line = "";
        continue;
      }
      if (line.length() < 64) line += c;
      else line = "";
    }
    delay(1);
  }
  return false;
}

String bytesToHex(const uint8_t* data, size_t len) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out += HEX_DIGITS[(data[i] >> 4) & 0x0F];
    out += HEX_DIGITS[data[i] & 0x0F];
  }
  return out;
}

void sendSdFileOverUsb(const String& requestedPath, uint32_t requestedOffset) {
  usbRecoveryTransferActive = true;
  Serial.setTxTimeoutMs(1000);

  if (!usbRecoveryCanAccessSd()) {
    Serial.setTxTimeoutMs(0);
    usbRecoveryTransferActive = false;
    return;
  }

  String path = requestedPath;
  path.trim();
  if (!path.startsWith("/")) path = "/" + path;

  if (path.length() <= 1 || !SD.exists(path)) {
    usbProtocolError("NOT_FOUND", path);
    Serial.setTxTimeoutMs(0);
    usbRecoveryTransferActive = false;
    return;
  }

  File file = SD.open(path, FILE_READ);
  if (!file) {
    usbProtocolError("OPEN_FAILED", path);
    Serial.setTxTimeoutMs(0);
    usbRecoveryTransferActive = false;
    return;
  }

  const uint32_t totalSize = (uint32_t)file.size();
  if (requestedOffset > totalSize) {
    file.close();
    usbProtocolError("BAD_OFFSET", String(requestedOffset));
    Serial.setTxTimeoutMs(0);
    usbRecoveryTransferActive = false;
    return;
  }

  if (!file.seek(requestedOffset)) {
    file.close();
    usbProtocolError("SEEK_FAILED", String(requestedOffset));
    Serial.setTxTimeoutMs(0);
    usbRecoveryTransferActive = false;
    return;
  }

  String dataLine = "USB_DATA\t";
  dataLine += String(totalSize);
  dataLine += "\t";
  dataLine += String(requestedOffset);
  dataLine += "\t";
  dataLine += path;

  if (!usbWriteLine(dataLine)) {
    file.close();
    Serial.setTxTimeoutMs(0);
    usbRecoveryTransferActive = false;
    return;
  }

  uint8_t buffer[USB_RECOVERY_CHUNK_SIZE];
  uint32_t offset = requestedOffset;
  bool ok = true;

  while (offset < totalSize) {
    size_t remaining = totalSize - offset;
    size_t toRead = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
    int bytesRead = file.read(buffer, toRead);
    if (bytesRead <= 0) {
      ok = false;
      break;
    }

    uint32_t crc = remusCrc32(buffer, (size_t)bytesRead);
    char crcBuf[9];
    snprintf(crcBuf, sizeof(crcBuf), "%08lX", (unsigned long)crc);

    String payloadHex = bytesToHex(buffer, (size_t)bytesRead);
    String chunkLine;
    chunkLine.reserve(64 + payloadHex.length());
    chunkLine = "USB_CHUNK\t";
    chunkLine += String(offset);
    chunkLine += "\t";
    chunkLine += String(bytesRead);
    chunkLine += "\t";
    chunkLine += crcBuf;
    chunkLine += "\t";
    chunkLine += payloadHex;

    bool chunkAccepted = false;
    for (int attempt = 0; attempt < 5 && !chunkAccepted; ++attempt) {
      if (!usbWriteLine(chunkLine)) {
        ok = false;
        break;
      }

      UsbChunkAckResult ack = waitForUsbChunkAck(offset, offset + (uint32_t)bytesRead);
      if (ack == UsbChunkAckResult::Ack) {
        chunkAccepted = true;
      } else if (ack == UsbChunkAckResult::Timeout) {
        ok = false;
        break;
      }
      // Retry => reenvia exatamente a mesma linha/bloco.
    }

    if (!ok || !chunkAccepted) {
      ok = false;
      break;
    }

    offset += (uint32_t)bytesRead;
    delay(1);
  }

  file.close();

  if (ok && offset == totalSize) {
    String endLine = "USB_END\t";
    endLine += String(totalSize);
    usbWriteLine(endLine);
    waitForUsbEndAck();
  }

  Serial.setTxTimeoutMs(0);
  usbRecoveryTransferActive = false;
}

void handleUsbRecoveryCommand(const String& rawCommand) {
  String command = rawCommand;
  command.trim();

  if (command.equalsIgnoreCase("USB_LIST")) {
    listSdFilesOverUsb();
    return;
  }

  if (command.startsWith("USB_GET\t") || command.startsWith("USB_GET ")) {
    int firstSep = command.indexOf('\t');
    if (firstSep < 0) firstSep = command.indexOf(' ');
    if (firstSep < 0) {
      usbProtocolError("BAD_COMMAND");
      return;
    }

    String args = command.substring(firstSep + 1);
    args.trim();

    String path;
    uint32_t offset = 0;

    int lastTab = args.lastIndexOf('\t');
    if (lastTab > 0) {
      path = args.substring(0, lastTab);
      String offsetText = args.substring(lastTab + 1);
      offsetText.trim();
      offset = (uint32_t)strtoul(offsetText.c_str(), nullptr, 10);
    } else {
      path = args;
    }

    path.trim();
    sendSdFileOverUsb(path, offset);
    return;
  }

  usbProtocolError("UNKNOWN_COMMAND", command);
}

void processUsbRecoveryInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (usbRecoveryCommandBuffer.length() == 0) {
      if (c == 'U' || c == 'u') {
        usbRecoveryCommandBuffer += c;
        continue;
      }

      if (c == 'r' || c == 'R') {
        printSavedReport();
      } else if (c == 'n' || c == 'N') {
        showRawNmea = !showRawNmea;
        Serial.printf("\n[GPS] Modo NMEA bruto %s!\n\n",
                      showRawNmea ? "ATIVADO (mostrando sentencas do GPS)" : "DESATIVADO");
      } else if (c == 's' || c == 'S') {
        if (isWorkoutActive) stopWorkoutRecording();
        else startWorkoutRecording();
      } else if (c == 'g' || c == 'G') {
        startFileTransfer();
      }
      continue;
    }

    if (c == '\r') continue;

    if (c == '\n') {
      String command = usbRecoveryCommandBuffer;
      usbRecoveryCommandBuffer = "";
      handleUsbRecoveryCommand(command);
      continue;
    }

    if (usbRecoveryCommandBuffer.length() < 180) {
      usbRecoveryCommandBuffer += c;
    } else {
      usbRecoveryCommandBuffer = "";
      usbProtocolError("COMMAND_TOO_LONG");
    }
  }
}

void printSavedReport() {
  prefs.begin("remus_gps", true);
  bool hadFix = prefs.getBool("had_fix", false);
  int savedFixCount = prefs.getInt("fix_count", 0);
  double savedLat = prefs.getDouble("lat", 0.0);
  double savedLng = prefs.getDouble("lng", 0.0);
  float savedSpeed = prefs.getFloat("speed", 0.0);
  String savedTime = prefs.getString("time", "--:--:--");
  String savedDate = prefs.getString("date", "--/--/----");
  prefs.end();

  Serial.println("\n=========================================================");
  Serial.println("     📡 RELATÓRIO DO ÚLTIMO PASSEIO / FIX NA MEMÓRIA     ");
  Serial.println("=========================================================");
  Serial.print("Dispositivo: "); Serial.println(remusDeviceName);
  Serial.print("Conseguiu Fix completo? ");
  Serial.println(hadFix ? "✅ SIM! (Coordenadas capturadas com sucesso)" : "⏳ Efemérides e relógio atômico sincronizados!");
  Serial.print("Total de leituras de GPS gravadas no NVS: ");
  Serial.println(savedFixCount);

  if (hadFix) {
    Serial.println("---------------------------------------------------------");
    Serial.print("Data / Hora UTC: "); Serial.print(savedDate); Serial.print(" "); Serial.println(savedTime);
    Serial.print("Latitude:  "); Serial.println(savedLat, 6);
    Serial.print("Longitude: "); Serial.println(savedLng, 6);
    Serial.print("Velocidade registrada: "); Serial.print(savedSpeed); Serial.println(" km/h");
    Serial.println("---------------------------------------------------------");
    Serial.print("📍 Link no Google Maps: https://www.google.com/maps?q=");
    Serial.print(savedLat, 6);
    Serial.print(",");
    Serial.println(savedLng, 6);
  }
  Serial.println("=========================================================");
  Serial.println("👉 Pressione 'r' ou ENTER para reimprimir este relatório.\n");
}

// =========================================================================
// TASK DEDICADA DE AMOSTRAGEM DO IMU (200 Hz = 5 ms) — ALTA PRIORIDADE (5)
// =========================================================================
// Esta task é deliberadamente mínima: lê o MPU, atualiza a telemetria, grava
// o registro IMU e produz um preview barato de 25 Hz para a fila de SPM.
// Nenhuma FFT/autocorrelação/estimação pesada é executada aqui.
void imuSamplingTask(void* pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(5); // 5 ms = 200 Hz

  uint32_t spmAccumulatorGeneration = 0;
  uint8_t spmAccumulatorCount = 0;
  float spmAxSum = 0.0f, spmAySum = 0.0f, spmAzSum = 0.0f;

  while (true) {
    vTaskDelayUntil(&lastWakeTime, frequency);

    // Se uma operação excepcional atrasar mais de um período, retoma a
    // cadência a partir de agora. Rodar iterações atrasadas em rajada repetiria
    // o mesmo registrador do MPU e criaria amostras falsas.
    const TickType_t currentTick = xTaskGetTickCount();
    if ((currentTick - lastWakeTime) >= frequency) {
      lastWakeTime = currentTick;
    }

    if (!recordingStateMutex) continue;
    xSemaphoreTake(recordingStateMutex, portMAX_DELAY);
    if (!imuOk || isConfiguringMpu) {
      xSemaphoreGive(recordingStateMutex);
      continue;
    }

    const unsigned long now = millis();
    const bool recording = isWorkoutActive;
    if (recording) {
      const unsigned long previous = lastImuLoop;
      const unsigned long intervalMs = previous == 0 ? 0 : now - previous;
      if (intervalMs > 0) updateMaximum(&maxImuGapMs, intervalMs);
      if (intervalMs > IMU_GAP_THRESHOLD_MS) incrementCounter(&imuGapCount);
      lastImuLoop = now;
    }

    remus::hal::ImuSample sample{};
    if (!imuDevice.read(sample)) {
      imuOk = imuDevice.healthy();
      if (recording) incrementCounter(&imuReadFailureCount);
      xSemaphoreGive(recordingStateMutex);
      continue;
    }

    portENTER_CRITICAL(&telemetryMux);
    rawAcX = sample.rawAx; rawAcY = sample.rawAy; rawAcZ = sample.rawAz;
    rawGyX = sample.rawGx; rawGyY = sample.rawGy; rawGyZ = sample.rawGz;
    ax = sample.accelX; ay = sample.accelY; az = sample.accelZ;
    gx = sample.gyroX; gy = sample.gyroY; gz = sample.gyroZ;
    portEXIT_CRITICAL(&telemetryMux);

    if (!recording) {
      spmAccumulatorCount = 0;
      spmAxSum = spmAySum = spmAzSum = 0.0f;
      xSemaphoreGive(recordingStateMutex);
      continue;
    }

    // 1) A fonte de verdade continua sendo o IMU bruto a 200 Hz no RBP2.
    RemusImuRecord imuRec;
    imuRec.type = 0x01;
    imuRec.timestamp_ms = (uint32_t)now;
    imuRec.ax = sample.rawAx;
    imuRec.ay = sample.rawAy;
    imuRec.az = sample.rawAz;
    imuRec.gx = sample.rawGx;
    imuRec.gy = sample.rawGy;
    imuRec.gz = sample.rawGz;

    if (!__atomic_load_n(&recordingStorageFault, __ATOMIC_RELAXED) && s_recordingRingBuf) {
      if (xRingbufferSend(s_recordingRingBuf, &imuRec, sizeof(imuRec), 0) == pdTRUE) {
        incrementCounter(&recordsQueued);
      } else {
        incrementCounter(&ringBufferOverflowCount);
      }
    }

    const uint32_t liveSequence = __atomic_fetch_add(
      &pcLiveSampleSequence, 1U, __ATOMIC_RELAXED);
    if (bleConnected && pcLiveImuQueue) {
      remus::blade::protocol::RawImuFrame liveFrame{};
      liveFrame.sampleSequence = liveSequence;
      liveFrame.nativeTimestampUs = static_cast<uint64_t>(esp_timer_get_time());
      liveFrame.ax = sample.rawAx;
      liveFrame.ay = sample.rawAy;
      liveFrame.az = sample.rawAz;
      liveFrame.gx = sample.rawGx;
      liveFrame.gy = sample.rawGy;
      liveFrame.gz = sample.rawGz;
      if (xQueueSend(pcLiveImuQueue, &liveFrame, 0) != pdTRUE) {
        incrementCounter(&pcLiveQueueDropCount);
      }
    }

    // 2) Preview SPM: média simples de 8 amostras = ~25 Hz. Isso reduz
    // vibração/aliasing e custa apenas algumas somas no caminho de 200 Hz.
    // A fila é não-bloqueante: se o preview não acompanhar, descartamos apenas
    // o preview; NUNCA atrasamos nem sacrificamos o IMU bruto.
    if (liveSpmSampleQueue && __atomic_load_n(&liveSpmPreviewEnabled, __ATOMIC_RELAXED)) {
      const uint32_t generation = __atomic_load_n(&liveSpmGeneration, __ATOMIC_RELAXED);
      if (spmAccumulatorGeneration != generation) {
        spmAccumulatorGeneration = generation;
        spmAccumulatorCount = 0;
        spmAxSum = spmAySum = spmAzSum = 0.0f;
      }

      spmAxSum += sample.accelX * 9.80665f;
      spmAySum += sample.accelY * 9.80665f;
      spmAzSum += sample.accelZ * 9.80665f;
      spmAccumulatorCount++;

      if (spmAccumulatorCount >= LIVE_SPM_DOWNSAMPLE_FACTOR) {
        const float invCount = 1.0f / (float)spmAccumulatorCount;
        LiveSpmInputSample preview{};
        preview.timestamp_ms = (uint32_t)now;
        preview.ax_mps2 = spmAxSum * invCount;
        preview.ay_mps2 = spmAySum * invCount;
        preview.az_mps2 = spmAzSum * invCount;
        preview.generation = generation;

        if (xQueueSend(liveSpmSampleQueue, &preview, 0) != pdTRUE) {
          incrementCounter(&liveSpmQueueDropCount);
        }

        spmAccumulatorCount = 0;
        spmAxSum = spmAySum = spmAzSum = 0.0f;
      }
    }

    xSemaphoreGive(recordingStateMutex);
  }
}

void notifyPcLiveBatch(const uint8_t* batch, size_t batchLength, uint32_t batchSequence) {
  namespace protocol = remus::blade::protocol;
  if (!bleConnected || !pImuStreamCharacteristic || !batch || batchLength == 0) return;
  const size_t mtuPayload = BLEDevice::getMTU() > 3 ? BLEDevice::getMTU() - 3 : 20;
  if (batchLength <= mtuPayload) {
    pImuStreamCharacteristic->setValue(const_cast<uint8_t*>(batch), batchLength);
    pImuStreamCharacteristic->notify();
    return;
  }
  if (mtuPayload <= protocol::kFragmentHeaderSize) {
    incrementCounter(&pcLiveQueueDropCount);
    return;
  }
  const size_t fragmentPayload = mtuPayload - protocol::kFragmentHeaderSize;
  const size_t fragmentCountSize = (batchLength + fragmentPayload - 1) / fragmentPayload;
  if (fragmentCountSize > 255) {
    incrementCounter(&pcLiveQueueDropCount);
    return;
  }
  std::array<uint8_t, 256> fragment{};
  const uint8_t fragmentCount = static_cast<uint8_t>(fragmentCountSize);
  for (uint8_t index = 0; index < fragmentCount && bleConnected; ++index) {
    const size_t offset = static_cast<size_t>(index) * fragmentPayload;
    const size_t length = std::min(fragmentPayload, batchLength - offset);
    const size_t encoded = protocol::encodeFragment(
      fragment.data(), fragment.size(), batchSequence, index, fragmentCount,
      batch + offset, length);
    pImuStreamCharacteristic->setValue(fragment.data(), encoded);
    pImuStreamCharacteristic->notify();
    taskYIELD();
  }
}

void notifyBladeRelayPacket(const BladeRelayPacket& packet) {
  namespace protocol = remus::blade::protocol;
  if (!bleConnected || !pBladeRelayCharacteristic || packet.length == 0) return;
  if (packet.channelIndex >= BLADE_CHANNEL_COUNT) return;
  BladeChannel& channel = bladeChannels[packet.channelIndex];
  std::array<uint8_t, protocol::kMaxRelayedPacketSize> relayed{};
  const size_t relayedLength = protocol::encodeRelayedPacket(
    relayed.data(), relayed.size(), packet.sourceIdentityHash,
    packet.receivedAtMs, packet.payload, packet.length);
  if (relayedLength == 0) {
    incrementCounter(&channel.liveDropCount);
    return;
  }

  uint16_t peerMtu = 23;
  if (pServer) {
    const uint16_t candidate = pServer->getPeerMTU(pServer->getConnId());
    if (candidate >= 23 && candidate <= 512) peerMtu = candidate;
  }
  const size_t mtuPayload = peerMtu - 3;
  if (relayedLength <= mtuPayload) {
    pBladeRelayCharacteristic->setValue(relayed.data(), relayedLength);
    pBladeRelayCharacteristic->notify();
    return;
  }
  if (mtuPayload <= protocol::kFragmentHeaderSize) {
    incrementCounter(&channel.liveDropCount);
    return;
  }
  const size_t fragmentPayload = mtuPayload - protocol::kFragmentHeaderSize;
  const size_t fragmentCountSize = (relayedLength + fragmentPayload - 1) / fragmentPayload;
  if (fragmentCountSize > 255) {
    incrementCounter(&channel.liveDropCount);
    return;
  }
  const uint32_t relaySequence = __atomic_fetch_add(
    &channel.liveSequence, 1U, __ATOMIC_RELAXED);
  std::array<uint8_t, 256> fragment{};
  const uint8_t fragmentCount = static_cast<uint8_t>(fragmentCountSize);
  for (uint8_t index = 0; index < fragmentCount && bleConnected; ++index) {
    const size_t offset = static_cast<size_t>(index) * fragmentPayload;
    const size_t length = std::min(fragmentPayload, relayedLength - offset);
    const size_t encoded = protocol::encodeFragment(
      fragment.data(), fragment.size(), relaySequence, index, fragmentCount,
      relayed.data() + offset, length);
    if (encoded == 0) {
      incrementCounter(&channel.liveDropCount);
      return;
    }
    pBladeRelayCharacteristic->setValue(fragment.data(), encoded);
    pBladeRelayCharacteristic->notify();
    taskYIELD();
  }
}

void pcLiveImuStreamingTask(void*) {
  namespace protocol = remus::blade::protocol;
  std::array<protocol::RawImuFrame, protocol::kMaxSamplesPerBatch> samples{};
  std::array<uint8_t, protocol::kMaxBatchSize> encoded{};
  for (;;) {
    if (!isWorkoutActive || !bleConnected || !pcLiveImuQueue) {
      if (pcLiveImuQueue) xQueueReset(pcLiveImuQueue);
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    size_t count = 0;
    if (xQueueReceive(pcLiveImuQueue, &samples[count], pdMS_TO_TICKS(100)) == pdTRUE) {
      ++count;
    }
    while (count < samples.size() &&
           xQueueReceive(pcLiveImuQueue, &samples[count], pdMS_TO_TICKS(6)) == pdTRUE) {
      ++count;
    }
    if (count == 0) continue;
    const uint32_t batchSequence = __atomic_fetch_add(
      &pcLiveBatchSequence, 1U, __ATOMIC_RELAXED);
    const size_t length = protocol::encodeImuBatch(
      encoded.data(), encoded.size(), batchSequence, samples.data(), count);
    notifyPcLiveBatch(encoded.data(), length, batchSequence);
  }
}

// =========================================================================
// TASK DE PREVIEW LIVE SPM (~25 Hz input, prioridade baixa)
// =========================================================================
// O cálculo pesado fica completamente fora da task de 200 Hz. No ESP32-C3
// (single core), a task IMU de prioridade 5 sempre pode preemptar esta task.
// O app recebe liveSpm pela telemetria BLE a 1 Hz; o estimador só precisa
// atualizar esse valor periodicamente, não produzir evidência final.
void liveSpmProcessingTask(void* pvParameters) {
  uint32_t estimatorGeneration = 0;
  LiveSpmInputSample sample{};

  while (true) {
    if (!liveSpmSampleQueue ||
        !__atomic_load_n(&liveSpmPreviewEnabled, __ATOMIC_RELAXED) ||
        isTransferActive || !isWorkoutActive) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (xQueueReceive(liveSpmSampleQueue, &sample, pdMS_TO_TICKS(100)) != pdTRUE) {
      continue;
    }

    // O estado pode ter mudado enquanto esperávamos a fila. Não inicia uma
    // autocorrelação pesada se o workout terminou ou o GET começou.
    if (!__atomic_load_n(&liveSpmPreviewEnabled, __ATOMIC_RELAXED) ||
        isTransferActive || !isWorkoutActive) {
      continue;
    }

    if (sample.generation != estimatorGeneration) {
      spmEstimator.reset();
      spmEstimator.setPreferredAxis(LIVE_SPM_PREFERRED_AXIS);
      estimatorGeneration = sample.generation;
    }

    const auto spmRes = spmEstimator.push(
      (double)sample.timestamp_ms / 1000.0,
      sample.ax_mps2,
      sample.ay_mps2,
      sample.az_mps2);

    if (!spmRes.updated) continue;

    // Resultado de uma sessão antiga que terminou enquanto calculávamos:
    // descarta silenciosamente, sem tocar na UI nem no arquivo atual.
    const uint32_t currentGeneration = __atomic_load_n(&liveSpmGeneration, __ATOMIC_RELAXED);
    if (sample.generation != currentGeneration ||
        !__atomic_load_n(&liveSpmPreviewEnabled, __ATOMIC_RELAXED) ||
        isTransferActive || !isWorkoutActive) continue;

    if (spmRes.available) {
      const float currentSpm = (float)spmRes.stroke_rate_spm;
      portENTER_CRITICAL(&telemetryMux);
      liveSpm = currentSpm;
      portEXIT_CRITICAL(&telemetryMux);

      // 0x03 continua significando um NOVO resultado aceito pelo estimator.
      // Quando held=true a UI conserva temporariamente o último SPM confiável,
      // mas não gravamos outro 0x03: análise offline continua distinguindo uma
      // medição aceita de um simples hold de apresentação.
      if (!spmRes.held && recordingStateMutex && s_recordingRingBuf &&
          !__atomic_load_n(&recordingStorageFault, __ATOMIC_RELAXED)) {
        xSemaphoreTake(recordingStateMutex, portMAX_DELAY);
        if (isWorkoutActive && sample.generation == __atomic_load_n(&liveSpmGeneration, __ATOMIC_RELAXED)) {
          RemusSpmRecord spmRec;
          spmRec.type = 0x03;
          spmRec.timestamp_ms = (uint32_t)millis();
          spmRec.spm_x10 = (uint16_t)(currentSpm * 10.0f);
          if (xRingbufferSend(s_recordingRingBuf, &spmRec, sizeof(spmRec), 0) == pdTRUE) {
            incrementCounter(&recordsQueued);
          } else {
            incrementCounter(&ringBufferOverflowCount);
          }
        }
        xSemaphoreGive(recordingStateMutex);
      }
    } else if (spmRes.progress >= 1.0 && spmRes.reason == "recent_quiet") {
      // Zero significa parada confirmada. Janela fraca, ambígua ou sem dados
      // não é convertida em zero, nem na UI nem no arquivo.
      portENTER_CRITICAL(&telemetryMux);
      const bool stoppedFromActiveCadence = liveSpm > 0.0f;
      liveSpm = 0.0f;
      portEXIT_CRITICAL(&telemetryMux);

      // Zero is evidence of a confirmed stop, not a substitute for an
      // unavailable/ambiguous estimate. Emit exactly on the non-zero -> quiet
      // transition; subsequent quiet windows see liveSpm already at zero.
      if (stoppedFromActiveCadence && recordingStateMutex && s_recordingRingBuf &&
          !__atomic_load_n(&recordingStorageFault, __ATOMIC_RELAXED)) {
        RemusSpmRecord spmRec{};
        spmRec.type = 0x03;
        spmRec.timestamp_ms = sample.timestamp_ms;
        spmRec.spm_x10 = 0;
        xSemaphoreTake(recordingStateMutex, portMAX_DELAY);
        if (isWorkoutActive && sample.generation ==
            __atomic_load_n(&liveSpmGeneration, __ATOMIC_RELAXED)) {
          if (xRingbufferSend(s_recordingRingBuf, &spmRec, sizeof(spmRec), 0) == pdTRUE) {
            incrementCounter(&recordsQueued);
          } else {
            incrementCounter(&ringBufferOverflowCount);
          }
        }
        xSemaphoreGive(recordingStateMutex);
      }
    }

    // Cortesia para loopTask/GPS/SD em um MCU single-core.
    taskYIELD();
  }
}

static void remusAppBeginImpl() {
  // Buffers maiores deixam o USB Serial/JTAG mais estável durante recuperação
  // de arquivos. Fora do modo de recuperação mantemos TX timeout 0 para que a
  // captura autônoma nunca dependa de um computador conectado.
  Serial.setTxBufferSize(4096);
  Serial.setRxBufferSize(1024);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); // Evita bloqueio por timeout se desconectado da USB no barco
  delay(1200);
  Serial.println("REMUS_USB_CDC_READY");
  Serial.flush();
  delay(800);

  Serial.println("\n=========================================================");
  Serial.println("       REMUS — SISTEMA MODULAR DE TELEMETRIA             ");
  Serial.printf("       Firmware %s | Session format RBP2\n", REMUS_FIRMWARE_VERSION);
  Serial.println("=========================================================");
  printHardwareProfile();
  loadBladeSlots();

  // 1. Inicializa sincronização, comandos e RingBuffer de gravação.
  recordingStateMutex = xSemaphoreCreateMutex();
  bladeOrientationMutex = xSemaphoreCreateMutex();
  controlCommandQueue = xQueueCreate(4, sizeof(ControlCommand));
  s_recordingRingBuf = xRingbufferCreate(RECORDING_RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
  liveSpmSampleQueue = xQueueCreate(LIVE_SPM_QUEUE_LENGTH, sizeof(LiveSpmInputSample));
  pcLiveImuQueue = xQueueCreate(
    PC_LIVE_IMU_QUEUE_LENGTH, sizeof(remus::blade::protocol::RawImuFrame));
  bladeRelayQueue = xQueueCreate(BLADE_RELAY_QUEUE_LENGTH, sizeof(BladeRelayPacket));
  if (s_recordingRingBuf && recordingStateMutex && controlCommandQueue) {
    Serial.printf("[BUF] ✅ Pipeline RAM alocado com sucesso (%u bytes)\n", (unsigned int)RECORDING_RING_BUFFER_SIZE);
  } else {
    Serial.println("[BUF] ❌ Falha crítica ao alocar fila, mutex ou RingBuffer!");
  }
  if (liveSpmSampleQueue) {
    Serial.printf("[SPM] ✅ Fila de preview criada (%u amostras a ~25 Hz).\n", (unsigned int)LIVE_SPM_QUEUE_LENGTH);
  } else {
    Serial.println("[SPM] ⚠️ Fila de preview indisponível. Gravação bruta continuará normalmente.");
  }
  if (pcLiveImuQueue) {
    Serial.printf("[BLE] ✅ Fila do stream bruto alocada (%u amostras).\n",
      static_cast<unsigned int>(PC_LIVE_IMU_QUEUE_LENGTH));
  } else {
    Serial.println("[BLE] ⚠️ Stream bruto ao app indisponível; gravação local continua operacional.");
  }
  if (bladeRelayQueue) {
    Serial.printf("[BLADE RELAY] ✅ Fila de backup criada (%u notificações).\n",
      static_cast<unsigned int>(BLADE_RELAY_QUEUE_LENGTH));
  } else {
    Serial.println("[BLADE RELAY] ⚠️ Backup interno do Blade indisponível.");
  }

  // 2. Inicializa BLE
  if (remus::hardware.hasBle) setupBLE();

  // 3. Inicializa GPS
  setupGPS();

  // 4. Inicializa IMU através da HAL
  setupIMU();

  // 5. Inicializa storage
  setupSD();

  // 6. Inicializa o TFT em software SPI, isolado do barramento do MicroSD.
  setupDisplay();
  updateDisplay(true);

  // 7. Inicia Task de Amostragem do IMU a 200 Hz com prioridade 5 (acima do SD/loopTask)
  BaseType_t taskCreated = xTaskCreate(
    imuSamplingTask,
    "imuTask",
    4096,
    NULL,
    5,
    &imuTaskHandle
  );
  if (taskCreated == pdPASS) {
    Serial.println("[TASK] ✅ Task IMU a 200 Hz iniciada com Prioridade 5.");
  } else {
    imuTaskHandle = NULL;
    imuOk = false;
    Serial.println("[TASK] ❌ Falha ao criar Task IMU.");
  }

  if (pcLiveImuQueue) {
    BaseType_t liveTaskCreated = xTaskCreate(
      pcLiveImuStreamingTask,
      "pcLiveImu",
      4096,
      NULL,
      2,
      &pcLiveImuTaskHandle
    );
    if (liveTaskCreated == pdPASS) {
      Serial.println("[TASK] ✅ Stream IMU bruto ao app iniciado (prioridade 2).");
    } else {
      pcLiveImuTaskHandle = NULL;
      Serial.println("[TASK] ⚠️ Stream IMU bruto ao app indisponível; SD permanece ativo.");
    }
  }

  if (bladeRelayQueue && remus::hardware.hasBle) {
    BaseType_t relayTaskCreated = xTaskCreate(
      bladeRelayClientTask,
      "bladeRelay",
      6144,
      NULL,
      1,
      &bladeRelayClientTaskHandle
    );
    if (relayTaskCreated == pdPASS) {
      Serial.println("[TASK] ✅ Cliente Blade/backup interno iniciado (prioridade 1).");
    } else {
      bladeRelayClientTaskHandle = NULL;
      Serial.println("[TASK] ⚠️ Cliente Blade indisponível; RBP2 e app continuam ativos.");
    }
  }

  // 8. Live SPM é apenas preview: prioridade baixa, mas acima da Idle Task.
  // Prioridade 1 garante CPU suficiente para consumir a fila no ESP32-C3
  // single-core sem competir com a task IMU, que permanece em prioridade 5.
  if (liveSpmSampleQueue) {
    BaseType_t spmTaskCreated = xTaskCreate(
      liveSpmProcessingTask,
      "liveSpmTask",
      4096,
      NULL,
      1,
      &liveSpmTaskHandle
    );
    if (spmTaskCreated == pdPASS) {
      Serial.println("[TASK] ✅ Live SPM iniciado com Prioridade 1 (preview somente).");
    } else {
      liveSpmTaskHandle = NULL;
      Serial.println("[TASK] ⚠️ Falha ao criar Live SPM. IMU/GPS/SD continuam operacionais.");
    }
  }

  // 9. Exibe relatório persistido em Flash NVS
  printSavedReport();
}

void processControlCommands() {
  if (!controlCommandQueue) return;
  ControlCommand command{};
  while (xQueueReceive(controlCommandQueue, &command, 0) == pdTRUE) {
    switch (command.type) {
      case ControlCommandType::Start:
        startWorkoutRecording();
        break;
      case ControlCommandType::Stop:
        stopWorkoutRecording();
        break;
      case ControlCommandType::Get:
        startFileTransfer(String(command.path));
        break;
    }
  }
}

static void remusAppTickImpl() {
  // --- RECONEXÃO BLE ---
  if (!bleConnected && oldBleConnected) {
    delay(500);
    pServer->startAdvertising();
    if (!usbRecoveryTransferActive) Serial.println("[BLE] Anúncio reiniciado para novos clientes.");
    oldBleConnected = bleConnected;
  }
  if (bleConnected && !oldBleConnected) {
    oldBleConnected = bleConnected;
  }

  // Comandos BLE apenas entram em uma fila no callback. Operações de SD
  // acontecem aqui, em um único contexto, evitando fechar logFile durante write.
  processControlCommands();

  // --- PROCESSO DE TRANSFERÊNCIA DE ARQUIVO (NÃO-BLOQUEANTE) ---
  if (isTransferActive) {
    // Continue draining the UART during a multi-minute SD transfer. Without
    // this poll the small hardware UART buffer overflows and the display looks
    // as if GNSS disappeared until the transfer finishes.
    if (remus::hardware.hasGps) gpsDevice.poll(false, false);
    // processFileTransfer() controla pacing/retry internamente. Não acrescente
    // delay aqui: o app atual encerra GET após 30 s totais.
    processFileTransfer();
    return;
  }

  // --- COMANDOS DO USUÁRIO / RECUPERAÇÃO USB DO MICROSD ---
  processUsbRecoveryInput();

  // --- LEITURA CONTÍNUA DO GPS ---
  if (remus::hardware.hasGps) gpsDevice.poll(showRawNmea, isWorkoutActive);

  // Cache the latest fix in RAM. NVS commits are intentionally kept out of an
  // active workout because flash compaction can block the single ESP32-C3 core
  // for hundreds of milliseconds and destroy IMU continuity.
  if (remus::hardware.hasGps && gpsDevice.navigationSolutionUpdated()) {
    const bool validFix = isGpsFixValid();
    incrementCounter(&gpsEpochsReceived);
    incrementCounter(validFix ? &gpsValidEpochs : &gpsInvalidEpochs);

    const uint32_t currentItowMs = gpsDevice.gpsTimeOfWeekMs();
    if (gpsDevice.enhancedNavigationAvailable() && previousGpsItowMs != UINT32_MAX) {
      // iTOW rolls every GPS week. Unsigned modulo arithmetic handles the
      // rollover as long as consecutive observations are less than one week apart.
      constexpr uint32_t GPS_WEEK_MS = 604800000UL;
      const uint32_t delta = currentItowMs >= previousGpsItowMs
        ? currentItowMs - previousGpsItowMs
        : GPS_WEEK_MS - previousGpsItowMs + currentItowMs;
      if (delta > 201) {
        incrementCounter(&gpsItowGapCount);
        unsigned long observedMax = readCounter(&gpsMaxItowGapMs);
        while (delta > observedMax && !__atomic_compare_exchange_n(
            &gpsMaxItowGapMs, &observedMax, delta, false,
            __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {}
      }
    }
    previousGpsItowMs = currentItowMs;

    if (validFix) {
      fixCount++;
      gpsSnapshotDirty = true;
      if (!isWorkoutActive && millis() - lastGpsNvsPersist >= GPS_NVS_IDLE_INTERVAL_MS) {
        persistLatestGpsFix();
      }
    }

    // O GPS usa o mesmo mutex da task do IMU, preservando registros inteiros e
    // garantindo que STOP não drene a fila durante um enqueue em voo.
    if (sdOk && isWorkoutActive && logFile && s_recordingRingBuf && recordingStateMutex) {
      RemusGpsV2Record gpsRec{};
      gpsRec.type = 0x04;
      gpsRec.timestamp_ms = gpsDevice.navigationReceivedAtMs();
      gpsRec.gps_itow_ms = gpsDevice.gpsTimeOfWeekMs();
      gpsRec.lat_e7 = (int32_t)(gpsDevice.latitude() * 1e7);
      gpsRec.lon_e7 = (int32_t)(gpsDevice.longitude() * 1e7);
      gpsRec.ground_speed_cm_s = gpsDevice.groundSpeedCmPerSecond();
      gpsRec.speed_accuracy_cm_s = gpsDevice.speedAccuracyCmPerSecond();
      gpsRec.course_deg_e5 = gpsDevice.courseDegreesE5();
      gpsRec.course_accuracy_deg_e5 = gpsDevice.courseAccuracyDegreesE5();
      gpsRec.horizontal_accuracy_mm = gpsDevice.enhancedNavigationAvailable()
        ? gpsDevice.horizontalAccuracyMm()
        : static_cast<uint32_t>(gpsDevice.hdop() * 2500.0f);
      gpsRec.sats_in_use = gpsDevice.satellitesInUse();
      gpsRec.max_snr = (uint8_t)gpsDevice.maxSnr();
      gpsRec.fix_type = gpsDevice.fixType();
      gpsRec.flags = validFix ? 0x01 : 0x00;

      xSemaphoreTake(recordingStateMutex, portMAX_DELAY);
      if (isWorkoutActive) {
        if (xRingbufferSend(s_recordingRingBuf, &gpsRec, sizeof(gpsRec), 0) == pdTRUE) {
          incrementCounter(&recordsQueued);
        } else {
          incrementCounter(&ringBufferOverflowCount);
        }
      }
      xSemaphoreGive(recordingStateMutex);
    }
  }

  unsigned long now = millis();

  // --- AUTO-RECUPERAÇÃO DE HARDWARE (IMU / MicroSD) a cada 5s ---
  static unsigned long lastHardwareRetry = 0;
  if (!isTransferActive && (!sdOk || !imuOk) && (now - lastHardwareRetry >= 5000)) {
    lastHardwareRetry = now;
    if (!imuOk) {
      setupIMU();
    }
    if (!sdOk && !(isWorkoutActive &&
        __atomic_load_n(&recordingStorageFault, __ATOMIC_RELAXED))) {
      setupSD();
    }
  }

  // --- DESPEJO CONTÍNUO DO RINGBUFFER NO MICROSD (SPI) ---
  // A escrita em disco consome blocos de até 4 KB da RAM. Se o cartão SD
  // sofrer picos de latência, a task imuSamplingTask (Prioridade 5) pode
  // preemptar a loopTask enquanto a fila absorve o backlog.
  if (sdOk && isWorkoutActive && logFile && s_recordingRingBuf) {
    size_t rxSize = 0;
    for (int b = 0; b < 2; b++) {
      uint8_t* pData = (uint8_t*)xRingbufferReceiveUpTo(s_recordingRingBuf, &rxSize, 0, 4096);
      if (pData != NULL && rxSize > 0) {
        const bool persisted = writeRecordingBytes(pData, rxSize);
        vRingbufferReturnItem(s_recordingRingBuf, pData);
        if (!persisted) {
          abortRecordingAfterStorageFailure(rxSize);
          break;
        }
        if (rxSize < 4096) break;
      } else {
        break;
      }
    }
  }

  // O loopTask continua sendo o único escritor no MicroSD. Pacotes recebidos
  // pelo callback BLE entram apenas na fila; a persistência do sidecar ocorre
  // aqui, depois do RBP2 principal e com orçamento limitado por iteração.
  if (bladeRelayQueue &&
      (isWorkoutActive ||
       __atomic_load_n(&bladeCalibrationRequested, __ATOMIC_RELAXED))) {
    processBladeRelayStorage(8);
  }

  // Flush periódico longo (apenas a cada 60s) para garantir integridade FAT em treinos longos
  if (sdOk && isWorkoutActive && logFile && (now - lastFlush >= 60000)) {
    lastFlush = now;
    logFile.flush();
    for (File& file : bladeRelayFiles) if (file) file.flush();
  }

  // --- PAINEL SERIAL E BROADCAST BLE DE TELEMETRIA AO VIVO (1 Hz) ---
  if (!isTransferActive && !usbRecoveryTransferActive && (now - lastSerialPrint >= 1000)) {
    lastSerialPrint = now;
    uint64_t now_us = esp_timer_get_time();
    float telemetryAx, telemetryAy, telemetryAz;
    float telemetryGx, telemetryGy, telemetryGz, telemetrySpm;
    portENTER_CRITICAL(&telemetryMux);
    telemetryAx = ax; telemetryAy = ay; telemetryAz = az;
    telemetryGx = gx; telemetryGy = gy; telemetryGz = gz;
    telemetrySpm = liveSpm;
    portEXIT_CRITICAL(&telemetryMux);
    const unsigned long writtenSnapshot = readCounter(&recordsPersisted);
    const unsigned long queuedSnapshot = readCounter(&recordsQueued);
    const unsigned long gapSnapshot = readCounter(&imuGapCount);
    const unsigned long maxIntervalSnapshot = readCounter(&maxImuGapMs);
    const unsigned long overflowSnapshot = readCounter(&ringBufferOverflowCount);
    const unsigned long readFailureSnapshot = readCounter(&imuReadFailureCount);
    const unsigned long spmPreviewDropSnapshot = readCounter(&liveSpmQueueDropCount);
    bool anyBladeConnected = false;
    unsigned long bladeNotifications = 0;
    unsigned long bladePacketsPersisted = 0;
    unsigned long bladeQueueDrops = 0;
    unsigned long bladeWriteFailures = 0;
    unsigned long bladeLiveDrops = 0;
    bool anyBladeStorageFault = false;
    for (BladeChannel& channel : bladeChannels) {
      anyBladeConnected = anyBladeConnected ||
        __atomic_load_n(&channel.connected, __ATOMIC_RELAXED);
      bladeNotifications += readCounter(&channel.notificationsReceived);
      bladePacketsPersisted += readCounter(&channel.packetsPersisted);
      bladeQueueDrops += readCounter(&channel.queueDropCount);
      bladeWriteFailures += readCounter(&channel.writeFailureCount);
      bladeLiveDrops += readCounter(&channel.liveDropCount);
      anyBladeStorageFault = anyBladeStorageFault ||
        __atomic_load_n(&channel.storageFault, __ATOMIC_RELAXED);
    }
    bool hasFix = isGpsFixValid();
    int satsInUse = hasFix ? gpsDevice.satellitesInUse() : 0;
    int satsInView = max(satsInUse, gpsDevice.satellitesInView());
    float accuracyMeters = hasFix
      ? (gpsDevice.enhancedNavigationAvailable()
          ? gpsDevice.horizontalAccuracyMm() / 1000.0f
          : (gpsDevice.hdopValidRecent() ? gpsDevice.hdop() * 2.5f : 0.0f))
      : 0.0f;

    // O TFT usa atualizações parciais a 1 Hz para não redesenhar a tela inteira
    // nem competir com a task de aquisição IMU a 200 Hz.
    updateDisplay();

    // 1. Transmissão BLE (notifica o app com telemetria ao vivo a 1 Hz)
    if (bleConnected && pCharacteristic) {
      notifyBladeRoster();
      char bleBuf[400];
      char satsStr[32];
      snprintf(satsStr, sizeof(satsStr), "%d/%d:%d:%.1fm", satsInUse, satsInView, gpsDevice.maxSnr(), accuracyMeters);

      unsigned long charsRx = remus::hardware.hasGps ? gpsDevice.charsProcessed() : 0;
      if (hasFix) {
        snprintf(bleBuf, sizeof(bleBuf), "%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%s,%lu,%lu,%.1f,%d,%d,%lu,%lu,%lu,%lu,%.2f,%.5f,%.5f,%u,%lu,%lu,%lu,%lu,%d,%lu,%lu,%lu,%lu,%d,%lu",
          (unsigned long)(now_us / 1000), telemetryAx, telemetryAy, telemetryAz, telemetryGx, telemetryGy, telemetryGz,
          gpsDevice.latitude(), gpsDevice.longitude(), gpsDevice.speedKmph(),
          satsStr, writtenSnapshot, charsRx, telemetrySpm,
          imuOk ? 1 : 0, sdOk ? 1 : 0, gapSnapshot, maxIntervalSnapshot,
          overflowSnapshot, readFailureSnapshot,
          gpsDevice.speedAccuracyCmPerSecond() / 100.0f,
          gpsDevice.courseDegreesE5() / 100000.0f,
          gpsDevice.courseAccuracyDegreesE5() / 100000.0f,
          gpsDevice.fixType(), (unsigned long)gpsDevice.gpsTimeOfWeekMs(),
          queuedSnapshot, readCounter(&storageWriteFailureCount),
          readCounter(&pcLiveQueueDropCount),
          anyBladeConnected ? 1 : 0,
          bladeNotifications, bladePacketsPersisted, bladeQueueDrops,
          bladeWriteFailures, anyBladeStorageFault ? 1 : 0, bladeLiveDrops);
      } else {
        snprintf(bleBuf, sizeof(bleBuf), "%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,,,,%s,%lu,%lu,%.1f,%d,%d,%lu,%lu,%lu,%lu,,,,%u,%lu,%lu,%lu,%lu,%d,%lu,%lu,%lu,%lu,%d,%lu",
          (unsigned long)(now_us / 1000), telemetryAx, telemetryAy, telemetryAz, telemetryGx, telemetryGy, telemetryGz,
          satsStr, writtenSnapshot, charsRx, telemetrySpm,
          imuOk ? 1 : 0, sdOk ? 1 : 0, gapSnapshot, maxIntervalSnapshot,
          overflowSnapshot, readFailureSnapshot,
          gpsDevice.fixType(), (unsigned long)gpsDevice.gpsTimeOfWeekMs(),
          queuedSnapshot, readCounter(&storageWriteFailureCount),
          readCounter(&pcLiveQueueDropCount),
          anyBladeConnected ? 1 : 0,
          bladeNotifications, bladePacketsPersisted, bladeQueueDrops,
          bladeWriteFailures, anyBladeStorageFault ? 1 : 0, bladeLiveDrops);
      }
      pCharacteristic->setValue(bleBuf);
      pCharacteristic->notify();
    }

    // 2. Painel Serial. Uma porta USB ausente nunca é dependência da
    // captura autônoma; sem espaço no endpoint, o painel daquele segundo cai.
    if (!(Serial && Serial.availableForWrite() >= 128)) return;
    Serial.print("[REMUS] ");
    if (imuOk) {
      Serial.printf("IMU: [AX:%.2f AY:%.2f AZ:%.2f] ", telemetryAx, telemetryAy, telemetryAz);
    } else {
      Serial.print("IMU: [Off] ");
    }

    Serial.printf("| GPS: [%d/%d Sats, SNR:%d, Acc:%.1fm, Fix:%s] | SPM: %.1f ",
      satsInUse, satsInView, gpsDevice.maxSnr(), accuracyMeters,
      hasFix ? "LOCK" : "Buscando", telemetrySpm);

    if (sdOk) {
      if (isWorkoutActive) {
        Serial.printf("| SD: [Gravando: %s (%lu/%lu persistidos/fila)] ",
          currentSessionFileName, writtenSnapshot, queuedSnapshot);
      } else if (writtenSnapshot > 0) {
        Serial.printf("| SD: [Finalizado: %s (%lu/%lu persistidos/fila)] ",
          currentSessionFileName, writtenSnapshot, queuedSnapshot);
      } else {
        Serial.print("| SD: [Standby] ");
      }
    } else {
      Serial.print("| SD: [Off] ");
    }

    Serial.printf("| BLE: [%s] ", bleConnected ? "CONECTADO" : "ANUNCIANDO");
    Serial.printf("| IMU gaps >%lums: %lu (max %lums) | Buf Overflow: %lu | I2C fail: %lu | SPM preview drops: %lu ",
      IMU_GAP_THRESHOLD_MS, gapSnapshot, maxIntervalSnapshot, overflowSnapshot, readFailureSnapshot, spmPreviewDropSnapshot);
    if (isWorkoutActive) {
      Serial.printf("| GPS epochs: %lu valid / %lu invalid (iTOW gaps: %lu, max %lums) ",
        readCounter(&gpsValidEpochs), readCounter(&gpsInvalidEpochs),
        readCounter(&gpsItowGapCount), readCounter(&gpsMaxItowGapMs));
    }
    Serial.println();
  }
}


void remus::app::RemusApp::begin() {
  remusAppBeginImpl();
}

void remus::app::RemusApp::tick() {
  remusAppTickImpl();
}
