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
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "remus/ActiveProfile.hpp"
#include "remus/app/RemusApp.hpp"
#include "remus/core/LiveSpmEstimator.hpp"
#include "remus/core/SessionFormat.hpp"
#include "remus/drivers/Gmt024Display.hpp"
#include "remus/drivers/Mpu6050Imu.hpp"
#include "remus/drivers/Neo6mGps.hpp"
#include "remus/drivers/SdCardStorage.hpp"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define REMUS_FIRMWARE_VERSION "0.3.2"

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

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
volatile bool bleConnected = false;
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
volatile unsigned long recordsWritten = 0;
volatile bool isWorkoutActive = false;
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

// Fila circular em RAM (16 KB = ~4,8s de IMU a 200 Hz). Ela absorve a
// latência normal do MicroSD sem alterar o contrato binário RBP1.
static RingbufHandle_t s_recordingRingBuf = NULL;
constexpr size_t RECORDING_RING_BUFFER_SIZE = 16384;
constexpr unsigned long IMU_GAP_THRESHOLD_MS = 6;
volatile unsigned long ringBufferOverflowCount = 0;
TaskHandle_t imuTaskHandle = NULL;
volatile bool isConfiguringMpu = false;
SemaphoreHandle_t recordingStateMutex = NULL;

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

class RemusCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String rxValue = pChar->getValue().c_str();
    rxValue.trim();
    if (rxValue.length() > 0) {
      if (!usbRecoveryTransferActive) Serial.printf("[BLE RX] 📥 Comando recebido: %s\n", rxValue.c_str());
      if (rxValue.startsWith("AID,")) {
        handleGpsAiding(rxValue);
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
  telemetry.recordsWritten = readCounter(&recordsWritten);
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
  if (isWorkoutActive && logFile) {
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
  uint32_t sessionId = esp_random();
  snprintf(currentSessionFileName, sizeof(currentSessionFileName), "/remus_sensor_%08X.bin", sessionId);

  logFile = SD.open(currentSessionFileName, FILE_WRITE);
  if (!logFile) {
    Serial.printf("[SD] ❌ Erro ao criar %s para gravação!\n", currentSessionFileName);
    isWorkoutActive = false;
    xSemaphoreGive(recordingStateMutex);
    return;
  }

  // RBP2 keeps the RBP1 IMU/SPM layout and adds coherent receiver-native GNSS.
  RemusFileHeader header = remus::session::makeHeaderV2(millis(), sessionId, 200);
  header.padding[0] = static_cast<uint8_t>(remus::hardware.id);
  header.padding[1] = static_cast<uint8_t>(remus::hardware.imuModel);
  header.padding[2] = static_cast<uint8_t>(remus::hardware.gpsModel);
  header.padding[3] = remus::esp32::hw::capabilityByte(remus::hardware);

  logFile.write((const uint8_t*)&header, sizeof(header));
  logFile.flush();

  __atomic_store_n(&recordsWritten, 0UL, __ATOMIC_RELAXED);
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
  Serial.printf("[SD] 🔴 Gravação do Workout INICIADA em binário: %s\n", currentSessionFileName);
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
          logFile.write(pData, rxSize);
          vRingbufferReturnItem(s_recordingRingBuf, pData);
        } else {
          break;
        }
      }
    }
    logFile.flush();
    logFile.close();
  }

  // Persist only the latest GPS recovery snapshot after the time-sensitive
  // recording path has stopped. Repeated NVS commits during capture caused
  // long IMU gaps and invalidated the 15-second cadence windows.
  persistLatestGpsFix();
  updateDisplay();
  Serial.printf("[SD] ⏹️ Gravação FINALIZADA! Arquivo '%s' preservado no SD. Registros: %lu (Overflows: %lu)\n",
    currentSessionFileName, readCounter(&recordsWritten), readCounter(&ringBufferOverflowCount));
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
    (unsigned int)transferMaxPayload, readCounter(&recordsWritten));

  char startBuf[112];
  snprintf(startBuf, sizeof(startBuf), "FILE_START:%s:%u:%lu",
    resolvedPath.c_str(), transferTotalBytes, readCounter(&recordsWritten));

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

    if (!recording || !s_recordingRingBuf) {
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

    if (xRingbufferSend(s_recordingRingBuf, &imuRec, sizeof(imuRec), 0) == pdTRUE) {
      incrementCounter(&recordsWritten);
    } else {
      incrementCounter(&ringBufferOverflowCount);
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
      if (!spmRes.held && recordingStateMutex && s_recordingRingBuf) {
        xSemaphoreTake(recordingStateMutex, portMAX_DELAY);
        if (isWorkoutActive && sample.generation == __atomic_load_n(&liveSpmGeneration, __ATOMIC_RELAXED)) {
          RemusSpmRecord spmRec;
          spmRec.type = 0x03;
          spmRec.timestamp_ms = (uint32_t)millis();
          spmRec.spm_x10 = (uint16_t)(currentSpm * 10.0f);
          if (xRingbufferSend(s_recordingRingBuf, &spmRec, sizeof(spmRec), 0) == pdTRUE) {
            incrementCounter(&recordsWritten);
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
      if (stoppedFromActiveCadence && recordingStateMutex && s_recordingRingBuf) {
        RemusSpmRecord spmRec{};
        spmRec.type = 0x03;
        spmRec.timestamp_ms = sample.timestamp_ms;
        spmRec.spm_x10 = 0;
        xSemaphoreTake(recordingStateMutex, portMAX_DELAY);
        if (isWorkoutActive && sample.generation ==
            __atomic_load_n(&liveSpmGeneration, __ATOMIC_RELAXED)) {
          if (xRingbufferSend(s_recordingRingBuf, &spmRec, sizeof(spmRec), 0) == pdTRUE) {
            incrementCounter(&recordsWritten);
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

  // 1. Inicializa sincronização, comandos e RingBuffer de gravação.
  recordingStateMutex = xSemaphoreCreateMutex();
  controlCommandQueue = xQueueCreate(4, sizeof(ControlCommand));
  s_recordingRingBuf = xRingbufferCreate(RECORDING_RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
  liveSpmSampleQueue = xQueueCreate(LIVE_SPM_QUEUE_LENGTH, sizeof(LiveSpmInputSample));
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
          incrementCounter(&recordsWritten);
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
    if (!sdOk) {
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
        logFile.write(pData, rxSize);
        vRingbufferReturnItem(s_recordingRingBuf, pData);
        if (rxSize < 4096) break;
      } else {
        break;
      }
    }
  }

  // Flush periódico longo (apenas a cada 60s) para garantir integridade FAT em treinos longos
  if (sdOk && isWorkoutActive && logFile && (now - lastFlush >= 60000)) {
    lastFlush = now;
    logFile.flush();
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
    const unsigned long writtenSnapshot = readCounter(&recordsWritten);
    const unsigned long gapSnapshot = readCounter(&imuGapCount);
    const unsigned long maxIntervalSnapshot = readCounter(&maxImuGapMs);
    const unsigned long overflowSnapshot = readCounter(&ringBufferOverflowCount);
    const unsigned long readFailureSnapshot = readCounter(&imuReadFailureCount);
    const unsigned long spmPreviewDropSnapshot = readCounter(&liveSpmQueueDropCount);
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
      char bleBuf[320];
      char satsStr[32];
      snprintf(satsStr, sizeof(satsStr), "%d/%d:%d:%.1fm", satsInUse, satsInView, gpsDevice.maxSnr(), accuracyMeters);

      unsigned long charsRx = remus::hardware.hasGps ? gpsDevice.charsProcessed() : 0;
      if (hasFix) {
        snprintf(bleBuf, sizeof(bleBuf), "%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%s,%lu,%lu,%.1f,%d,%d,%lu,%lu,%lu,%lu,%.2f,%.5f,%.5f,%u,%lu",
          (unsigned long)(now_us / 1000), telemetryAx, telemetryAy, telemetryAz, telemetryGx, telemetryGy, telemetryGz,
          gpsDevice.latitude(), gpsDevice.longitude(), gpsDevice.speedKmph(),
          satsStr, writtenSnapshot, charsRx, telemetrySpm,
          imuOk ? 1 : 0, sdOk ? 1 : 0, gapSnapshot, maxIntervalSnapshot,
          overflowSnapshot, readFailureSnapshot,
          gpsDevice.speedAccuracyCmPerSecond() / 100.0f,
          gpsDevice.courseDegreesE5() / 100000.0f,
          gpsDevice.courseAccuracyDegreesE5() / 100000.0f,
          gpsDevice.fixType(), (unsigned long)gpsDevice.gpsTimeOfWeekMs());
      } else {
        snprintf(bleBuf, sizeof(bleBuf), "%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,,,,%s,%lu,%lu,%.1f,%d,%d,%lu,%lu,%lu,%lu,,,,%u,%lu",
          (unsigned long)(now_us / 1000), telemetryAx, telemetryAy, telemetryAz, telemetryGx, telemetryGy, telemetryGz,
          satsStr, writtenSnapshot, charsRx, telemetrySpm,
          imuOk ? 1 : 0, sdOk ? 1 : 0, gapSnapshot, maxIntervalSnapshot,
          overflowSnapshot, readFailureSnapshot,
          gpsDevice.fixType(), (unsigned long)gpsDevice.gpsTimeOfWeekMs());
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
        Serial.printf("| SD: [Gravando: %s (%lu reg)] ", currentSessionFileName, writtenSnapshot);
      } else if (writtenSnapshot > 0) {
        Serial.printf("| SD: [Finalizado: %s (%lu reg)] ", currentSessionFileName, writtenSnapshot);
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
