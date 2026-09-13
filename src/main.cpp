#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ==========================================
// PINOS DE HARDWARE DO REMUS (ESP32-C3)
// ==========================================
// IMU (I2C)
#define I2C_SDA 0
#define I2C_SCL 1
#define MPU_ADDR 0x68

// GPS (Hardware Serial 1)
#define GPS_RX_PIN 21 // RX do ESP32 -> TX do GPS (GPIO 21)
#define GPS_TX_PIN 20 // TX do ESP32 -> RX do GPS (GPIO 20)

// MicroSD (SPI integrado no ITEAD GPS Shield - Pinos 9, 10, 11, 12 do ESP32 = GPIOs 5, 6, 7, 8)
#define SD_SCK  5  // Pino 9 da placa (GPIO 5) -> D13 (SCK) do Shield
#define SD_MISO 6  // Pino 10 da placa (GPIO 6) -> D12 (MISO) do Shield
#define SD_MOSI 7  // Pino 11 da placa (GPIO 7) -> D11 (MOSI) do Shield
#define SD_CS   8  // Pino 12 da placa (GPIO 8) -> D10 (CS) do Shield

// BLE UUIDs canônicos do REMUS
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#include "LiveSpmEstimator.hpp"

TinyGPSPlus gps;
TinyGPSCustom gpgsvSats(gps, "GPGSV", 3);
TinyGPSCustom gngsvSats(gps, "GNGSV", 3);
TinyGPSCustom glgsvSats(gps, "GLGSV", 3);
TinyGPSCustom bdgsvSats(gps, "BDGSV", 3);
TinyGPSCustom gbgsvSats(gps, "GBGSV", 3);
TinyGPSCustom gagsvSats(gps, "GAGSV", 3);

int satsInViewGP = 0;
int satsInViewGL = 0;
int satsInViewBD = 0;
int satsInViewGA = 0;
int satsInViewGN = 0;

remus::live::LiveSpmEstimator spmEstimator;
float liveSpm = 0.0;

Preferences prefs;
File logFile;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool bleConnected = false;
bool oldBleConnected = false;
char remusDeviceName[24] = "REMUS-ESP32";

bool imuOk = false;
bool sdOk = false;
bool showRawNmea = true;

inline bool isGpsFixValid() {
  return gps.location.isValid() && (gps.location.age() < 2500);
}

void sendNmeaWithChecksum(const char* body) {
  uint8_t cs = 0;
  for (const char* p = body; *p; p++) {
    cs ^= (uint8_t)(*p);
  }
  char buf[96];
  snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, cs);
  Serial1.print(buf);
}

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

unsigned long lastImuLoop = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastFlush = 0;
int liveSatsInView = 0;
int liveMaxSnr = 0;
int fixCount = 0;
unsigned long linesWritten = 0;
bool isWorkoutActive = false;

const char* sessionFileName = "/remus_session.csv";
char sdBuffer[1024];
size_t sdBufLen = 0;

void startWorkoutRecording();
void stopWorkoutRecording();

class RemusBLEServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {

    bleConnected = true;
    Serial.println("[BLE] 📲 Cliente conectado com sucesso!");
  };

  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    Serial.println("[BLE] 📴 Cliente desconectado. Reiniciando anúncio...");
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
    Serial.printf("[GPS AID] 📍 Injetando assistência A-GPS recebida do iPhone: Lat=%.6f, Lon=%.6f\n", lat, lon);

    if (!isGpsFixValid()) {
      char mtkBuf[64];
      snprintf(mtkBuf, sizeof(mtkBuf), "PMTK740,%.6f,%.6f,0,2026,09,12,00,00,00", lat, lon);
      sendNmeaWithChecksum(mtkBuf);

      char casicBuf[64];
      snprintf(casicBuf, sizeof(casicBuf), "PCAS05,%.6f,%.6f,0.0", lat, lon);
      sendNmeaWithChecksum(casicBuf);
    }
  }
}

class RemusCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String rxValue = pChar->getValue().c_str();
    rxValue.trim();
    if (rxValue.length() > 0) {
      Serial.printf("[BLE RX] 📥 Comando recebido: %s\n", rxValue.c_str());
      if (rxValue.startsWith("AID,")) {
        handleGpsAiding(rxValue);
      } else if (rxValue.equalsIgnoreCase("START") || rxValue.startsWith("START")) {
        startWorkoutRecording();
      } else if (rxValue.equalsIgnoreCase("STOP") || rxValue.startsWith("STOP")) {
        stopWorkoutRecording();
      }
    }
  }
};

void setupBLE() {
  uint64_t chipid = ESP.getEfuseMac();
  snprintf(remusDeviceName, sizeof(remusDeviceName), "REMUS-%04X", (uint16_t)(chipid & 0xFFFF));

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

void setupMPU() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    imuOk = false;
    Serial.println("[IMU] ⚠️ MPU-6050 não encontrado no endereço 0x68!");
    return;
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);

  imuOk = true;
  Serial.println("[IMU] ✅ MPU-6050 inicializado com sucesso (8g, 500dps)!");
}

bool tryInitSD(int sck, int miso, int mosi, int cs, uint32_t freq) {
  SPI.end();
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  pinMode(miso, INPUT_PULLUP);
  delay(50);

  SPI.begin(sck, miso, mosi, cs);
  delay(50);

  return SD.begin(cs, SPI, freq);
}

void setupSD() {
  Serial.printf("[SD] 💾 Inicializando MicroSD (SCK=GPIO %d, MISO=GPIO %d, MOSI=GPIO %d, CS=GPIO %d)...\n",
    SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (tryInitSD(SD_SCK, SD_MISO, SD_MOSI, SD_CS, 400000)) {
    Serial.println("[SD] ✅ MicroSD montado com sucesso a 400kHz!");
    sdOk = true;
  } else {
    Serial.println("[SD] ❌ Falha ao montar o cartão MicroSD!");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[SD] ❌ Nenhum cartão MicroSD detectado no slot!");
    sdOk = false;
    return;
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("[SD] ✅ Cartão reconhecido! Capacidade: %llu MB\n", cardSize);

  // Toda vez que o dispositivo ligar, apaga o arquivo da sessão anterior
  if (SD.exists(sessionFileName)) {
    SD.remove(sessionFileName);
    Serial.println("[SD] 🧹 Arquivo de sessão anterior apagado do MicroSD no boot.");
  }
  Serial.println("[SD] ⏸️ MicroSD em Standby. Aguardando comando START para iniciar gravação do workout.");
}

void startWorkoutRecording() {
  if (!sdOk) {
    Serial.println("[SD] ⚠️ Não é possível iniciar gravação: MicroSD offline.");
    return;
  }
  if (isWorkoutActive && logFile) {
    Serial.println("[SD] ⚠️ Gravação do workout já está ativa.");
    return;
  }

  if (SD.exists(sessionFileName)) {
    SD.remove(sessionFileName);
  }

  logFile = SD.open(sessionFileName, FILE_WRITE);
  if (!logFile) {
    Serial.println("[SD] ❌ Erro ao criar /remus_session.csv para gravação!");
    isWorkoutActive = false;
    return;
  }

  logFile.println("timestamp_us,ax,ay,az,gx,gy,gz,lat,lon,speed_kmh,sats");
  logFile.flush();
  sdBufLen = 0;
  linesWritten = 0;
  isWorkoutActive = true;
  Serial.println("[SD] 🔴 Gravação do Workout INICIADA! (200 Hz -> /remus_session.csv)");
}

void stopWorkoutRecording() {
  if (!isWorkoutActive) {
    Serial.println("[SD] ⚠️ Nenhuma gravação de workout ativa para parar.");
    return;
  }

  if (logFile) {
    if (sdBufLen > 0) {
      logFile.write((const uint8_t*)sdBuffer, sdBufLen);
      sdBufLen = 0;
    }
    logFile.flush();
    logFile.close();
  }
  isWorkoutActive = false;
  Serial.printf("[SD] ⏹️ Gravação do Workout FINALIZADA! Total de linhas salvas: %lu\n", linesWritten);
}

void readIMU() {
  if (!imuOk) return;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    imuOk = false;
    return;
  }

  size_t bytesRead = Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true);
  if (bytesRead != 14) {
    imuOk = false;
    return;
  }

  if (Wire.available() == 14) {
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t GyX = Wire.read() << 8 | Wire.read();
    int16_t GyY = Wire.read() << 8 | Wire.read();
    int16_t GyZ = Wire.read() << 8 | Wire.read();

    ax = AcX / 4096.0; ay = AcY / 4096.0; az = AcZ / 4096.0;
    gx = GyX / 65.5;   gy = GyY / 65.5;   gz = GyZ / 65.5;
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

void parseGsvLine(const char* line) {
  if (line[0] != '$' || strlen(line) < 10) return;
  if (strstr(line, "GSV") == NULL) return;

  const char* p = line;
  char tokens[21][16];
  int tCount = 0;
  int tLen = 0;

  while (*p && tCount < 21) {
    if (*p == ',' || *p == '*') {
      tokens[tCount][tLen] = '\0';
      tCount++;
      tLen = 0;
      if (*p == '*') break;
    } else {
      if (tLen < 15) {
        tokens[tCount][tLen++] = *p;
      }
    }
    p++;
  }
  if (tCount < 4) return;

  int totalMsgs = atoi(tokens[1]);
  int msgNum = atoi(tokens[2]);
  int totalSatsInView = atoi(tokens[3]);

  static int tempSatCount = 0;
  static int tempPrn[16];
  static int tempSnr[16];

  if (msgNum == 1) {
    tempSatCount = 0;
  }

  for (int i = 4; i + 3 < tCount && tempSatCount < 16; i += 4) {
    int prn = atoi(tokens[i]);
    int snr = atoi(tokens[i + 3]);
    if (prn > 0) {
      tempPrn[tempSatCount] = prn;
      tempSnr[tempSatCount] = snr;
      tempSatCount++;
    }
  }

  if (msgNum == totalMsgs && totalMsgs > 0) {
    int withSignal = 0;
    int maxSnr = 0;
    String satList = "";

    int strongSats = 0;
    for (int i = 0; i < tempSatCount; i++) {
      if (tempSnr[i] > 0) {
        withSignal++;
        if (tempSnr[i] >= 28) strongSats++;
        if (tempSnr[i] > maxSnr) maxSnr = tempSnr[i];
        satList += "[PRN" + String(tempPrn[i]) + ":" + String(tempSnr[i]) + "dB] ";
      }
    }
    
    liveMaxSnr = maxSnr; // Update global SNR metric

    Serial.print("[SINAL GPS] 📡 ");
    Serial.printf("Sats c/ sinal: %d/%d (Fortes >=28dB: %d) | Max: %02d dB-Hz | ", withSignal, totalSatsInView, strongSats, maxSnr);
    if (withSignal > 0) {
      Serial.print(satList);
      if (gps.location.isValid()) {
        Serial.println("-> 🟢 LOCK 3D ATIVO! (Lat/Lon OK)");
      } else if (strongSats >= 4) {
        Serial.println("-> 🟡 SINAL FORTE (4+ sats >= 28dB · aguardando efemérides...)");
      } else if (strongSats > 0) {
        Serial.printf("-> ⚠️ INSUFICIENTE (%d/4 sats >= 28dB · precisa de 4 para fix)\n", strongSats);
      } else {
        Serial.println("-> ⚠️ MUITO FRACO (todos < 28dB · coloque sob céu aberto)");
      }
    } else {
      Serial.println("❌ NENHUM SINAL RF CAPTADO (Cabo solto ou antena sem ganho)");
    }
  }
}

void setupGPS() {
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);

  if (GPS_TX_PIN >= 0) {
    Serial.println("[GPS] 📡 Configurando módulo GPS (Multi-Constelação e desativação de Deadband)...");

    // 1. Desativa Static Navigation (permite rastrear passos e velocidades lentas sem congelar coordenadas)
    sendNmeaWithChecksum("PSRF105,0"); // SiRFstar III / IV (Static Navigation OFF)
    delay(40);
    sendNmeaWithChecksum("PMTK386,0"); // MediaTek (threshold de velocidade estática = 0 m/s)
    delay(40);
    sendNmeaWithChecksum("PCAS11,0");  // CASIC (limiar estático = 0)
    delay(40);

    // 2. Configurações de Constelação e Sentenças NMEA
    sendNmeaWithChecksum("PCAS04,7");
    delay(40);
    sendNmeaWithChecksum("PCAS03,1,1,1,1,1,1,0,0");
    delay(40);
    sendNmeaWithChecksum("PMTK353,1,1,1,0,0");
    delay(40);
    sendNmeaWithChecksum("PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0");
    delay(40);

    // SiRFstar: Habilita GGA (1s), GSA (1s), GSV (5s), RMC (1s)
    sendNmeaWithChecksum("PSRF103,00,00,01,01"); // GGA 1s
    delay(40);
    sendNmeaWithChecksum("PSRF103,02,00,01,01"); // GSA 1s
    delay(40);
    sendNmeaWithChecksum("PSRF103,03,00,05,01"); // GSV 5s
    delay(40);
    sendNmeaWithChecksum("PSRF103,04,00,01,01"); // RMC 1s
    delay(40);

    while (Serial1.available()) Serial1.read();
  }

  Serial.printf("[GPS] ✅ Conectado em GPIO %d (RX) e GPIO %d (TX) a 9600 baud.\n",
    GPS_RX_PIN, GPS_TX_PIN);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=========================================================");
  Serial.println("       REMUS — SISTEMA COMPLETO DE TELEMETRIA            ");
  Serial.println("   (ESP32-C3 + MPU-6050 + GPS + MicroSD + BLE 5.0)       ");
  Serial.println("=========================================================");

  // 1. Inicializa BLE
  setupBLE();

  // 2. Inicializa GPS com Multi-Constelação
  setupGPS();

  // 3. Inicializa IMU (MPU-6050 a 100 Hz)
  setupMPU();

  // 4. Inicializa MicroSD (SPI com buffer e clock suave)
  setupSD();

  // 5. Exibe relatorio persistido em Flash NVS
  printSavedReport();
}

void loop() {
  // --- RECONEXÃO BLE ---
  if (!bleConnected && oldBleConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("[BLE] Anúncio reiniciado para novos clientes.");
    oldBleConnected = bleConnected;
  }
  if (bleConnected && !oldBleConnected) {
    oldBleConnected = bleConnected;
  }

  // --- COMANDOS DO USUÁRIO ---
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      printSavedReport();
    } else if (cmd == 'n' || cmd == 'N') {
      showRawNmea = !showRawNmea;
      Serial.printf("\n[GPS] Modo NMEA bruto %s!\n\n", showRawNmea ? "ATIVADO (mostrando sentenças do GPS)" : "DESATIVADO");
    } else if (cmd == 's' || cmd == 'S') {
      if (isWorkoutActive) {
        stopWorkoutRecording();
      } else {
        startWorkoutRecording();
      }
    }
  }



  // --- LEITURA CONTÍNUA DO GPS ---
  static char nmeaLineBuf[128];
  static size_t nmeaLinePos = 0;
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (showRawNmea) {
      Serial.write(c);
    }
    gps.encode(c);

    if (c == '\n' || c == '\r') {
      if (nmeaLinePos > 0) {
        nmeaLineBuf[nmeaLinePos] = '\0';
        parseGsvLine(nmeaLineBuf);
        nmeaLinePos = 0;
      }
    } else if (nmeaLinePos < sizeof(nmeaLineBuf) - 1) {
      nmeaLineBuf[nmeaLinePos++] = c;
    }
  }

  // Atualiza Sats in View a partir dos campos customizados NMEA das constelações
  if (gpgsvSats.isUpdated()) satsInViewGP = atoi(gpgsvSats.value());
  if (glgsvSats.isUpdated()) satsInViewGL = atoi(glgsvSats.value());
  if (bdgsvSats.isUpdated()) satsInViewBD = atoi(bdgsvSats.value());
  if (gbgsvSats.isUpdated()) satsInViewBD = atoi(gbgsvSats.value());
  if (gagsvSats.isUpdated()) satsInViewGA = atoi(gagsvSats.value());
  if (gngsvSats.isUpdated()) satsInViewGN = atoi(gngsvSats.value());

  int multiTotal = satsInViewGP + satsInViewGL + satsInViewBD + satsInViewGA;
  liveSatsInView = max(satsInViewGN, max(multiTotal, max(satsInViewGP, max(satsInViewGL, satsInViewBD))));

  // Grava fix no NVS permanente se atualizou
  if (isGpsFixValid() && gps.location.isUpdated()) {
    fixCount++;
    prefs.begin("remus_gps", false);
    prefs.putBool("had_fix", true);
    prefs.putInt("fix_count", fixCount);
    prefs.putDouble("lat", gps.location.lat());
    prefs.putDouble("lng", gps.location.lng());
    prefs.putFloat("speed", gps.speed.kmph());
    
    if (gps.time.isValid()) {
      char timeBuf[16];
      sprintf(timeBuf, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      prefs.putString("time", String(timeBuf));
    }
    if (gps.date.isValid()) {
      char dateBuf[16];
      sprintf(dateBuf, "%02d/%02d/%04d", gps.date.day(), gps.date.month(), gps.date.year());
      prefs.putString("date", String(dateBuf));
    }
    prefs.end();
  }

  // --- LOOP DE AMOSTRAGEM DO IMU (200 Hz = 5ms) ---
  unsigned long now = millis();
  if (imuOk && (now - lastImuLoop >= 5)) {
    lastImuLoop = now;
    readIMU();

    // 200 Hz: Alimenta estimador de voga em C++ nativo (Edge Computing)
    double time_s = (double)now / 1000.0;
    auto spmRes = spmEstimator.push(time_s, ax * 9.80665, ay * 9.80665, az * 9.80665);
    if (spmRes.updated && spmRes.available) {
      liveSpm = (float)spmRes.stroke_rate_spm;
    } else if (spmRes.updated && !spmRes.available && spmRes.progress >= 1.0) {
      liveSpm = 0.0;
    }

    // Grava no MicroSD somente se o treino estiver ativo (comando START recebido)
    if (sdOk && isWorkoutActive && logFile) {
      uint64_t timestamp_us = esp_timer_get_time();
      bool hasFix = isGpsFixValid();
      int satsInUse = (hasFix && gps.satellites.isValid() && gps.satellites.age() < 2500) ? gps.satellites.value() : 0;
      int satsInView = max(satsInUse, liveSatsInView);
      float accuracyMeters = (hasFix && gps.hdop.isValid() && gps.hdop.age() < 2500) ? (float)(gps.hdop.hdop() * 2.5) : 0.0;
      char lineBuf[140];
      int lineLen = 0;

      if (hasFix) {
        lineLen = snprintf(lineBuf, sizeof(lineBuf), "%llu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d/%d:%d:%.1fm\n",
          timestamp_us, ax, ay, az, gx, gy, gz,
          gps.location.lat(), gps.location.lng(), gps.speed.kmph(),
          satsInUse, satsInView, liveMaxSnr, accuracyMeters);
      } else {
        lineLen = snprintf(lineBuf, sizeof(lineBuf), "%llu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,,,,%d/%d:%d:0.0m\n",
          timestamp_us, ax, ay, az, gx, gy, gz,
          satsInUse, satsInView, liveMaxSnr);
      }

      if (lineLen > 0 && lineLen < (int)sizeof(lineBuf)) {
        if (sdBufLen + lineLen >= sizeof(sdBuffer)) {
          // Descarrega o bloco acumulado em uma única rajada SPI rápida (deixando o barramento silencioso 95% do tempo)
          logFile.write((const uint8_t*)sdBuffer, sdBufLen);
          sdBufLen = 0;
        }
        memcpy(sdBuffer + sdBufLen, lineBuf, lineLen);
        sdBufLen += lineLen;
        linesWritten++;
      }
    }
  }

  if (sdOk && isWorkoutActive && logFile && (now - lastFlush >= 2000)) {
    lastFlush = now;
    if (sdBufLen > 0) {
      logFile.write((const uint8_t*)sdBuffer, sdBufLen);
      sdBufLen = 0;
    }
    logFile.flush();
  }

  // --- PAINEL SERIAL E BROADCAST BLE (A CADA 1 SEGUNDO) ---
  if (now - lastSerialPrint >= 1000) {
    lastSerialPrint = now;
    uint64_t now_us = esp_timer_get_time();
    bool hasFix = isGpsFixValid();
    int satsInUse = (hasFix && gps.satellites.isValid() && gps.satellites.age() < 2500) ? gps.satellites.value() : 0;
    int satsInView = max(satsInUse, liveSatsInView);
    float accuracyMeters = (hasFix && gps.hdop.isValid() && gps.hdop.age() < 2500) ? (float)(gps.hdop.hdop() * 2.5) : 0.0;

    // 1. Transmissão BLE (notifica o app com SPM computado na borda e precisão em metros)
    if (bleConnected && pCharacteristic) {
      char bleBuf[160];
      char satsStr[32];
      snprintf(satsStr, sizeof(satsStr), "%d/%d:%d:%.1fm", satsInUse, satsInView, liveMaxSnr, accuracyMeters);

      unsigned long charsRx = gps.charsProcessed();
      if (hasFix) {
        snprintf(bleBuf, sizeof(bleBuf), "%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%s,%lu,%lu,%.1f",
          (unsigned long)(now_us / 1000), ax, ay, az, gx, gy, gz,
          gps.location.lat(), gps.location.lng(), gps.speed.kmph(),
          satsStr, linesWritten, charsRx, liveSpm);
      } else {
        snprintf(bleBuf, sizeof(bleBuf), "%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,,,,%s,%lu,%lu,%.1f",
          (unsigned long)(now_us / 1000), ax, ay, az, gx, gy, gz,
          satsStr, linesWritten, charsRx, liveSpm);
      }
      pCharacteristic->setValue(bleBuf);
      pCharacteristic->notify();
    }

    // 2. Painel Serial
    Serial.print("[REMUS] ");
    if (imuOk) {
      Serial.printf("IMU: [AX:%.2f AY:%.2f AZ:%.2f] ", ax, ay, az);
    } else {
      Serial.print("IMU: [Off] ");
    }

    Serial.printf("| GPS: [%d/%d Sats, SNR:%d, Acc:%.1fm, Fix:%s] | SPM: %.1f (Chars RX:%lu) ", 
      satsInUse,
      satsInView, 
      liveMaxSnr,
      accuracyMeters,
      hasFix ? "LOCK" : "Buscando",
      liveSpm,
      gps.charsProcessed());

    if (sdOk) {
      if (isWorkoutActive) {
        Serial.printf("| SD: [Gravando: %lu linhas] ", linesWritten);
      } else if (linesWritten > 0) {
        Serial.printf("| SD: [Finalizado (%lu linhas)] ", linesWritten);
      } else {
        Serial.print("| SD: [Standby] ");
      }
    } else {
      Serial.print("| SD: [Off] ");
    }

    Serial.printf("| BLE: [%s] ", bleConnected ? "CONECTADO" : "ANUNCIANDO");
    Serial.println();
  }
}
