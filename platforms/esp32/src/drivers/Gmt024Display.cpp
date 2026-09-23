#include "remus/drivers/Gmt024Display.hpp"

#include <Arduino.h>
#include <cstdio>
#include <cstring>

namespace remus::drivers {

namespace {
constexpr uint16_t kBackground = 0x0000;
constexpr uint16_t kMuted = 0x7BEF;
constexpr uint16_t kAccent = 0x05FF;
constexpr uint16_t kWhite = 0xFFFF;
constexpr uint16_t kRed = 0xF800;
constexpr uint16_t kGreen = 0x07E0;
constexpr uint16_t kBlue = 0x001F;
constexpr uint16_t kYellow = 0xFFE0;
constexpr uint16_t kLightGreen = 0x87F0;
constexpr uint16_t kOk = kGreen;
constexpr uint16_t kWarning = kYellow;
constexpr uint16_t kError = kRed;
}  // namespace

Gmt024Display::Gmt024Display(int sck, int mosi, int dc, int rst, int cs)
    : bus_(dc, cs, sck, mosi, GFX_NOT_DEFINED),
      tft_(&bus_, rst, 1, false, 240, 320),
      sck_(sck), mosi_(mosi), dc_(dc), rst_(rst), cs_(cs) {}

bool Gmt024Display::begin() {
  healthy_ = false;

  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  // Arduino_GFX's ST7789 driver selects the ESP32-compatible MODE3 sequence.
  // Software SPI keeps the display electrically independent from the SD bus.
  if (!tft_.begin()) {
    Serial.println("[TFT] Falha local ao iniciar o driver Arduino_GFX.");
    return false;
  }
  tft_.setRotation(1);
  runBootSelfTest();
  drawStaticLayout();
  healthy_ = true;

  Serial.printf("[TFT] Sequencia Arduino_GFX/MODE3 enviada para %s via software SPI: SCK=%d MOSI=%d DC=%d RST=%d CS=%d\n",
                name(), sck_, mosi_, dc_, rst_, cs_);
  Serial.println("[TFT] Autoteste visual esperado: faixas VERMELHA, VERDE e AZUL antes do painel.");
  return true;
}

void Gmt024Display::runBootSelfTest() {
  tft_.fillScreen(kBackground);
  tft_.fillRect(0, 0, 107, 240, kRed);
  tft_.fillRect(107, 0, 106, 240, kGreen);
  tft_.fillRect(213, 0, 107, 240, kBlue);
  delay(900);
  tft_.fillScreen(kBackground);
}

void Gmt024Display::drawStaticLayout() {
  tft_.fillRect(0, 0, 320, 30, 0x0210);
  tft_.setTextColor(kWhite);
  tft_.setTextSize(2);
  tft_.setCursor(10, 7);
  tft_.print("REMUS");

  tft_.drawFastHLine(0, 31, 320, kAccent);
  tft_.setTextColor(kMuted);
  tft_.setTextSize(1);
  tft_.setCursor(12, 43);
  tft_.print("CADENCIA");
  tft_.setCursor(174, 43);
  tft_.print("VELOCIDADE");
  tft_.setCursor(12, 158);
  tft_.print("SENSORES");
  tft_.drawFastVLine(160, 40, 108, 0x2104);
  tft_.drawFastHLine(10, 150, 300, 0x2104);
}

void Gmt024Display::drawField(int16_t x, int16_t y, int16_t width, int16_t height,
                              const char* value, uint16_t color, uint8_t textSize,
                              char* cache, size_t cacheSize, bool force) {
  if (!force && strncmp(cache, value, cacheSize) == 0) return;

  tft_.fillRect(x, y, width, height, kBackground);
  tft_.setTextColor(color);
  tft_.setTextSize(textSize);
  tft_.setCursor(x, y);
  tft_.print(value);
  snprintf(cache, cacheSize, "%s", value);
}

void Gmt024Display::render(const DisplayTelemetry& telemetry, bool force) {
  if (!healthy_) return;

  const char* status = telemetry.transferring ? "TRANSFERINDO" :
                       telemetry.recording ? "GRAVANDO" : "PRONTO";
  const uint16_t statusColor = telemetry.transferring ? kWarning :
                               telemetry.recording ? kError : kOk;
  drawField(166, 7, 144, 18, status, statusColor, 2,
            statusCache_, sizeof(statusCache_), force);

  char value[40];
  if (telemetry.strokeRateSpm > 0.0f) {
    snprintf(value, sizeof(value), "%.1f", telemetry.strokeRateSpm);
  } else {
    snprintf(value, sizeof(value), "--.-");
  }
  drawField(10, 62, 142, 70, value, kWhite, 6,
            spmCache_, sizeof(spmCache_), force);
  tft_.setTextColor(kMuted);
  tft_.setTextSize(2);
  tft_.setCursor(112, 122);
  tft_.print("SPM");

  if (telemetry.gpsFix) {
    snprintf(value, sizeof(value), "%.1f", telemetry.groundSpeedKmph);
  } else {
    snprintf(value, sizeof(value), "--.-");
  }
  drawField(170, 62, 142, 70, value, kAccent, 5,
            speedCache_, sizeof(speedCache_), force);
  tft_.setTextColor(kMuted);
  tft_.setTextSize(1);
  tft_.setCursor(274, 126);
  tft_.print("km/h");

  uint16_t gpsColor = kError;
  if (telemetry.gpsFix) {
    gpsColor = kLightGreen;
    if (telemetry.gpsAccuracyEstimateAvailable) {
      snprintf(value, sizeof(value), "GPS:FIX SAT:%u ~%.1fm",
               telemetry.satellitesInUse, telemetry.gpsAccuracyEstimateMeters);
    } else {
      snprintf(value, sizeof(value), "GPS:FIX SAT:%u --m",
               telemetry.satellitesInUse);
    }
  } else if (telemetry.satellitesInView > 0) {
    gpsColor = kWarning;
    snprintf(value, sizeof(value), "GPS:BUSCA SAT:%u", telemetry.satellitesInView);
  } else {
    snprintf(value, sizeof(value), "GPS:SEM SINAL");
  }
  drawField(12, 174, 300, 18, value, gpsColor, 2,
            gpsCache_, sizeof(gpsCache_), force);

  snprintf(value, sizeof(value), "IMU:%s  SD:%s  BLE:%s  REG:%lu",
           telemetry.imuHealthy ? "OK" : "OFF",
           telemetry.sdHealthy ? "OK" : "OFF",
           telemetry.bleConnected ? "ON" : "OFF",
           telemetry.recordsWritten);
  const uint16_t healthColor = telemetry.imuHealthy && telemetry.sdHealthy ? kOk : kError;
  drawField(12, 207, 300, 20, value, healthColor, 1,
            healthCache_, sizeof(healthCache_), force);
}

}  // namespace remus::drivers
