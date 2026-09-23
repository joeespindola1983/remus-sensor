#include "remus/drivers/Gmt024Display.hpp"

#include <Arduino.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace remus::drivers {

namespace {
constexpr uint16_t kBackground = 0xFFFF;
constexpr uint16_t kText = 0x0000;
constexpr uint16_t kMuted = 0x4208;
constexpr uint16_t kDivider = 0xC618;
constexpr uint16_t kRed = 0xF800;
constexpr uint16_t kGreen = 0x0400;
constexpr uint16_t kBlue = 0x001F;
constexpr uint16_t kYellow = 0xFFE0;
constexpr uint16_t kLightGreen = 0x87F0;
constexpr uint16_t kOk = kGreen;
constexpr uint16_t kWarning = kYellow;
constexpr uint16_t kError = kRed;

void formatSpm(char* target, size_t size, float spm) {
  if (spm <= 0.0f) {
    snprintf(target, size, "--");
    return;
  }
  const int doubled = static_cast<int>(roundf(spm * 2.0f));
  if ((doubled & 1) == 0) snprintf(target, size, "%d", doubled / 2);
  else snprintf(target, size, "%d.5", doubled / 2);
}

void formatPace(char* target, size_t size, uint16_t seconds) {
  if (seconds == 0) {
    snprintf(target, size, "--:--");
    return;
  }
  snprintf(target, size, "%u:%02u", seconds / 60, seconds % 60);
}
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
  tft_.fillScreen(kBackground);
  tft_.setTextColor(kText);
  tft_.setTextSize(2);
  tft_.setCursor(10, 7);
  tft_.print("REMUS");

  tft_.drawFastHLine(0, 31, 320, kText);
  tft_.setTextColor(kMuted);
  tft_.setTextSize(1);
  tft_.setCursor(10, 40);
  tft_.print("SPM ATUAL");
  tft_.setCursor(170, 40);
  tft_.print("PACE /500m");
  tft_.setCursor(10, 115);
  tft_.print("SPM MEDIANO");
  tft_.setCursor(170, 115);
  tft_.print("PACE MEDIANO");
  tft_.drawFastVLine(160, 36, 148, kDivider);
  tft_.drawFastHLine(8, 108, 304, kDivider);
  tft_.drawFastHLine(8, 187, 304, kDivider);
}

void Gmt024Display::drawIndicatorField(int16_t x, int16_t y, int16_t width,
                                       int16_t height, const char* value,
                                       uint16_t indicatorColor, uint8_t textSize,
                                       char* cache, size_t cacheSize, bool force) {
  if (!force && strncmp(cache, value, cacheSize) == 0) return;

  tft_.fillRect(x, y, width, height, kBackground);
  const int16_t radius = textSize > 1 ? 5 : 3;
  tft_.fillCircle(x + radius, y + height / 2, radius, indicatorColor);
  tft_.setTextColor(kText);
  tft_.setTextSize(textSize);
  tft_.setCursor(x + radius * 2 + 5, y);
  tft_.print(value);
  snprintf(cache, cacheSize, "%s", value);
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
  drawIndicatorField(166, 7, 144, 18, status, statusColor, 2,
                     statusCache_, sizeof(statusCache_), force);

  char value[48];
  formatSpm(value, sizeof(value), telemetry.strokeRateSpm);
  drawField(10, 56, 142, 46, value, kText, 5,
            spmCache_, sizeof(spmCache_), force);

  formatPace(value, sizeof(value), telemetry.paceSecondsPer500m);
  drawField(170, 56, 142, 46, value, kText, 4,
            paceCache_, sizeof(paceCache_), force);

  formatSpm(value, sizeof(value), telemetry.medianStrokeRateSpm);
  drawField(10, 133, 142, 44, value, kText, 4,
            medianSpmCache_, sizeof(medianSpmCache_), force);

  formatPace(value, sizeof(value), telemetry.medianPaceSecondsPer500m);
  drawField(170, 133, 142, 44, value, kText, 4,
            medianPaceCache_, sizeof(medianPaceCache_), force);

  uint16_t gpsColor = kError;
  if (telemetry.gpsFix) {
    gpsColor = kLightGreen;
    if (telemetry.gpsAccuracyEstimateAvailable) {
      snprintf(value, sizeof(value), "GPS: %u SAT / %.1fm",
               telemetry.satellitesInUse, telemetry.gpsAccuracyEstimateMeters);
    } else {
      snprintf(value, sizeof(value), "GPS: %u SAT / --m",
               telemetry.satellitesInUse);
    }
  } else if (telemetry.satellitesInView > 0) {
    gpsColor = kWarning;
    snprintf(value, sizeof(value), "GPS: %u SAT / BUSCA", telemetry.satellitesInView);
  } else {
    snprintf(value, sizeof(value), "GPS: 0 SAT / SEM SINAL");
  }
  drawIndicatorField(12, 195, 300, 15, value, gpsColor, 1,
                     gpsCache_, sizeof(gpsCache_), force);

  snprintf(value, sizeof(value), "IMU:%s  SD:%s  BLE:%s  REG:%lu",
           telemetry.imuHealthy ? "OK" : "OFF",
           telemetry.sdHealthy ? "OK" : "OFF",
           telemetry.bleConnected ? "ON" : "OFF",
           telemetry.recordsWritten);
  drawField(12, 219, 300, 12, value, kText, 1,
            healthCache_, sizeof(healthCache_), force);
}

}  // namespace remus::drivers
