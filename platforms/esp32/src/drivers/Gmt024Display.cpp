#include "remus/drivers/Gmt024Display.hpp"

#include <Arduino.h>
#include <algorithm>
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
  tft_.setTextColor(kMuted);
  tft_.setTextSize(2);
  tft_.setCursor(8, 5);
  tft_.print("SPM ATUAL");
  tft_.setCursor(8, 112);
  tft_.print("PACE /500m");
  tft_.drawFastHLine(0, 104, 320, kDivider);
  tft_.drawFastHLine(0, 215, 320, kText);
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

void Gmt024Display::drawCenteredField(int16_t y, int16_t height,
                                      const char* value, uint16_t color,
                                      uint8_t textSize, char* cache,
                                      size_t cacheSize, bool force) {
  if (!force && strncmp(cache, value, cacheSize) == 0) return;

  tft_.fillRect(0, y, 320, height, kBackground);
  const int16_t characterWidth = 6 * textSize;
  const int16_t textWidth = static_cast<int16_t>(strlen(value)) * characterWidth;
  const int16_t x = std::max<int16_t>(4, (320 - textWidth) / 2);
  tft_.setTextColor(color);
  tft_.setTextSize(textSize);
  tft_.setCursor(x, y);
  tft_.print(value);
  snprintf(cache, cacheSize, "%s", value);
}

void Gmt024Display::render(const DisplayTelemetry& telemetry, bool force) {
  if (!healthy_) return;

  if (telemetry.bladeAlignmentAvailable) {
    drawBladeAlignment(telemetry, force);
    return;
  }
  if (alignmentMode_) {
    alignmentMode_ = false;
    alignmentDegreesCache_ = 1000.0f;
    drawStaticLayout();
    force = true;
  }

  const char* status = telemetry.transferring ? "XFR" :
                       telemetry.recording ? "GRAV" : "PRONTO";
  const uint16_t statusColor = telemetry.transferring ? kWarning :
                               telemetry.recording ? kError : kOk;
  char value[48];
  formatSpm(value, sizeof(value), telemetry.strokeRateSpm);
  drawCenteredField(28, 72, value, kText, 9,
                    spmCache_, sizeof(spmCache_), force);

  formatPace(value, sizeof(value), telemetry.paceSecondsPer500m);
  drawCenteredField(136, 68, value, kText, 8,
                    paceCache_, sizeof(paceCache_), force);

  const char* gpsState = "--";
  char gpsValue[8];
  if (telemetry.gpsFix) {
    snprintf(gpsValue, sizeof(gpsValue), "%u", telemetry.satellitesInUse);
    gpsState = gpsValue;
  } else if (telemetry.satellitesInView > 0) {
    snprintf(gpsValue, sizeof(gpsValue), "~%u", telemetry.satellitesInView);
    gpsState = gpsValue;
  }

  snprintf(value, sizeof(value), "%s IMU:%s SD:%s BLE:%s GPS:%s R:%lu",
           status,
           telemetry.imuHealthy ? "OK" : "OFF",
           telemetry.sdHealthy ? "OK" : "OFF",
           telemetry.bleConnected ? "ON" : "OFF",
           gpsState,
           telemetry.recordsWritten);
  drawIndicatorField(4, 220, 312, 12, value, statusColor, 1,
                     healthCache_, sizeof(healthCache_), force);
}

void Gmt024Display::drawBladeAlignment(const DisplayTelemetry& telemetry, bool force) {
  const float delta = telemetry.relativeEquipmentAlignmentDegrees;
  if (!force && alignmentMode_ && std::abs(delta - alignmentDegreesCache_) < 0.5f) return;
  alignmentMode_ = true;
  alignmentDegreesCache_ = delta;
  tft_.fillScreen(kBackground);

  tft_.setTextColor(kMuted);
  tft_.setTextSize(2);
  tft_.setCursor(45, 8);
  tft_.print("ALINHAMENTO DAS PAS");

  const float halfRadians = delta * 0.5f * 3.14159265358979323846f / 180.0f;
  constexpr int16_t centerX = 160;
  constexpr int16_t centerY = 140;
  constexpr int16_t bladeLength = 112;
  const int16_t horizontal = static_cast<int16_t>(std::cos(halfRadians) * bladeLength);
  const int16_t vertical = static_cast<int16_t>(std::sin(halfRadians) * bladeLength);
  tft_.drawLine(centerX, centerY, centerX - horizontal, centerY - vertical, kBlue);
  tft_.drawLine(centerX, centerY + 1, centerX - horizontal, centerY - vertical + 1, kBlue);
  tft_.drawLine(centerX, centerY, centerX + horizontal, centerY - vertical, kGreen);
  tft_.drawLine(centerX, centerY + 1, centerX + horizontal, centerY - vertical + 1, kGreen);
  tft_.fillCircle(centerX, centerY, 5, kText);

  char value[32];
  snprintf(value, sizeof(value), "DIF %.1f graus", delta);
  tft_.setTextColor(telemetry.orientationQuality >= 3 ? kOk : kWarning);
  tft_.setTextSize(2);
  const int16_t textWidth = static_cast<int16_t>(strlen(value)) * 12;
  tft_.setCursor(std::max<int16_t>(4, (320 - textWidth) / 2), 195);
  tft_.print(value);
  tft_.setTextSize(1);
  tft_.setCursor(6, 226);
  tft_.print(telemetry.orientationQuality >= 3
      ? "RELATIVO CALIBRADO"
      : "QUALIDADE DEGRADADA");
}

}  // namespace remus::drivers
