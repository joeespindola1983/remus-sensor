#pragma once

#include <Arduino_GFX_Library.h>
#include <cstdint>

namespace remus::drivers {

struct DisplayTelemetry {
  bool recording = false;
  bool transferring = false;
  bool bleConnected = false;
  bool imuHealthy = false;
  bool sdHealthy = false;
  bool gpsFix = false;
  bool gpsAccuracyEstimateAvailable = false;
  float strokeRateSpm = 0.0f;
  float medianStrokeRateSpm = 0.0f;
  uint16_t paceSecondsPer500m = 0;
  uint16_t medianPaceSecondsPer500m = 0;
  float gpsAccuracyEstimateMeters = 0.0f;
  uint8_t satellitesInUse = 0;
  uint8_t satellitesInView = 0;
  unsigned long recordsWritten = 0;
  bool bladeAlignmentAvailable = false;
  float relativeEquipmentAlignmentDegrees = 0.0f;
  uint8_t orientationQuality = 0;
};

class Gmt024Display {
public:
  Gmt024Display(int sck, int mosi, int dc, int rst, int cs);

  bool begin();
  void render(const DisplayTelemetry& telemetry, bool force = false);
  bool healthy() const { return healthy_; }
  const char* name() const { return "GMT024-10 V2.1 / ST7789"; }

private:
  void runBootSelfTest();
  void drawStaticLayout();
  void drawField(int16_t x, int16_t y, int16_t width, int16_t height,
                 const char* value, uint16_t color, uint8_t textSize,
                 char* cache, size_t cacheSize, bool force);
  void drawCenteredField(int16_t y, int16_t height, const char* value,
                         uint16_t color, uint8_t textSize, char* cache,
                         size_t cacheSize, bool force);
  void drawIndicatorField(int16_t x, int16_t y, int16_t width, int16_t height,
                          const char* value, uint16_t indicatorColor,
                          uint8_t textSize, char* cache, size_t cacheSize,
                          bool force);
  void drawBladeAlignment(const DisplayTelemetry& telemetry, bool force);

  Arduino_SWSPI bus_;
  Arduino_ST7789 tft_;
  int sck_;
  int mosi_;
  int dc_;
  int rst_;
  int cs_;
  bool healthy_ = false;
  char spmCache_[16]{};
  char paceCache_[16]{};
  char healthCache_[64]{};
  bool alignmentMode_ = false;
  float alignmentDegreesCache_ = 1000.0f;
};

}  // namespace remus::drivers
