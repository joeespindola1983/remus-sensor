#pragma once

#include <cstdint>

namespace remus::hal {

class IGps {
public:
  virtual ~IGps() = default;
  virtual bool begin() = 0;
  virtual void poll(bool showRawNmea, bool workoutActive) = 0;
  virtual bool healthy() const = 0;
  virtual bool fixValid() const = 0;
  virtual bool locationUpdated() const = 0;
  virtual double latitude() const = 0;
  virtual double longitude() const = 0;
  virtual float speedKmph() const = 0;
  // Enhanced navigation fields. Drivers without native receiver estimates keep
  // the safe defaults so Prototype 2 remains source-compatible.
  virtual bool enhancedNavigationAvailable() const { return false; }
  virtual bool navigationSolutionUpdated() const { return locationUpdated(); }
  virtual uint32_t gpsTimeOfWeekMs() const { return 0; }
  virtual uint32_t groundSpeedCmPerSecond() const {
    return static_cast<uint32_t>(speedKmph() * 27.7777778f);
  }
  virtual uint32_t speedAccuracyCmPerSecond() const { return 0; }
  virtual int32_t courseDegreesE5() const { return 0; }
  virtual uint32_t courseAccuracyDegreesE5() const { return 0; }
  virtual uint32_t horizontalAccuracyMm() const { return 0; }
  virtual uint8_t fixType() const { return fixValid() ? 3 : 0; }
  virtual uint8_t satellitesInUse() const = 0;
  virtual int satellitesInView() const = 0;
  virtual int maxSnr() const = 0;
  virtual bool hdopValidRecent() const = 0;
  virtual float hdop() const = 0;
  virtual uint64_t charsProcessed() const = 0;
  virtual bool timeValid() const = 0;
  virtual uint8_t hour() const = 0;
  virtual uint8_t minute() const = 0;
  virtual uint8_t second() const = 0;
  virtual bool dateValid() const = 0;
  virtual uint8_t day() const = 0;
  virtual uint8_t month() const = 0;
  virtual uint16_t year() const = 0;
  virtual void aidPosition(float lat, float lon) = 0;
  virtual const char* name() const = 0;
};

}  // namespace remus::hal
