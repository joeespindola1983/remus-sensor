#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "remus/hal/IGps.hpp"

namespace remus::drivers {

class Neo6mGps final : public hal::IGps {
public:
  Neo6mGps(HardwareSerial& serial, int rxPin, int txPin, uint32_t baud = 9600);

  bool begin() override;
  void poll(bool showRawNmea, bool workoutActive) override;
  bool healthy() const override { return healthy_; }
  bool fixValid() const override;
  bool locationUpdated() const override;
  double latitude() const override;
  double longitude() const override;
  float speedKmph() const override;
  bool enhancedNavigationAvailable() const override { return ubxNavigationActive_; }
  bool navigationSolutionUpdated() const override;
  uint32_t gpsTimeOfWeekMs() const override { return navigation_.iTowMs; }
  uint32_t groundSpeedCmPerSecond() const override {
    return ubxNavigationActive_ ? navigation_.groundSpeedCmS
                                : static_cast<uint32_t>(gps_.speed.kmph() * 27.7777778f);
  }
  uint32_t speedAccuracyCmPerSecond() const override {
    return ubxNavigationActive_ ? navigation_.speedAccuracyCmS : 0;
  }
  int32_t courseDegreesE5() const override {
    return ubxNavigationActive_ ? navigation_.courseDegE5 : 0;
  }
  uint32_t courseAccuracyDegreesE5() const override {
    return ubxNavigationActive_ ? navigation_.courseAccuracyDegE5 : 0;
  }
  uint32_t horizontalAccuracyMm() const override {
    return ubxNavigationActive_ ? navigation_.horizontalAccuracyMm : 0;
  }
  uint8_t fixType() const override {
    return ubxNavigationActive_ ? navigation_.fixType : (fixValid() ? 3 : 0);
  }
  uint8_t satellitesInUse() const override;
  int satellitesInView() const override { return liveSatsInView_; }
  int maxSnr() const override { return liveMaxSnr_; }
  bool hdopValidRecent() const override;
  float hdop() const override { return gps_.hdop.hdop(); }
  uint64_t charsProcessed() const override { return charsProcessed_; }
  bool timeValid() const override { return gps_.time.isValid(); }
  uint8_t hour() const override { return gps_.time.hour(); }
  uint8_t minute() const override { return gps_.time.minute(); }
  uint8_t second() const override { return gps_.time.second(); }
  bool dateValid() const override { return gps_.date.isValid(); }
  uint8_t day() const override { return gps_.date.day(); }
  uint8_t month() const override { return gps_.date.month(); }
  uint16_t year() const override { return gps_.date.year(); }
  void aidPosition(float lat, float lon) override;
  const char* name() const override { return "NEO-6M V2"; }

private:
  void sendUbx(uint8_t messageClass, uint8_t messageId,
               const uint8_t* payload, uint16_t payloadLength);
  bool waitForUbxAck(uint8_t messageClass, uint8_t messageId,
                     uint32_t timeoutMs = 350);
  bool configureUbxMessage(uint8_t messageClass, uint8_t messageId,
                           uint8_t rate);
  bool configureUbxRate(uint16_t measurementRateMs);
  void parseGsvLine(const char* line, bool showRawNmea, bool workoutActive);
  void updateSatellitesInView();
  bool parseUbxByte(uint8_t value);
  void processUbxMessage();
  void refreshCoherentNavigationSolution();

  struct NavigationState {
    uint32_t iTowMs = 0;
    int32_t latE7 = 0;
    int32_t lonE7 = 0;
    uint32_t groundSpeedCmS = 0;
    uint32_t speedAccuracyCmS = 0;
    int32_t courseDegE5 = 0;
    uint32_t courseAccuracyDegE5 = 0;
    uint32_t horizontalAccuracyMm = 0;
    uint8_t fixType = 0;
    bool fixOk = false;
    uint32_t receivedAtMs = 0;
  };

  HardwareSerial& serial_;
  int rxPin_;
  int txPin_;
  uint32_t baud_;
  bool healthy_ = false;
  bool ubxNavigationActive_ = false;
  uint64_t charsProcessed_ = 0;
  NavigationState navigation_{};
  uint32_t positionITowMs_ = UINT32_MAX;
  uint32_t velocityITowMs_ = UINT32_MAX;
  uint32_t statusITowMs_ = UINT32_MAX;
  uint32_t publishedITowMs_ = UINT32_MAX;
  uint32_t navigationGeneration_ = 0;
  mutable uint32_t consumedNavigationGeneration_ = 0;

  enum class UbxState : uint8_t { Sync1, Sync2, Class, Id, Length1, Length2, Payload, CkA, CkB };
  UbxState ubxState_ = UbxState::Sync1;
  uint8_t ubxClass_ = 0;
  uint8_t ubxId_ = 0;
  uint16_t ubxLength_ = 0;
  uint16_t ubxIndex_ = 0;
  uint8_t ubxPayload_[64]{};
  uint8_t ubxCkA_ = 0;
  uint8_t ubxCkB_ = 0;
  uint8_t ubxReceivedCkA_ = 0;

  mutable TinyGPSPlus gps_;
  TinyGPSCustom gpgsvSats_;
  TinyGPSCustom gngsvSats_;
  TinyGPSCustom glgsvSats_;
  TinyGPSCustom bdgsvSats_;
  TinyGPSCustom gbgsvSats_;
  TinyGPSCustom gagsvSats_;
  TinyGPSCustom gpggaFixQuality_;
  TinyGPSCustom gnggaFixQuality_;
  TinyGPSCustom gpgsaFixDimension_;
  TinyGPSCustom gngsaFixDimension_;

  int satsInViewGP_ = 0;
  int satsInViewGL_ = 0;
  int satsInViewBD_ = 0;
  int satsInViewGA_ = 0;
  int satsInViewGN_ = 0;
  int liveSatsInView_ = 0;
  int liveMaxSnr_ = 0;
  int ggaFixQuality_ = 0;
  int gsaFixDimension_ = 1;
  bool hasGgaFixQuality_ = false;
  bool hasGsaFixDimension_ = false;
  uint32_t lastFixStatusMs_ = 0;

  char gsvTalker_[3]{};
  int gsvTempSatCount_ = 0;
  int gsvTempPrn_[16]{};
  int gsvTempSnr_[16]{};

  char nmeaLineBuf_[128]{};
  size_t nmeaLinePos_ = 0;
};

}  // namespace remus::drivers
