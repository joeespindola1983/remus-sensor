#include "remus/drivers/Neo6mGps.hpp"

#include <Arduino.h>
#include <cstring>
#include <cstdlib>

namespace remus::drivers {

namespace {
uint32_t readU32(const uint8_t* bytes) {
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) |
         (static_cast<uint32_t>(bytes[3]) << 24);
}

int32_t readI32(const uint8_t* bytes) {
  return static_cast<int32_t>(readU32(bytes));
}
}  // namespace

Neo6mGps::Neo6mGps(HardwareSerial& serial, int rxPin, int txPin, uint32_t baud)
    : serial_(serial), rxPin_(rxPin), txPin_(txPin), baud_(baud),
      gpgsvSats_(gps_, "GPGSV", 3), gngsvSats_(gps_, "GNGSV", 3),
      glgsvSats_(gps_, "GLGSV", 3), bdgsvSats_(gps_, "BDGSV", 3),
      gbgsvSats_(gps_, "GBGSV", 3), gagsvSats_(gps_, "GAGSV", 3),
      gpggaFixQuality_(gps_, "GPGGA", 6), gnggaFixQuality_(gps_, "GNGGA", 6),
      gpgsaFixDimension_(gps_, "GPGSA", 2), gngsaFixDimension_(gps_, "GNGSA", 2) {}

void Neo6mGps::sendUbx(uint8_t messageClass, uint8_t messageId,
                       const uint8_t* payload, uint16_t payloadLength) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  auto writeChecksummed = [&](uint8_t value) {
    serial_.write(value);
    ckA = static_cast<uint8_t>(ckA + value);
    ckB = static_cast<uint8_t>(ckB + ckA);
  };

  serial_.write(0xB5);
  serial_.write(0x62);
  writeChecksummed(messageClass);
  writeChecksummed(messageId);
  writeChecksummed(static_cast<uint8_t>(payloadLength & 0xFF));
  writeChecksummed(static_cast<uint8_t>(payloadLength >> 8));
  for (uint16_t i = 0; i < payloadLength; ++i) writeChecksummed(payload[i]);
  serial_.write(ckA);
  serial_.write(ckB);
  serial_.flush();
}

bool Neo6mGps::waitForUbxAck(uint8_t messageClass, uint8_t messageId,
                             uint32_t timeoutMs) {
  enum class State : uint8_t { Sync1, Sync2, Class, Id, Length1, Length2, Payload, CkA, CkB };
  State state = State::Sync1;
  uint8_t responseClass = 0;
  uint8_t responseId = 0;
  uint16_t length = 0;
  uint16_t payloadIndex = 0;
  uint8_t payload[16]{};
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  uint8_t receivedCkA = 0;
  const uint32_t startedAt = millis();

  while (millis() - startedAt < timeoutMs) {
    if (!serial_.available()) {
      delay(1);
      continue;
    }
    const uint8_t value = static_cast<uint8_t>(serial_.read());
    auto checksum = [&](uint8_t byte) {
      ckA = static_cast<uint8_t>(ckA + byte);
      ckB = static_cast<uint8_t>(ckB + ckA);
    };
    switch (state) {
      case State::Sync1:
        if (value == 0xB5) state = State::Sync2;
        break;
      case State::Sync2:
        state = value == 0x62 ? State::Class : State::Sync1;
        ckA = ckB = 0;
        break;
      case State::Class:
        responseClass = value;
        checksum(value);
        state = State::Id;
        break;
      case State::Id:
        responseId = value;
        checksum(value);
        state = State::Length1;
        break;
      case State::Length1:
        length = value;
        checksum(value);
        state = State::Length2;
        break;
      case State::Length2:
        length |= static_cast<uint16_t>(value) << 8;
        checksum(value);
        payloadIndex = 0;
        state = length == 0 ? State::CkA : State::Payload;
        break;
      case State::Payload:
        if (payloadIndex < sizeof(payload)) payload[payloadIndex] = value;
        ++payloadIndex;
        checksum(value);
        if (payloadIndex >= length) state = State::CkA;
        break;
      case State::CkA:
        receivedCkA = value;
        state = State::CkB;
        break;
      case State::CkB: {
        const bool checksumValid = receivedCkA == ckA && value == ckB;
        const bool isAck = responseClass == 0x05 && responseId == 0x01;
        const bool isNak = responseClass == 0x05 && responseId == 0x00;
        if (checksumValid && (isAck || isNak) && length >= 2 &&
            payload[0] == messageClass && payload[1] == messageId) {
          return isAck;
        }
        state = State::Sync1;
        break;
      }
    }
  }
  return false;
}

bool Neo6mGps::configureUbxMessage(uint8_t messageClass, uint8_t messageId,
                                  uint8_t rate) {
  const uint8_t payload[] = {messageClass, messageId, rate};
  sendUbx(0x06, 0x01, payload, sizeof(payload));
  return waitForUbxAck(0x06, 0x01);
}

bool Neo6mGps::configureUbxRate(uint16_t measurementRateMs) {
  const uint8_t payload[] = {
    static_cast<uint8_t>(measurementRateMs & 0xFF),
    static_cast<uint8_t>(measurementRateMs >> 8),
    0x01, 0x00,  // one navigation solution per measurement
    0x01, 0x00   // align to GPS time
  };
  sendUbx(0x06, 0x08, payload, sizeof(payload));
  return waitForUbxAck(0x06, 0x08);
}

bool Neo6mGps::begin() {
  serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
  delay(100);
  healthy_ = true;

  if (txPin_ >= 0) {
    Serial.printf("[GPS] 📡 Configurando %s via protocolo UBX...\n", name());
    while (serial_.available()) serial_.read();
    // Keep low-rate NMEA for UTC/date and human diagnostics while using UBX
    // for coherent 5 Hz position/velocity and receiver-native accuracy.
    bool ubxOk = configureUbxMessage(0xF0, 0x00, 5);      // GGA at 1 Hz
    ubxOk = configureUbxMessage(0xF0, 0x02, 0) && ubxOk;  // GSA off
    ubxOk = configureUbxMessage(0xF0, 0x03, 25) && ubxOk; // GSV every 5 s
    ubxOk = configureUbxMessage(0xF0, 0x04, 5) && ubxOk;  // RMC at 1 Hz
    ubxOk = configureUbxMessage(0xF0, 0x01, 0) && ubxOk;  // GLL off
    ubxOk = configureUbxMessage(0xF0, 0x05, 0) && ubxOk;  // VTG off
    ubxOk = configureUbxMessage(0x01, 0x02, 1) && ubxOk;  // NAV-POSLLH at 5 Hz
    ubxOk = configureUbxMessage(0x01, 0x12, 1) && ubxOk;  // NAV-VELNED at 5 Hz
    ubxOk = configureUbxMessage(0x01, 0x03, 1) && ubxOk;  // NAV-STATUS at 5 Hz
    ubxOk = configureUbxRate(200) && ubxOk;
    ubxNavigationActive_ = ubxOk;
    if (ubxOk) {
      Serial.println("[GPS] ✅ UBX coerente ativo: posição + velocidade a 5 Hz, NMEA diagnóstico a 1 Hz.");
    } else {
      // Best-effort recovery to the previous interoperable NMEA profile.
      configureUbxRate(1000);
      configureUbxMessage(0xF0, 0x00, 1);
      configureUbxMessage(0xF0, 0x02, 1);
      configureUbxMessage(0xF0, 0x03, 5);
      configureUbxMessage(0xF0, 0x04, 1);
      Serial.println("[GPS] ⚠️ Perfil UBX 5 Hz não confirmado; fallback NMEA 1 Hz ativo.");
    }
  }

  Serial.printf("[GPS] ✅ %s em GPIO %d (RX) / GPIO %d (TX), %lu baud.\n",
                name(), rxPin_, txPin_, (unsigned long)baud_);
  return true;
}

bool Neo6mGps::fixValid() const {
  if (ubxNavigationActive_) {
    return navigation_.fixOk && navigation_.fixType >= 2 &&
           (millis() - navigation_.receivedAtMs) < 1500;
  }
  const bool statusRecent = (millis() - lastFixStatusMs_) < 2500;
  // GGA is the authoritative quality field when present. GSA is a fallback for
  // receivers/talkers that do not emit GGA under their current configuration.
  const bool receiverReportsFix = statusRecent &&
      (hasGgaFixQuality_ ? ggaFixQuality_ > 0
                         : (hasGsaFixDimension_ && gsaFixDimension_ >= 2));
  return receiverReportsFix && gps_.location.isValid() && gps_.location.age() < 2500 &&
         gps_.satellites.isValid() && gps_.satellites.age() < 2500 &&
         gps_.satellites.value() >= 3;
}

bool Neo6mGps::locationUpdated() const {
  return ubxNavigationActive_ ? navigationSolutionUpdated() : gps_.location.isUpdated();
}

bool Neo6mGps::navigationSolutionUpdated() const {
  if (!ubxNavigationActive_) return gps_.location.isUpdated();
  if (navigationGeneration_ == consumedNavigationGeneration_) return false;
  consumedNavigationGeneration_ = navigationGeneration_;
  return true;
}

double Neo6mGps::latitude() const {
  return ubxNavigationActive_ ? navigation_.latE7 / 1e7 : gps_.location.lat();
}

double Neo6mGps::longitude() const {
  return ubxNavigationActive_ ? navigation_.lonE7 / 1e7 : gps_.location.lng();
}

float Neo6mGps::speedKmph() const {
  return ubxNavigationActive_ ? navigation_.groundSpeedCmS * 0.036f : gps_.speed.kmph();
}

bool Neo6mGps::parseUbxByte(uint8_t value) {
  auto checksum = [&](uint8_t byte) {
    ubxCkA_ = static_cast<uint8_t>(ubxCkA_ + byte);
    ubxCkB_ = static_cast<uint8_t>(ubxCkB_ + ubxCkA_);
  };
  switch (ubxState_) {
    case UbxState::Sync1:
      if (value == 0xB5) { ubxState_ = UbxState::Sync2; return true; }
      return false;
    case UbxState::Sync2:
      if (value == 0x62) {
        ubxState_ = UbxState::Class;
        ubxCkA_ = ubxCkB_ = 0;
      } else {
        ubxState_ = UbxState::Sync1;
      }
      return true;
    case UbxState::Class:
      ubxClass_ = value; checksum(value); ubxState_ = UbxState::Id; return true;
    case UbxState::Id:
      ubxId_ = value; checksum(value); ubxState_ = UbxState::Length1; return true;
    case UbxState::Length1:
      ubxLength_ = value; checksum(value); ubxState_ = UbxState::Length2; return true;
    case UbxState::Length2:
      ubxLength_ |= static_cast<uint16_t>(value) << 8;
      checksum(value);
      ubxIndex_ = 0;
      ubxState_ = ubxLength_ == 0 ? UbxState::CkA : UbxState::Payload;
      return true;
    case UbxState::Payload:
      if (ubxIndex_ < sizeof(ubxPayload_)) ubxPayload_[ubxIndex_] = value;
      ++ubxIndex_;
      checksum(value);
      if (ubxIndex_ >= ubxLength_) ubxState_ = UbxState::CkA;
      return true;
    case UbxState::CkA:
      ubxReceivedCkA_ = value; ubxState_ = UbxState::CkB; return true;
    case UbxState::CkB:
      if (ubxReceivedCkA_ == ubxCkA_ && value == ubxCkB_ &&
          ubxLength_ <= sizeof(ubxPayload_)) {
        processUbxMessage();
      }
      ubxState_ = UbxState::Sync1;
      return true;
  }
  return false;
}

void Neo6mGps::processUbxMessage() {
  if (ubxClass_ != 0x01) return;
  if (ubxId_ == 0x02 && ubxLength_ >= 28) { // NAV-POSLLH
    positionITowMs_ = readU32(ubxPayload_);
    navigation_.lonE7 = readI32(ubxPayload_ + 4);
    navigation_.latE7 = readI32(ubxPayload_ + 8);
    navigation_.horizontalAccuracyMm = readU32(ubxPayload_ + 20);
  } else if (ubxId_ == 0x12 && ubxLength_ >= 36) { // NAV-VELNED
    velocityITowMs_ = readU32(ubxPayload_);
    navigation_.groundSpeedCmS = readU32(ubxPayload_ + 20);
    navigation_.courseDegE5 = readI32(ubxPayload_ + 24);
    navigation_.speedAccuracyCmS = readU32(ubxPayload_ + 28);
    navigation_.courseAccuracyDegE5 = readU32(ubxPayload_ + 32);
  } else if (ubxId_ == 0x03 && ubxLength_ >= 16) { // NAV-STATUS
    statusITowMs_ = readU32(ubxPayload_);
    navigation_.fixType = ubxPayload_[4];
    navigation_.fixOk = (ubxPayload_[5] & 0x01) != 0;
  }
  refreshCoherentNavigationSolution();
}

void Neo6mGps::refreshCoherentNavigationSolution() {
  if (positionITowMs_ != velocityITowMs_ || positionITowMs_ != statusITowMs_ ||
      positionITowMs_ == publishedITowMs_) return;
  navigation_.iTowMs = positionITowMs_;
  navigation_.receivedAtMs = millis();
  publishedITowMs_ = positionITowMs_;
  ++navigationGeneration_;
}

uint8_t Neo6mGps::satellitesInUse() const {
  if (!gps_.satellites.isValid() || gps_.satellites.age() >= 2500) return 0;
  return static_cast<uint8_t>(gps_.satellites.value());
}

bool Neo6mGps::hdopValidRecent() const {
  if (!gps_.hdop.isValid() || gps_.hdop.age() >= 2500) return false;
  const float value = gps_.hdop.hdop();
  return value > 0.0f && value < 50.0f;
}

void Neo6mGps::updateSatellitesInView() {
  if (gpgsvSats_.isUpdated()) satsInViewGP_ = atoi(gpgsvSats_.value());
  if (glgsvSats_.isUpdated()) satsInViewGL_ = atoi(glgsvSats_.value());
  if (bdgsvSats_.isUpdated()) satsInViewBD_ = atoi(bdgsvSats_.value());
  if (gbgsvSats_.isUpdated()) satsInViewBD_ = atoi(gbgsvSats_.value());
  if (gagsvSats_.isUpdated()) satsInViewGA_ = atoi(gagsvSats_.value());
  if (gngsvSats_.isUpdated()) satsInViewGN_ = atoi(gngsvSats_.value());

  const int multiTotal = satsInViewGP_ + satsInViewGL_ + satsInViewBD_ + satsInViewGA_;
  liveSatsInView_ = max(satsInViewGN_, max(multiTotal, max(satsInViewGP_, max(satsInViewGL_, satsInViewBD_))));
}

void Neo6mGps::parseGsvLine(const char* line, bool showRawNmea, bool workoutActive) {
  if (line[0] != '$' || strlen(line) < 10 || strstr(line, "GSV") == nullptr) return;

  const char* p = line;
  char tokens[21][16];
  int tCount = 0;
  int tLen = 0;
  while (*p && tCount < 21) {
    if (*p == ',' || *p == '*') {
      tokens[tCount][tLen] = '\0';
      ++tCount;
      tLen = 0;
      if (*p == '*') break;
    } else if (tLen < 15) {
      tokens[tCount][tLen++] = *p;
    }
    ++p;
  }
  if (tCount < 4) return;

  const int totalMsgs = atoi(tokens[1]);
  const int msgNum = atoi(tokens[2]);
  const int totalSatsInView = atoi(tokens[3]);
  const char talker[] = {line[1], line[2], '\0'};
  if (msgNum == 1 || strcmp(gsvTalker_, talker) != 0) {
    strncpy(gsvTalker_, talker, sizeof(gsvTalker_) - 1);
    gsvTalker_[sizeof(gsvTalker_) - 1] = '\0';
    gsvTempSatCount_ = 0;
  }

  for (int i = 4; i + 3 < tCount && gsvTempSatCount_ < 16; i += 4) {
    const int prn = atoi(tokens[i]);
    const int snr = atoi(tokens[i + 3]);
    if (prn > 0) {
      gsvTempPrn_[gsvTempSatCount_] = prn;
      gsvTempSnr_[gsvTempSatCount_] = snr;
      ++gsvTempSatCount_;
    }
  }

  if (msgNum != totalMsgs || totalMsgs <= 0) return;

  int withSignal = 0;
  int strongSats = 0;
  int maxSnr = 0;
  String satList;
  for (int i = 0; i < gsvTempSatCount_; ++i) {
    if (gsvTempSnr_[i] <= 0) continue;
    ++withSignal;
    if (gsvTempSnr_[i] >= 28) ++strongSats;
    if (gsvTempSnr_[i] > maxSnr) maxSnr = gsvTempSnr_[i];
    satList += "[PRN" + String(gsvTempPrn_[i]) + ":" + String(gsvTempSnr_[i]) + "dB] ";
  }
  liveMaxSnr_ = maxSnr;

  if (!(showRawNmea || (!workoutActive && (withSignal > 0 || totalSatsInView > 0)))) return;
  Serial.print("[SINAL GPS] 📡 ");
  Serial.printf("Sats c/ sinal: %d/%d (Fortes >=28dB: %d) | Max: %02d dB-Hz | ",
                withSignal, totalSatsInView, strongSats, maxSnr);
  if (withSignal > 0) {
    Serial.print(satList);
    if (fixValid()) Serial.println("-> 🟢 FIX VALIDO (GGA/GSA + Lat/Lon recentes)");
    else if (strongSats > 0) Serial.println("-> 🟡 SINAL PRESENTE, receptor ainda sem solucao valida");
    else Serial.println("-> ⚠️ SINAL MUITO FRACO (coloque a antena voltada para o ceu)");
  } else {
    Serial.println("❌ NENHUM SINAL RF CAPTADO (Cabo solto ou antena sem ganho)");
  }
}

void Neo6mGps::poll(bool showRawNmea, bool workoutActive) {
  while (serial_.available() > 0) {
    const uint8_t value = static_cast<uint8_t>(serial_.read());
    ++charsProcessed_;
    if (parseUbxByte(value)) continue;
    const char c = static_cast<char>(value);
    if (showRawNmea && !workoutActive && Serial && Serial.availableForWrite() > 0) Serial.write(c);
    gps_.encode(c);

    if (c == '\n' || c == '\r') {
      if (nmeaLinePos_ > 0) {
        nmeaLineBuf_[nmeaLinePos_] = '\0';
        parseGsvLine(nmeaLineBuf_, showRawNmea, workoutActive);
        nmeaLinePos_ = 0;
      }
    } else if (nmeaLinePos_ < sizeof(nmeaLineBuf_) - 1) {
      nmeaLineBuf_[nmeaLinePos_++] = c;
    }
  }
  updateSatellitesInView();
  if (gpggaFixQuality_.isUpdated()) {
    ggaFixQuality_ = atoi(gpggaFixQuality_.value());
    hasGgaFixQuality_ = true;
    lastFixStatusMs_ = millis();
  }
  if (gnggaFixQuality_.isUpdated()) {
    ggaFixQuality_ = atoi(gnggaFixQuality_.value());
    hasGgaFixQuality_ = true;
    lastFixStatusMs_ = millis();
  }
  if (gpgsaFixDimension_.isUpdated()) {
    gsaFixDimension_ = atoi(gpgsaFixDimension_.value());
    hasGsaFixDimension_ = true;
    if (!hasGgaFixQuality_) lastFixStatusMs_ = millis();
  }
  if (gngsaFixDimension_.isUpdated()) {
    gsaFixDimension_ = atoi(gngsaFixDimension_.value());
    hasGsaFixDimension_ = true;
    if (!hasGgaFixQuality_) lastFixStatusMs_ = millis();
  }
}

void Neo6mGps::aidPosition(float lat, float lon) {
  (void)lat;
  (void)lon;
  // A position alone is not valid u-blox assistance. UBX-AID requires qualified
  // time/position uncertainty and/or ephemeris data. Silently sending PMTK/PCAS
  // commands to a NEO-6 can hide a wrong module identity and cannot be called A-GPS.
  Serial.println("[GPS AID] Ignorado: assistencia UBX ainda nao possui tempo/incerteza/efemerides qualificados.");
}

}  // namespace remus::drivers
