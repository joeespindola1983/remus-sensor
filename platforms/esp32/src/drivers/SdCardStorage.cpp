#include "remus/drivers/SdCardStorage.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

namespace remus::drivers {

SdCardStorage::SdCardStorage(int sck, int miso, int mosi, int cs)
    : sck_(sck), miso_(miso), mosi_(mosi), cs_(cs) {}

uint8_t SdCardStorage::probeRawCmd0() {
  SPI.end();
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  pinMode(miso_, INPUT_PULLUP);
  pinMode(mosi_, OUTPUT);
  digitalWrite(mosi_, HIGH);
  pinMode(sck_, OUTPUT);
  digitalWrite(sck_, LOW);
  delay(10);

  SPI.begin(sck_, miso_, mosi_, -1);
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_, HIGH);
  for (int i = 0; i < 15; ++i) SPI.transfer(0xFF);
  digitalWrite(cs_, LOW);
  delayMicroseconds(20);

  const uint8_t cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
  for (uint8_t value : cmd0) SPI.transfer(value);

  uint8_t resp = 0xFF;
  for (int i = 0; i < 32; ++i) {
    resp = SPI.transfer(0xFF);
    if (resp != 0xFF) break;
  }
  digitalWrite(cs_, HIGH);
  SPI.transfer(0xFF);
  SPI.endTransaction();
  SPI.end();
  return resp;
}

bool SdCardStorage::tryInit(uint32_t freq) {
  SD.end();
  SPI.end();
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  pinMode(miso_, INPUT_PULLUP);
  pinMode(mosi_, OUTPUT);
  pinMode(sck_, OUTPUT);
  delay(20);
  SPI.begin(sck_, miso_, mosi_, -1);
  delay(10);

  digitalWrite(cs_, HIGH);
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < 15; ++i) SPI.transfer(0xFF);
  SPI.endTransaction();
  digitalWrite(cs_, HIGH);
  delay(10);

  const bool mounted = freq > 0 ? SD.begin(cs_, SPI, freq) : SD.begin(cs_, SPI);
  if (mounted && SD.cardType() != CARD_NONE) return true;
  SD.end();
  return false;
}

bool SdCardStorage::begin() {
  healthy_ = false;
  Serial.printf("[SD] 💾 %s: SCK=%d MISO=%d MOSI=%d CS=%d\n", name(), sck_, miso_, mosi_, cs_);
  const uint8_t r1 = probeRawCmd0();
  if (r1 == 0x01 || r1 == 0x00) Serial.printf("[SD DIAG] 🎯 CMD0 respondeu 0x%02X.\n", r1);
  else Serial.printf("[SD DIAG] ⚠️ CMD0 sem resposta válida (0x%02X); tentando mount.\n", r1);

  const uint32_t freqs[] = {4000000, 1000000, 400000, 0};
  for (const uint32_t freq : freqs) {
    if (!tryInit(freq)) continue;
    healthy_ = true;
    const uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    const uint8_t cardType = SD.cardType();
    const char* typeStr = "Desconhecido";
    if (cardType == CARD_MMC) typeStr = "MMC";
    else if (cardType == CARD_SD) typeStr = "SDSC";
    else if (cardType == CARD_SDHC) typeStr = "SDHC/SDXC";
    Serial.printf("[SD] ✅ Montado: Tipo=%s, %llu MB, CS=GPIO %d\n", typeStr, cardSize, cs_);
    Serial.println("[SD] ⏸️ Dados anteriores preservados no cartão.");
    return true;
  }

  Serial.println("[SD] ❌ Falha ao montar MicroSD.");
  Serial.printf("     Confira GND/alimentação e fiação: SCK=%d MISO=%d MOSI=%d CS=%d\n",
                sck_, miso_, mosi_, cs_);
  return false;
}

}  // namespace remus::drivers
