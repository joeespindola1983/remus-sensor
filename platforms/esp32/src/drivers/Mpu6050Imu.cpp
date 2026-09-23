#include "remus/drivers/Mpu6050Imu.hpp"

#include <Arduino.h>

namespace remus::drivers {

Mpu6050Imu::Mpu6050Imu(TwoWire& wire, int sda, int scl, uint16_t rateHz)
    : wire_(wire), sda_(sda), scl_(scl), rateHz_(rateHz) {}

bool Mpu6050Imu::configureAddress(uint8_t addr) {
  wire_.beginTransmission(addr);
  if (wire_.endTransmission() != 0) return false;

  wire_.beginTransmission(addr);
  wire_.write(0x6B);
  wire_.write(0x00);
  if (wire_.endTransmission(true) != 0) return false;
  delay(10);

  wire_.beginTransmission(addr);
  wire_.write(0x1C);
  wire_.write(0x10); // ±8g
  if (wire_.endTransmission(true) != 0) return false;

  wire_.beginTransmission(addr);
  wire_.write(0x1B);
  wire_.write(0x08); // ±500 dps
  if (wire_.endTransmission(true) != 0) return false;

  wire_.beginTransmission(addr);
  wire_.write(0x1A);
  wire_.write(0x03); // DLPF ~44/42 Hz
  if (wire_.endTransmission(true) != 0) return false;

  // MPU6050 internal sample rate is 1 kHz with DLPF enabled.
  const uint16_t divider = rateHz_ >= 1000 ? 0 : (1000 / rateHz_) - 1;
  wire_.beginTransmission(addr);
  wire_.write(0x19);
  wire_.write(static_cast<uint8_t>(divider));
  if (wire_.endTransmission(true) != 0) return false;

  return true;
}

bool Mpu6050Imu::begin() {
  healthy_ = false;
  wire_.begin(sda_, scl_, 400000);
  delay(30);

  if (configureAddress(0x68)) {
    address_ = 0x68;
    healthy_ = true;
  } else if (configureAddress(0x69)) {
    address_ = 0x69;
    healthy_ = true;
  }

  if (healthy_) {
    Serial.printf("[IMU] ✅ %s inicializado no endereço 0x%02X (SDA=%d, SCL=%d, %u Hz)\n",
                  name(), address_, sda_, scl_, rateHz_);
  } else {
    Serial.printf("[IMU] ⚠️ %s não encontrado em 0x68/0x69 (SDA=%d, SCL=%d)\n",
                  name(), sda_, scl_);
  }
  return healthy_;
}

bool Mpu6050Imu::read(hal::ImuSample& sample) {
  if (!healthy_) return false;

  wire_.beginTransmission(address_);
  wire_.write(0x3B);
  if (wire_.endTransmission(false) != 0) {
    healthy_ = false;
    return false;
  }

  const size_t bytesRead = wire_.requestFrom((uint16_t)address_, (uint8_t)14, true);
  if (bytesRead != 14 || wire_.available() != 14) {
    healthy_ = false;
    return false;
  }

  sample.rawAx = wire_.read() << 8 | wire_.read();
  sample.rawAy = wire_.read() << 8 | wire_.read();
  sample.rawAz = wire_.read() << 8 | wire_.read();
  wire_.read(); wire_.read(); // Temperature
  sample.rawGx = wire_.read() << 8 | wire_.read();
  sample.rawGy = wire_.read() << 8 | wire_.read();
  sample.rawGz = wire_.read() << 8 | wire_.read();

  sample.accelX = sample.rawAx / 4096.0f;
  sample.accelY = sample.rawAy / 4096.0f;
  sample.accelZ = sample.rawAz / 4096.0f;
  sample.gyroX = sample.rawGx / 65.5f;
  sample.gyroY = sample.rawGy / 65.5f;
  sample.gyroZ = sample.rawGz / 65.5f;
  return true;
}

}  // namespace remus::drivers
