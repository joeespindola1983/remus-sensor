#include "remus/drivers/LinuxSsd1306.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

namespace remus::drivers {
namespace {

#define G(a,b,c,d,e) {a,b,c,d,e}
static const uint8_t kBlank[5] = G(0,0,0,0,0);
static const uint8_t k0[5] = G(0x3E,0x51,0x49,0x45,0x3E);
static const uint8_t k1[5] = G(0x00,0x42,0x7F,0x40,0x00);
static const uint8_t k2[5] = G(0x42,0x61,0x51,0x49,0x46);
static const uint8_t k3[5] = G(0x21,0x41,0x45,0x4B,0x31);
static const uint8_t k4[5] = G(0x18,0x14,0x12,0x7F,0x10);
static const uint8_t k5[5] = G(0x27,0x45,0x45,0x45,0x39);
static const uint8_t k6[5] = G(0x3C,0x4A,0x49,0x49,0x30);
static const uint8_t k7[5] = G(0x01,0x71,0x09,0x05,0x03);
static const uint8_t k8[5] = G(0x36,0x49,0x49,0x49,0x36);
static const uint8_t k9[5] = G(0x06,0x49,0x49,0x29,0x1E);
static const uint8_t kS[5] = G(0x46,0x49,0x49,0x49,0x31);
static const uint8_t kP[5] = G(0x7F,0x09,0x09,0x09,0x06);
static const uint8_t kM[5] = G(0x7F,0x02,0x0C,0x02,0x7F);
static const uint8_t kR[5] = G(0x7F,0x09,0x19,0x29,0x46);
static const uint8_t kE[5] = G(0x7F,0x49,0x49,0x49,0x41);
static const uint8_t kC[5] = G(0x3E,0x41,0x41,0x41,0x22);
static const uint8_t kG[5] = G(0x3E,0x41,0x49,0x49,0x7A);
static const uint8_t kColon[5] = G(0x00,0x36,0x36,0x00,0x00);
static const uint8_t kDot[5] = G(0x00,0x60,0x60,0x00,0x00);
static const uint8_t kDash[5] = G(0x08,0x08,0x08,0x08,0x08);
#undef G

}  // namespace

LinuxSsd1306::LinuxSsd1306(std::string device, uint8_t address)
    : device_(std::move(device)), address_(address) {}

LinuxSsd1306::~LinuxSsd1306() {
  if (fd_ >= 0) ::close(fd_);
}

bool LinuxSsd1306::command(const uint8_t* bytes, size_t count) {
  if (fd_ < 0) return false;
  std::vector<uint8_t> packet(count + 1);
  packet[0] = 0x00;
  std::copy(bytes, bytes + count, packet.begin() + 1);
  return ::write(fd_, packet.data(), packet.size()) == static_cast<ssize_t>(packet.size());
}

bool LinuxSsd1306::data(const uint8_t* bytes, size_t count) {
  if (fd_ < 0) return false;
  // Keep Linux I2C writes small; 16-byte payloads are accepted by common adapters.
  size_t offset = 0;
  while (offset < count) {
    const size_t n = std::min<size_t>(16, count - offset);
    uint8_t packet[17];
    packet[0] = 0x40;
    std::memcpy(packet + 1, bytes + offset, n);
    if (::write(fd_, packet, n + 1) != static_cast<ssize_t>(n + 1)) return false;
    offset += n;
  }
  return true;
}

bool LinuxSsd1306::begin() {
  fd_ = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0 || ::ioctl(fd_, I2C_SLAVE, address_) < 0) {
    std::cerr << "[OLED] Failed to open " << device_ << " addr=0x" << std::hex
              << int(address_) << std::dec << ": " << std::strerror(errno) << '\n';
    healthy_ = false;
    return false;
  }

  const uint8_t init[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
    0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
    0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
  };
  healthy_ = command(init, sizeof(init));
  clear();
  flush();
  if (healthy_) std::cout << "[OLED] OK SSD1306 128x64 @ 0x" << std::hex << int(address_) << std::dec << '\n';
  return healthy_;
}

void LinuxSsd1306::clear() {
  framebuffer_.fill(0);
}

void LinuxSsd1306::setPixel(int x, int y, bool on) {
  if (x < 0 || x >= 128 || y < 0 || y >= 64) return;
  const size_t index = static_cast<size_t>(x + (y / 8) * 128);
  const uint8_t mask = static_cast<uint8_t>(1u << (y & 7));
  if (on) framebuffer_[index] |= mask;
  else framebuffer_[index] &= static_cast<uint8_t>(~mask);
}

const uint8_t* LinuxSsd1306::glyph(char ch) {
  switch (ch) {
    case '0': return k0; case '1': return k1; case '2': return k2; case '3': return k3; case '4': return k4;
    case '5': return k5; case '6': return k6; case '7': return k7; case '8': return k8; case '9': return k9;
    case 'S': return kS; case 'P': return kP; case 'M': return kM; case 'R': return kR; case 'E': return kE;
    case 'C': return kC; case 'G': return kG; case ':': return kColon; case '.': return kDot; case '-': return kDash;
    default: return kBlank;
  }
}

void LinuxSsd1306::drawChar(int x, int y, char ch, int scale) {
  const uint8_t* g = glyph(ch);
  for (int col = 0; col < 5; ++col) {
    for (int row = 0; row < 7; ++row) {
      if ((g[col] & (1u << row)) == 0) continue;
      for (int dx = 0; dx < scale; ++dx)
        for (int dy = 0; dy < scale; ++dy)
          setPixel(x + col * scale + dx, y + row * scale + dy);
    }
  }
}

void LinuxSsd1306::drawText(int x, int y, const std::string& text, int scale) {
  for (char ch : text) {
    drawChar(x, y, ch, scale);
    x += 6 * scale;
  }
}

void LinuxSsd1306::drawGpsBars(int x, int y, int bars) {
  bars = std::clamp(bars, 0, 3);
  for (int i = 0; i < 3; ++i) {
    const int h = 3 + i * 3;
    for (int dx = 0; dx < 3; ++dx)
      for (int dy = 0; dy < h; ++dy)
        setPixel(x + i * 5 + dx, y - dy, i < bars);
  }
}

void LinuxSsd1306::flush() {
  if (!healthy_) return;
  const uint8_t window[] = {0x21, 0, 127, 0x22, 0, 7};
  if (!command(window, sizeof(window)) || !data(framebuffer_.data(), framebuffer_.size())) healthy_ = false;
}

void LinuxSsd1306::render(float spm, double splitSeconds, int gpsBars, bool recording) {
  if (!healthy_) return;
  clear();

  char spmBuf[16];
  if (spm > 0.0f) std::snprintf(spmBuf, sizeof(spmBuf), "%.1f", spm);
  else std::snprintf(spmBuf, sizeof(spmBuf), "--.-");
  drawText(0, 1, "SPM", 1);
  drawText(0, 12, spmBuf, 2);

  char splitBuf[16];
  if (splitSeconds > 0.0) {
    const int total = static_cast<int>(std::lround(splitSeconds));
    std::snprintf(splitBuf, sizeof(splitBuf), "%d:%02d", total / 60, total % 60);
  } else {
    std::snprintf(splitBuf, sizeof(splitBuf), "--:--");
  }
  drawText(0, 40, "500", 1);
  drawText(24, 38, splitBuf, 2);
  if (recording) drawText(93, 1, "REC", 1);
  drawGpsBars(112, 12, gpsBars);
  flush();
}

}  // namespace remus::drivers
