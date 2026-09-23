#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace remus::drivers {

class LinuxSsd1306 {
public:
  LinuxSsd1306(std::string device, uint8_t address = 0x3C);
  ~LinuxSsd1306();

  bool begin();
  bool healthy() const { return healthy_; }
  void render(float spm, double splitSeconds, int gpsBars, bool recording);

private:
  bool command(const uint8_t* bytes, size_t count);
  bool data(const uint8_t* bytes, size_t count);
  void clear();
  void flush();
  void setPixel(int x, int y, bool on = true);
  void drawChar(int x, int y, char ch, int scale = 1);
  void drawText(int x, int y, const std::string& text, int scale = 1);
  void drawGpsBars(int x, int y, int bars);
  static const uint8_t* glyph(char ch);

  std::string device_;
  uint8_t address_;
  int fd_ = -1;
  bool healthy_ = false;
  std::array<uint8_t, 128 * 64 / 8> framebuffer_{};
};

}  // namespace remus::drivers
