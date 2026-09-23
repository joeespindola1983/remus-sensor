#pragma once

#include <cstdint>
#include "remus/hal/IStorage.hpp"

namespace remus::drivers {

class SdCardStorage final : public hal::IStorage {
public:
  SdCardStorage(int sck, int miso, int mosi, int cs);
  bool begin() override;
  bool healthy() const override { return healthy_; }
  const char* name() const override { return "MicroSD SPI"; }

private:
  uint8_t probeRawCmd0();
  bool tryInit(uint32_t freq);

  int sck_;
  int miso_;
  int mosi_;
  int cs_;
  bool healthy_ = false;
};

}  // namespace remus::drivers
