#include <cassert>
#include <iostream>

#include "remus/profiles/Prototype1.hpp"
#include "remus/profiles/BladeDev.hpp"

int main() {
  constexpr const auto& profile = remus::profiles::Prototype1;

  static_assert(remus::esp32::hw::profileIsValid(profile));
  static_assert(remus::esp32::hw::capabilityByte(profile) == 0x1F);

  assert(profile.imuPins.sda == 5);
  assert(profile.imuPins.scl == 6);
  assert(profile.gpsPins.rx == 0);
  assert(profile.gpsPins.tx == 1);
  assert(profile.sdPins.miso == 8);
  assert(profile.sdPins.mosi == 10);
  assert(profile.sdPins.sck == 20);
  assert(profile.sdPins.cs == 21);
  assert(profile.displayPins.sck == 4);
  assert(profile.displayPins.mosi == 2);
  assert(profile.displayPins.dc == 7);
  assert(profile.displayPins.rst == 3);
  assert(profile.displayPins.cs == 9);

  constexpr const auto& blade = remus::profiles::BladeDev;
  static_assert(remus::esp32::hw::profileIsValid(blade));
  static_assert(remus::esp32::hw::capabilityByte(blade) == 0x0C);
  assert(blade.imuPins.sda == 6);
  assert(blade.imuPins.scl == 5);
  assert(!blade.hasGps);
  assert(!blade.hasStorage);
  assert(!blade.hasDisplay);
  assert(blade.hasBle);

  std::cout << "esp32_hardware_profile_smoke OK\n";
}
