#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

#include "remus/app/RemusPiApp.hpp"
#include "remus/profiles/Prototype2.hpp"

namespace {
volatile sig_atomic_t gStopRequested = 0;

void onSignal(int) {
  gStopRequested = 1;
}

void usage(const char* argv0) {
  std::cout << "Usage: " << argv0 << " [options]\n"
            << "  --auto-start           start recording immediately\n"
            << "  --no-oled              disable SSD1306\n"
            << "  --no-gps               disable GPS (blade/paddle IMU-only mode)\n"
            << "  --raw-nmea             print raw NMEA while idle\n"
            << "  --i2c DEVICE           default /dev/i2c-1\n"
            << "  --gps DEVICE           default /dev/serial0 (or 'none')\n"
            << "  --session-dir PATH     default ./sessions\n";
}
}

int main(int argc, char** argv) {
  const auto& p = remus::profiles::Prototype2;
  remus::app::PiAppOptions options;
  options.i2cDevice = p.i2cDevice;
  options.gpsDevice = p.gpsDevice;
  options.sessionDir = p.defaultSessionDir;
  options.autoStart = false;
  options.oledEnabled = p.hasOled;
  options.gpsEnabled = true;
  options.rawNmea = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--auto-start") options.autoStart = true;
    else if (arg == "--no-oled") options.oledEnabled = false;
    else if (arg == "--no-gps") options.gpsEnabled = false;
    else if (arg == "--raw-nmea") options.rawNmea = true;
    else if (arg == "--i2c" && i + 1 < argc) options.i2cDevice = argv[++i];
    else if (arg == "--gps" && i + 1 < argc) {
      options.gpsDevice = argv[++i];
      if (options.gpsDevice == "none" || options.gpsDevice == "off") options.gpsEnabled = false;
    }
    else if (arg == "--session-dir" && i + 1 < argc) options.sessionDir = argv[++i];
    else if (arg == "--help" || arg == "-h") { usage(argv[0]); return 0; }
    else { std::cerr << "Unknown argument: " << arg << '\n'; usage(argv[0]); return 2; }
  }

  remus::app::RemusPiApp app(std::move(options));
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  if (!app.begin()) return 1;
  return app.run(&gStopRequested);
}
