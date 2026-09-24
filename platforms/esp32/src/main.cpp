#include <Arduino.h>

#if defined(REMUS_TARGET_BLADE)
  #include "remus/app/BladeApp.hpp"
  using SelectedApp = remus::app::BladeApp;
#elif defined(REMUS_TARGET_FULL)
  #include "remus/app/RemusApp.hpp"
  using SelectedApp = remus::app::RemusApp;
#else
  #error "No REMUS application target selected"
#endif

namespace {
SelectedApp app;
}

void setup() {
  app.begin();
}

void loop() {
  app.tick();
}
