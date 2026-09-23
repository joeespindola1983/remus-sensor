#include <Arduino.h>
#include "remus/app/RemusApp.hpp"

namespace {
remus::app::RemusApp app;
}

void setup() {
  app.begin();
}

void loop() {
  app.tick();
}
