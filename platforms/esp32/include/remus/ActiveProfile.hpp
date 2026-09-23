#pragma once

#if defined(REMUS_PROFILE_PROTO1)
  #include "remus/profiles/Prototype1.hpp"
  namespace remus { inline constexpr const auto& hardware = profiles::Prototype1; }
#else
  #error "No ESP32 REMUS profile selected. Build env remus-proto1."
#endif
