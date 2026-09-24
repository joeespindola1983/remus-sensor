#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

c++ -std=c++17 -O2 \
  -I"$ROOT/lib/remus-core/include" \
  "$ROOT/tests/core_smoke.cpp" \
  "$ROOT/lib/remus-core/src/LiveSpmEstimator.cpp" \
  -o "$TMP/core_smoke"
"$TMP/core_smoke"

c++ -std=c++17 -O2 \
  -I"$ROOT/platforms/esp32/include" \
  "$ROOT/tests/esp32_hardware_profile_smoke.cpp" \
  -o "$TMP/esp32_hardware_profile_smoke"
"$TMP/esp32_hardware_profile_smoke"

c++ -std=c++17 -O2 \
  -I"$ROOT/lib/remus-core/include" \
  "$ROOT/tests/blade_protocol_smoke.cpp" \
  -o "$TMP/blade_protocol_smoke"
"$TMP/blade_protocol_smoke"

c++ -std=c++17 -O2 -pthread \
  -I"$ROOT/lib/remus-core/include" \
  -I"$ROOT/platforms/raspberrypi/include" \
  "$ROOT/tests/session_storage_smoke.cpp" \
  "$ROOT/platforms/raspberrypi/src/drivers/LinuxSessionStorage.cpp" \
  -o "$TMP/session_storage_smoke"
"$TMP/session_storage_smoke"

if [[ "$(uname -s)" == "Linux" ]]; then
  cmake -S "$ROOT/platforms/raspberrypi" -B "$TMP/pi-build" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build "$TMP/pi-build" -j2 >/dev/null
  "$TMP/pi-build/remus-proto2" --help >/dev/null
else
  echo "[VALIDATE] Note: Skipping Linux-specific remus-proto2 binary build on non-Linux host ($(uname -s))."
fi

python3 "$ROOT/tests/test_platformio_custom_targets.py"

echo "REMUS validation OK"
