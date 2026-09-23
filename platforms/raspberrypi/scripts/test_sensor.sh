#!/usr/bin/env bash
# REMUS Sensor — Sensor hardware diagnostic script for Raspberry Pi
# Run this directly on the Raspberry Pi to test MPU-6050 connectivity.

set -euo pipefail

echo "=========================================================="
echo " REMUS Blade Sensor — Hardware Diagnostics"
echo "=========================================================="

echo "1. Checking I2C device node (/dev/i2c-1)..."
if [ ! -e "/dev/i2c-1" ]; then
  echo "❌ Error: /dev/i2c-1 not found!"
  echo "Make sure I2C is enabled by running ./prepare_pi.sh and rebooting."
  exit 1
fi
echo "✅ /dev/i2c-1 exists."

echo ""
echo "2. Probing I2C bus 1 for MPU-6050 (expected address: 0x68)..."
if ! command -v i2cdetect >/dev/null 2>&1; then
  echo "Installing i2c-tools..."
  sudo apt-get update -y && sudo apt-get install -y i2c-tools
fi

I2C_OUTPUT=$(i2cdetect -y 1)
echo "$I2C_OUTPUT"

if echo "$I2C_OUTPUT" | grep -q "68"; then
  echo "✅ MPU-6050 detected at address 0x68!"
elif echo "$I2C_OUTPUT" | grep -q "UU.*68\|68.*UU"; then
  echo "ℹ️ Address 0x68 is currently claimed by a kernel driver or background process."
else
  echo "⚠️ Warning: Device at 0x68 not detected. Check physical wiring:"
  echo "  - VCC -> 3.3V (Pin 1)"
  echo "  - GND -> GND  (Pin 6 or 9)"
  echo "  - SDA -> GPIO 2 (Pin 3)"
  echo "  - SCL -> GPIO 3 (Pin 5)"
  echo "  - AD0 -> GND (to select address 0x68)"
fi

echo ""
echo "3. Checking background service status..."
if systemctl is-active --quiet remus-proto2; then
  echo "ℹ️ Background service 'remus-proto2' is currently ACTIVE and recording."
  echo "You can check live logs with:"
  echo "  journalctl -u remus-proto2 -f"
else
  echo "ℹ️ Background service is not currently running."
  BIN_PATH="/opt/remus-sensor/remus-proto2"
  if [ ! -f "$BIN_PATH" ]; then
    BIN_PATH="$(cd "$(dirname "$0")/.." && pwd)/build/remus-proto2"
  fi
  if [ -f "$BIN_PATH" ]; then
    echo ""
    echo "Running 5-second test with: $BIN_PATH --no-gps"
    timeout 5 "$BIN_PATH" --no-gps || true
  else
    echo "Executable not found yet. Run ./install_service.sh first."
  fi
fi

echo ""
echo "=========================================================="
echo " Diagnostics complete!"
echo "=========================================================="
