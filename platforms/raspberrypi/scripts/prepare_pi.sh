#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y build-essential cmake i2c-tools git rsync python3-dbus python3-gi bluez
sudo rfkill unblock bluetooth || true
sudo systemctl enable --now bluetooth || true

REMUS_USER="${SUDO_USER:-$USER}"
sudo usermod -aG i2c "$REMUS_USER" || true

# Locate boot config file (Debian Bullseye/Buster uses /boot/config.txt, Bookworm uses /boot/firmware/config.txt)
BOOT_CONFIG="/boot/config.txt"
if [ ! -f "$BOOT_CONFIG" ] && [ -f "/boot/firmware/config.txt" ]; then
  BOOT_CONFIG="/boot/firmware/config.txt"
fi

echo "Checking I2C configuration in $BOOT_CONFIG..."
if [ -f "$BOOT_CONFIG" ]; then
  # Ensure I2C is enabled and set to 400kHz for high-rate IMU sampling (200 Hz)
  if grep -q "^dtparam=i2c_arm=" "$BOOT_CONFIG"; then
    sudo sed -i 's/^dtparam=i2c_arm=.*/dtparam=i2c_arm=on,i2c_arm_baudrate=400000/' "$BOOT_CONFIG"
  else
    echo "dtparam=i2c_arm=on,i2c_arm_baudrate=400000" | sudo tee -a "$BOOT_CONFIG" >/dev/null
  fi
  echo "✅ I2C configured at 400 kHz (Fast-Mode) in $BOOT_CONFIG"
fi

# Load i2c-dev module if not already loaded
sudo modprobe i2c-dev || true
if ! grep -q "^i2c-dev" /etc/modules 2>/dev/null; then
  echo "i2c-dev" | sudo tee -a /etc/modules >/dev/null
fi

echo ""
echo "=========================================================="
echo " REMUS Blade Sensor (Raspberry Pi) Prepared!"
echo "=========================================================="
echo " - I2C: Enabled @ 400 kHz (SDA: GPIO 2 / pin 3, SCL: GPIO 3 / pin 5)"
echo " - GPS: Not needed for blade IMU capture"
echo " - User '$REMUS_USER' added to group 'i2c'"
echo ""
echo "Please reboot if this was the first time enabling I2C:"
echo "  sudo reboot"
echo ""
echo "After boot/reboot, verify that the MPU-6050 is detected at 0x68:"
echo "  i2cdetect -y 1"
echo "=========================================================="
