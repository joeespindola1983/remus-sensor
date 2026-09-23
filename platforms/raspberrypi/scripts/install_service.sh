#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
BUILD_DIR="$ROOT/platforms/raspberrypi/build"
INSTALL_DIR="/opt/remus-sensor"
DATA_DIR="/var/lib/remus/sessions"
SERVICE_SRC="$ROOT/platforms/raspberrypi/deploy/remus-proto2.service"
REMUS_USER="${SUDO_USER:-$USER}"

cmake -S "$ROOT/platforms/raspberrypi" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j2

sudo mkdir -p "$INSTALL_DIR" "$DATA_DIR"

if ! python3 -c "import dbus, gi" 2>/dev/null || ! command -v rfkill >/dev/null 2>&1; then
  echo "Installing Python D-Bus, BlueZ, and rfkill dependencies..."
  sudo apt-get update && sudo apt-get install -y python3-dbus python3-gi bluez rfkill i2c-tools
fi

# Ensure bluetooth is unblocked and interface is up
sudo rfkill unblock bluetooth 2>/dev/null || true
sudo hciconfig hci0 up 2>/dev/null || true

sudo cp "$BUILD_DIR/remus-proto2" "$INSTALL_DIR/remus-proto2"
if [ -f "$ROOT/platforms/raspberrypi/ble/remus_ble_service.py" ]; then
  sudo cp "$ROOT/platforms/raspberrypi/ble/remus_ble_service.py" "$INSTALL_DIR/remus_ble_service.py"
  sudo chmod +x "$INSTALL_DIR/remus_ble_service.py"
fi

# Ensure /run/remus exists with 0777 for IPC socket
sudo mkdir -p /run/remus
sudo chmod 0777 /run/remus
echo "d /run/remus 0777 root root -" | sudo tee /etc/tmpfiles.d/remus.conf >/dev/null

sudo chown -R "$REMUS_USER:$REMUS_USER" "$INSTALL_DIR" "$DATA_DIR"
sudo usermod -aG i2c,dialout,bluetooth "$REMUS_USER" || true

# Configure default options in /etc/default/remus-sensor if not already present
DEFAULT_CONF="/etc/default/remus-sensor"
DEFAULT_OPTS="--auto-start --no-gps --session-dir $DATA_DIR"
if [ ! -f "$DEFAULT_CONF" ]; then
  echo "REMUS_OPTS=\"$DEFAULT_OPTS\"" | sudo tee "$DEFAULT_CONF" >/dev/null
  echo "Created $DEFAULT_CONF with defaults: $DEFAULT_OPTS"
else
  echo "Keeping existing $DEFAULT_CONF config"
fi

sed "s/__REMUS_USER__/$REMUS_USER/g" "$SERVICE_SRC" | sudo tee /etc/systemd/system/remus-proto2.service >/dev/null
if [ -f "$ROOT/platforms/raspberrypi/deploy/remus-ble.service" ]; then
  sudo cp "$ROOT/platforms/raspberrypi/deploy/remus-ble.service" /etc/systemd/system/remus-ble.service
fi

# Disable legacy remus service if present to avoid I2C contention
sudo systemctl stop remus.service 2>/dev/null || true
sudo systemctl disable remus.service 2>/dev/null || true

sudo systemctl daemon-reload
sudo systemctl enable remus-proto2.service
sudo systemctl restart remus-proto2.service

if [ -f "/etc/systemd/system/remus-ble.service" ]; then
  sudo systemctl enable remus-ble.service
  sudo systemctl restart remus-ble.service
fi

echo "=========================================================="
echo " ✅ REMUS Prototype 2 + BLE GATT Server Installed!"
echo " User:       $REMUS_USER"
echo " Services:   remus-proto2 (IMU/SPM) & remus-ble (Bluetooth)"
echo " Status:     sudo systemctl status remus-proto2 remus-ble"
echo " Logs:       journalctl -u remus-proto2 -u remus-ble -f"
echo "=========================================================="
