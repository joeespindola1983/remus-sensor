#!/usr/bin/env bash
# REMUS Sensor — Monitor live service logs on Raspberry Pi
# Usage:
#   ./monitor_pi.sh [user@host]
# Default target: pi@gramulho-pi.local

set -euo pipefail

TARGET="${1:-${PI_HOST:-pi@gramulho-pi.local}}"

echo "=========================================================="
echo " REMUS Blade Sensor — Live Service Monitor"
echo " Target: $TARGET"
echo " Press Ctrl+C to exit monitoring."
echo "=========================================================="

ssh -t "$TARGET" "sudo journalctl -u remus-proto2 -u remus-ble -f -n 25"
