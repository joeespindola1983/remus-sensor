#!/usr/bin/env bash
# REMUS Sensor — Fetch recorded sessions from Raspberry Pi
# Usage:
#   ./fetch_sessions.sh [user@host] [local_dest_dir] [--delete-remote]
# Default target: pi@gramulho-pi.local
# Default dest:   ./downloaded_sessions

set -euo pipefail

TARGET="pi@gramulho-pi.local"
LOCAL_DEST="./downloaded_sessions"
DELETE_REMOTE=false

for arg in "$@"; do
  if [ "$arg" == "--delete-remote" ]; then
    DELETE_REMOTE=true
  elif [[ "$arg" == *"@"* ]] || [[ "$arg" == *".local"* ]] || [[ "$arg" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    TARGET="$arg"
  else
    LOCAL_DEST="$arg"
  fi
done

REMOTE_SESSIONS="/var/lib/remus/sessions"

echo "=========================================================="
echo " REMUS Blade Sensor — Fetch Sessions"
echo " Target: $TARGET"
echo " Remote path: $REMOTE_SESSIONS"
echo " Destination: $LOCAL_DEST"
echo "=========================================================="

mkdir -p "$LOCAL_DEST"

# Setup SSH connection multiplexing
CONTROL_DIR="$(mktemp -d /tmp/remus_fetch.XXXXXX)"
CONTROL_PATH="$CONTROL_DIR/ctrl_%h_%p_%r"

cleanup() {
  ssh -O exit -o ControlPath="$CONTROL_PATH" "$TARGET" 2>/dev/null || true
  rm -rf "$CONTROL_DIR"
}
trap cleanup EXIT

SSH_CMD="ssh -o ControlMaster=auto -o ControlPath=$CONTROL_PATH -o ControlPersist=5m"

echo "1. Checking files on Raspberry Pi..."
REMOTE_FILES=$($SSH_CMD "$TARGET" "ls -lh $REMOTE_SESSIONS/*.bin 2>/dev/null || true")

if [ -z "$REMOTE_FILES" ]; then
  echo "ℹ️ No .bin session files found on $TARGET in $REMOTE_SESSIONS."
  exit 0
fi

echo "Found session files:"
echo "$REMOTE_FILES"
echo ""

echo "2. Downloading session files via rsync..."
rsync -avz --progress -e "$SSH_CMD" "$TARGET:$REMOTE_SESSIONS/*.bin" "$LOCAL_DEST/"

echo ""
echo "3. Downloaded files in $LOCAL_DEST:"
ls -lh "$LOCAL_DEST"/*.bin 2>/dev/null || true

if [ "$DELETE_REMOTE" = true ]; then
  echo ""
  echo "4. Removing downloaded sessions from Raspberry Pi as requested (--delete-remote)..."
  $SSH_CMD "$TARGET" "sudo rm -f $REMOTE_SESSIONS/*.bin"
  echo "✅ Remote sessions cleared."
fi

echo ""
echo "=========================================================="
echo " ✅ Sessions fetch complete! Saved to $LOCAL_DEST"
echo "=========================================================="
