#!/usr/bin/env bash
# REMUS Sensor — Deploy to Raspberry Pi
# Usage:
#   ./deploy_to_pi.sh [user@host]
# Default target: pi@gramulho-pi.local

set -euo pipefail

TARGET="${1:-${PI_HOST:-pi@gramulho-pi.local}}"
REMOTE_DIR="remus-sensor"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "=========================================================="
echo " REMUS Blade Sensor — Deploy to Raspberry Pi"
echo " Target: $TARGET"
echo " Local source: $ROOT"
echo "=========================================================="

# 1. Check if passwordless SSH key authentication is already working
if ! ssh -o BatchMode=yes -o ConnectTimeout=3 "$TARGET" "true" 2>/dev/null; then
  PUB_KEY=""
  if [ -f "$HOME/.ssh/id_ed25519.pub" ]; then
    PUB_KEY="$HOME/.ssh/id_ed25519.pub"
  elif [ -f "$HOME/.ssh/id_rsa.pub" ]; then
    PUB_KEY="$HOME/.ssh/id_rsa.pub"
  fi

  if [ -n "$PUB_KEY" ]; then
    echo "🔑 Chave SSH detectada ($PUB_KEY), mas ainda não autorizada no Pi."
    echo "Deseja salvar a credencial no Raspberry Pi para NUNCA MAIS pedir senha? [S/n]"
    read -r resp || resp="s"
    if [[ "$resp" =~ ^[Ss]?$ ]]; then
      echo "Copiando chave SSH para $TARGET (digite a senha do Pi uma única vez)..."
      ssh-copy-id -i "$PUB_KEY" "$TARGET" || {
        echo "⚠️ Não foi possível copiar a chave automaticamente. Prosseguindo com multiplexação..."
      }
    fi
  fi
fi

# 2. Setup SSH Multiplexing (ControlMaster)
# Reuses a single authenticated connection for all SSH and rsync commands,
# asking for password at most ONCE even without an SSH key!
CONTROL_DIR="$(mktemp -d /tmp/remus_ssh.XXXXXX)"
CONTROL_PATH="$CONTROL_DIR/ctrl_%h_%p_%r"

cleanup() {
  ssh -O exit -o ControlPath="$CONTROL_PATH" "$TARGET" 2>/dev/null || true
  rm -rf "$CONTROL_DIR"
}
trap cleanup EXIT

SSH_CMD="ssh -o ControlMaster=auto -o ControlPath=$CONTROL_PATH -o ControlPersist=5m"

echo "1. Establishing authenticated connection to $TARGET..."
$SSH_CMD "$TARGET" "echo '✅ Conexão estabelecida com sucesso com o Raspberry Pi'"

echo "2. Synchronizing source files to $TARGET:~/$REMOTE_DIR..."
rsync -avz --delete \
  -e "$SSH_CMD" \
  --exclude='.git' \
  --exclude='.pio' \
  --exclude='.vscode' \
  --exclude='build' \
  --exclude='*.bin' \
  --exclude='__pycache__' \
  --exclude='.DS_Store' \
  "$ROOT/" "$TARGET:~/$REMOTE_DIR/"

echo "3. Building and installing service on $TARGET..."
$SSH_CMD -t "$TARGET" "cd ~/$REMOTE_DIR/platforms/raspberrypi && sudo ./scripts/install_service.sh"

echo "4. Checking service status..."
$SSH_CMD "$TARGET" "sudo systemctl status remus-proto2 remus-ble --no-pager" || true

echo ""
echo "5. Recent logs:"
$SSH_CMD "$TARGET" "sudo journalctl -u remus-proto2 -u remus-ble -n 10 --no-pager" || true

echo ""
echo "=========================================================="
echo " ✅ Deploy complete!"
echo " Live logs:    ./monitor_pi.sh $TARGET"
echo " Download:     ./fetch_sessions.sh $TARGET"
echo "=========================================================="
