#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${script_dir}/platformio_install_and_monitor.sh" \
  remus-blade-dev \
  "Remus Blade" \
  "$(basename "$0")" \
  "$@"
