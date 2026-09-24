#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "Uso interno: $0 <ambiente> <nome> <comando> [--port PORTA] [--no-monitor] [--monitor-only]"
  exit 2
fi

environment="$1"
device_name="$2"
entrypoint="$3"
shift 3

port=""
monitor=true
upload=true

usage() {
  cat <<EOF
Instala e monitora o ${device_name} com PlatformIO.

Uso: ${entrypoint} [opções]

Opções:
  -p, --port PORTA   Porta serial, por exemplo /dev/cu.usbmodem101
      --no-monitor   Instala e encerra sem abrir o monitor serial
      --monitor-only Abre o monitor sem compilar ou instalar
  -h, --help         Exibe esta ajuda

Se houver exatamente uma porta serial, ela será selecionada automaticamente.
Com mais de uma porta conectada, informe --port para evitar gravar a placa errada.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--port)
      if [[ $# -lt 2 ]]; then
        echo "Erro: $1 exige uma porta." >&2
        exit 2
      fi
      port="$2"
      shift 2
      ;;
    --no-monitor)
      monitor=false
      shift
      ;;
    --monitor-only)
      upload=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Erro: opção desconhecida: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v pio >/dev/null 2>&1; then
  echo "Erro: PlatformIO CLI (pio) não foi encontrado no PATH." >&2
  echo "Instale o PlatformIO Core ou execute pelo terminal integrado do PlatformIO." >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
cd "$project_dir"

if [[ -z "$port" ]]; then
  mapfile_command='import json,sys; data=json.load(sys.stdin); [print(item["port"]) for item in data if item.get("port")]'
  ports_output="$(pio device list --serial --json-output | python3 -c "$mapfile_command")"
  ports=()
  while IFS= read -r detected_port; do
    [[ -n "$detected_port" ]] && ports+=("$detected_port")
  done <<< "$ports_output"

  if [[ ${#ports[@]} -eq 0 ]]; then
    echo "Erro: nenhuma porta serial foi encontrada. Conecte o ${device_name} por USB." >&2
    exit 1
  fi

  if [[ ${#ports[@]} -gt 1 ]]; then
    echo "Erro: mais de uma porta serial foi encontrada:" >&2
    printf '  %s\n' "${ports[@]}" >&2
    echo "Execute novamente com --port PORTA." >&2
    exit 1
  fi

  port="${ports[0]}"
fi

if [[ ! -e "$port" ]]; then
  echo "Erro: a porta serial não existe: $port" >&2
  exit 1
fi

echo "Dispositivo: ${device_name}"
echo "Ambiente PlatformIO: ${environment}"
echo "Porta serial: ${port}"

if [[ "$upload" == true ]]; then
  echo "Compilando e instalando..."
  pio run -e "$environment" -t upload --upload-port "$port"
fi

if [[ "$monitor" == true ]]; then
  echo "Abrindo monitor serial a 115200 baud. Pressione Ctrl+C para sair."
  exec pio device monitor --port "$port" --baud 115200
fi
