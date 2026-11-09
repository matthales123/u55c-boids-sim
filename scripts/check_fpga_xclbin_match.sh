#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ENV="$ROOT_DIR/config/project.env"

XRT_SMI="${WEB_XRT_SMI:-/opt/xilinx/xrt/bin/xrt-smi}"
XCLBINUTIL="${WEB_XCLBINUTIL:-/opt/xilinx/xrt/bin/xclbinutil}"

log() {
  echo "[xclbin-check] $*"
}

expand_path() {
  local p="$1"
  p="${p/#\~/$HOME}"
  p="${p//\$HOME/$HOME}"
  printf '%s' "$p"
}

if [[ -f "$PROJECT_ENV" ]]; then
  # shellcheck disable=SC1090
  source "$PROJECT_ENV"
fi

DEVICE="${WEB_DEVICE:-${U55C_DEVICE_INDEX:-0}}"
EXPECTED_XCLBIN_RAW="${WEB_REQUIRED_XCLBIN:-${U55C_XCLBIN:-}}"
EXPECTED_XCLBIN="$(expand_path "$EXPECTED_XCLBIN_RAW")"
ACCEPT_ANY_PROJECT_ARTIFACT="${WEB_ACCEPT_ANY_PROJECT_ARTIFACT:-0}"
ARTIFACT_DIR="${WEB_ARTIFACT_DIR:-$HOME/.local/share/fpga-dashboard-web/artifacts/xclbin}"
ARTIFACT_PREFIX="${WEB_ARTIFACT_PREFIX:-${PROJECT_NAME:-u55c_boids_sim}_}"

resolve_device_bdf() {
  local raw="$1"
  if [[ "$raw" == *:*.* ]]; then
    printf '%s' "$raw"
    return
  fi

  if [[ "$raw" =~ ^[0-9]+$ ]]; then
    local idx="$raw"
    mapfile -t bdfs < <("$XRT_SMI" examine 2>/dev/null | sed -nE 's/.*\[([0-9A-Fa-f:.]+)\].*/\1/p')
    if (( idx < ${#bdfs[@]} )); then
      printf '%s' "${bdfs[$idx]}"
      return
    fi
  fi

  printf '%s' "$raw"
}

if [[ ! -x "$XRT_SMI" ]]; then
  log "xrt-smi not found/executable at $XRT_SMI"
  exit 1
fi

if [[ ! -x "$XCLBINUTIL" ]]; then
  log "xclbinutil not found/executable at $XCLBINUTIL"
  exit 1
fi

DEVICE_BDF="$(resolve_device_bdf "$DEVICE")"

FLASHED_UUID="$("$XRT_SMI" examine --device "$DEVICE_BDF" --report dynamic-regions 2>/dev/null \
  | sed -nE 's/.*Xclbin UUID:[[:space:]]*([A-Fa-f0-9-]+).*/\1/p' \
  | head -n 1 \
  | tr '[:upper:]' '[:lower:]')"

if [[ -z "$FLASHED_UUID" ]]; then
  log "Could not read flashed xclbin UUID from device '$DEVICE' (resolved: '$DEVICE_BDF')"
  exit 1
fi

if [[ "$ACCEPT_ANY_PROJECT_ARTIFACT" == "1" ]]; then
  if [[ ! -d "$ARTIFACT_DIR" ]]; then
    log "Artifact directory not found: $ARTIFACT_DIR"
    exit 1
  fi

  shopt -s nullglob
  candidates=("$ARTIFACT_DIR"/"$ARTIFACT_PREFIX"*.xclbin)
  shopt -u nullglob

  if (( ${#candidates[@]} == 0 )); then
    log "No candidate artifacts found with prefix '$ARTIFACT_PREFIX' in $ARTIFACT_DIR"
    exit 1
  fi

  for candidate in "${candidates[@]}"; do
    base="$(basename "$candidate")"
    if [[ "$base" == *_hw_emu.xclbin || "$base" == *_sw_emu.xclbin ]]; then
      continue
    fi
    uuid="$("$XCLBINUTIL" --info --input "$candidate" 2>/dev/null \
      | sed -nE 's/.*UUID \(xclbin\):[[:space:]]*([A-Fa-f0-9-]+).*/\1/p' \
      | head -n 1 \
      | tr '[:upper:]' '[:lower:]')"
    if [[ -n "$uuid" && "$uuid" == "$FLASHED_UUID" ]]; then
      log "UUID match OK on device '$DEVICE' (resolved: '$DEVICE_BDF') ($FLASHED_UUID)"
      log "Matched artifact: $candidate"
      exit 0
    fi
  done

  log "No matching UUID found for flashed image among project artifacts."
  log "Flashed UUID: $FLASHED_UUID"
  exit 42
fi

if [[ -z "$EXPECTED_XCLBIN" ]]; then
  log "No expected xclbin configured (WEB_REQUIRED_XCLBIN/U55C_XCLBIN)."
  exit 1
fi

if [[ ! -f "$EXPECTED_XCLBIN" ]]; then
  log "Expected xclbin not found: $EXPECTED_XCLBIN"
  exit 1
fi

EXPECTED_UUID="$("$XCLBINUTIL" --info --input "$EXPECTED_XCLBIN" 2>/dev/null \
  | sed -nE 's/.*UUID \(xclbin\):[[:space:]]*([A-Fa-f0-9-]+).*/\1/p' \
  | head -n 1 \
  | tr '[:upper:]' '[:lower:]')"

if [[ -z "$EXPECTED_UUID" ]]; then
  log "Could not parse expected xclbin UUID from $EXPECTED_XCLBIN"
  exit 1
fi

if [[ "$EXPECTED_UUID" != "$FLASHED_UUID" ]]; then
  log "UUID mismatch on device '$DEVICE' (resolved: '$DEVICE_BDF')"
  log "Expected: $EXPECTED_UUID"
  log "Flashed : $FLASHED_UUID"
  exit 42
fi

log "UUID match OK on device '$DEVICE' (resolved: '$DEVICE_BDF') ($FLASHED_UUID)"
