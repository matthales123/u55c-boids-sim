#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set -a
source "$ROOT_DIR/config/project.env"
set +a

if [[ -n "${VITIS_SETTINGS:-}" && -f "${VITIS_SETTINGS}" ]]; then
  # shellcheck disable=SC1090
  source "${VITIS_SETTINGS}"
fi

TARGET="${TARGET:-hw}"
PLATFORM="${PLATFORM:-${U55C_PLATFORM:-xilinx_u55c_gen3x16_xdma_3_202210_1}}"
PART="${PART:-${U55C_PART:-xcu55c-fsvh2892-2L-e}}"

make -C "$ROOT_DIR/hardware/rtl_kernel" xclbin \
  TARGET="$TARGET" \
  PLATFORM="$PLATFORM" \
  PART="$PART"
