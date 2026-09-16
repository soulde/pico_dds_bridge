#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export LD_LIBRARY_PATH="${ROOT}/.deps/xrobotoolkit/lib:${ROOT}/.deps/cyclonedds/lib:${LD_LIBRARY_PATH:-}"

exec "${ROOT}/build/pico_dds_bridge" "$@"
