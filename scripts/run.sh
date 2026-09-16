#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export CYCLONEDDS_URI="file://${ROOT}/config/cyclonedds.xml"
export LD_LIBRARY_PATH="${ROOT}/.deps/xrobotoolkit/lib:${ROOT}/.deps/cyclonedds/lib:${ROOT}/.deps/iceoryx/lib:${ROOT}/.deps/simdjson/lib:${LD_LIBRARY_PATH:-}"

exec "${ROOT}/build/pico_dds_bridge" "$@"
