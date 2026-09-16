#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cmake -S "${ROOT}" -B "${ROOT}/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${ROOT}/.deps/cyclonedds;${ROOT}/.deps/iceoryx;${ROOT}/.deps/simdjson" \
    -DPXREA_ROOT="${ROOT}/.deps/xrobotoolkit"

cmake --build "${ROOT}/build" --parallel
