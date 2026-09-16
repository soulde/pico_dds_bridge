#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cmake -S "${ROOT}" -B "${ROOT}/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH="${ROOT}/.deps/cyclonedds" \
    -DPXREA_ROOT="${ROOT}/.deps/xrobotoolkit"

cmake --build "${ROOT}/build" --parallel
