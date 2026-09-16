#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="${ROOT}/.cache"
PREFIX="${ROOT}/.deps/simdjson"
REPO="${SIMDJSON_REPO:-https://github.com/simdjson/simdjson.git}"
REF="${SIMDJSON_REF:-v3.13.0}"
SRC="${CACHE}/simdjson"

mkdir -p "${CACHE}" "${PREFIX}"
if [[ ! -d "${SRC}/.git" ]]; then
    git clone "${REPO}" "${SRC}"
fi

git -C "${SRC}" fetch --tags --prune
git -C "${SRC}" checkout "${REF}"

cmake -S "${SRC}" -B "${SRC}/build-pico-dds" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DSIMDJSON_BUILD_STATIC=OFF \
    -DSIMDJSON_BUILD_SHARED=ON \
    -DSIMDJSON_BUILD_TESTS=OFF \
    -DSIMDJSON_BUILD_BENCHMARKS=OFF \
    -DSIMDJSON_BUILD_EXAMPLES=OFF

cmake --build "${SRC}/build-pico-dds" --parallel
cmake --install "${SRC}/build-pico-dds"

echo "simdjson installed to: ${PREFIX}"