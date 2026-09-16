#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="${ROOT}/.cache"
PREFIX="${ROOT}/.deps/iceoryx"
REPO="${ICEORYX_REPO:-https://github.com/eclipse-iceoryx/iceoryx.git}"
REF="${ICEORYX_REF:-release_2.0}"
SRC="${CACHE}/iceoryx"

mkdir -p "${CACHE}" "${PREFIX}"
if [[ ! -d "${SRC}/.git" ]]; then
    git clone --branch "${REF}" "${REPO}" "${SRC}"
else
    git -C "${SRC}" fetch --all --prune
    git -C "${SRC}" checkout "${REF}"
fi

cmake -S "${SRC}/iceoryx_meta" -B "${SRC}/build-pico-dds" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DROUDI_ENVIRONMENT=ON \
    -DBUILD_SHARED_LIBS=ON

cmake --build "${SRC}/build-pico-dds" --parallel
cmake --install "${SRC}/build-pico-dds"

echo "iceoryx installed to: ${PREFIX}"