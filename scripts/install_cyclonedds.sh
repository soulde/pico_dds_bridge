#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="${ROOT}/.cache"
PREFIX="${ROOT}/.deps/cyclonedds"

CYCLONEDDS_REPO="${CYCLONEDDS_REPO:-https://github.com/eclipse-cyclonedds/cyclonedds.git}"
CYCLONEDDS_REF="${CYCLONEDDS_REF:-11.0.1}"

mkdir -p "${CACHE}" "${PREFIX}"

SRC="${CACHE}/cyclonedds"

if [[ ! -d "${SRC}/.git" ]]; then
    git clone "${CYCLONEDDS_REPO}" "${SRC}"
fi

git -C "${SRC}" fetch --all --tags --prune
git -C "${SRC}" checkout "${CYCLONEDDS_REF}"

cmake -S "${SRC}" -B "${SRC}/build-pico-dds" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF

cmake --build "${SRC}/build-pico-dds" --parallel
cmake --install "${SRC}/build-pico-dds"

echo "CycloneDDS installed to: ${PREFIX}"
