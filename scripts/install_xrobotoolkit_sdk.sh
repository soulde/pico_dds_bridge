#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="${ROOT}/.cache"
PREFIX="${ROOT}/.deps/xrobotoolkit"

XROBO_REPO="${XROBO_REPO:-https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git}"
XROBO_REF="${XROBO_REF:-main}"

mkdir -p "${CACHE}" "${PREFIX}/include" "${PREFIX}/lib"

SRC="${CACHE}/XRoboToolkit-PC-Service"

if [[ ! -d "${SRC}/.git" ]]; then
    git clone "${XROBO_REPO}" "${SRC}"
fi

git -C "${SRC}" fetch --all --tags --prune
if git -C "${SRC}" show-ref --verify --quiet "refs/remotes/origin/${XROBO_REF}"; then
    git -C "${SRC}" checkout -B "${XROBO_REF}" "origin/${XROBO_REF}"
else
    git -C "${SRC}" checkout "${XROBO_REF}"
fi

SDK_DIR="${SRC}/RoboticsService/PXREARobotSDK"

if [[ ! -d "${SDK_DIR}" ]]; then
    echo "PXREARobotSDK directory not found: ${SDK_DIR}" >&2
    exit 1
fi

pushd "${SDK_DIR}" >/dev/null
bash build.sh
popd >/dev/null

HEADER="${SDK_DIR}/PXREARobotSDK.h"
LIB="${SDK_DIR}/build/libPXREARobotSDK.so"

if [[ ! -f "${HEADER}" ]]; then
    echo "SDK header not produced: ${HEADER}" >&2
    exit 1
fi

if [[ ! -f "${LIB}" ]]; then
    echo "SDK library not produced: ${LIB}" >&2
    exit 1
fi

cp "${HEADER}" "${PREFIX}/include/"
cp "${LIB}" "${PREFIX}/lib/"

# PXREARobotSDK.h in upstream builds may reference the bundled nlohmann headers.
if [[ -d "${SDK_DIR}/nlohmann" ]]; then
    rm -rf "${PREFIX}/include/nlohmann"
    cp -r "${SDK_DIR}/nlohmann" "${PREFIX}/include/nlohmann"
fi

echo "XRoboToolkit Robot SDK installed to: ${PREFIX}"
echo
echo "Dynamic dependencies:"
ldd "${PREFIX}/lib/libPXREARobotSDK.so" || true
