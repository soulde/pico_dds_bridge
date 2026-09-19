#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="${ROOT}/.cache"

VERSION="${XROBO_PC_SERVICE_VERSION:-1.0.0}"
ARCH="$(dpkg --print-architecture)"
if [[ "${ARCH}" != "amd64" ]]; then
    echo "XRoboToolkit PC Service .deb currently supports amd64 only; found ${ARCH}." >&2
    exit 1
fi

URL="${XROBO_PC_SERVICE_URL:-https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v${VERSION}/XRoboToolkit_PC_Service_${VERSION}_ubuntu_22.04_amd64.deb}"
DEB="${CACHE}/XRoboToolkit_PC_Service_${VERSION}_ubuntu_22.04_amd64.deb"

mkdir -p "${CACHE}"

if [[ ! -s "${DEB}" ]]; then
    if ! command -v curl >/dev/null 2>&1; then
        echo "curl is required to download the XRoboToolkit PC Service package." >&2
        exit 1
    fi
    echo "Downloading XRoboToolkit PC Service ${VERSION}..."
    curl --fail --location --retry 3 --continue-at - "${URL}" --output "${DEB}"
else
    echo "Using cached package: ${DEB}"
fi

echo "Installing XRoboToolkit PC Service from ${DEB}..."
sudo dpkg --install "${DEB}" || {
    echo "Fixing package dependencies..."
    sudo apt-get update
    sudo apt-get install --yes --fix-broken
}

echo "XRoboToolkit PC Service installed. Start it from the Ubuntu applications menu before teleoperation."
