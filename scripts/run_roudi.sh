#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROUDI="${ROOT}/.deps/iceoryx/bin/iox-roudi"

if [[ ! -x "${ROUDI}" ]]; then
    echo "[roudi] not found: ${ROUDI}" >&2
    echo "Run ./scripts/install_iceoryx.sh first." >&2
    exit 1
fi

export LD_LIBRARY_PATH="${ROOT}/.deps/iceoryx/lib:${ROOT}/.deps/cyclonedds/lib:${LD_LIBRARY_PATH:-}"
exec "${ROUDI}" "$@"