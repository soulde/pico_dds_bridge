#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${ROOT}/.venv/bin/python"

if [[ ! -x "${PYTHON}" ]]; then
    echo "Python environment not found: ${PYTHON}" >&2
    echo "Run ./scripts/bootstrap.sh first." >&2
    exit 1
fi

export CYCLONEDDS_URI="file://${ROOT}/config/cyclonedds.xml"
export LD_LIBRARY_PATH="${ROOT}/.deps/xrobotoolkit/lib:${ROOT}/.deps/cyclonedds/lib:${ROOT}/.deps/iceoryx/lib:${ROOT}/.deps/simdjson/lib:${LD_LIBRARY_PATH:-}"

exec "${PYTHON}" "${ROOT}/scripts/pico_dds_retarget_node.py" "$@"
