#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${ROOT}/.venv"

python3 -m venv "${VENV}"
"${VENV}/bin/pip" install --upgrade pip
export CYCLONEDDS_HOME="${ROOT}/.deps/cyclonedds"
export LD_LIBRARY_PATH="${ROOT}/.deps/xrobotoolkit/lib:${ROOT}/.deps/cyclonedds/lib:${ROOT}/.deps/iceoryx/lib:${LD_LIBRARY_PATH:-}"
"${VENV}/bin/pip" install \
	git+https://github.com/eclipse-cyclonedds/cyclonedds-python
"${VENV}/bin/pip" install -e "${ROOT}/python"
"${VENV}/bin/pip" install -e "${ROOT}/third_party/GMR"

echo "Python SDK installed in: ${VENV}"
echo "Activate with: source ${VENV}/bin/activate"
