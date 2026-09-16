#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"${ROOT}/scripts/install_deps_ubuntu.sh"
"${ROOT}/scripts/install_iceoryx.sh"
"${ROOT}/scripts/install_cyclonedds.sh"
"${ROOT}/scripts/install_simdjson.sh"
"${ROOT}/scripts/install_xrobotoolkit_sdk.sh"
"${ROOT}/scripts/install_python.sh"

echo
echo "Bootstrap complete."
echo "Next:"
echo "  ./scripts/build.sh"
echo "  ./scripts/run.sh"
