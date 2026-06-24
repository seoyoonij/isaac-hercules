#!/usr/bin/env bash
# Launch the Hercules Isaac Sim simulation.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ROOT}/.env.local"
SCRIPT="examples/21_ardupilot_hercules_exp.py"

error() { echo "ERROR: $*" >&2; exit 1; }

[[ -f "$ENV_FILE" ]] || error "Missing ${ENV_FILE}. Run ./setup.sh first."

# shellcheck source=/dev/null
source "$ENV_FILE"

[[ -x "$ISAACSIM_PYTHON" ]] || error "Isaac Sim Python not found: ${ISAACSIM_PYTHON}"
[[ -f "${ROOT}/${SCRIPT}" ]] || error "Simulation script not found: ${ROOT}/${SCRIPT}"

# Match the working manual launch: Isaac Sim via python.sh, SITL via python3 on PATH.
# Put the ArduPilot conda/venv first, but do not export ARDUPILOT_PYTHON (Pegasus uses python3).
if [[ -n "${ARDUPILOT_PYTHON:-}" && -x "$ARDUPILOT_PYTHON" ]]; then
    export PATH="$(dirname "$ARDUPILOT_PYTHON"):$PATH"
fi
unset ARDUPILOT_PYTHON

export ISAACSIM_PATH
cd "$ROOT"

# Leftover sim_vehicle / arduplane from a crashed run steals UDP port 9002 and breaks JSON FDM sync.
kill_stale_sitl() {
    local pattern
    for pattern in sim_vehicle.py arduplane arducopter mavproxy.py MAVProxy; do
        pkill -TERM -f "$pattern" 2>/dev/null || true
    done
    sleep 0.5
}
kill_stale_sitl

exec "$ISAACSIM_PYTHON" "$SCRIPT" "$@"
