#!/usr/bin/env bash
# One-time setup for Hercules Isaac Sim package.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ROOT}/.env.local"
CONFIG_YAML="${ROOT}/extensions/pegasus.simulator/config/configs.yaml"
HERCULES_USD="${ROOT}/assets/Hercules/Hercules_v3_without_prop.usd"
PEGASUS_EXT="${ROOT}/extensions/pegasus.simulator"

info()  { echo "==> $*"; }
warn()  { echo "WARNING: $*" >&2; }
error() { echo "ERROR: $*" >&2; exit 1; }

# Isaac Sim's python.sh must not inherit conda or ROS env vars (breaks pxr/omni at runtime).
clear_isaacsim_env() {
    if [[ -n "${CONDA_PREFIX:-}" ]]; then
        warn "Conda env '${CONDA_DEFAULT_ENV:-active}' detected — clearing conda/ROS vars for Isaac Sim commands."
        warn "Keep conda for ArduPilot builds; use a clean shell for ./run.sh, or deactivate conda first."
    fi

    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
    unset PYTHONHOME PYTHONEXE
    unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH

    if [[ -n "${PATH:-}" ]]; then
        PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda|anaconda|/opt/ros/' | paste -sd':' - || true)"
        export PATH
    fi

    if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
        for ros_path in /opt/ros/humble /opt/ros/jazzy /opt/ros/iron; do
            LD_LIBRARY_PATH="$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "^${ros_path}" | paste -sd':' - || true)"
        done
        export LD_LIBRARY_PATH
    fi
}

prompt_path() {
    local label="$1"
    local default="${2:-}"
    local result=""

    if [[ -n "$default" ]]; then
        read -r -p "${label} [${default}]: " result
        result="${result:-$default}"
    else
        read -r -p "${label}: " result
    fi
    echo "$result"
}

find_isaacsim() {
    local candidates=()

    if [[ -n "${ISAACSIM_PATH:-}" && -x "${ISAACSIM_PATH}/python.sh" ]]; then
        echo "$ISAACSIM_PATH"
        return 0
    fi

    candidates+=(
        "${HOME}/isaacsim"
        "${HOME}/isaac_sim"
        "${HOME}/IsaacSim"
    )

    local path
    for path in "${candidates[@]}"; do
        if [[ -x "${path}/python.sh" ]]; then
            echo "$path"
            return 0
        fi
    done
    return 1
}

find_ardupilot() {
    local candidates=()

    if [[ -n "${ARDUPILOT_DIR:-}" && -f "${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py" ]]; then
        echo "$ARDUPILOT_DIR"
        return 0
    fi

    candidates+=(
        "${HOME}/ardupilot"
        "${HOME}/workspaces/ardupilot"
        "${HOME}/workspace/ardupilot"
    )

    local path
    for path in "${candidates[@]}"; do
        if [[ -f "${path}/Tools/autotest/sim_vehicle.py" ]]; then
            echo "$path"
            return 0
        fi
    done
    return 1
}

validate_isaacsim() {
    local path="$1"
    [[ -x "${path}/python.sh" ]] || error "Isaac Sim python.sh not found at: ${path}/python.sh"
    [[ -x "${path}/isaac-sim.sh" ]] || warn "isaac-sim.sh not found at ${path}/isaac-sim.sh (simulation may still work in standalone mode)"
}

validate_ardupilot() {
    local path="$1"
    [[ -f "${path}/Tools/autotest/sim_vehicle.py" ]] || error "ArduPilot sim_vehicle.py not found at: ${path}/Tools/autotest/sim_vehicle.py"
}

detect_ardupilot_python() {
    if [[ -n "${ARDUPILOT_PYTHON:-}" && -x "${ARDUPILOT_PYTHON}" ]]; then
        echo "$ARDUPILOT_PYTHON"
        return 0
    fi
    if [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/bin/python3" ]]; then
        echo "${CONDA_PREFIX}/bin/python3"
        return 0
    fi
    if [[ -x "${HOME}/miniconda3/envs/ardupilot/bin/python3" ]]; then
        echo "${HOME}/miniconda3/envs/ardupilot/bin/python3"
        return 0
    fi
    command -v python3
}

info "Hercules Isaac setup"
info "Package root: ${ROOT}"

# --- Isaac Sim ---
ISAACSIM_PATH=""
if ISAACSIM_PATH="$(find_isaacsim)"; then
    info "Detected Isaac Sim: ${ISAACSIM_PATH}"
    read -r -p "Use this path? [Y/n]: " confirm
    if [[ "${confirm,,}" == "n" ]]; then
        ISAACSIM_PATH=""
    fi
fi

if [[ -z "$ISAACSIM_PATH" ]]; then
    ISAACSIM_PATH="$(prompt_path "Enter Isaac Sim install directory" "${HOME}/isaacsim")"
fi
validate_isaacsim "$ISAACSIM_PATH"

ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

# --- ArduPilot ---
ARDUPILOT_DIR=""
if ARDUPILOT_DIR="$(find_ardupilot)"; then
    info "Detected ArduPilot: ${ARDUPILOT_DIR}"
    read -r -p "Use this path? [Y/n]: " confirm
    if [[ "${confirm,,}" == "n" ]]; then
        ARDUPILOT_DIR=""
    fi
fi

if [[ -z "$ARDUPILOT_DIR" ]]; then
    ARDUPILOT_DIR="$(prompt_path "Enter ArduPilot source directory" "${HOME}/ardupilot")"
fi
validate_ardupilot "$ARDUPILOT_DIR"

ARDUPILOT_PYTHON="$(detect_ardupilot_python)"
[[ -n "$ARDUPILOT_PYTHON" && -x "$ARDUPILOT_PYTHON" ]] || error "Could not find python3 for ArduPilot SITL (sim_vehicle.py)"
info "ArduPilot Python: ${ARDUPILOT_PYTHON}"

# --- Hercules asset ---
[[ -f "$HERCULES_USD" ]] || error "Hercules USD not found: ${HERCULES_USD}"

# --- Write .env.local ---
info "Writing ${ENV_FILE}"
cat > "$ENV_FILE" <<EOF
# Generated by setup.sh — do not commit this file.
ISAACSIM_PATH=${ISAACSIM_PATH}
ISAACSIM_PYTHON=${ISAACSIM_PYTHON}
ISAACSIM=${ISAACSIM}
ARDUPILOT_DIR=${ARDUPILOT_DIR}
ARDUPILOT_PYTHON=${ARDUPILOT_PYTHON}
EOF

# --- Update configs.yaml ---
info "Updating ardupilot_dir in configs.yaml"
sed -i "s|^ardupilot_dir:.*|ardupilot_dir: ${ARDUPILOT_DIR}|" "$CONFIG_YAML"

# --- Install Pegasus ---
info "Installing Pegasus Simulator (editable)"
clear_isaacsim_env
export ISAACSIM_PATH
"${ISAACSIM_PYTHON}" -m pip install --editable "${PEGASUS_EXT}"

# --- Verify ---
# pegasus.simulator imports pxr/omni and only works inside a SimulationApp — verify pip install instead.
info "Verifying Pegasus pip install"
"${ISAACSIM_PYTHON}" -m pip show pegasus-simulator >/dev/null
"${ISAACSIM_PYTHON}" -c "import pymavlink; print('pegasus-simulator OK')"

echo
info "Setup complete."
echo "  Run the simulation:  cd ${ROOT} && ./run.sh"
echo "  Re-run setup later:  ./setup.sh"
