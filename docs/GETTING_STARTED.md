# Getting Started

Complete step-by-step guide to run the Hercules tailsitter simulation on a new machine.

**Estimated time:** 1–3 hours on first setup (mostly Isaac Sim download and ArduPilot build).  
**After setup:** launching the simulation takes one command (`./run.sh`).

---

## Table of Contents

1. [Prerequisites checklist](#1-prerequisites-checklist)
2. [Step 1 — System requirements](#step-1--system-requirements)
3. [Step 2 — Install Isaac Sim](#step-2--install-isaac-sim)
4. [Step 3 — Clone and build ArduPilot](#step-3--clone-and-build-ardupilot)
5. [Step 4 — Get the Hercules_Isaac package](#step-4--get-the-hercules_isaac-package)
6. [Step 5 — Run package setup](#step-5--run-package-setup)
7. [Step 6 — Launch the simulation](#step-6--launch-the-simulation)
8. [Step 7 — Verify it works](#step-7--verify-it-works)
9. [Optional configuration](#optional-configuration)
10. [Troubleshooting](#troubleshooting)

---

## 1. Prerequisites checklist

Before you begin, confirm you have:

| # | Requirement | Version | How to verify |
|---|-------------|---------|---------------|
| 1 | Ubuntu Linux | 22.04 LTS | `lsb_release -a` |
| 2 | NVIDIA GPU | CUDA-capable | `nvidia-smi` |
| 3 | NVIDIA driver | 550.x recommended | `nvidia-smi` (top-right driver version) |
| 4 | Isaac Sim | 5.1.0 | `[ISAACSIM_PATH]/python.sh --version` |
| 5 | ArduPilot source + SITL build | 4.4.x stable | `ls [ARDUPILOT_DIR]/build/sitl/bin/arduplane` |
| 6 | Hercules_Isaac package | this folder | `ls assets/Hercules/Hercules_v3_without_prop.usd` |

**Not required before `./setup.sh`:**

- Pegasus Simulator — bundled in this package; installed automatically
- Python packages (`numpy`, `scipy`, `pymavlink`, `pyyaml`) — installed by `./setup.sh`

**Important:** Do **not** source ROS (`source /opt/ros/...`) in the same terminal before running the simulation. ROS can break Isaac Sim's Python. `run.sh` clears ROS variables automatically, but a clean terminal is safest.

---

## Step 1 — System requirements

### Hardware

- NVIDIA GPU with a recent driver (tested with driver **550.163.01**)
- Enough disk space for Isaac Sim (~15 GB) and ArduPilot build artifacts (~2 GB)

### Operating system

- **Ubuntu 22.04 LTS** (64-bit)
- Windows and macOS are **not** supported for this workflow

### Verify GPU and driver

```bash
nvidia-smi
```

You should see your GPU listed and a driver version in the top-right corner. If this command fails, install or update your NVIDIA driver before continuing.

---

## Step 2 — Install Isaac Sim

### Download

Download Isaac Sim **5.1.0** standalone for Linux:

https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip

Official docs: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html

### Extract

```bash
mkdir -p ~/isaacsim
unzip isaac-sim-standalone-5.1.0-linux-x86_64.zip -d ~/isaacsim
```

Your install path will typically be `~/isaacsim` (or a subfolder inside it — use the directory that contains `python.sh`).

### Verify

```bash
ls ~/isaacsim/python.sh
~/isaacsim/python.sh --version
```

Both commands must succeed. Note your Isaac Sim path — you will need it in Step 5.

---

## Step 3 — Clone and build ArduPilot

This simulation uses **ArduPlane SITL** with the `plane-tailsitter` frame.

### Clone

```bash
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
git submodule update --init --recursive
```

Use a recent stable branch or tag (4.4.x). `master` also works.

### Install build dependencies

```bash
cd ~/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

If that script fails, install packages manually:

```bash
sudo apt install git make cmake python3-pip build-essential ccache g++ gawk \
  wget valgrind screen python3-pexpect pkg-config libtool libxml2-dev \
  libxslt1-dev xterm python3-wxgtk4.0
```

Log out and back in (or open a new terminal) after the prereq script if it updates your shell profile.

### Install Python packages

ArduPilot does **not** ship a root `requirements.txt`. The official installer is:

```bash
cd ~/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

That script installs system packages with `apt` and Python packages with `pip3 --user` for **system Python** (on Ubuntu 22.04 it does not create a venv).

**If you build inside conda or another venv** (e.g. `(ardupilot)`), `install-prereqs-ubuntu.sh` will not install into that environment. `./waf` uses whichever `python3` is active, so install the same pip packages there. On Ubuntu 22.04, ArduPilot's installer pulls this set (plus `pexpect`, which it normally gets from the `python3-pexpect` apt package):

```bash
cd ~/ardupilot
python3 -m pip install --upgrade pip packaging setuptools wheel
python3 -m pip install --upgrade \
  empy==3.3.4 pexpect ptyprocess \
  lxml pymavlink pyserial MAVProxy geocoder dronecan \
  flake8 junitparser wsproto tabulate pygame intelhex
```

Or install one package at a time (same as the upstream script):

```bash
for pkg in lxml pymavlink pyserial MAVProxy geocoder empy==3.3.4 ptyprocess dronecan \
           flake8 junitparser wsproto tabulate pygame intelhex pexpect; do
  python3 -m pip install --upgrade "$pkg"
done
```

Activate your conda/venv first if you use one.

> **Tip:** If you do not need a separate conda env, deactivate it and build with system `python3` after running `install-prereqs-ubuntu.sh` — that is the path ArduPilot tests on Ubuntu 22.04.

### Build ArduPlane SITL

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf plane
```

This may take several minutes on first build.

### Verify

`sim_vehicle.py` is part of the source tree and exists after clone; it does **not** confirm the SITL build succeeded. Check the built binary instead:

```bash
cd /path/to/ardupilot   # e.g. ~/ardupilot or ~/workspaces/ardupilot
ls build/sitl/bin/arduplane
```

This file must exist. Note your ArduPilot path — you will need it in Step 5.

---

## Step 4 — Get the Hercules_Isaac package

Copy, clone, or unzip the `Hercules_Isaac` folder to your machine.

```bash
cd /path/to/Hercules_Isaac
```

### Verify the Hercules asset is present

```bash
ls assets/Hercules/Hercules_v3_without_prop.usd
```

If this file is missing, the simulation cannot spawn the aircraft.

---

## Step 5 — Run package setup

This step is **one-time per machine**. It configures paths and installs Pegasus into Isaac Sim's Python.

```bash
cd /path/to/Hercules_Isaac
./setup.sh
```

### What `setup.sh` does

1. Detects or asks for your **Isaac Sim** install path
2. Detects or asks for your **ArduPilot** source path
3. Writes `.env.local` with machine-specific paths (not committed to git)
4. Updates `extensions/pegasus.simulator/config/configs.yaml`
5. Runs `pip install --editable` for Pegasus Simulator (clears conda/ROS env vars for Isaac Sim's Python)
6. Verifies the Pegasus pip package is installed (`pegasus-simulator OK`)

> **Conda users:** You can keep `(ardupilot)` active for ArduPilot builds, but `setup.sh` and `./run.sh` clear conda when calling Isaac Sim — do not rely on conda for the simulation runtime.

### Prompts you may see

```
==> Detected Isaac Sim: /home/you/isaacsim
Use this path? [Y/n]:
```

Press **Enter** to accept, or **n** to type a different path.

Same for ArduPilot:

```
==> Detected ArduPilot: /home/you/ardupilot
Use this path? [Y/n]:
```

### Success output

You should see:

```
pegasus-simulator OK

==> Setup complete.
  Run the simulation:  cd /path/to/Hercules_Isaac && ./run.sh
```

If setup fails, see [Troubleshooting](#troubleshooting) below.

---

## Step 6 — Launch the simulation

Every time you want to run the simulation:

```bash
cd /path/to/Hercules_Isaac
./run.sh
```

### What happens

1. `run.sh` loads `.env.local` and runs the same command as `isaacsim/python.sh examples/21_ardupilot_hercules_exp.py`
2. Puts your ArduPilot Python (conda) first on `PATH` for `sim_vehicle.py`
3. Isaac Sim starts in a window (not headless)
4. Default grid environment loads
5. Hercules aircraft spawns at 1.1 m altitude
6. ArduPilot SITL launches automatically (`plane-tailsitter` / ArduPlane)
7. Physics and MAVLink communication begin

### Stop the simulation

Close the Isaac Sim window, or press `Ctrl+C` in the terminal.

---

## Step 7 — Verify it works

After `./run.sh`, check:

| Check | Expected |
|-------|----------|
| Isaac Sim window | Opens without Python import errors |
| Terminal output | `✓ Fixed-wing simulation initialized successfully!` |
| Scene | Hercules model visible in the viewport |
| ArduPilot SITL | Starts in background (MAVProxy console may appear) |
| Simulation | Timeline plays; aircraft responds to physics |

If the model orientation looks wrong, try adjusting spawn pose (see [Optional configuration](#optional-configuration)).

---

## Optional configuration

Set environment variables **before** `./run.sh`:

```bash
# Debug aerodynamics without ArduPilot control inputs
PEGASUS_FIXEDWING_SIM_MODE=thrust_only ./run.sh

# Change spawn orientation (degrees, XYZ Euler)
PEGASUS_FIXEDWING_SPAWN_EULER_DEG="0,0,0" ./run.sh

# Use standard fixed-wing frame instead of tailsitter
PEGASUS_ARDUPILOT_FRAME=plane ./run.sh
```

Full reference: [CONFIGURATION.md](CONFIGURATION.md)

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `Missing .env.local` | Setup not run | Run `./setup.sh` |
| `Hercules USD not found` | Asset missing from package | Restore `assets/Hercules/Hercules_v3_without_prop.usd` |
| `Isaac Sim python.sh not found` | Wrong Isaac Sim path | Re-run `./setup.sh` with correct path |
| `ArduPilot sim_vehicle.py not found` (from `./setup.sh`) | Wrong ArduPilot path or repo not cloned | Point `./setup.sh` at your clone (e.g. `~/workspaces/ardupilot`); verify `build/sitl/bin/arduplane` exists |
| `ModuleNotFoundError: pxr` during setup | Old verify step or conda/ROS conflict | Re-run `./setup.sh` (updated script verifies pip install only); deactivate conda if it persists |
| `import pegasus` fails at runtime | Pegasus not installed in Isaac Python | Re-run `./setup.sh` |
| `you need to install empy` / `pexpect` during `./waf plane` | Pip packages missing from active Python env (common with conda) | Run the pip install block in [Install Python build helpers](#install-python-build-helpers), then re-run `./waf plane` |
| SITL fails to start | ArduPlane not built or wrong Python for `sim_vehicle.py` | Run `./waf plane`; re-run `./setup.sh` so `ARDUPILOT_PYTHON` points at your conda env |
| `./run.sh` fails but `isaacsim/python.sh examples/...` works | `run.sh` could not find ArduPilot Python | Add `ARDUPILOT_PYTHON=/path/to/conda/envs/ardupilot/bin/python3` to `.env.local` |
| Python/library errors after ROS | ROS env vars conflict | Open a fresh terminal; do not `source /opt/ros/...` before `./run.sh` |
| Wrong aircraft orientation | USD axis mismatch | Set `PEGASUS_FIXEDWING_SPAWN_EULER_DEG` (see [CONFIGURATION.md](CONFIGURATION.md)) |

### Re-run setup after moving paths

If you move Isaac Sim or ArduPilot to a new location:

```bash
cd /path/to/Hercules_Isaac
./setup.sh
```

---

## Next steps

| Topic | Document |
|-------|----------|
| Environment variables | [CONFIGURATION.md](CONFIGURATION.md) |
| Aerodynamic parameters | [AIRFRAME.md](AIRFRAME.md) |
| Software versions | [REQUIREMENTS.md](REQUIREMENTS.md) |
