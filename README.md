# Hercules Isaac

Hercules fixed-wing / tailsitter simulation in [Isaac Sim](https://developer.nvidia.com/isaac-sim) with [ArduPilot](https://ardupilot.org/) SITL over MAVLink, powered by [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/).

![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.1.0-76B900?logo=nvidia)
![Pegasus](https://img.shields.io/badge/Pegasus%20Simulator-5.1.0-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)
![ArduPilot](https://img.shields.io/badge/ArduPilot-4.4.x-0078D4)
![License](https://img.shields.io/badge/License-BSD--3--Clause-blue)

## Table of Contents

- [Requirements](#requirements)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Features](#features)
- [Configuration](#configuration)
- [Documentation](#documentation)
- [Project Structure](#project-structure)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Requirements

You need these installed **before** running `./setup.sh`:

| Component | Version | Verify |
|-----------|---------|--------|
| Ubuntu | 22.04 LTS | `lsb_release -a` |
| NVIDIA GPU + driver | CUDA-capable, **550.x** recommended | `nvidia-smi` |
| [Isaac Sim](https://developer.nvidia.com/isaac-sim) | **5.1.0** | `~/isaacsim/python.sh --version` |
| [ArduPilot](https://github.com/ArduPilot/ardupilot) SITL | **4.4.x** stable | `ls /path/to/ardupilot/build/sitl/bin/arduplane` |

**Installed automatically by `./setup.sh`:** Pegasus Simulator 5.1.0, `numpy`, `scipy`, `pymavlink`, `pyyaml`.

> **Notes**
> - Use Isaac Sim's bundled Python (`python.sh`), not system Python.
> - Do not source ROS (`source /opt/ros/...`) before `./run.sh` — it can break imports. `run.sh` clears ROS variables automatically.
> - Full version matrix: [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md)

---

## Getting Started

One-time setup on a new machine. Estimated time: 1–3 hours (mostly Isaac Sim download and ArduPilot build).

For screenshots, verification steps, and extended troubleshooting, see [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md).

### 1. Install Isaac Sim

1. Download [Isaac Sim 5.1.0 for Linux](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip).
2. Extract to your install location (e.g. `~/isaacsim`).
3. Verify:

```bash
~/isaacsim/python.sh --version
```

### 2. Build ArduPilot SITL

```bash
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
git submodule update --init --recursive
./Tools/environment_install/install-prereqs-ubuntu.sh -y
# conda/venv only — install ArduPilot pip deps into the same Python as ./waf (see docs/GETTING_STARTED.md):
python3 -m pip install --upgrade empy==3.3.4 pexpect ptyprocess lxml pymavlink pyserial MAVProxy geocoder dronecan flake8 junitparser wsproto tabulate pygame intelhex
./waf configure --board sitl
./waf plane
```

Verify (use your ArduPilot clone path — e.g. `~/ardupilot` or `~/workspaces/ardupilot`):

```bash
cd /path/to/ardupilot
ls build/sitl/bin/arduplane
```

### 3. Set up this package

```bash
cd /path/to/Hercules_Isaac
./setup.sh
```

`setup.sh` will detect (or ask for) your Isaac Sim and ArduPilot paths, write `.env.local` (including `ARDUPILOT_PYTHON` for SITL), configure Pegasus, and install dependencies into Isaac Sim's Python. You should see `pegasus-simulator OK` when it finishes.

### 4. Launch the simulation

```bash
./run.sh
```

Isaac Sim opens, spawns Hercules, and connects to ArduPilot SITL automatically.

---

## Usage

### Run

```bash
cd /path/to/Hercules_Isaac
./run.sh
```

### What to expect

- Isaac Sim window with the default grid environment
- Hercules aircraft at the spawn position
- Terminal message: `✓ Fixed-wing simulation initialized successfully!`
- ArduPilot SITL running in the background (`plane-tailsitter` frame)

### Stop

Close the Isaac Sim window or press `Ctrl+C` in the terminal.

### Debug modes

```bash
# Aerodynamics debugging (no ArduPilot control)
PEGASUS_FIXEDWING_SIM_MODE=thrust_only ./run.sh

# Frame debugging (manual forces, no aerodynamics)
PEGASUS_FIXEDWING_SIM_MODE=manual ./run.sh

# Adjust spawn orientation if the model appears tilted
PEGASUS_FIXEDWING_SPAWN_EULER_DEG="0,0,0" ./run.sh
```

See [docs/CONFIGURATION.md](docs/CONFIGURATION.md) for all environment variables.

---

## Features

- **Hercules USD asset** — `Hercules_v3_without_prop.usd` included
- **Fixed-wing aerodynamics** — lift, drag, and stability derivatives via `FixedWingConfig`
- **ArduPilot tailsitter** — auto-launches ArduPlane SITL with `plane-tailsitter` frame
- **Pegasus backend** — MAVLink integration between Isaac Sim and ArduPilot
- **Simulation modes** — `autonomous`, `thrust_only`, and `manual`

---

## Configuration

| Topic | Document |
|-------|----------|
| Environment variables | [docs/CONFIGURATION.md](docs/CONFIGURATION.md) |
| Aerodynamic parameters | [docs/AIRFRAME.md](docs/AIRFRAME.md) |
| Software versions | [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md) |
| Full installation guide | [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) |

---

## Documentation

| File | Description |
|------|-------------|
| [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) | Complete setup guide with verification and troubleshooting |
| [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md) | Tested software versions and dependencies |
| [docs/CONFIGURATION.md](docs/CONFIGURATION.md) | Environment variables and config file reference |
| [docs/AIRFRAME.md](docs/AIRFRAME.md) | Hercules aerodynamic and propulsion parameters |

---

## Project Structure

```
Hercules_Isaac/
├── README.md                          # This file
├── setup.sh                           # One-time setup (paths + pip install)
├── run.sh                             # Launch simulation
├── .env.local                         # Generated by setup.sh (git-ignored)
├── assets/Hercules/
│   └── Hercules_v3_without_prop.usd   # Hercules 3D model
├── docs/
│   ├── GETTING_STARTED.md             # Full installation guide
│   ├── REQUIREMENTS.md                # Version matrix
│   ├── CONFIGURATION.md               # Env vars reference
│   └── AIRFRAME.md                    # Aerodynamic parameters
├── examples/
│   └── 21_ardupilot_hercules_exp.py   # Main simulation script
└── extensions/pegasus.simulator/      # Pegasus Simulator (installed by setup.sh)
    └── config/configs.yaml
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `Missing .env.local` | Run `./setup.sh` first |
| `Hercules USD not found` | Keep `assets/Hercules/Hercules_v3_without_prop.usd` in place |
| `Isaac Sim python.sh not found` | Re-run `./setup.sh` with the correct Isaac Sim path |
| `you need to install empy` / `pexpect` during `./waf plane` | Install ArduPilot pip deps into the active Python env (see [Getting Started §2](#2-build-ardupilot-sitl)), then re-run `./waf plane` |
| `arduplane` not found / ArduPilot path wrong | Build SITL: `./waf plane` in your ArduPilot directory; re-run `./setup.sh` with the correct path |
| `import pegasus` fails | Re-run `./setup.sh` |
| SITL fails to start | Verify ArduPilot build; test `sim_vehicle.py` manually |
| ROS / Python conflicts | Use a fresh terminal without `source /opt/ros/...` |
| Wrong aircraft orientation | Set `PEGASUS_FIXEDWING_SPAWN_EULER_DEG` (see [CONFIGURATION.md](docs/CONFIGURATION.md)) |

More detail: [docs/GETTING_STARTED.md#troubleshooting](docs/GETTING_STARTED.md#troubleshooting)

---

## License

Pegasus Simulator is [BSD-3-Clause](extensions/pegasus.simulator/docs/README.md).
