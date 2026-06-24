# Requirements

Tested software and library versions for the Hercules Isaac Sim package.

> **Installation guide:** For step-by-step setup instructions, see [GETTING_STARTED.md](GETTING_STARTED.md).

## System

| Component | Version | Notes |
|-----------|---------|-------|
| **OS** | Ubuntu 22.04 LTS | Linux only; Windows not tested |
| **NVIDIA driver** | 550.163.01 | Tested with Pegasus Simulator upstream |
| **GPU** | NVIDIA (CUDA-capable) | Required for Isaac Sim |

## Core software

| Component | Version | Install |
|-----------|---------|---------|
| **Isaac Sim** | 5.1.0 | [Standalone download](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip) |
| **Pegasus Simulator** | 5.1.0 | Included in this package; installed by `./setup.sh` |
| **ArduPilot** | 4.4.x (stable) | [ArduPilot repo](https://github.com/ArduPilot/ardupilot); SITL built with `./waf plane` |

Isaac Sim 5.1+ should work; this package was developed against **5.1.0**.

For ArduPilot, use a recent stable release (upstream Pegasus ArduPilot docs reference **ArduCopter 4.4.0**). For this Hercules tailsitter example, build **ArduPlane** SITL:

```bash
cd /path/to/ardupilot
git checkout master   # or a recent stable tag
./Tools/environment_install/install-prereqs-ubuntu.sh -y
# conda/venv only — same Python as ./waf (full Ubuntu 22.04 pip set from install-prereqs-ubuntu.sh):
python3 -m pip install --upgrade empy==3.3.4 pexpect ptyprocess lxml pymavlink pyserial MAVProxy geocoder dronecan flake8 junitparser wsproto tabulate pygame intelhex
./waf configure --board sitl
./waf plane
```

## Python environment

Use **Isaac Sim's bundled Python** (`python.sh`). Do not use system Python or a separate virtualenv unless you follow [Isaac Sim Python setup](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.html).

Isaac Sim 5.1 ships with Python 3.11.

## Python packages (installed by `./setup.sh`)

Installed into Isaac Sim's Python via `pip install --editable extensions/pegasus.simulator`:

| Package | Purpose |
|---------|---------|
| `numpy` | Numerics |
| `scipy` | Rotations, linear algebra |
| `pymavlink` | ArduPilot MAVLink backend |
| `pyyaml` | Pegasus config files |

Additional ArduPilot SITL tools (install in your user environment or ArduPilot venv if needed):

| Package | Purpose |
|---------|---------|
| `MAVProxy` | ArduPilot SITL console (auto-launched by backend) |
| `pymavlink` | MAVLink protocol |
| `empy==3.3.4`, `pexpect`, `ptyprocess`, `lxml`, `pymavlink`, `pyserial`, `MAVProxy`, `geocoder`, `dronecan`, `flake8`, `junitparser`, `wsproto`, `tabulate`, `pygame`, `intelhex` | ArduPilot SITL pip packages on Ubuntu 22.04 (installed by `install-prereqs-ubuntu.sh` for system Python; install manually in conda/venv — see [GETTING_STARTED.md](GETTING_STARTED.md)) |

## ArduPilot system packages

If SITL build fails, install ArduPilot prerequisites:

```bash
sudo apt install git make cmake python3-pip build-essential ccache g++ gawk \
  wget valgrind screen python3-pexpect pkg-config libtool libxml2-dev \
  libxslt1-dev xterm python3-wxgtk4.0
```

## Version summary (badges)

| | |
|---|---|
| Isaac Sim | 5.1.0 |
| Pegasus Simulator | 5.1.0 |
| Ubuntu | 22.04 LTS |
| NVIDIA driver | 550.163.01 |
| ArduPilot | 4.4.x stable (ArduPlane SITL) |
