# Configuration

Environment variables and config file options for the Hercules Isaac Sim simulation.

> **First-time setup:** See [GETTING_STARTED.md](GETTING_STARTED.md) for prerequisites and installation steps.

## Config file

After running `./setup.sh`, the ArduPilot path is written to:

```
extensions/pegasus.simulator/config/configs.yaml
```

Key field:

```yaml
ardupilot_dir: /path/to/ardupilot
```

Re-run `./setup.sh` to change this path, or edit the file manually.

## Environment variables

All variables are optional unless noted. Set them before `./run.sh` or prefix a single command.

### Simulation mode

| Variable | Default | Values | Description |
|----------|---------|--------|-------------|
| `PEGASUS_FIXEDWING_SIM_MODE` | `autonomous` | `autonomous`, `thrust_only`, `manual` | Flight dynamics mode |

- **autonomous** — Full aerodynamics + ArduPilot control inputs (production mode).
- **thrust_only** — User thrust only; aerodynamics still computed (aero debugging).
- **manual** — UI force/torque only; no aerodynamics (frame debugging).

Example:

```bash
PEGASUS_FIXEDWING_SIM_MODE=thrust_only ./run.sh
```

### Spawn pose

| Variable | Default | Description |
|----------|---------|-------------|
| `PEGASUS_FIXEDWING_SPAWN_EULER_DEG` | `0,-90,0` | Spawn orientation in degrees (XYZ Euler) |

Per ArduPilot tailsitter docs, the default orientation corresponds to fixed-wing flight attitude. Override if your USD forward axis differs.

Example:

```bash
PEGASUS_FIXEDWING_SPAWN_EULER_DEG="0,0,0" ./run.sh
```

### ArduPilot SITL

| Variable | Default | Description |
|----------|---------|-------------|
| `PEGASUS_ARDUPILOT_FRAME` | `plane-tailsitter` | ArduPilot vehicle model passed to SITL |
| `PEGASUS_ARDUPILOT_VEHICLE` | `ArduPlane` | SITL vehicle binary |

Example (standard fixed-wing instead of tailsitter):

```bash
PEGASUS_ARDUPILOT_FRAME=plane PEGASUS_ARDUPILOT_VEHICLE=ArduPlane ./run.sh
```

### USD layout hints

Use these when the Hercules mesh is nested inside the USD hierarchy.

| Variable | Default | Description |
|----------|---------|-------------|
| `PEGASUS_VEHICLE_USD_INNER_PREFIX` | *(empty)* | Inner USD prim prefix (e.g. `Hercules`) |
| `PEGASUS_VEHICLE_ROBOT_ARTICULATION_SUFFIX` | *(empty)* | Articulation root suffix (e.g. `body`) |

Example:

```bash
PEGASUS_VEHICLE_USD_INNER_PREFIX=Hercules \
PEGASUS_VEHICLE_ROBOT_ARTICULATION_SUFFIX=body \
./run.sh
```

### Control sign overrides

| Variable | Default | Description |
|----------|---------|-------------|
| `PEGASUS_FW_SIGNS` | *(empty)* | Comma-separated signs: Aileron, Elevator, Throttle, Rudder |

Example:

```bash
PEGASUS_FW_SIGNS="1,-1,1,-1" ./run.sh
```

### Propeller joint

| Variable | Default | Description |
|----------|---------|-------------|
| `PEGASUS_FIXEDWING_PROPELLER_JOINT_NAME` | *(disabled)* | Revolute joint name for visual prop spin; leave unset to disable |

Example:

```bash
PEGASUS_FIXEDWING_PROPELLER_JOINT_NAME=propeller_joint ./run.sh
```

## Local environment file

`setup.sh` writes `.env.local` (git-ignored) with machine-specific paths:

```bash
ISAACSIM_PATH=...
ISAACSIM_PYTHON=...
ISAACSIM=...
ARDUPILOT_DIR=...
```

`run.sh` sources this file automatically. Do not commit `.env.local`.

## ArduPilot build (first time)

If SITL has not been built yet:

```bash
cd /path/to/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf plane
```

The simulation script auto-launches ArduPlane SITL with the configured frame.
