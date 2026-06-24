# Pegasus Simulator — `logic` Package Report

**Scope:** `extensions/pegasus.simulator/pegasus/simulator/logic/`  
**Purpose:** Architecture and responsibility map for simulation logic (vehicles, control backends, sensors, globe integration, and ancillary systems).

---

## 1. Executive summary

The `logic` package is the core of the Pegasus Simulator extension. It bridges **Isaac Sim** (`isaacsim.core`, PhysX, USD, dynamic control) with **high-level vehicle behavior**: spawning articulated robots, stepping physics with custom forces, synthesizing sensor streams, and exchanging data with **PX4**, **ArduPilot** (MAVLink), or **ROS 2**.

Design themes:

- **Singleton facades** (`PegasusInterface`, `VehicleManager`, `PeopleManager`) for global coordination.
- **Composition over deep inheritance** for vehicles: base `Vehicle` wires callbacks; concrete types (`Multirotor`, `FixedWing`) implement `update()` physics.
- **Backend abstraction** for control/telemetry: each backend implements MAVLink/ROS-facing behavior and exposes `input_reference()` for actuation.
- **Frame conventions:** simulation uses **ENU** inertial and **FLU** body; `State` and `rotations.py` convert to **NED/FRD** for autopilots.

---

## 2. Directory layout and roles

| Path | Role |
|------|------|
| `__init__.py` | Re-exports `PegasusInterface` as the public entry point for `pegasus.simulator.logic`. |
| `interface/pegasus_interface.py` | Singleton: world lifecycle, config (`configs.yaml`), environment loading, vehicle registry access, path/airframe persistence. |
| `vehicle_manager.py` | Singleton registry: `stage_prefix` → vehicle instance; thread-safe single instance. |
| `state.py` | `State`: pose, velocities, acceleration; ENU/FLU storage with NED/FRD getters for FCs. |
| `rotations.py` | Fixed `scipy` rotations: ENU↔NED, FLU↔FRD. |
| `vehicles/vehicle.py` | Base class: USD spawn, Isaac `Robot`, physics/render callbacks, sensors, backends, tracking camera prim. |
| `vehicles/multirotor.py` | Quad-style multirotor: thrust curve, drag, per-rotor forces, propeller visuals. |
| `vehicles/multirotors/iris.py` | Preset `MultirotorConfig` / `Iris` wrapper for the Iris asset. |
| `vehicles/fixedwing.py` | Fixed-wing: aerodynamic coefficients, control surfaces, propeller thrust, multiple simulation modes, optional debug/UI hooks. |
| `backends/backend.py` | Abstract `Backend` / `BackendConfig`: `update_state`, `update_sensor`, `input_reference`, `update`, lifecycle. |
| `backends/px4_mavlink_backend.py` | PX4 SITL / MAVLink integration. |
| `backends/ardupilot_mavlink_backend.py` | ArduPilot SITL / MAVLink; configurable rotor count and scaling for motors. |
| `backends/ros2_backend.py` | ROS 2 pub/sub (optional import if ROS 2 stack present). |
| `backends/tools/` | Launch helpers (`px4_launch_tool`, `ardupilot_launch_tool`, `ArduPilotPlugin`). |
| `sensors/` | Physics-step sensors: `Sensor` base, IMU, barometer, magnetometer (+ geo utils), GPS. |
| `graphical_sensors/` | Render-step sensors: `GraphicalSensor`, monocular camera, LiDAR. |
| `dynamics/` | `Drag` / `LinearDrag` for body-frame damping-style forces. |
| `thrusters/` | `ThrustCurve` API; `QuadraticThrustCurve` (rotor ω² thrust, rolling moment, saturation). |
| `graphs/` | OmniGraph hooks: `Graph` base, `ros2_camera_graph`. |
| `people/` | `Person`, `PersonController`, `LinePersonController` for pedestrian-style agents. |
| `people_manager.py` | Singleton registry for people; nav mesh rebuild coordination. |
| `people_backends/` | `PeopleBackend`, `ROS2PeopleBackend` — mirror backend pattern for human agents. |
| `cesium_globe.py` | Optional Cesium for Omniverse: globe anchor, ion token sync with Pegasus config, tileset presets. |
| `cesium_physics.py` | PhysX collision setup for Cesium tilesets (reparenting, collision APIs, fallback ground). |

---

## 3. Runtime architecture

### 3.1 Singletons and registries

- **`PegasusInterface`**: Owns the Isaac `World` reference (once `initialize_world()` is used), reads/writes `configs.yaml` for PX4/ArduPilot paths, airframes, global origin lat/lon/alt, and Cesium-related settings. Exposes `vehicle_manager` and helpers to clear/load environments.
- **`VehicleManager`**: Every `Vehicle` registers on construction and unregisters on destruction. Lookup by full stage path string.
- **`PeopleManager`**: Same pattern for `Person` instances; triggers nav mesh rebuild on init.

### 3.2 Vehicle lifecycle (`Vehicle`)

1. Resolve a free USD path under the current stage (`get_stage_next_free_path`).
2. Reference the robot USD, then call `isaacsim.core.api.robots.robot.Robot` with quaternion order adapted for Isaac (w-first).
3. Register with `World.scene` and `VehicleManager`.
4. Install callbacks:
   - **Physics:** `update_state`, `update` (vehicle-specific dynamics), sensor update aggregator, backend `update_sim_state`.
   - **Render:** graphical sensors.
   - **Timeline:** `sim_start_stop` for play/stop alignment.

Subclasses must implement `update`, `start`, `stop`, `reset` semantics as used by the base (see `Vehicle` for full contract).

### 3.3 Control loop (multirotor)

Typical flow each physics step:

1. `Vehicle.update_state` refreshes `State` from simulation.
2. `Multirotor.update` reads **first** backend’s `input_reference()` as per-rotor angular rates (rad/s).
3. `QuadraticThrustCurve` maps rates to forces and yaw rolling moment.
4. Forces applied at `/rotor0`…`/rotor3` (see §5); drag on `/body`; backends’ `update(dt)` for I/O.

### 3.4 Backends

Backends receive sensor and state callbacks from the vehicle and push/pull MAVLink or ROS topics. For multirotors, **`input_reference()`** is the primary actuator interface. Fixed-wing code paths may also inspect backend-specific fields (e.g. rotor/servo structures) where implemented.

---

## 4. Subsystem details

### 4.1 State and frames (`state.py`, `rotations.py`)

- Internal state is **ENU** position/velocity/acceleration and **FLU** body angular rates / body velocity as documented in `State`.
- `get_*_ned` / `get_*_frd` helpers apply the constant rotations in `rotations.py` so PX4/ArduPilot-facing code can emit consistent MAVLink state.

### 4.2 Multirotor (`multirotor.py`)

- **`MultirotorConfig`**: USD path, `QuadraticThrustCurve`, `LinearDrag`, sensor list, graphical sensors, graphs, backends.
- **`Multirotor.update`**: Uses dynamic control articulation for propeller **visual** DOFs (`joint0`…`jointN`) and rigid bodies for thrust application.
- **`force_and_torques_to_velocities`**: Pseudoinverse-based allocation from desired thrust/torque to squared angular rates (quadratic model).

### 4.3 Fixed wing (`fixedwing.py`)

- **`FixedWingConfig`**: Large coefficient set (lift/drag/moments, control derivatives), geometry, propeller model, `LinearDrag`, simulation mode (`autonomous`, `thrust_only`, `manual`, plus legacy `full` → `autonomous`), sign maps for surfaces/throttle vs. body frame.
- **`FixedWing`**: Extends `Vehicle`; implements a substantially different `update` (aero + prop + optional UI debugger imports).

### 4.4 Sensors

- **Physics sensors** subclass `Sensor`: rate-limited `update` via decorator; `initialize` receives world origin geo for GPS/mag, etc.
- **Graphical sensors** subclass `GraphicalSensor`: updated on **render** callbacks; same rate-limiting pattern.

### 4.5 Dynamics and thrusters

- **`LinearDrag`**: Body-frame damping proportional to body velocity components.
- **`QuadraticThrustCurve`**: Configurable `num_rotors`, per-rotor constants, saturation, rotation direction, rolling moment coefficients.

### 4.6 Graphs (`graphs/`)

Lightweight base `Graph` plus concrete graphs (e.g. ROS 2 camera bridge). Wired from `Vehicle` during construction if listed in config.

### 4.7 People simulation (`people/`, `people_manager.py`, `people_backends/`)

Parallel to vehicles: controllable humanoids with backends for ROS 2 or custom logic. `PeopleManager` mirrors `VehicleManager` and interacts with Kit navigation mesh commands.

### 4.8 Cesium (`cesium_globe.py`, `cesium_physics.py`)

Optional, extension-gated integration:

- **Globe:** WGS84 anchoring, ion token discovery (Carb settings ↔ Pegasus `configs.yaml`), preset tilesets.
- **Physics:** Ensures streamed terrain/buildings can participate in PhysX collisions (prim hierarchy, collision APIs, selection sanitization).

---

## 5. USD / articulation assumptions (multirotor)

For `Multirotor` to behave correctly, the robot USD is expected to expose:

- Articulation root at the vehicle stage prefix used in code.
- Rigid bodies such as **`/body`** and **`/rotor0`…`/rotor3`** (force application paths).
- Revolute DOFs named **`joint0`…** for propeller spin animation.

The **force application loop** in `Multirotor.update` iterates a **fixed range of four rotors**; this should be kept in mind when extending to hexacopters or variable motor counts (thruster models may allow `num_rotors != 4`, but the apply-force loop must stay consistent).

---

## 6. Dependencies (external to `logic`)

- **NVIDIA Isaac Sim:** `isaacsim.core.api.world.World`, `Robot`, stage utilities, viewports, nucleus assets.
- **Omniverse:** `omni.usd`, `pxr` (USD), `carb`, `omni.isaac.dynamic_control`.
- **Scientific Python:** `numpy`, `scipy.spatial.transform`.
- **YAML:** extension `configs.yaml` via `pegasus.simulator.params.CONFIG_FILE`.

---

## 7. Relationship to the rest of the extension

- **`parser/`** (outside this folder): YAML → `MultirotorConfig` via `VehicleParser` and sub-parsers; complements hand-written configs in examples.
- **`ui/`**: Uses `PegasusInterface`, `VehicleManager`, `ROBOTS` from `params.py`, and spawns `Multirotor` with selected backend (UI does not drive `FixedWing` by default).
- **`params.py`**: World settings per backend, asset paths, not part of `logic/` but tightly coupled.

---

## 8. Extension points (for contributors)

1. **New rotorcraft:** New USD + `MultirotorConfig` (or subclass `Multirotor` if actuator layout differs materially).
2. **New autopilot bridge:** Implement `Backend` / `BackendConfig`; wire `input_reference()` and MAVLink/ROS handling.
3. **New physics sensor:** Subclass `Sensor`, register in vehicle config; ensure backend forwards types if needed.
4. **New vision sensor:** Subclass `GraphicalSensor`, add render callback path (already generic in `Vehicle`).
5. **New vehicle class:** Subclass `Vehicle`, implement dynamics in `update`, register sensors/backends similarly.

---

## 9. Gaps and risks noted in code review

| Item | Description |
|------|-------------|
| **Missing method** | `PegasusInterface.generate_quadrotor_config_from_yaml()` calls `generate_quadrotor_config_from_dict(data)`, but **no `generate_quadrotor_config_from_dict` is defined** on `PegasusInterface` in the current tree. Any caller of the YAML API will fail at runtime until implemented or routed to `VehicleParser`. |
| **Rotor count vs. loop** | `QuadraticThrustCurve` supports configurable `num_rotors`, but `Multirotor.update` applies forces with a **hard-coded four-rotor loop**; configurations with `num_rotors != 4` need code alignment. |
| **`set_global_coordinates` bug** | In `pegasus_interface.py`, `set_global_coordinates` uses `if self.altitude is not None` when updating altitude; **`self.altitude` is a property** and is effectively always truthy, which can block clearing altitude. Worth fixing separately. |
| **Fixed-wing ↔ UI** | Extension UI spawns `Multirotor` only; fixed-wing workflows are script-driven (`FixedWing` examples). |

---

## 10. File inventory (49 Python modules)

Top-level: `__init__.py`, `vehicle_manager.py`, `state.py`, `rotations.py`, `people_manager.py`, `cesium_globe.py`, `cesium_physics.py`.

Subpackages: `interface/` (1), `vehicles/` (4 + `multirotors/`), `backends/` (5 + `tools/`), `sensors/` (6), `thrusters/` (3), `dynamics/` (3), `graphical_sensors/` (4), `graphs/` (3), `people/` (3), `people_backends/` (3).

---

*Generated as a static architecture report from repository analysis. Update this document when large refactors land.*
