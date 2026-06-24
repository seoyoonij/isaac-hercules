# Flight dynamics simulation in Pegasus / Isaac Sim

This document describes how **flight dynamics** are modeled in the **Pegasus Simulator** extension used with Isaac Sim in this repository. The design is a **hybrid**: **NVIDIA PhysX** integrates the rigid-body equations of motion (mass, inertia, contacts, gravity from the scene), while **Python physics callbacks** inject **custom aerodynamic and propulsion forces and torques** each simulation step.

---

## 1. High-level architecture

| Layer | Role |
|--------|------|
| **PhysX (Isaac Sim)** | Time integration of articulated rigid bodies; collision response; scene gravity; joint constraints. |
| **`Vehicle.update_state`** | Reads pose, velocities from PhysX into an internal `State` (ENU position, FLU body velocity, etc.). |
| **`Multirotor.update` / `FixedWing.update`** | Computes thrust, drag, and (for fixed wing) lift/side force and aerodynamic moments; applies them via the Dynamic Control API. |
| **Backends (MAVLink, etc.)** | Provide actuator commands (`input_reference`) and consume `State` for sensors / state streams. |

Forces and torques are applied with:

- `Vehicle.apply_force(force, pos, body_part)` → `apply_body_force` on the chosen rigid body (force in **body FLU**).
- `Vehicle.apply_torque(torque, body_part)` → `apply_body_torque` (torque in **body FLU**).

Implementation: `pegasus/simulator/logic/vehicles/vehicle.py`.

---

## 2. Reference frames

Understanding frames is required to interpret coefficients and MAVLink-facing data.

### 2.1 Inertial frame: ENU

Isaac Sim’s world frame is treated as **ENU** (East, North, Up):

- Position and linear velocity in `State` use **ENU**.
- Linear acceleration in `State` is approximated in ENU as \((\mathbf{v}_k - \mathbf{v}_{k-1}) / \Delta t\) (see `Vehicle.update_state`).

### 2.2 Body frame: FLU

Simulated forces, torques, and `State.linear_body_velocity` \([u, v, w]\) use **FLU** (Forward, Left, Up) on the vehicle **body** link (`…/body`).

### 2.3 Flight-controller conventions: NED / FRD

PX4 / ArduPilot often use **NED** inertial and **FRD** body frames. `State` provides helpers that compose fixed rotations (`pegasus/simulator/logic/rotations.py`):

- `rot_ENU_to_NED`, `rot_FLU_to_FRD`

Backends convert state for MAVLink using these mappings where implemented.

---

## 3. Physics callback pipeline

Each vehicle registers several **world physics callbacks** (`Vehicle.__init__` in `vehicle.py`):

1. **`{stage_prefix}/state`** → `update_state(dt)` — refresh `State` from PhysX.
2. **`{stage_prefix}/update`** → `update(dt)` — vehicle-specific dynamics (apply forces/torques).
3. **`{stage_prefix}/Sensors`** → `update_sensors(dt)` — sensors, then `backend.update_sensor(...)`.
4. **`{stage_prefix}/mav_state`** → `update_sim_state(dt)` — `backend.update_state(self._state)`.

Callback ordering within a single physics step follows Isaac/Omniverse rules for the world; Pegasus assumes `update_state` runs before or in a consistent order with `update` so that `self._state` reflects the latest integrated motion when forces are computed. In practice, designs should avoid relying on same-step algebraic loops without checking Omniverse callback ordering for your Kit version.

---

## 4. State vector (`State`)

`pegasus/simulator/logic/state.py` holds:

| Field | Meaning |
|--------|---------|
| `position` | \([x,y,z]\) in **ENU**, inertial frame. |
| `attitude` | Quaternion \([q_x, q_y, q_z, q_w]\): FLU body relative to ENU. |
| `linear_velocity` | \([\dot x,\dot y,\dot z]\) in **ENU**. |
| `linear_body_velocity` | \([u,v,w]\) in **FLU** body. |
| `angular_velocity` | \([p,q,r]\) in **FLU** body (rad/s). |
| `linear_acceleration` | Finite-difference approximation in **ENU**. |

Angle of attack and sideslip for fixed-wing aerodynamics are derived from `linear_body_velocity` inside `FixedWing._calculate_aerodynamics`.

---

## 5. Multirotor flight dynamics

**Class:** `Multirotor` (`pegasus/simulator/logic/vehicles/multirotor.py`).

### 5.1 Control input

The **first** backend’s `input_reference()` returns a list of **target rotor angular velocities** in **rad/s**, one per rotor (default model: four rotors).

### 5.2 Rotor thrust model

`QuadraticThrustCurve` (`pegasus/simulator/logic/thrusters/quadratic_thrust_curve.py`) implements an **algebraic** (no spin-up dynamics) map:

- Per rotor \(i\): \(\omega_i\) is clipped to \([\omega_{i,\min}, \omega_{i,\max}]\).
- Thrust: \(T_i = k_i \, \omega_i^2\) with per-rotor constant \(k_i\).
- **Yaw rolling moment** (reaction torque from rotors): aggregated as  
  \(\tau_{z,\mathrm{roll}} = \sum_i c_i \, \omega_i^2 \, \mathrm{rot\_dir}_i\)  
  using `rolling_moment_coefficient` and rotation direction signs.

The model does **not** currently use airspeed or inflow; `state` and `dt` are passed into `update` for API compatibility only.

### 5.3 Where forces are applied

- For each rotor \(i\), force **`[0, 0, T_i]`** is applied on rigid body **`/rotor{i}`** (in that link’s frame — aligned with the USD rotor setup).
- **`[0, 0, \tau_{z,\mathrm{roll}}]`** is applied as **body torque** on **`/body`**.

So net thrust and roll/pitch moments arise from **different lever arms** of the rotor forces relative to the body center of mass (handled implicitly by PhysX because forces are applied at rotor links).

### 5.4 Body-axis linear drag

After rotors, **`LinearDrag`** is evaluated on the **body** and applied on **`/body`**:

\[
\mathbf{F}_{\mathrm{drag}} = -\mathrm{diag}(d_x, d_y, d_z) \, \mathbf{v}_{\mathrm{body}}
\]

Default multirotor config uses `LinearDrag([0.50, 0.30, 0.0])` in `MultirotorConfig`.

### 5.5 What PhysX still provides

- **Mass and inertia** from USD / PhysX on `body` and rotor links.
- **Gravity** from the simulation scene (not explicitly added in `Multirotor.update`).
- **Collisions** and joint articulation for the quad frame.

---

## 6. Fixed-wing flight dynamics

**Class:** `FixedWing` (`pegasus/simulator/logic/vehicles/fixedwing.py`).  
**Configuration:** `FixedWingConfig`.

### 6.1 Simulation modes

| Mode | Thrust | Coefficient aerodynamics | `LinearDrag` | Moments |
|------|--------|---------------------------|--------------|---------|
| **`autonomous`** | Prop model from backend throttle | Yes | Yes | Aero + damping |
| **`thrust_only`** | Force UI along body X | Yes | Yes | Aero + optional UI pitch torque |
| **`manual`** | Force UI (full 3D force + torque) | **No** | **No** | UI only |

(`full` is accepted as an alias for `autonomous`.)

### 6.2 Propulsion

Throttle \(\in [0,1]\) sets RPM scale; thrust magnitude:

\[
T = \min\left(T_{\max},\, k_{\mathrm{th}} \, (\omega_{\mathrm{ref}})^2\right), \quad \omega_{\mathrm{ref}} = \mathrm{throttle} \cdot \omega_{\max}.
\]

Thrust is applied as a body force **`[T, 0, 0]`** on **`/body`**.

### 6.3 Quasi-steady aerodynamic model

At each step, **airspeed** \(V = \|\mathbf{v}_{\mathrm{body}}\|\). If \(V < 0.1\,\mathrm{m/s}\), aerodynamic force and moment contributions are zero.

**Angles** (from FLU body velocity, with conventions documented in code):

- Angle of attack: \(\alpha = \mathrm{atan2}(-w, u)\).
- Sideslip: \(\beta = \arcsin(\mathrm{clip}(-v/V, -1, 1))\).

**Dynamic pressure:**

\[
q = \tfrac{1}{2} \rho V^2
\]

with configurable \(\rho\) and wing area \(S\).

**Lift coefficient** (elevator \(\delta_e\) in \([-1,1]\)):

\[
C_L = \mathrm{clip}\left(C_{L,0} + C_{L,\alpha}\alpha + C_{L,\delta_e}\delta_e,\; C_{L,\min},\; C_{L,\max}\right).
\]

**Drag coefficient** (parasitic + AoA terms):

\[
C_D = C_{D,0} + C_{D,\alpha}|\alpha| + C_{D,\alpha2}\alpha^2.
\]

**Side-force coefficient** uses \(\beta\) and rudder. **Moment coefficients** \(C_l, C_m, C_n\) use \(\alpha, \beta\), control deflections, and **nondimensionalized rates** \(\hat p, \hat q, \hat r\) (span and chord scaling as in code).

**Forces in stability axes** are converted to **aero body** components, then to **FLU** for application (sign fixes for lateral and vertical axes).

**Moments** scale as:

- Roll / yaw: \(\sim q S b\) (span \(b\)).
- Pitch: \(\sim q S \bar c\) (chord \(\bar c\)).

### 6.4 Additional linear drag on fixed wing

Independently of \(C_D\), `FixedWingConfig.drag` (default `LinearDrag([0.1, 0.1, 0.1])`) adds **extra** body-axis damping proportional to \(\mathbf{v}_{\mathrm{body}}\). This is **not** a substitute for induced drag; it is an extra tuning / stability term.

### 6.5 Control inputs (autonomous)

The first backend’s `input_reference()` is decoded into **aileron, elevator, throttle, rudder** (with optional ArduPilot servo scaling when metadata exists). Sign conventions can be adjusted with `aileron_sign`, `elevator_sign`, `rudder_sign`, `throttle_sign` on `FixedWingConfig`.

### 6.6 Clipping and logging

Forces and torques passed to `apply_force` / `apply_torque` are clipped to broad numerical limits in code. Optional CSV logging and `debug_mode` visualization exist for development.

---

## 7. Gravity, mass, and inertia

- **Gravity** is not applied in Python; it comes from the **Isaac Sim physics scene** settings.
- **Mass and inertia** for each link come from **USD** (`UsdPhysics.MassAPI`, collision geometry, etc.). `FixedWing` can read mass from `body` for diagnostics (`_get_vehicle_mass_kg`).

If the airframe USD is a placeholder, dynamics will reflect **that** mass/inertia — tuning lift/thrust without updating USD mass can produce unrealistic accelerations.

---

## 8. Coordinate summary for implementers

- Apply **all** Pegasus-computed forces/torques in **FLU** on the **correct link** (`/body` vs `/rotorN`).
- **Multirotor**: thrust along **local Z** of each **rotor** link; yaw reaction as **body Z torque**.
- **Fixed wing**: thrust along **body +X**; aerodynamic resultants computed internally and already converted to **FLU** before `apply_force` / `apply_torque`.

---

## 9. Limitations and modeling assumptions

This stack is aimed at **HIL/SIL-style** integration with autopilots, not high-fidelity CFD.

- **No** explicit propeller slipstream / **no** ground effect in the coefficient model.
- **No** stall hysteresis or unsteady aerodynamics beyond clipping \(C_L\).
- Rotor **spin-up dynamics** are not modeled (instantaneous \(\omega \rightarrow T\)).
- **Wind** is not a first-class input in the core force equations unless added elsewhere (body velocity is used as a proxy for relative wind only if the state already includes wind in velocity — typically it does **not** unless the scene or state is extended).
- **Linear drag** is a simple diagonal damping model, not a full \(q\,S\,C_D\) breakdown for multirotors.
- Sensor noise models depend on individual sensor classes (not covered here).

---

## 10. Tuning entry points

| Vehicle | Primary config | Typical parameters |
|---------|----------------|--------------------|
| Multirotor | `MultirotorConfig` | `QuadraticThrustCurve` \(k_i\), \(\omega_{\max}\), rotor positions (USD), `LinearDrag` coefficients |
| Fixed wing | `FixedWingConfig` | `wing_area`, `wing_span`, `chord`, `air_density`, all \(C_*\) derivatives, `prop_*`, `simulation_mode`, `drag` |

Examples in the repo: `examples/12_ardupilot_fixedwing.py`, `examples/14_ardupilot_fixedwing_multy_cesium.py`, and multirotor examples under `examples/`.

---

## 11. Related files (quick index)

| Topic | Path |
|--------|------|
| Base vehicle, state readout, apply force/torque | `pegasus/simulator/logic/vehicles/vehicle.py` |
| Multirotor loop | `pegasus/simulator/logic/vehicles/multirotor.py` |
| Fixed-wing loop and aero | `pegasus/simulator/logic/vehicles/fixedwing.py` |
| `State` | `pegasus/simulator/logic/state.py` |
| ENU/NED, FLU/FRD | `pegasus/simulator/logic/rotations.py` |
| Linear drag | `pegasus/simulator/logic/dynamics/linear_drag.py` |
| Rotor thrust | `pegasus/simulator/logic/thrusters/quadratic_thrust_curve.py` |
| Broader extension overview | `docs/logic-package-report.md` |

---

*Document version: matches Pegasus logic under `extensions/pegasus.simulator` in the isaac-uav workspace. If you change callback registration or force application APIs, update this document accordingly.*
