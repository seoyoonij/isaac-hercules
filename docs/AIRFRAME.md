# Airframe Parameters — Hercules

Reference for the aerodynamic and propulsion values used in `examples/21_ardupilot_hercules_exp.py`.

This document is a practical blueprint for the **Hercules** airframe using `FixedWingConfig`. It focuses on the airframe physics parameters you need to define, not code internals.

## High-Level Model Flow

Each update step computes:

1. **Propulsion thrust** from throttle and motor/prop parameters.
2. **Aerodynamic forces** (lift, drag, side-force) from airspeed, angle of attack, sideslip, and control deflections.
3. **Aerodynamic moments** (roll, pitch, yaw) from stability derivatives and angular-rate damping.
4. Optional additional **body drag** term.

So your airframe definition is essentially:

- geometry + mass,
- propulsion capability,
- aerodynamic/stability derivatives,
- control-surface effectiveness.

---

## Airframe Parameters You Must Provide

### 1) Mass and Geometry (first-pass sizing)

- `mass_kg`: aircraft mass used by diagnostics and force sanity checks.
- `wing_area` (`S`): sets lift/drag scaling.
- `wing_span` (`b`): sets roll/yaw moment authority and normalized rates.
- `chord` (`c`): sets pitch moment scaling and normalized pitch rate.
- `air_density` (`rho`): usually `1.225` at sea level; reduce for high-altitude scenarios.

Rule of thumb:

- If `wing_area` is too small for mass, you will need high speed to take off.
- If `wing_span` is too small, roll/yaw moments become weak for the same coefficients.

### 2) Propulsion

- `prop_max_thrust`: max available thrust (N) at full throttle.
- `prop_max_rpm`: max motor RPM.
- `prop_thrust_coefficient`: map RPM to thrust (`T = k * RPM^2`).

Practical calibration:

1. Choose target static thrust-to-weight ratio (for fixed-wing often about `0.3` to `0.8`, mission dependent).
2. Set `prop_max_thrust ~= ratio * mass_kg * g`.
3. Tune `prop_thrust_coefficient` so full throttle approximately reaches that thrust.

### 3) Longitudinal Aerodynamics (lift/drag/pitch)

Lift model:

- `CL_0`: lift at zero AoA.
- `CL_alpha`: lift slope vs AoA.
- `CL_max`, `CL_min`: stall-like clipping limits.

Drag model:

- `CD_0`: baseline parasite drag.
- `CD_alpha`: linear AoA drag rise.
- `CD_alpha2`: quadratic AoA drag rise.

Pitch stability/control:

- `Cm_0`: trim pitch bias.
- `Cm_alpha`: static longitudinal stability (typically negative for stable aircraft).
- `Cm_q`: pitch-rate damping (more negative = stronger damping).
- `CL_elevator`, `Cm_elevator`: elevator effectiveness on lift and pitch moment.

### 4) Lateral-Directional Aerodynamics (sideslip/roll/yaw)

Side-force:

- `CY_beta`: side-force from sideslip.
- `CY_rudder`: side-force from rudder.

Roll dynamics:

- `Cl_beta`: dihedral/sideslip-to-roll coupling.
- `Cl_p`: roll-rate damping.
- `Cl_r`: yaw-rate to roll coupling.
- `Cl_aileron`: aileron roll authority.

Yaw dynamics:

- `Cn_beta`: weathercock stability (typically positive for stable yaw).
- `Cn_p`: roll-rate to yaw coupling.
- `Cn_r`: yaw-rate damping (typically negative).
- `Cn_rudder`: rudder yaw authority.

### 5) Optional Extra Drag Term

- `drag` (`LinearDrag([...])`): non-aero residual damping term in body axes.

Use it to represent:

- fuselage/gear damping not captured by simple coefficient model,
- numerical stabilization for aggressive conditions.

Keep it small; overusing it can hide poor aerodynamic tuning.

---

## Minimum Viable Airframe Dataset (Checklist)

For a new aircraft, collect these first:

- **Mass/inertia proxy**: at least `mass_kg`.
- **Reference geometry**: `wing_area`, `wing_span`, `chord`.
- **Propulsion ceiling**: realistic `prop_max_thrust`.
- **Trim/stability core**: `CL_0`, `CL_alpha`, `CD_0`, `Cm_0`, `Cm_alpha`.
- **Damping**: `Cm_q`, `Cl_p`, `Cn_r`.
- **Control authority**: `CL_elevator`, `Cm_elevator`, `Cl_aileron`, `Cn_rudder`, `CY_rudder`.
- **Coefficient bounds**: `CL_max`, `CL_min`.

If you only have sparse data, tune in this order:

1. Takeoff feasibility (`mass_kg`, `wing_area`, `prop_max_thrust`)
2. Straight-and-level trim (`CL_0`, `Cm_0`, `Cm_alpha`)
3. Drag/energy behavior (`CD_0`, `CD_alpha`, `CD_alpha2`)
4. Control response (`*_elevator`, `Cl_aileron`, `Cn_rudder`)
5. Damping and Dutch-roll behavior (`Cl_p`, `Cn_r`, `CY_beta`, `Cn_beta`)

---

## Hercules Values (Copy-Ready)

These are the values configured in `examples/21_ardupilot_hercules_exp.py`:

```python
cfg = FixedWingConfig()

# Mass / geometry
cfg.mass_kg = 10.5
cfg.wing_area = 1.337
cfg.wing_span = 2.744
cfg.chord = 0.506
cfg.air_density = 1.225

# Propulsion
cfg.prop_max_thrust = 111.2
cfg.prop_max_rpm = 3110.0
cfg.prop_thrust_coefficient = 1.058e-5

# Lift / drag
cfg.CL_0 = -0.13
cfg.CL_alpha = 4.18
cfg.CL_max = 0.9
cfg.CL_min = -0.9
cfg.CD_0 = 0.07
cfg.CD_alpha = 0.5
cfg.CD_alpha2 = 2.1

# Pitch
cfg.Cm_0 = 0.05
cfg.Cm_alpha = -0.382
cfg.Cm_q = -1.51416
cfg.CL_elevator = 2.534
cfg.Cm_elevator = 0.77291

# Lateral-directional
cfg.CY_beta = -0.58585
cfg.Cl_beta = -0.02769
cfg.Cl_p = -0.41015
cfg.Cl_r = 0.09431
cfg.Cn_beta = 0.07522
cfg.Cn_p = -0.06698
cfg.Cn_r = -0.02369
cfg.Cl_aileron = 0.45521
cfg.CY_rudder = -0.35687
cfg.Cn_rudder = 0.05155
```

Treat these values as **starting points**. Final values should come from flight-test matching, wind-tunnel/CFD data, or known reference aircraft.

---

## Acceptance Criteria for a "Good" Airframe

Use these checks after first tuning:

- Full throttle gives positive acceleration at low speed on runway.
- Aircraft can rotate and build positive climb without extreme elevator.
- At cruise throttle, trim requires modest elevator and no divergent pitch oscillation.
- Roll and yaw are responsive but damped (no sustained divergence).
- Sideslip disturbances decay (or stay bounded) with realistic rudder usage.
