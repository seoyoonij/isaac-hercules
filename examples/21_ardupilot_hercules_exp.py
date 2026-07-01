#!/usr/bin/env python
"""
Hercules fixed-wing tailsitter simulation with ArduPilot SITL backend.

Standalone Isaac Sim app. Requires Isaac Sim, Pegasus Simulator (included),
and a local ArduPilot checkout (see ../README.md).
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp
import os

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Ensure local extension sources are imported before any installed pegasus package
from pathlib import Path
import sys
repo_root = Path(__file__).resolve().parents[1]
utils_dir = Path(__file__).resolve().parent / "utils"
uav_extensions = repo_root / "extensions"
uav_simulator = uav_extensions / "pegasus.simulator"

for p in (utils_dir, uav_extensions, uav_simulator):
    p_str = str(p)
    if p_str not in sys.path:
        sys.path.insert(0, p_str)

# Import the Pegasus API for simulating vehicles
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (
    ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig
)

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation

# Import the FixedWing class
from pegasus.simulator.logic.vehicles.fixedwing import FixedWing, FixedWingConfig

_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HERCULES_USD_PATH = os.path.join(
    _PACKAGE_ROOT, "assets", "Hercules", "Hercules_v3_without_prop.usd"
)


class FixedWingApp:

    def __init__(self):
        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World - controls physics, spawning assets, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load simulation environment
        # Options: "Curved Gridroom", "Default Environment", "Black Gridroom", "Hospital", "Office", "Warehouse"
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])

        # Create the fixed-wing aircraft
        self.create_fixedwing_vehicle()

        # Reset the simulation environment so that all articulations are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback
        self.stop_sim = False

        print("✓ Fixed-wing simulation initialized successfully!")

    def create_fixedwing_vehicle(self):
        """
        Create a single fixed-wing aircraft with configured backend
        """
        
        config = FixedWingConfig()
        
        # Hercules propulsion settings
        config.prop_max_thrust = 111.2
        config.prop_max_rpm = 3110.0
        config.prop_thrust_coefficient = 1.058e-5 # verify if same for all three
        config.prop_rotation_dir = 1
        
        # Hercules mass/geometry settings
        config.mass_kg = 10.5
        config.wing_area = 1.337
        config.wing_span = 2.744
        config.chord = 0.506
        config.air_density = 1.225
        
        # Hercules aerodynamic coefficients
        config.CL_0 = -0.13
        config.CL_alpha = 4.18
        config.CL_max = 0.9
        config.CL_min = -0.9
        config.CD_0 = 0.07
        config.CD_alpha = 0.5
        config.CD_alpha2 = 2.1

        # propwash tuning
        config.propwash_gain = float(os.environ.get("PEGASUS_FW_PROPWASH_GAIN", "2.0"))
        config.propwash_max_scale = float(os.environ.get("PEGASUS_FW_PROPWASH_MAX_SCALE", "4.0"))
        config.propwash_log = os.environ.get("PEGASUS_FW_PROPWASH_LOG", "1").strip() not in ("0", "false", "False")
        
        # Hercules pitch stability/control
        config.Cm_0 = 0.05
        config.Cm_alpha = -0.382
        config.Cm_q = -1.51416
        config.CL_elevator = 2.534
        config.Cm_elevator = 0.77291

        # Hercules lateral-directional derivatives
        config.CY_beta = -0.58585
        config.Cl_beta = -0.02769
        config.Cl_p = -0.41015
        config.Cl_r = 0.09431
        config.Cn_beta = 0.07522
        config.Cn_p = -0.06698
        config.Cn_r = -0.02369
        config.Cl_aileron = 0.45521
        config.CY_rudder = -0.35687
        config.Cn_rudder = 0.05155

        # Simulation
        # Manual      : User provides forces. Aerodynamics are NOT calculated. Useful for frame debugging.
        # Thrust Only : User provides forces. Aerodynamics are calculated. Useful for Aerodynamics Coefficient debugging
        # Autonomous  : Needs backend, no user control needed. Aerodynamics are calculated. Production mode.
        config.simulation_mode = os.environ.get(
            "PEGASUS_FIXEDWING_SIM_MODE", "autonomous"
        ).strip().lower()  # autonomous, thrust_only, manual

        # Disable visual prop spin unless a propeller joint name is provided.
        prop_joint_name = os.environ.get("PEGASUS_FIXEDWING_PROPELLER_JOINT_NAME", "").strip()
        if prop_joint_name:
            config.propeller_joint_name = prop_joint_name
        else:
            config.propeller_joint_name = "__disabled__"

        # Optional layout hints for nested custom USDs:
        #   PEGASUS_VEHICLE_USD_INNER_PREFIX=Hercules
        #   PEGASUS_VEHICLE_ROBOT_ARTICULATION_SUFFIX=body
        inner = os.environ.get("PEGASUS_VEHICLE_USD_INNER_PREFIX", "").strip().strip("/")
        robot_suffix = os.environ.get("PEGASUS_VEHICLE_ROBOT_ARTICULATION_SUFFIX", "").strip().strip("/")
        if inner:
            config.usd_inner_prefix = inner
        if robot_suffix:
            config.robot_articulation_suffix = robot_suffix

        # Optional control-sign overrides for quick frame-validation without code edits.
        # Example:
        #   PEGASUS_FW_SIGNS="1,-1,1,-1"  -> Aileron, Elevator, Throttle, Rudder
        signs_raw = os.environ.get("PEGASUS_FW_SIGNS", "").strip()
        if signs_raw:
            try:
                a_s, e_s, t_s, r_s = [float(x.strip()) for x in signs_raw.split(",")]
                config.aileron_sign = 1.0 if a_s >= 0 else -1.0
                config.elevator_sign = 1.0 if e_s >= 0 else -1.0
                config.throttle_sign = 1.0 if t_s >= 0 else -1.0
                config.rudder_sign = 1.0 if r_s >= 0 else -1.0
            except ValueError:
                carb.log_warn(
                    f"Invalid PEGASUS_FW_SIGNS={signs_raw!r}; expected four comma-separated numbers."
                )
        
        # ArduPilot tailsitter guide:
        # - Use a tailsitter-capable frame so SITL boots with QuadPlane tailsitter defaults.
        # - Keep ArduPlane vehicle type (tailsitters are QuadPlane in Plane firmware).
        # Override with:
        #   PEGASUS_ARDUPILOT_FRAME=plane
        #   PEGASUS_ARDUPILOT_VEHICLE=ArduPlane
        ardupilot_config = ArduPilotMavlinkBackendConfig({
            "vehicle_id": 0,
            "ardupilot_autolaunch": True,
            "ardupilot_dir": self.pg.ardupilot_path,
            "ardupilot_vehicle_model": os.environ.get("PEGASUS_ARDUPILOT_FRAME", "plane-tailsitter"),
            "ardupilot_vehicle": os.environ.get("PEGASUS_ARDUPILOT_VEHICLE", "ArduPlane"),
            # JSON FDM (--model JSON) requires lock-step state/servo exchange with SITL.
            "enable_lockstep": True,
        })
        
        # Combine backends
        config.backends = [
            ArduPilotMavlinkBackend(config=ardupilot_config),
        ]

        spawn_euler_deg_raw = os.environ.get("PEGASUS_FIXEDWING_SPAWN_EULER_DEG", "0,-90,0").strip()
        try:
            spawn_euler_deg = [float(x.strip()) for x in spawn_euler_deg_raw.split(",")]
            if len(spawn_euler_deg) != 3:
                raise ValueError("expected 3 values")
        except ValueError:
            carb.log_warn(
                f"Invalid PEGASUS_FIXEDWING_SPAWN_EULER_DEG={spawn_euler_deg_raw!r}; using 0,0,0."
            )
            spawn_euler_deg = [0.0, 0.0, 0.0]

        if not os.path.isfile(HERCULES_USD_PATH):
            raise FileNotFoundError(
                f"Hercules USD not found at {HERCULES_USD_PATH}. "
                "Ensure assets/Hercules/Hercules_v3_without_prop.usd is present."
            )

        self.aircraft = FixedWing(
            stage_prefix="/World/fixedwing0",
            usd_file=HERCULES_USD_PATH,
            vehicle_id=0,
            init_pos=[0.0, 0.0, 1.1],
            # Per ArduPilot tailsitter docs, "normal" orientation is fixed-wing flight attitude.
            # Use env override if your USD forward axis differs.
            init_orientation=Rotation.from_euler("XYZ", spawn_euler_deg, degrees=True).as_quat(),
            config=config
        )
        
        print("✓ Fixed-wing aircraft created.")

    def run(self):
        """
        Main application loop - executes physics steps
        """

        self.timeline.play()
        print("▶ Simulation started!")

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)
            
        # Cleanup and stop
        carb.log_warn("Fixed-wing Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    """
    Main entry point
    """
    app = FixedWingApp()
    app.run()


if __name__ == "__main__":
    main()
