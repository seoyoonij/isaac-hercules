#!/usr/bin/env python
"""
| File: 12_ardupilot_fixedwing.py
| License: BSD-3-Clause
| Description: Isaac Sim standalone app for the Pegasus fixed-wing asset with ArduPilot SITL backend
"""

# Imports to start Isaac Sim from this script
import carb
import os
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

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
FIXED_WING_USD_PATH = os.path.join(
    _PACKAGE_ROOT, "assets", "fixed_wing", "fixed_wing.usd"
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

        # Pegasus fixed_wing.usd: defaultPrim "World" with body at …/World/body
        config.usd_inner_prefix = "World"
        
        # Propeller/Motor settings
        config.prop_max_thrust = 100.0          # Maximum thrust in Newtons (Targeting ~1kg mass)
        config.prop_max_rpm = 10000.0          # Maximum RPM
        config.prop_thrust_coefficient = 0.000075  # Thrust coefficient
        config.prop_rotation_dir = 1           # 1: CCW, -1: CW
        
        # Aircraft geometry
        config.wing_area = 2.36                # Wing area (m²) (Span 4.46 * Chord 0.53)
        config.wing_span = 4.46                 # Wing span (m)
        config.chord = 0.53                    # Mean aerodynamic chord (m)
        
        # Aerodynamic coefficients (adjust based on your aircraft)
        config.CL_0 = 0.3                      # Zero AoA lift coefficient
        config.CL_alpha = 4.0                  # Lift curve slope
        config.CL_max = 1.5                    # Stall limit (upper)
        config.CD_0 = 0.025                    # Parasitic drag
        
        # Control surface effectiveness
        config.Cm_elevator = -1.5              # Elevator pitch moment
        config.Cl_aileron = 0.3                # Aileron roll moment
        config.Cn_rudder = -0.05               # Rudder yaw moment

        # Simulation
        # Manual      : User provides forces. Aerodynamics are NOT calculated. Useful for frame debugging.
        # Thrust Only : User provides forces. Aerodynamics are calculated. Useful for Aerodynamics Coefficient debugging
        # Autonomous  : Needs backend, no user control needed. Aerodynamics are calculated. Production mode.
        config.simulation_mode = 'autonomous' # autonomous, thrust_only, manual
        #config.debug_mode = True
        
        ardupilot_config = ArduPilotMavlinkBackendConfig({
            "vehicle_id": 0,
            "ardupilot_autolaunch": True,
            "ardupilot_dir": self.pg.ardupilot_path,
            "ardupilot_vehicle_model": "plane",
            "ardupilot_vehicle": "ArduPlane",
            # JSON FDM (--model JSON) requires lock-step state/servo exchange with SITL.
            "enable_lockstep": True,
        })
        
        # Combine backends
        config.backends = [
            ArduPilotMavlinkBackend(config=ardupilot_config),  # Uncomment for Ardupilot
        ]

        if not os.path.isfile(FIXED_WING_USD_PATH):
            raise FileNotFoundError(
                f"Fixed-wing USD not found at {FIXED_WING_USD_PATH}. "
                "Ensure assets/fixed_wing/fixed_wing.usd is present."
            )

        self.aircraft = FixedWing(
            stage_prefix="/World/fixedwing0",
            usd_file=FIXED_WING_USD_PATH,
            vehicle_id=0,
            init_pos=[0.0, 0.0, 1.0],                    # Start 0.2m above ground
            init_orientation=Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config
        )
        
        print(f"✓ Fixed-wing aircraft created.")

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