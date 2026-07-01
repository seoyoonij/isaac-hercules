# Setting up Pegasus Simulator
1. Clone the repository (isaac-uav)
2. Tell IsaacSim's python where UAV code lives
  - ``cd C:\isaac-uav\extensions``
      ``C:\isaac-sim\python.bat -m pip install --editable pegasus.simulator``
  - or if error, directly inject path in .py files before Pegasus imports (e.g. example_12 line 23)

3. Adapted backend config in ``12_ardupilot_fixedwing.py`` line 102
    - Consideration: 
        - Default ArduPilot+MissionPlanner SITL + now external Flight Dynamics Model (IsaacSim)
        - ArduPilot must run inside Cygwin + MAVProxy network bridge

    - Orignal pipeline: IsaacSim FixedWing -> ArdupilotMavlinkBackend -> ArduP SITL/MissionPlanner
    - Adapted pipeline: 

4. Launch simulation
  - ``cd C:\isaac-uav``
  - ``C:\isaac-sim\python.bat examples\12_ardupilot_fixedwing.py``
  - or ``C:\isaac-uav\app\isaac-sim.bat --ext-folder C:\isaac-uav\extensions``

  - if crashes, 
    - Command Prompt (CMD):
        ``` set ROS_DISTRO=humble
        set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        set PATH=%PATH%;c:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
        ```

# Fixed-wing UAV specific files
1. USD assets: ``extensions\pegasus\simulator\assets\Robots\fixed_wing`` 
2. FixedWing Class definition: ``extensions\pegasus\simulator\logic\vehicles\fixedwing.py``
3. Simulation example script: ``examples\12_ardupilot_fixedwing.py``
4. ROBOTS list: ``extensions\pegasus.simulator\pegasus\simulator\params.py``
  ``ROBOTS = {"Iris": ROBOTS_ASSETS + "/Iris/iris.usd",
          "Fixed Wing": ROBOTS_ASSETS + "/fixed_wing/fixed_wing.usd",
          "Flying Cube": ROBOTS_ASSETS + "/yoda_fixed_wing/cube.usd",}``

---
## Legacy: Setting up PX4 SITL
1. Install PX4 in Ubuntu, configure ip address
2. skip this: (if needed: Update line 219 ``extensions\pegasus\simulator\logic\backends\px4_mavlink_backend.py`` with correct IP and port for MAVLink connection
    e.g. ``self.connection_ip = self.config.get("connection_ip", "172.25.145.97")``)
3. Launch PX4 on WSL2 using 
    ``cd ~/isaac_sim_project/PX4-Autopilot
      make px4_sitl_default none_iris``   
   QGroundControl autoconnects. 
   Then spawn simulation on Pegasus.
---

# Aerodynamics model assumptions
[Ardupilot] --MAVLink/Json-- [Pegasus] --scaling-- [IsaacSim] --aerodynamics

    - Ignoring physical joints
    - Treats 3d model as ghost asset (not identical reflection of physical shape)
    - Vehicles are abstracted with aerodynamic parameters


# Launching with Hercules files
## Setting up
1. Intially, IsaacSim was installed on Windows. It has ``python.bat``
2. Hercules setup.sh was written for Linux host. When prompting Isaac Sim install directory, it looks for ``python.sh``
3. Thus during initial setup,  
    (Cygwin64 terminal)
    `` /cygdrive/c/Hercules_workspace/isaac-hercules``
    ``$ ./setup.sh ``
    and if needed, ``sed -i 's/\r$//' /cygdrive/c/Hercules_workspace/isaac-hercules/setup.sh`` to convert Windows-style to Linux-style.

    Then create a dummy file just for setup (When running sim on Windows, batch execution will look for ``python.bat`` anyways) so (Cygwin64 terminal)
    ``touch /cygdrive/c/isaac-sim/python.sh``
    ``chmod +x /cygdrive/c/isaac-sim/python.sh``
4. Then in setup, 
    "Enter Isaac Sim install directory [/home/crrl/isaacsim]:"  ``/cygdrive/c/isaac-sim``
    "Enter ArduPilot source directory [/home/crrl/ardupilot]:" ``/cygdrive/c/ardupilot_workspace/ardupilot``

5. Since Isaac Sim is running in Windows in this case, manually open ``.env.local`` and inject the paths in Windows syntax.
    ```
    ISAACSIM_PATH=C:\isaac-sim
    ISAACSIM_PYTHON=C:\isaac-sim\python.bat
    ISAACSIM=C:\isaac-sim\isaac-sim.bat
    ARDUPILOT_DIR=C:\ardupilot_workspace\ardupilot
    ARDUPILOT_PYTHON=python3
    ```

6. Ensure local extension sources are imported before any installed pegasus package: ``21_ardupilot_hercules_exp.py`` (line 24)
    ```
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
    ```

7. Launch IsaacSim using Hercules
    ``cd C:\Hercules_workspace\isaac-hercules``
    ``C:\isaac-sim\python.bat examples\21_ardupilot_hercules_exp.py``
    n.b. IsaacSim tries autolaunching ArduPilot, crashes on Windows' os.setsid error, abandons auto-launch, then proceeds with main loop anyway.

## Execution
Follow guidline in devnote-ArduPilot.md ##Execution
    