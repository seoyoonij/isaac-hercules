# Getting started
## Overview:
1. OS translator: ArduPilot(Linux) --[Cygwin]-- User(Windows)
2. Network bridge: ArduPilot data stream(MAVLink) --[TCP:5760]--[MAVProxy]--splits stream-- [UDP:14550] MissionPlanner command/HUD
[UDP:14551/14555] 

## Initial setup
0. Clone ArduPilot source code + Install MissionPlanner
1. Cygwin setup: execute Ardupilot's Linux command from Windows (Windows PowerShell)
    ``Set-ExecutionPolicy Bypass -Scope Process``
    ``.\Tools\environment_install\install-prereqs-windows.ps1 -SelectDefaults``
2. Telemetry bridge setup (Windows CMD) 
    ``pip install mavproxy prompt-toolkit pyreadline3 wxPython``

## Execution
3. Launch standalone Ardupilot SITL w/o IsaacSim (Cygwin64 terminal)
    ``cd /cygdrive/c/ardupilot_workspace/ardupilot/ArduPlane``
    ``python3 ../Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane-tailsitter:127.0.0.1 --no-mavproxy -A "--sim-address=127.0.0.1 --sim-port-in=9002"``

    (legacy: for standard fixed wing
    ``cd /cygdrive/c/ardupilot_workspace/ardupilot/ArduPlane``
    then bypass Linux GUI + launch isaac-uav:
    ``python3 ../Tools/autotest/sim_vehicle.py -v ArduPlane -f json:127.0.0.1 --no-mavproxy -A "--sim-address=127.0.0.1 --sim-port-in=9002"``)

    (legacy: w/o IsaacSim
    ``python3 ../Tools/autotest/sim_vehicle.py -v ArduPlane --no-mavproxy`` # for traditional fixed-wing
     or ``python3 ../Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane-tailsitter --no-mavproxy``  # for tailsitter)
     
4. Launch intermediate network proxy router for telemetry bridge (Windows CMD) 
    ``python -m MAVProxy.mavproxy --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551``

    (legacy: w/o IsaacSim
    ``python -m MAVProxy.mavproxy --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --nodisplay`` )
    MAVProxy connects to Cygwin over 5760 and pushes MAVLink data directly out to Windows UDP port 14550, which mathces 'connection_baseport' of isaacsim code

5. Launch IsaacSim fixed-wing example standalone
    Launch with Hercules example
    ``cd C:\Hercules_Isaac``
    ``C:\isaac-sim\python.bat examples\21_ardupilot_hercules_exp.py``

    (legacy: standard fixedwing
    ``cd C:\isaac-uav``
    ``C:\isaac-sim\python.bat examples\12_ardupilot_fixedwing.py``)

6. Open and connect MissionPlanner: 
    - UDP default baud rate 115200
    - Connect
    - Remote host 127.0.01 (or localhost)
    - UDP port 14550
    
----
n.b. 
- GCS options: MissionPlanner (GUI) / MAVProxy (CLI)
- Connection ports
    - tcp://127.0.0.1:5670
    - udp://127.0.0.1:14550
    - udpcl://192.168.1.255:14550
    - serial:com4:115200
----

# Sending commands (MissionPlanner GUI)
1. Parameter config for VTOL SITL: 
    - bypass failsafe: Config/ Full Parameter List/ THR_FAILSAFE set to 0 (disable) > Write params
    - bypass sensor calib guard:                  / ARMING_SKIPCHK set to 1
    - enable VTOL:                                / Q_ENABLE set to 1
    - telemetry rate:        / Planner/ Telemetry Rates set all to 10

2. Actions: Waypoint flight test
    - STABILIZE > Set Mode
    - Arm/Disarm
    - right-click on satellite map > Fly to (or Takeoff) > set altitude > initiate flight
    - Expectation: Loiter around waypoint

---

