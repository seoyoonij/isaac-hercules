"""
| File: vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Vehicle class which is used as the base for all the vehicles.
"""

# Numerical computations
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation

# Low level APIs
import carb
from pxr import Usd, Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

# High level Isaac sim APIs
import omni.usd
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from omni.usd import get_stage_next_free_path
from isaacsim.core.prims import SingleArticulation
from omni.isaac.dynamic_control import _dynamic_control

# Extension APIs
from pegasus.simulator.logic.rotations import body_axis_align_mode, body_axis_align_rotation
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicle_manager import VehicleManager


def get_world_transform_xform(prim: Usd.Prim):
    """
    Get the local transformation of a prim using omni.usd.get_world_transform_matrix().
    See https://docs.omniverse.nvidia.com/kit/docs/omni.usd/latest/omni.usd/omni.usd.get_world_transform_matrix.html
    Args:
        prim (Usd.Prim): The prim to calculate the world transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    return rotation


def _usd_path_segment(s: str) -> str:
    """Normalize a single USD path segment (no leading/trailing slashes)."""
    return (s or "").strip().strip("/")


def _dedupe_paths(paths: list) -> list:
    seen = set()
    out = []
    for p in paths:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


def _collect_articulation_root_paths(stage: Usd.Stage, spawn_path: str, max_depth: int = 10) -> list:
    """All prim paths under ``spawn_path`` that carry PhysicsArticulationRootAPI."""
    roots = []
    root_prim = stage.GetPrimAtPath(spawn_path)
    if not root_prim.IsValid():
        return roots

    def walk(prim: Usd.Prim, depth: int) -> None:
        if depth > max_depth:
            return
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(prim.GetPath().pathString)
        for child in prim.GetChildren():
            walk(child, depth + 1)

    walk(root_prim, 0)
    return roots


def _fallback_robot_prim_from_body_link(
    stage: Usd.Stage, spawn: str, asset_root: str
) -> str | None:
    """
    Pegasus fixed_wing puts the articulation root on ``body``. Right after ``AddReference``,
    ``HasAPI(ArticulationRootAPI)`` is often still false even though PhysX uses ``body`` at reset.
    """
    for base in _dedupe_paths([asset_root, spawn]):
        bp = f"{base}/body"
        prim = stage.GetPrimAtPath(bp)
        if prim.IsValid():
            return bp
    return None


def _ensure_articulation_root_api(stage: Usd.Stage, robot_path: str) -> bool:
    """
    Ensure the selected robot prim carries ArticulationRootAPI so SingleArticulation can initialize.
    Returns True when API exists (pre-existing or newly applied), else False.
    """
    prim = stage.GetPrimAtPath(robot_path)
    if not prim.IsValid():
        return False
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return True
    try:
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        return prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    except Exception as e:
        carb.log_warn(f"Failed to apply ArticulationRootAPI on '{robot_path}': {e}")
        return False


def _pick_robot_articulation_path(
    stage: Usd.Stage,
    spawn: str,
    asset_root: str,
    robot_seg: str,
    articulation_roots: list,
) -> str | None:
    """
    Isaac SingleArticulation must wrap a **PhysX articulation root**, not an arbitrary link Xform.
    Prefer a root that is an ancestor of ``asset_root/body``; else the shallowest root under spawn.
    """
    body_path_str = f"{asset_root}/body"
    body_sdf = Sdf.Path(body_path_str) if stage.GetPrimAtPath(body_path_str).IsValid() else None

    if articulation_roots:
        if body_sdf is not None:
            for r in sorted(articulation_roots, key=len):
                if body_sdf.HasPrefix(Sdf.Path(r)):
                    return r
        return sorted(articulation_roots, key=len)[0]

    # Explicit hints: only accept if that prim is actually an articulation root
    hinted = []
    if robot_seg:
        hinted.append(f"{asset_root}/{robot_seg}")
        hinted.append(f"{spawn}/{robot_seg}")
    hinted.extend([asset_root, spawn, f"{spawn}/body", f"{asset_root}/body"])
    for path in _dedupe_paths(hinted):
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    return None


def _resolve_asset_root_for_body(stage: Usd.Stage, spawn: str, inner: str) -> str:
    """
    Base path such that ``{asset_root}/body`` is the main airframe link (forces, state, sensors).

    Handles:
    - ``spawn/body`` when the reference flattens the default prim (Iris, many assets).
    - ``spawn/<defaultPrim>/body`` e.g. pegasus ``fixed_wing.usd`` (defaultPrim ``World`` → …/World/body).
    """
    def valid(path: str) -> bool:
        return bool(path) and stage.GetPrimAtPath(path).IsValid()

    if inner and valid(f"{spawn}/{inner}/body"):
        return f"{spawn}/{inner}"
    if valid(f"{spawn}/body"):
        return spawn

    spawn_prim = stage.GetPrimAtPath(spawn)
    if not spawn_prim.IsValid():
        return f"{spawn}/{inner}" if inner and valid(f"{spawn}/{inner}") else spawn

    # Depth-first: first ``body`` under spawn. Prefer prims that already expose physics APIs;
    # if none do yet (common immediately after reference), use any Xform named ``body``.
    with_physics: list[tuple[int, str]] = []
    any_body: list[tuple[int, str]] = []

    def walk(prim: Usd.Prim) -> None:
        for ch in prim.GetChildren():
            if ch.GetName() == "body" and ch.IsValid():
                parent = ch.GetPath().GetParentPath().pathString
                tup = (len(parent), parent)
                any_body.append(tup)
                if ch.HasAPI(UsdPhysics.RigidBodyAPI) or ch.HasAPI(UsdPhysics.MassAPI):
                    with_physics.append(tup)
            walk(ch)

    walk(spawn_prim)
    pick = with_physics if with_physics else any_body
    if pick:
        pick.sort(key=lambda x: (x[0], x[1]))
        return pick[0][1]

    if inner and valid(f"{spawn}/{inner}"):
        return f"{spawn}/{inner}"
    return spawn


def _resolve_vehicle_paths_after_reference(
    stage: Usd.Stage, spawn: str, inner: str, robot_seg: str
) -> tuple:
    """
    After composing the asset reference under ``spawn``, find where ``body`` actually lives and
    which prim should back Isaac's Robot (articulation root). Default-prim merge often places
    ``body`` directly under ``spawn``, not under ``spawn/<defaultPrimName>``.
    """
    asset_root = _resolve_asset_root_for_body(stage, spawn, inner)

    articulation_roots = _collect_articulation_root_paths(stage, spawn)
    robot_path = _pick_robot_articulation_path(
        stage, spawn, asset_root, robot_seg, articulation_roots
    )
    robot_via_body_fallback = False
    if robot_path is None:
        robot_path = _fallback_robot_prim_from_body_link(stage, spawn, asset_root)
        robot_via_body_fallback = robot_path is not None

    if robot_path is None:
        raise RuntimeError(
            f"No articulation root found under '{spawn}' (ArticulationRootAPI scan found "
            f"{len(articulation_roots)} prim(s)), and no ``body`` child under asset_root/spawn. "
            "For custom assets, add UsdPhysics.ArticulationRootAPI on the vehicle root in USD. "
            f"Resolved asset_root for forces was '{asset_root}'."
        )

    api_ready = _ensure_articulation_root_api(stage, robot_path)
    if not api_ready:
        carb.log_warn(
            f"Selected robot articulation prim '{robot_path}' has no ArticulationRootAPI. "
            "SingleArticulation initialization may fail."
        )

    # Quiet default: stock pegasus assets use spawn + spawn/body with no env hints — no extra noise.
    user_hinted_layout = bool(inner or robot_seg)
    non_default_layout = asset_root != spawn or robot_path not in (spawn, f"{spawn}/body")
    if user_hinted_layout or non_default_layout:
        how = "body fallback (API not composed yet)" if robot_via_body_fallback else "ArticulationRootAPI"
        carb.log_info(
            f"Vehicle USD paths resolved: asset_root='{asset_root}' (for /body, /propeller, …), "
            f"robot_articulation='{robot_path}' ({how})"
        )

    return asset_root, robot_path


def _ensure_articulation_root_xform_orient(prim: Usd.Prim) -> None:
    """
    Isaac Sim ``set_world_poses`` / ``set_local_poses`` requires a typed ``xformOp:orient`` on the
    articulation root. With ``reset_xform_properties=False`` we skip the pass that creates it.

    Many DCC exports use only ``rotateXYZ`` on ``body``; add identity ``orient`` so spawn works.
    """
    if not prim.IsValid():
        return
    attr_name = "xformOp:orient"
    xformable = UsdGeom.Xformable(prim)
    if prim.HasAttribute(attr_name):
        attr = prim.GetAttribute(attr_name)
        if attr.GetTypeName():
            return
        try:
            xformable.DeleteXformOp(UsdGeom.XformOp(attr))
        except Exception:
            try:
                prim.RemoveProperty(attr_name)
            except Exception:
                carb.log_warn(
                    f"Could not remove invalid {attr_name} on {prim.GetPath()}; "
                    "Isaac may fail to set spawn orientation."
                )
                return
    orient_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
    orient_op.Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


# region agent log
_AGENT_DEBUG_LOG = "/home/nikolaos/workspaces/isaac-uav/.cursor/debug-43a8f7.log"
_AGENT_SESSION = "43a8f7"


def _agent_debug_log(
    hypothesis_id: str,
    location: str,
    message: str,
    data: dict,
    run_id: str = "pre-fix",
) -> None:
    try:
        with open(_AGENT_DEBUG_LOG, "a", encoding="utf-8") as _f:
            _f.write(
                json.dumps(
                    {
                        "sessionId": _AGENT_SESSION,
                        "hypothesisId": hypothesis_id,
                        "location": location,
                        "message": message,
                        "data": data,
                        "timestamp": int(time.time() * 1000),
                        "runId": run_id,
                    },
                    default=str,
                )
                + "\n"
            )
    except Exception:
        pass


def _agent_flu_axes_stage(stage: Usd.Stage, path: str) -> dict:
    p = stage.GetPrimAtPath(path)
    if not p.IsValid():
        return {"path": path, "valid": False}
    xb = UsdGeom.Xformable(p)
    wt = xb.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def _n(axis: Gf.Vec3d):
        v = Gf.GetNormalized(wt.TransformDir(axis))
        return [round(float(v[0]), 5), round(float(v[1]), 5), round(float(v[2]), 5)]

    ops = [f"{op.GetName()}={op.Get()}" for op in xb.GetOrderedXformOps()]
    return {
        "path": path,
        "valid": True,
        "world_plusX": _n(Gf.Vec3d(1, 0, 0)),
        "world_plusZ": _n(Gf.Vec3d(0, 0, 1)),
        "xform_ops": ops,
    }


# endregion


class Vehicle(SingleArticulation):
    _body_axis_align_log_done = False

    def __init__(
        self,
        stage_prefix: str,
        usd_path: str = None,
        init_pos=[0.0, 0.0, 0.0],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        sensors=[],
        graphical_sensors=[],
        graphs=[],
        backends=[],
        usd_inner_prefix: str = "",
        robot_articulation_suffix: str = "",
    ):
        """
        Class that initializes a vehicle in the isaac sim's curent stage

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "quadrotor".
            usd_path (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.0].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
            usd_inner_prefix (str): First Xform under the spawn root where /body, /propeller, etc. live (e.g. "Shahed").
            robot_articulation_suffix (str): Extra segment under that root for the PhysX articulation root Isaac Robot
                binds to (often "" if root is on the inner Xform, or "body" if ArticulationRootAPI is on body).
        """

        # Get the current world at which we want to spawn the vehicle
        self._world = PegasusInterface().world
        self._current_stage = self._world.stage

        # Save the name with which the vehicle will appear in the stage
        # and the name of the .usd file that contains its description
        self._stage_prefix = get_stage_next_free_path(self._current_stage, stage_prefix, False)
        self._usd_file = usd_path

        inner = _usd_path_segment(usd_inner_prefix)
        robot_seg = _usd_path_segment(robot_articulation_suffix)

        # Get the vehicle name by taking the last part of vehicle stage prefix
        self._vehicle_name = self._stage_prefix.rpartition("/")[-1]

        # Spawn the vehicle primitive in the world's stage
        self._prim = define_prim(self._stage_prefix, "Xform")
        self._prim = get_prim_at_path(self._stage_prefix)
        self._prim.GetReferences().AddReference(self._usd_file)
        # print(f"\n@ Vehicle: {self._prim}")

        self._asset_root, self._robot_prim_path = _resolve_vehicle_paths_after_reference(
            self._current_stage, self._stage_prefix, inner, robot_seg
        )

        # region agent log
        _agent_debug_log(
            "H-B",
            "vehicle.py:after_resolve_paths",
            "asset_root and robot prim",
            {
                "stage_prefix": self._stage_prefix,
                "usd_file": self._usd_file,
                "inner": inner,
                "robot_seg": robot_seg,
                "asset_root": self._asset_root,
                "robot_prim_path": self._robot_prim_path,
                "body_path": f"{self._asset_root}/body",
            },
        )
        _agent_debug_log(
            "H-C",
            "vehicle.py:before_ensure_orient",
            "FLU axes articulation root",
            {
                "robot": _agent_flu_axes_stage(self._current_stage, self._robot_prim_path),
                "body_mesh": _agent_flu_axes_stage(
                    self._current_stage, f"{self._asset_root}/body/body_mesh"
                ),
            },
        )
        # endregion

        _ensure_articulation_root_xform_orient(
            self._current_stage.GetPrimAtPath(self._robot_prim_path)
        )

        # region agent log
        _agent_debug_log(
            "H-E",
            "vehicle.py:after_ensure_orient",
            "FLU axes after adding xformOp:orient",
            {
                "robot": _agent_flu_axes_stage(self._current_stage, self._robot_prim_path),
            },
        )
        _agent_debug_log(
            "H-A",
            "vehicle.py:before_single_articulation",
            "spawn pose inputs",
            {
                "init_pos": list(init_pos),
                "init_orientation_qxyzw": list(init_orientation),
                "isaac_orientation_wxyz": [
                    init_orientation[3],
                    init_orientation[0],
                    init_orientation[1],
                    init_orientation[2],
                ],
            },
        )
        # endregion

        # SingleArticulation (same stack as isaacsim Robot) with reset_xform_properties=False so Isaac does not run
        # XFormPrim._set_xform_properties(), which strips rotateXYZ / pivot stacks and rebuilds translate+orient+scale
        # on the articulation root — that can discard Composer edits and break DCC-exported rigs (e.g. fixed-wing body).
        # Quaternion: Isaac expects scalar-first [w, x, y, z]; init_orientation is [qx, qy, qz, qw].
        SingleArticulation.__init__(
            self,
            prim_path=self._robot_prim_path,
            name=self._stage_prefix,
            position=init_pos,
            orientation=[init_orientation[3], init_orientation[0], init_orientation[1], init_orientation[2]],
            articulation_controller=None,
            reset_xform_properties=False,
        )

        # region agent log
        _agent_debug_log(
            "H-D",
            "vehicle.py:after_single_articulation",
            "FLU axes after Isaac spawn (before World.reset in app)",
            {
                "robot": _agent_flu_axes_stage(self._current_stage, self._robot_prim_path),
                "body_mesh": _agent_flu_axes_stage(
                    self._current_stage, f"{self._asset_root}/body/body_mesh"
                ),
            },
        )
        # endregion

        self._vehicle_dc_interface = None

        # Add this object for the world to track, so that if we clear the world, this object is deleted from memory and
        # as a consequence, from the VehicleManager as well
        self._world.scene.add(self)

        # Add the current vehicle to the vehicle manager, so that it knows
        # that a vehicle was instantiated
        VehicleManager.get_vehicle_manager().add_vehicle(self._stage_prefix, self)

        # Variable that will hold the current state of the vehicle
        self._state = State()

        # Add a callback to the physics engine to update the current state of the system
        self._world.add_physics_callback(self._stage_prefix + "/state", self.update_state)

        # Add the update method to the physics callback if the world was received
        # so that we can apply forces and torques to the vehicle. Note, this method should        # be implemented in classes that inherit the vehicle object
        self._world.add_physics_callback(self._stage_prefix + "/update", self.update)

        # Set the flag that signals if the simulation is running or not
        self._sim_running = False

        # Add a callback to start/stop of the simulation once the play/stop button is hit
        self._world.add_timeline_callback(self._stage_prefix + "/start_stop_sim", self.sim_start_stop)

        # --------------------------------------------------------------------
        # -------------------- Add sensors to the vehicle --------------------
        # --------------------------------------------------------------------
        self._sensors = sensors
        
        for sensor in self._sensors:
            sensor.initialize(self, PegasusInterface().latitude, PegasusInterface().longitude, PegasusInterface().altitude)

        # Add callbacks to the physics engine to update each sensor at every timestep
        # and let the sensor decide depending on its internal update rate whether to generate new data
        self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

        # --------------------------------------------------------------------
        # -------------------- Add the graphical sensors to the vehicle ------
        # --------------------------------------------------------------------
        self._graphical_sensors = graphical_sensors

        for graphical_sensor in self._graphical_sensors:
            graphical_sensor.initialize(self)

        # Add callbacks to the rendering engine to update each graphical sensor at every timestep of the rendering engine
        self._world.add_render_callback(self._stage_prefix + "/GraphicalSensors", self.update_graphical_sensors)


        # --------------------------------------------------------------------
        # -------------------- Add the graphs to the vehicle -----------------
        # --------------------------------------------------------------------
        self._graphs = graphs

        # --------------------------------------------------------------------
        # -------------------- Add Tracking Camera ---------------------------
        # --------------------------------------------------------------------
        
        # 1. Define a path for the camera (placing it outside the vehicle hierarchy for a 'fixed' position)
        camera_path = self._stage_prefix + "_tracking_cam"
        self._tracking_cam = define_prim(self._asset_root + "/body" + camera_path, "Camera")

        xformable = UsdGeom.Xformable(self._tracking_cam)
        
        # This ensures the 'xformOp:translate' attribute is actually created before setting it
        # Clear existing ops to avoid conflicts and add a clean translation op
        xformable.ClearXformOpOrder()
        translate_op = xformable.AddTranslateOp()
        rotate_op = xformable.AddRotateXYZOp()
        
        # Some magic numbers :)
        offset_pos = Gf.Vec3d(
            init_pos[0] + 14.82, 
            init_pos[1] + 14.12, 
            init_pos[2] + 9.49
        )

        translate_op.Set(offset_pos)

        # Some magic numbers here also :)
        rotate_op.Set(Gf.Vec3f(64.994, -1.141, 134.45))

        for graph in self._graphs:
            graph.initialize(self)
        
        # --------------------------------------------------------------------
        # ---- Add (communication/control) backends to the vehicle -----------
        # --------------------------------------------------------------------
        self._backends = backends

        # Initialize the backends
        for backend in self._backends:
            backend.initialize(self)

        # Add a callbacks for the
        self._world.add_physics_callback(self._stage_prefix + "/mav_state", self.update_sim_state)


    def __del__(self):
        """
        Method that is invoked when a vehicle object gets destroyed. When this happens, we also invoke the 
        'remove_vehicle' from the VehicleManager in order to remove the vehicle from the list of active vehicles.
        """

        # Remove this object from the vehicleHandler
        VehicleManager.get_vehicle_manager().remove_vehicle(self._stage_prefix)

    """
    Properties
    """

    @property
    def state(self):
        """The state of the vehicle.

        Returns:
            State: The current state of the vehicle, i.e., position, orientation, linear and angular velocities...
        """
        return self._state
    
    @property
    def vehicle_name(self) -> str:
        """Vehicle name.

        Returns:
            Vehicle name (str): last prim name in vehicle prim path
        """
        return self._vehicle_name

    """
    Operations
    """

    def sim_start_stop(self, event):
        """
        Callback that is called every time there is a timeline event such as starting/stoping the simulation.

        Args:
            event: A timeline event generated from Isaac Sim, such as starting or stoping the simulation.
        """

        # If the start/stop button was pressed, then call the start and stop methods accordingly
        if self._world.is_playing() and self._sim_running == False:
            self._sim_running = True

            # Initialize the sensors
            for sensor in self._sensors:
                sensor.start()

            # Initialize the graphical sensors
            for graphical_sensor in self._graphical_sensors:
                graphical_sensor.start()

            # Intializes the communication with all the backends. This method is invoked automatically when the simulation starts
            for backend in self._backends:
                backend.start()

            # Invoke the start method of the vehicle (if it exists)
            self.start()

        if self._world.is_stopped() and self._sim_running == True:
            self._sim_running = False

            # Reset the DC interface
            self._vehicle_dc_interface = None

            # Stop the sensors
            for sensor in self._sensors:
                sensor.stop()

            # Stop the graphical sensors
            for graphical_sensor in self._graphical_sensors:
                graphical_sensor.stop()

            # Signal all the backends that the simulation has stoped. This method is invoked automatically when the simulation stops
            for backend in self._backends:
                backend.stop()

            self.stop()

    def apply_force(self, force, pos=[0.0, 0.0, 0.0], body_part="/body"):
        """
        Method that will apply a force on the rigidbody, on the part specified in the 'body_part' at its relative position
        given by 'pos' (following a FLU) convention. 

        Args:
            force (list): A 3-dimensional vector of floats with the force [Fx, Fy, Fz] on the body axis of the vehicle according to a FLU convention.
            pos (list): _description_. Defaults to [0.0, 0.0, 0.0].
            body_part (str): . Defaults to "/body".
        """

        # Get the handle of the rigidbody that we will apply the force to
        rb = self.get_dc_interface().get_rigid_body(self._asset_root + body_part)
        carb.log_info(f"Applying force to '{self._asset_root + body_part}', F = {force}, At = {pos}")

        # Apply the force to the rigidbody. The force should be expressed in the rigidbody frame
        self.get_dc_interface().apply_body_force(rb, carb._carb.Float3(force), carb._carb.Float3(pos), False)

    def apply_torque(self, torque, body_part="/body"):
        """
        Method that when invoked applies a given torque vector to /<rigid_body_name>/"body" or to /<rigid_body_name>/<body_part>.

        Args:
            torque (list): A 3-dimensional vector of floats with the force [Tx, Ty, Tz] on the body axis of the vehicle according to a FLU convention.
            body_part (str): . Defaults to "/body".
        """

        # Get the handle of the rigidbody that we will apply a torque to
        rb = self.get_dc_interface().get_rigid_body(self._asset_root + body_part)

        # Apply the torque to the rigidbody. The torque should be expressed in the rigidbody frame
        self.get_dc_interface().apply_body_torque(rb, carb._carb.Float3(torque), False)

    def update_state(self, dt: float):
        """
        Method that is called at every physics step to retrieve and update the current state of the vehicle, i.e., get
        the current position, orientation, linear and angular velocities and acceleration of the vehicle.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the body frame interface of the vehicle (this will be the frame used to get the position, orientation, etc.)
        body = self.get_dc_interface().get_rigid_body(self._asset_root + "/body")

        # Get the current position and orientation in the inertial frame
        pose = self.get_dc_interface().get_rigid_body_pose(body)
        self._state.position = np.array(pose.p)

        # Attitude: prefer PhysX rigid-body quaternion (DcTransform.r is [qx,qy,qz,qw] per Isaac docs).
        # Position already comes from the same pose; using USD ``get_world_transform_matrix`` here
        # can disagree with PhysX on some DCC exports (stacked xformOps, pivots, non-uniform scale),
        # which shows up as "wrong orientation" in ArduPilot while the mesh looks fine.
        quat_phys = getattr(pose, "r", None)
        if quat_phys is not None and len(quat_phys) >= 4:
            self._state.attitude = np.array(
                [
                    float(quat_phys[0]),
                    float(quat_phys[1]),
                    float(quat_phys[2]),
                    float(quat_phys[3]),
                ],
                dtype=float,
            )
            n = np.linalg.norm(self._state.attitude)
            if n > 1e-12:
                self._state.attitude /= n
        else:
            prim = self._world.stage.GetPrimAtPath(self._asset_root + "/body")
            rotation_quat = get_world_transform_xform(prim).GetQuaternion()
            rotation_quat_real = rotation_quat.GetReal()
            rotation_quat_img = rotation_quat.GetImaginary()
            self._state.attitude = np.array(
                [rotation_quat_img[0], rotation_quat_img[1], rotation_quat_img[2], rotation_quat_real]
            )

        # Map rigid-body / USD body axes to Pegasus FLU (+X fwd, +Z up) for state, MAVLink, and aero.
        r_align = body_axis_align_rotation()
        if r_align is not None:
            r_cur = Rotation.from_quat(self._state.attitude)
            if body_axis_align_mode() == "pre":
                q_new = (r_align * r_cur).as_quat()
            else:
                q_new = (r_cur * r_align).as_quat()
            n = np.linalg.norm(q_new)
            if n > 1e-12:
                self._state.attitude = q_new / n
            if not Vehicle._body_axis_align_log_done:
                Vehicle._body_axis_align_log_done = True
                _m = body_axis_align_mode()
                carb.log_info(
                    f"PEGASUS_BODY_AXIS_ALIGN_EULER_DEG is set (mode={_m}): "
                    f"{'R_align * R_rigid' if _m == 'pre' else 'R_rigid * R_align'} on state.attitude "
                    f"(intrinsic XYZ deg). If ArduPilot looks upside-down, unset align or try mode=pre / "
                    f"different Euler (90,0,0 is roll about +X, not yaw for nose on +Y)."
                )

        # Get the angular velocity of the vehicle expressed in the body frame of reference
        ang_vel = self.get_dc_interface().get_rigid_body_angular_velocity(body)

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        linear_vel = self.get_dc_interface().get_rigid_body_linear_velocity(body)

        # Get the linear acceleration of the body relative to the inertial frame, expressed in the inertial frame
        # Note: we must do this approximation, since the Isaac sim does not output the acceleration of the rigid body directly
        linear_acceleration = (np.array(linear_vel) - self._state.linear_velocity) / dt

        # Express the velocity of the vehicle in the inertial frame X_dot = [x_dot, y_dot, z_dot]
        self._state.linear_velocity = np.array(linear_vel)

        # The linear velocity V =[u,v,w] of the vehicle's body frame expressed in the body frame of reference
        # Note that: x_dot = Rot * V
        self._state.linear_body_velocity = (
            Rotation.from_quat(self._state.attitude).inv().apply(self._state.linear_velocity)
        )

        # omega = [p,q,r]
        self._state.angular_velocity = Rotation.from_quat(self._state.attitude).inv().apply(np.array(ang_vel))

        # The acceleration of the vehicle expressed in the inertial frame X_ddot = [x_ddot, y_ddot, z_ddot]
        self._state.linear_acceleration = linear_acceleration

    def start(self):
        """
        Method that should be implemented by the class that inherits the vehicle object.
        """
        pass

    def stop(self):
        """
        Method that should be implemented by the class that inherits the vehicle object.
        """
        pass

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type and it's called periodically by the physics engine.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        pass

    def update_sensors(self, dt: float):
        """Callback that is called at every physics steps and will call the sensor.update method to generate new
        sensor data. For each data that the sensor generates, the backend.update_sensor method will also be called for
        every backend. For example, if new data is generated for an IMU and we have a PX4MavlinkBackend, then the update_sensor
        method will be called for that backend so that this data can latter be sent thorugh mavlink.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Call the update method for the sensor to update its values internally (if applicable)
        for sensor in self._sensors:
            sensor_data = sensor.update(self._state, dt)

            # If some data was updated and we have a mavlink backend or ros backend (or other), then just update it
            if sensor_data is not None:
                for backend in self._backends:
                    backend.update_sensor(sensor.sensor_type, sensor_data)

    def update_graphical_sensors(self, event):
        """Callback that is called at every rendering steps and will call the graphical_sensor.update method to generate new
        sensor data. For each data that the sensor generates, the backend.update_graphical_sensor method will also be called for
        every backend. For example, if new data is generated for a monocular camera and we have a ROS2Backend, then the update_graphical_sensor
        method will be called for that backend so that this data can latter be sent through a ROS2 topic.

        Args:
            event (float): The timer event that contains the time elapsed between the previous and current function calls (s).
        """

        # Call the update method for the sensor to update its values internally (if applicable)
        for sensor in self._graphical_sensors:
            sensor_data = sensor.update(self._state, event.payload['dt'])

            # If some data was updated and we have a ros backend (or other), then just update it
            if sensor_data is not None:
                for backend in self._backends:
                    backend.update_graphical_sensor(sensor.sensor_type, sensor_data)

    def update_sim_state(self, dt: float):
        """
        Callback that is used to "send" the current state for each backend being used to control the vehicle. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        for backend in self._backends:
            backend.update_state(self._state)

    def get_dc_interface(self):

        if self._vehicle_dc_interface is None:
            self._vehicle_dc_interface = _dynamic_control.acquire_dynamic_control_interface()

        return self._vehicle_dc_interface