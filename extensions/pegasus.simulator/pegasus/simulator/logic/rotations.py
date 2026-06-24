"""
| File: rotations.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Implements utilitary rotations between ENU and NED inertial frame conventions and FLU and FRD body frame conventions.
"""
import os
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

# ``PEGASUS_BODY_AXIS_ALIGN_EULER_DEG`` cache.
_cached_body_align_env: str | None = None
_cached_body_align_rot: Rotation | None = None

# Quaternion for rotation between ENU and NED INERTIAL frames
# NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
# ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
# This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
# Note: this quaternion follows the convention [qx, qy, qz, qw]
q_ENU_to_NED = np.array([0.70711, 0.70711, 0.0, 0.0])

# A scipy rotation from the ENU inertial frame to the NED inertial frame of reference
rot_ENU_to_NED = Rotation.from_quat(q_ENU_to_NED)

# Quaternion for rotation between body FLU and body FRD frames
# +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
# to Forward, Left, Up (base_link) frames and vice-versa.
# This rotation is symmetric, so q_FLU_to_FRD == q_FRD_to_FLU.
# Note: this quaternion follows the convention [qx, qy, qz, qw]
q_FLU_to_FRD = np.array([1.0, 0.0, 0.0, 0.0])

# A scipy rotation from the FLU body frame to the FRD body frame
rot_FLU_to_FRD = Rotation.from_quat(q_FLU_to_FRD)


def usd_body_prim_frame_is_frd() -> bool:
    """
    Pegasus ``State.attitude`` is normally **FLU** (+X forward, +Z up) relative to ENU.

    Set ``PEGASUS_USD_BODY_FRAME=FRD`` **only** when the rigid ``body`` prim’s **local** axes are
    aerospace **FRD** (+Z down). That flag removes the internal 180°-about-X FLU→FRD step in the
    MAVLink attitude path; it does **not** fix mesh axes that are off by 90° (nose along +Y, etc.).
    For constant roll/pitch/yaw offsets between the USD/PhysX body and Pegasus FLU, use
    ``PEGASUS_BODY_AXIS_ALIGN_EULER_DEG`` instead (see ``body_axis_align_rotation``).

    A world spawn Euler (``PEGASUS_FIXEDWING_SPAWN_EULER_DEG``) rotates the asset in Isaac but
    does not change the body-frame convention flags above.
    """
    return os.environ.get("PEGASUS_USD_BODY_FRAME", "").strip().upper() == "FRD"


def body_axis_align_mode() -> str:
    """
    How ``R_align`` combines with the rigid-body attitude (see ``body_axis_align_rotation``).

    ``PEGASUS_BODY_AXIS_ALIGN_MODE`` (default ``post``):

    - ``post`` / ``right`` / ``after``: ``R_enu_flu = R_enu_rigid * R_align`` — correct **body**
      axis labeling (nose along +Y in the mesh, etc.).
    - ``pre`` / ``left`` / ``before``: ``R_enu_flu = R_align * R_enu_rigid`` — fixed rotation in
      **world** frame first; try if ``post`` makes ArduPilot show inverted (upside-down) while
      the mesh orientation error is different.
    """
    m = os.environ.get("PEGASUS_BODY_AXIS_ALIGN_MODE", "post").strip().lower()
    if m in ("pre", "left", "before"):
        return "pre"
    return "post"


def body_axis_align_rotation() -> Rotation | None:
    """
    Optional fixed rotation ``R_align`` (from env). Combine with rigid attitude using
    ``body_axis_align_mode()`` (default **post**):

    - **post:** ``R_enu_flu = R_enu_rigid * R_align``
    - **pre:** ``R_enu_flu = R_align * R_enu_rigid``

    ``R_align = Rotation.from_euler("XYZ", parts, degrees=True)`` (intrinsic **XYZ**, same
    convention as ``PEGASUS_FIXEDWING_SPAWN_EULER_DEG`` in the examples).

    Environment: ``PEGASUS_BODY_AXIS_ALIGN_EULER_DEG=roll,pitch,yaw`` (degrees). Unset or empty
    disables alignment.

    **Intrinsic ``90,0,0``** is roll about **body +X** (forward). It rotates which axis is
    “up” in the logical FLU frame and often makes the AHRS look **inverted** if your real problem
    was nose along **+Y** or **+Z** (try **yaw** ``0,0,±90`` or **pitch** ``0,±90,0`` instead).

    Do **not** set ``PEGASUS_USD_BODY_FRAME=FRD`` unless the prim is really FRD.
    """
    global _cached_body_align_env, _cached_body_align_rot
    raw = os.environ.get("PEGASUS_BODY_AXIS_ALIGN_EULER_DEG", "").strip()
    if raw == _cached_body_align_env:
        return _cached_body_align_rot

    _cached_body_align_env = raw
    if not raw:
        _cached_body_align_rot = None
        return None
    try:
        parts = [float(x.strip()) for x in raw.split(",")]
        if len(parts) != 3:
            raise ValueError("expected three comma-separated numbers")
        _cached_body_align_rot = Rotation.from_euler("XYZ", parts, degrees=True)
        return _cached_body_align_rot
    except (ValueError, TypeError) as exc:
        warnings.warn(
            f"PEGASUS_BODY_AXIS_ALIGN_EULER_DEG={raw!r} ignored ({exc}).",
            UserWarning,
            stacklevel=2,
        )
        _cached_body_align_rot = None
        return None


def attitude_ned_frd_quat_from_body_to_enu(body_q_xyzw) -> np.ndarray:
    """
    Quaternion [qx,qy,qz,qw] for FRD body vs NED, from the body's world attitude quaternion
    (same convention as ``State.attitude``).
    """
    r_body_enu = Rotation.from_quat(body_q_xyzw)
    if usd_body_prim_frame_is_frd():
        return (rot_ENU_to_NED * r_body_enu).as_quat()
    return (rot_ENU_to_NED * r_body_enu * rot_FLU_to_FRD).as_quat()


def angular_velocity_to_frd_for_publish(omega_body: np.ndarray) -> np.ndarray:
    """Angular rate in body frame, for ArduPilot FRD when the USD body is FLU or FRD."""
    w = np.asarray(omega_body, dtype=float)
    if usd_body_prim_frame_is_frd():
        return w.copy()
    return rot_FLU_to_FRD.apply(w)


def linear_acceleration_to_frd_for_publish(accel_body: np.ndarray) -> np.ndarray:
    """Linear acceleration in body frame (from ``State``), for ArduPilot FRD output."""
    a = np.asarray(accel_body, dtype=float)
    if usd_body_prim_frame_is_frd():
        return a.copy()
    return rot_FLU_to_FRD.apply(a)
