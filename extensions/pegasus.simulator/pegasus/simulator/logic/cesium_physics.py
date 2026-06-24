"""
PhysX collision for Cesium ion tilesets so Pegasus vehicles contact terrain (static colliders).

Isaac default environments (e.g. Black Gridroom) load under /World/environment with explicit
physics:collisionEnabled on ground meshes. Cesium tilesets are created at the stage root; we
reparent them under a single Xform (default /World/environment/cesium) when enabled so Google tiles,
OSM buildings, and terrain share one parent under World, then apply the same CollisionAPI /
PhysxCollisionAPI / MeshCollisionAPI pattern as typical Isaac assets.
"""

from __future__ import annotations

import inspect
import carb
from typing import List, Optional


def _sanitize_usd_selection(stage) -> None:
    """
    Keep only selected paths that resolve to valid prims on the current stage.
    This avoids supportui manipulators touching stale/null prims after scene edits.
    """
    try:
        import omni.usd

        ctx = omni.usd.get_context()
        sel = ctx.get_selection()
        if not stage:
            sel.set_selected_prim_paths([], False)
            return
        paths = list(sel.get_selected_prim_paths())
        valid = []
        for p in paths:
            p_str = str(p).strip()
            if not p_str or not p_str.startswith("/"):
                continue
            if stage.GetPrimAtPath(p_str).IsValid():
                valid.append(p_str)
        if len(valid) != len(paths):
            sel.set_selected_prim_paths(valid, False)
    except Exception:
        pass


def _remap_usd_selection_paths_for_move(stage, src_path: str, dst_path: str) -> None:
    """
    Remap selected paths under src_path to dst_path after a MovePrim operation,
    then drop any paths that are still invalid.
    """
    try:
        import omni.usd

        ctx = omni.usd.get_context()
        sel = ctx.get_selection()
        original = list(sel.get_selected_prim_paths())
        src = str(src_path).rstrip("/")
        dst = str(dst_path).rstrip("/")
        prefix = src + "/"
        updated = []
        for p in original:
            path = str(p)
            if path == src:
                path = dst
            elif path.startswith(prefix):
                path = dst + path[len(src) :]
            path = str(path).strip()
            if not path or not path.startswith("/"):
                continue
            if stage.GetPrimAtPath(path).IsValid():
                updated.append(path)
        if updated != [str(p) for p in original]:
            sel.set_selected_prim_paths(updated, False)
    except Exception:
        _sanitize_usd_selection(stage)


def get_cesium_fallback_ground_config() -> dict:
    """Return resolved cesium_fallback_ground options (same keys as configs.yaml section)."""
    return dict(_read_cesium_fallback_ground_config())


def _read_cesium_fallback_ground_config() -> dict:
    """
    Optional large invisible static collider at the Cesium local origin plane (georeference lat/lon/height).

    Pegasus global_coordinates.altitude matches Cesium georeference height (WGS84 ellipsoid meters). Terrain and
    buildings are not necessarily exactly at y=0 in local space, but this gives a flat safety net when tile
    collision is missing or incomplete. Tune height_offset_m against your site (e.g. from a DEM).
    """
    try:
        from pegasus.simulator.params import CONFIG_FILE
        import yaml

        with open(CONFIG_FILE, "r") as f:
            data = yaml.safe_load(f) or {}
        section = data.get("cesium_fallback_ground", {}) or {}
        surf = str(section.get("collision_surface", "plane")).strip().lower() or "plane"
        if surf not in ("plane", "mesh"):
            surf = "plane"
        return {
            "enabled": bool(section.get("enabled", False)),
            "half_extent_m": float(section.get("half_extent_m", 5000.0)),
            "height_offset_m": float(section.get("height_offset_m", 0.0)),
            "thickness_m": max(1e-3, float(section.get("thickness_m", 0.2))),
            "prim_path": str(section.get("prim_path", "/World/environment/CesiumFallbackGround")).strip()
            or "/World/environment/CesiumFallbackGround",
            # Pegasus uses ENU (Z-up); the stage must match for correct physics and sensor math.
            "local_up_axis": str(section.get("local_up_axis", "Z")).strip().upper() or "Z",
            # "plane" = Isaac GroundPlane (PhysX half-space); "mesh" = legacy thin box mesh collider.
            "collision_surface": surf,
        }
    except Exception:
        return {
            "enabled": False,
            "half_extent_m": 5000.0,
            "height_offset_m": 0.0,
            "thickness_m": 0.2,
            "prim_path": "/World/environment/CesiumFallbackGround",
            "local_up_axis": "Z",
            "collision_surface": "plane",
        }


def _import_isaac_ground_plane_class():
    try:
        from isaacsim.core.api.objects.ground_plane import GroundPlane

        return GroundPlane
    except Exception:
        pass
    try:
        from isaacsim.core.api.objects import GroundPlane

        return GroundPlane
    except Exception:
        pass
    try:
        from omni.isaac.core.objects import GroundPlane

        return GroundPlane
    except Exception:
        return None


def _hide_prim_subtree_invisible(root_prim) -> None:
    try:
        from pxr import Usd, UsdGeom
    except ImportError:
        return
    try:
        for prim in Usd.PrimRange(root_prim):
            if not prim.IsValid():
                continue
            try:
                UsdGeom.Imageable(prim).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            except Exception:
                pass
    except Exception:
        pass


def _apply_fallback_ground_transform(prim, height_offset_m: float, up: str) -> None:
    """Translate (and orient for Y-up) the fallback ground root prim."""
    try:
        from pxr import Gf, UsdGeom

        xf = UsdGeom.Xformable(prim)
        if not xf:
            carb.log_warn(f"Cesium fallback ground: prim is not xformable ({prim.GetPath()})")
            return

        # GroundPlane roots may carry an op stack that is incompatible with XformCommonAPI.
        # Author a clean explicit stack to avoid _GetOrAddCommonXformOps warnings.
        xf.ClearXformOpOrder()

        h = float(height_offset_m)
        t = Gf.Vec3f(0.0, 0.0, h) if up == "Z" else Gf.Vec3f(0.0, h, 0.0)
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat).Set(t)
        if up != "Z":
            # Isaac GroundPlane hardcodes "Z" as the up axis in PhysicsSchemaTools.addGroundPlane,
            # so the collision normal already points along +Z. For Y-up stages, rotate -90° about X
            # to tilt the normal from Z to Y.
            xf.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(-90.0, 0.0, 0.0))
    except Exception as exc:
        carb.log_warn(f"Cesium fallback ground: could not set transform ({exc})")


def _ensure_cesium_fallback_ground_plane(stage, cfg: dict) -> bool:
    """
    Spawn Isaac Sim GroundPlane (PhysX infinite plane / half-space), not a triangle mesh floor.
    """
    GroundPlane = _import_isaac_ground_plane_class()
    if GroundPlane is None:
        return False

    try:
        from pxr import Sdf
    except ImportError:
        return False

    half = max(1.0, float(cfg["half_extent_m"]))
    y_off = float(cfg["height_offset_m"])
    up = cfg["local_up_axis"]
    prim_path = cfg["prim_path"]
    if not prim_path.startswith("/"):
        prim_path = "/" + prim_path

    parent_path = str(Sdf.Path(prim_path).GetParentPath())
    if parent_path and parent_path != "/":
        _ensure_xform_path_exists(stage, parent_path)

    p = Sdf.Path(prim_path)
    if stage.GetPrimAtPath(p).IsValid():
        try:
            stage.RemovePrim(p)
        except Exception:
            pass

    size = max(half * 2.0, 100.0)
    params = inspect.signature(GroundPlane.__init__).parameters
    kwargs = {"prim_path": prim_path}
    if "size" in params:
        kwargs["size"] = size
    if "color" in params:
        try:
            import numpy as np

            kwargs["color"] = np.array([0.15, 0.15, 0.15], dtype=float)
        except Exception:
            pass
    if "visible" in params:
        kwargs["visible"] = False

    try:
        GroundPlane(**kwargs)
    except TypeError:
        try:
            GroundPlane(prim_path=prim_path)
        except Exception as exc:
            carb.log_warn(f"Cesium fallback ground: GroundPlane ctor failed ({exc})")
            return False
    except Exception as exc:
        carb.log_warn(f"Cesium fallback ground: GroundPlane failed ({exc})")
        return False

    root = stage.GetPrimAtPath(p)
    if not root.IsValid():
        carb.log_warn(f"Cesium fallback ground: GroundPlane did not create {prim_path}")
        return False

    _apply_fallback_ground_transform(root, y_off, up)
    _hide_prim_subtree_invisible(root)

    carb.log_info(
        f"Cesium fallback ground: Isaac GroundPlane (half-space) at {prim_path} "
        f"(size={size} m, height_offset={y_off} m, up={up})."
    )
    return True


def _ensure_cesium_fallback_ground_mesh(stage, cfg: dict) -> bool:
    """Legacy thin box mesh with PhysX mesh collision (finite extent)."""
    try:
        from pxr import Gf, Sdf, UsdGeom
    except ImportError:
        carb.log_warn("Cesium fallback ground: pxr not available.")
        return False

    half = max(1.0, float(cfg["half_extent_m"]))
    y_off = float(cfg["height_offset_m"])
    th = float(cfg["thickness_m"])
    prim_path = cfg["prim_path"]
    if not prim_path.startswith("/"):
        prim_path = "/" + prim_path

    parent_path = str(Sdf.Path(prim_path).GetParentPath())
    if parent_path and parent_path != "/":
        _ensure_xform_path_exists(stage, parent_path)

    up = cfg["local_up_axis"]
    if up == "Z":
        hx, hy, hz = half, half, th * 0.5
        center = Gf.Vec3f(0.0, 0.0, y_off)
    else:
        hx, hy, hz = half, th * 0.5, half
        center = Gf.Vec3f(0.0, y_off, 0.0)

    def box_vertices(cx: float, cy: float, cz: float) -> list:
        x0, x1 = cx - hx, cx + hx
        y0, y1 = cy - hy, cy + hy
        z0, z1 = cz - hz, cz + hz
        return [
            Gf.Vec3f(x0, y0, z0),
            Gf.Vec3f(x1, y0, z0),
            Gf.Vec3f(x1, y1, z0),
            Gf.Vec3f(x0, y1, z0),
            Gf.Vec3f(x0, y0, z1),
            Gf.Vec3f(x1, y0, z1),
            Gf.Vec3f(x1, y1, z1),
            Gf.Vec3f(x0, y1, z1),
        ]

    c = (float(center[0]), float(center[1]), float(center[2]))
    pts = box_vertices(c[0], c[1], c[2])
    if up == "Z":
        faces = [
            0,
            2,
            1,
            0,
            3,
            2,
            4,
            5,
            6,
            4,
            6,
            7,
            0,
            1,
            5,
            0,
            5,
            4,
            3,
            7,
            6,
            3,
            6,
            2,
            0,
            4,
            7,
            0,
            7,
            3,
            1,
            2,
            6,
            1,
            6,
            5,
        ]
    else:
        faces = [
            0,
            1,
            5,
            0,
            5,
            4,
            3,
            7,
            6,
            3,
            6,
            2,
            0,
            4,
            7,
            0,
            7,
            3,
            1,
            2,
            6,
            1,
            6,
            5,
            0,
            3,
            2,
            0,
            2,
            1,
            4,
            5,
            6,
            4,
            6,
            7,
        ]

    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.GetPointsAttr().Set(pts)
    mesh.GetFaceVertexCountsAttr().Set([3] * 12)
    mesh.GetFaceVertexIndicesAttr().Set(faces)
    mesh.CreateDoubleSidedAttr().Set(True)

    try:
        UsdGeom.Imageable(mesh.GetPrim()).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    except Exception:
        pass

    approx = _read_cesium_tile_collision_config().get("approximation", "mesh")
    if _apply_static_mesh_collider(mesh.GetPrim(), approx):
        carb.log_info(
            f"Cesium fallback ground: invisible mesh slab at {prim_path} "
            f"(half_extent={half} m, height_offset={y_off} m, up={up})."
        )
        return True
    return False


def ensure_cesium_fallback_ground(stage) -> bool:
    """
    Create (or replace) Cesium fallback collider: Isaac GroundPlane (default) or thin mesh box (optional).

    GroundPlane uses PhysX half-space collision (stable contact vs mesh planes). Mesh mode is finite extent.
    """
    cfg = _read_cesium_fallback_ground_config()
    if not cfg["enabled"]:
        return False
    if not stage:
        return False

    want = cfg.get("collision_surface", "plane")
    if want == "mesh":
        return _ensure_cesium_fallback_ground_mesh(stage, cfg)

    ok = _ensure_cesium_fallback_ground_plane(stage, cfg)
    if ok:
        return True
    carb.log_warn(
        "Cesium fallback ground: GroundPlane unavailable or failed; falling back to mesh slab. "
        "Set collision_surface: mesh in configs.yaml to silence this, or fix Isaac imports."
    )
    return _ensure_cesium_fallback_ground_mesh(stage, cfg)


def remove_cesium_fallback_ground(stage) -> bool:
    """Remove the fallback ground prim if present (e.g. after disabling the option at runtime)."""
    if not stage:
        return False
    cfg = _read_cesium_fallback_ground_config()
    prim_path = str(cfg.get("prim_path", "/World/environment/CesiumFallbackGround")).strip()
    if not prim_path.startswith("/"):
        prim_path = "/" + prim_path
    try:
        from pxr import Sdf

        p = Sdf.Path(prim_path)
        prim = stage.GetPrimAtPath(p)
        if not prim.IsValid():
            return False
        stage.RemovePrim(p)
        carb.log_info(f"Cesium fallback ground: removed {prim_path}")
        return True
    except Exception as exc:
        carb.log_warn(f"Cesium fallback ground: could not remove {prim_path}: {exc}")
        return False


def _read_cesium_tile_collision_config() -> dict:
    try:
        from pegasus.simulator.params import CONFIG_FILE
        import yaml

        with open(CONFIG_FILE, "r") as f:
            data = yaml.safe_load(f) or {}
        section = data.get("cesium_tile_collision", {}) or {}
        return {
            "enabled": bool(section.get("enabled", True)),
            "max_meshes": int(section.get("max_meshes", 800)),
            "approximation": str(section.get("approximation", "mesh")).strip() or "mesh",
            "settle_frames": int(section.get("settle_frames", 120)),
            "reparent_under_world_environment": bool(
                section.get(
                    "reparent_under_world_environment",
                    section.get("reparent_under_world_layout", True),
                )
            ),
            # Single Xform under /World/environment holding all Cesium tilesets (Google, OSM, terrain, etc.).
            "tilesets_parent_path": str(
                section.get("tilesets_parent_path", "/World/environment/cesium")
            ).strip()
            or "/World/environment/cesium",
            "pre_reparent_settle_frames": int(section.get("pre_reparent_settle_frames", 48)),
            "post_reparent_settle_frames": int(section.get("post_reparent_settle_frames", 32)),
            "disable_frustum_culling_for_physics": bool(section.get("disable_frustum_culling_for_physics", True)),
            "disable_fog_culling_for_physics": bool(section.get("disable_fog_culling_for_physics", True)),
            "collision_pass_retries": max(1, int(section.get("collision_pass_retries", 4))),
            "retry_settle_frames": int(section.get("retry_settle_frames", 90)),
            "disable_geometry_pool_for_physics": bool(section.get("disable_geometry_pool_for_physics", True)),
            "log_hierarchy_scales": bool(section.get("log_hierarchy_scales", False)),
        }
    except Exception:
        return {
            "enabled": True,
            "max_meshes": 800,
            "approximation": "mesh",
            "settle_frames": 120,
            "reparent_under_world_environment": True,
            "tilesets_parent_path": "/World/environment/cesium",
            "pre_reparent_settle_frames": 48,
            "post_reparent_settle_frames": 32,
            "disable_frustum_culling_for_physics": True,
            "disable_fog_culling_for_physics": True,
            "collision_pass_retries": 4,
            "retry_settle_frames": 90,
            "disable_geometry_pool_for_physics": True,
            "log_hierarchy_scales": False,
        }


def _apply_identity_xform_if_empty(prim) -> None:
    """
    New container Xforms from UsdGeom.Xform.Define often have no xform ops; Isaac/PhysX expect
    explicit meters-consistent stacking (scale 1,1,1). Only writes when the prim has no ordered ops.
    """
    try:
        from pxr import UsdGeom

        xf = UsdGeom.Xformable(prim)
        if xf.GetOrderedXformOps():
            return
        api = UsdGeom.XformCommonAPI(prim)
        api.SetTranslate((0.0, 0.0, 0.0))
        api.SetRotate((0.0, 0.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        api.SetScale((1.0, 1.0, 1.0))
    except Exception as exc:
        carb.log_warn(f"Cesium physics: could not set identity xform on {prim.GetPath()}: {exc}")


def _local_scale_triple(prim) -> Optional[tuple]:
    """Approximate local per-axis scale from the local matrix (column basis lengths)."""
    try:
        from pxr import Usd, UsdGeom, Gf

        if not prim.IsValid():
            return None
        xf = UsdGeom.Xformable(prim)
        m, _ = xf.GetLocalTransformation(Usd.TimeCode.Default())
        c0 = Gf.Vec3d(m[0][0], m[1][0], m[2][0])
        c1 = Gf.Vec3d(m[0][1], m[1][1], m[2][1])
        c2 = Gf.Vec3d(m[0][2], m[1][2], m[2][2])
        return (c0.GetLength(), c1.GetLength(), c2.GetLength())
    except Exception:
        return None


def log_cesium_hierarchy_scales(stage, tilesets_parent_path: str) -> None:
    """Log local scale (matrix column lengths) for World chain and each tileset root; warn if not ~1."""
    paths = ["/World", "/World/environment", tilesets_parent_path]
    try:
        from cesium.omniverse.usdUtils import get_tileset_paths

        for ts in get_tileset_paths():
            if ts not in paths:
                paths.append(ts)
    except Exception:
        pass

    eps = 1e-3
    summary: List[str] = []
    for p in paths:
        prim = stage.GetPrimAtPath(p)
        if not prim.IsValid():
            summary.append(f"{p}=<missing>")
            continue
        s = _local_scale_triple(prim)
        if s is None:
            summary.append(f"{p}=?")
            continue
        summary.append(f"{p}=({s[0]:.6g},{s[1]:.6g},{s[2]:.6g})")
        if abs(s[0] - 1.0) > eps or abs(s[1] - 1.0) > eps or abs(s[2] - 1.0) > eps:
            carb.log_warn(
                f"Cesium physics: local scale on {p} is not unit (expect 1,1,1 for meters): {s}"
            )

    carb.log_info("Cesium physics: hierarchy local scales — " + " ".join(summary))


def apply_cesium_debug_disable_geometry_pool_for_physx(stage) -> bool:
    """
    Cesium streams most tile geometry through a Fabric geometry pool; PhysX only cooks collision from
    USD UsdGeom.Mesh prims. Setting cesium:debug:disableGeometryPool on /Cesium (see Cesium Data
    debug options) disables pooling so meshes can appear on the USD stage for tagging.

    Must run before tile content loads (before ADD_ION_ASSET / add_tileset_ion) when possible.
    """
    cfg = _read_cesium_tile_collision_config()
    if not cfg.get("disable_geometry_pool_for_physics", True):
        return False
    try:
        from cesium.omniverse.usdUtils import get_or_create_cesium_data
    except ImportError as e:
        carb.log_warn(f"Cesium disableGeometryPool: skipped ({e}).")
        return False
    try:
        data = get_or_create_cesium_data()
        prim = data.GetPrim()
        if not prim.IsValid():
            return False
        attr = data.GetDebugDisableGeometryPoolAttr()
        if attr:
            attr.Set(True)
        else:
            data.CreateDebugDisableGeometryPoolAttr().Set(True)
        carb.log_info(
            "Cesium /Cesium: cesium:debug:disableGeometryPool=true so tile meshes can exist as USD prims for PhysX."
        )
        return True
    except Exception as exc:
        carb.log_warn(f"Cesium disableGeometryPool: could not set ({exc}).")
        return False


def log_cesium_tileset_usd_mesh_diagnostics(stage, tileset_paths: List[str], as_warning: bool = True) -> None:
    """When collision tagging finds 0 meshes, log what USD actually has under each tileset."""
    try:
        from pxr import Usd, UsdGeom
    except ImportError:
        return

    for ts_path in tileset_paths:
        root = stage.GetPrimAtPath(ts_path)
        log_fn = carb.log_warn if as_warning else carb.log_info
        if not root.IsValid():
            log_fn(f"Cesium tile collision diagnostics: invalid tileset root {ts_path}")
            continue
        mesh_count = 0
        n = 0
        type_counts: dict = {}
        for prim in Usd.PrimRange(root):
            n += 1
            if prim.IsA(UsdGeom.Mesh):
                mesh_count += 1
            tn = prim.GetTypeName()
            type_counts[tn] = type_counts.get(tn, 0) + 1
        top = sorted(type_counts.items(), key=lambda x: -x[1])[:15]
        log_fn(
            f"Cesium tile collision diagnostics for {ts_path}: total prims={n}, "
            f"UsdGeom.Mesh={mesh_count}, type histogram (top)={top}"
        )


def apply_cesium_tileset_streaming_settings_for_physics(stage) -> int:
    """
    Cesium tilesets default to frustum (and often fog) culling. Culled tiles are not refined and
    typically do not contribute USD mesh geometry that PhysX can use — the drone then has nothing
    to collide with unless those tiles are in view. For simulation, disable these culls so tile
    loading can populate collision meshes around the scene (higher load cost; needed for contact).

    Returns the number of tileset prims updated.
    """
    cfg = _read_cesium_tile_collision_config()
    if not cfg.get("disable_frustum_culling_for_physics") and not cfg.get("disable_fog_culling_for_physics"):
        return 0

    try:
        from cesium.usd.plugins.CesiumUsdSchemas import Tileset as CesiumTileset
        from cesium.omniverse.usdUtils import get_tileset_paths
    except ImportError as e:
        carb.log_warn(f"Cesium physics streaming settings: skipped ({e}).")
        return 0

    updated = 0
    for ts_path in get_tileset_paths():
        try:
            tileset = CesiumTileset.Get(stage, ts_path)
            if not tileset or not tileset.GetPrim().IsValid():
                continue
            if cfg.get("disable_frustum_culling_for_physics"):
                attr = tileset.GetEnableFrustumCullingAttr()
                if attr:
                    attr.Set(False)
                else:
                    tileset.CreateEnableFrustumCullingAttr().Set(False)
            if cfg.get("disable_fog_culling_for_physics"):
                attr = tileset.GetEnableFogCullingAttr()
                if attr:
                    attr.Set(False)
                else:
                    tileset.CreateEnableFogCullingAttr().Set(False)
            updated += 1
            carb.log_info(
                f"Cesium tileset physics: disabled view culling on {ts_path} "
                f"(frustum={cfg.get('disable_frustum_culling_for_physics')}, "
                f"fog={cfg.get('disable_fog_culling_for_physics')})."
            )
        except Exception as exc:
            carb.log_warn(f"Cesium tileset physics: could not update {ts_path}: {exc}")
    return updated


def _ensure_xform_path_exists(stage, abs_path: str) -> None:
    """Create each segment of abs_path as UsdGeom.Xform if missing (e.g. /World/environment/cesium)."""
    from pxr import UsdGeom

    parts = [p for p in abs_path.strip().split("/") if p]
    cur = ""
    for part in parts:
        cur = cur + "/" + part
        if not stage.GetPrimAtPath(cur).IsValid():
            prim = UsdGeom.Xform.Define(stage, cur).GetPrim()
            _apply_identity_xform_if_empty(prim)
            carb.log_info(f"Cesium physics: created {cur} (environment / tileset parent, scale=1).")


def ensure_world_environment_hierarchy(stage) -> None:
    """Create /World/environment and the configured Cesium tileset parent Xform (default /World/environment/cesium)."""
    try:
        cfg = _read_cesium_tile_collision_config()
        parent = str(cfg.get("tilesets_parent_path", "/World/environment/cesium")).strip() or "/World/environment/cesium"
        if not parent.startswith("/"):
            parent = "/" + parent
        _ensure_xform_path_exists(stage, parent)
    except Exception as exc:
        carb.log_warn(f"Cesium physics: ensure_world_environment_hierarchy failed: {exc}")


def reparent_cesium_tilesets_under_world_environment(stage) -> int:
    """
    Move Cesium tileset roots under <tilesets_parent_path>/<TilesetName> (default
    /World/environment/cesium/<TilesetName>) so all tilesets share one Xform under World.

    Returns number of prims reparented.
    """
    try:
        from cesium.omniverse.usdUtils import get_tileset_paths
        import omni.kit.commands
        from pxr import Sdf
    except ImportError as e:
        carb.log_warn(f"Cesium physics: reparent skipped ({e}).")
        return 0

    cfg = _read_cesium_tile_collision_config()
    parent = str(cfg.get("tilesets_parent_path", "/World/environment/cesium")).strip() or "/World/environment/cesium"
    if not parent.startswith("/"):
        parent = "/" + parent
    parent_sdf = Sdf.Path(parent)

    ensure_world_environment_hierarchy(stage)
    _sanitize_usd_selection(stage)
    moved = 0
    for ts_path in list(get_tileset_paths()):
        ts_path = str(ts_path).strip()
        if not ts_path:
            carb.log_warn("Cesium physics: skipping empty tileset path from get_tileset_paths().")
            continue
        if not ts_path.startswith("/"):
            carb.log_warn(f"Cesium physics: skipping non-absolute tileset path '{ts_path}'.")
            continue

        try:
            ts_sdf = Sdf.Path(ts_path)
        except Exception:
            carb.log_warn(f"Cesium physics: skipping ill-formed tileset path '{ts_path}'.")
            continue
        if ts_sdf.GetParentPath() == parent_sdf:
            continue
        name = ts_sdf.name
        if not name:
            carb.log_warn(f"Cesium physics: skipping tileset path without prim name '{ts_path}'.")
            continue
        dst_path = str(parent_sdf.AppendChild(name))
        if stage.GetPrimAtPath(dst_path).IsValid():
            continue
        try:
            omni.kit.commands.execute(
                "MovePrim",
                path_from=ts_sdf,
                path_to=Sdf.Path(dst_path),
            )
            _remap_usd_selection_paths_for_move(stage, ts_path, dst_path)
            carb.log_info(f"Cesium physics: reparented tileset {ts_path} -> {dst_path}")
            moved += 1
        except Exception as exc:
            carb.log_warn(f"Cesium physics: MovePrim failed for {ts_path}: {exc}")
            _sanitize_usd_selection(stage)
    return moved


def _apply_static_mesh_collider(prim, approximation: str) -> bool:
    """
    Match Isaac asset pattern: PhysicsCollisionAPI + explicit collisionEnabled,
    MeshCollisionAPI + PhysxCollisionAPI (+ PhysxMeshCollisionAPI when available).
    """
    try:
        from pxr import UsdGeom, UsdPhysics

        if not prim.IsA(UsdGeom.Mesh):
            return False

        cap = UsdPhysics.CollisionAPI.Apply(prim)
        en = cap.GetCollisionEnabledAttr()
        if en:
            en.Set(True)
        else:
            cap.CreateCollisionEnabledAttr().Set(True)

        mapi = UsdPhysics.MeshCollisionAPI.Apply(prim)
        a = mapi.GetApproximationAttr()
        if a:
            a.Set(approximation)
        else:
            mapi.CreateApproximationAttr().Set(approximation)

        try:
            from pxr import PhysxSchema

            PhysxSchema.PhysxCollisionAPI.Apply(prim)
            if hasattr(PhysxSchema, "PhysxMeshCollisionAPI"):
                PhysxSchema.PhysxMeshCollisionAPI.Apply(prim)
        except Exception:
            pass

        return True
    except Exception as exc:
        carb.log_warn(f"Cesium collision: could not apply APIs to {prim.GetPath()}: {exc}")
        return False


def apply_cesium_tile_mesh_collisions(
    max_meshes: Optional[int] = None,
    approximation: Optional[str] = None,
    tileset_paths: Optional[List[str]] = None,
) -> int:
    """
    Traverse Cesium tileset prims and enable static mesh collision on UsdGeom.Mesh descendants.

    Returns:
        Number of meshes that received collision APIs.
    """
    cfg = _read_cesium_tile_collision_config()
    if not cfg["enabled"]:
        return 0

    if max_meshes is None:
        max_meshes = cfg["max_meshes"]
    if approximation is None:
        approximation = cfg["approximation"]

    try:
        import omni.usd
        from pxr import Usd, UsdGeom
    except ImportError:
        carb.log_warn("Cesium tile collision: omni.usd / pxr not available.")
        return 0

    try:
        from cesium.omniverse.usdUtils import get_tileset_paths
    except ImportError:
        carb.log_warn("Cesium tile collision: cesium.omniverse.usdUtils not available.")
        return 0

    stage = omni.usd.get_context().get_stage()
    if not stage:
        return 0

    paths = tileset_paths if tileset_paths is not None else get_tileset_paths()
    if not paths:
        carb.log_info("Cesium tile collision: no tileset prims found yet (tiles may still be streaming).")
        return 0

    applied = 0
    for ts_path in paths:
        root = stage.GetPrimAtPath(ts_path)
        if not root.IsValid():
            continue
        for prim in Usd.PrimRange(root):
            if applied >= max_meshes:
                carb.log_warn(
                    f"Cesium tile collision: stopped after {max_meshes} meshes (cesium_tile_collision.max_meshes). "
                    "Increase max_meshes in configs.yaml if you need more coverage."
                )
                return applied
            if prim.IsA(UsdGeom.Mesh):
                if _apply_static_mesh_collider(prim, approximation):
                    applied += 1

    carb.log_info(f"Cesium tile collision: applied static colliders to {applied} mesh(es).")
    return applied


async def prepare_cesium_stage_for_isaac_physics_async() -> int:
    """
    Wait for tileset prims, ensure /World/environment exists, optionally reparent Cesium tilesets under
    tilesets_parent_path (default /World/environment/cesium) so all tilesets share one Xform under World.
    Returns number of tilesets reparented.
    """
    import omni.kit.app as omni_app
    import omni.usd

    cfg = _read_cesium_tile_collision_config()

    for _ in range(cfg["pre_reparent_settle_frames"]):
        await omni_app.get_app().next_update_async()

    stage = omni.usd.get_context().get_stage()
    if not stage:
        return 0

    n = 0
    if cfg["reparent_under_world_environment"]:
        n = reparent_cesium_tilesets_under_world_environment(stage)
    else:
        _ensure_xform_path_exists(stage, "/World/environment")

    if cfg.get("log_hierarchy_scales"):
        parent = str(cfg.get("tilesets_parent_path", "/World/environment/cesium")).strip() or "/World/environment/cesium"
        if not parent.startswith("/"):
            parent = "/" + parent
        log_cesium_hierarchy_scales(stage, parent)

    for _ in range(cfg["post_reparent_settle_frames"]):
        await omni_app.get_app().next_update_async()

    return n


async def apply_cesium_tile_mesh_collisions_after_streaming_async(
    max_meshes: Optional[int] = None,
    approximation: Optional[str] = None,
    settle_frames: Optional[int] = None,
) -> int:
    """
    Prepare stage tree (World/environment + reparent), wait for streaming, then tag mesh colliders.
    """
    import omni.kit.app as omni_app
    import omni.usd

    cfg = _read_cesium_tile_collision_config()
    if settle_frames is None:
        settle_frames = cfg.get("settle_frames", 120)

    if cfg["enabled"]:
        stage = omni.usd.get_context().get_stage()
        if stage:
            apply_cesium_tileset_streaming_settings_for_physics(stage)

    await prepare_cesium_stage_for_isaac_physics_async()

    if not cfg["enabled"]:
        return 0

    stage = omni.usd.get_context().get_stage()
    if stage:
        apply_cesium_tileset_streaming_settings_for_physics(stage)

    for _ in range(settle_frames):
        await omni_app.get_app().next_update_async()

    _sanitize_usd_selection(omni.usd.get_context().get_stage())

    retries = int(cfg.get("collision_pass_retries", 4))
    retry_wait = max(0, int(cfg.get("retry_settle_frames", 90)))
    applied = 0
    for attempt in range(retries):
        applied = apply_cesium_tile_mesh_collisions(max_meshes=max_meshes, approximation=approximation)
        if applied > 0:
            break
        if attempt + 1 < retries and retry_wait > 0:
            carb.log_info(
                f"Cesium tile collision: pass {attempt + 1}/{retries} tagged 0 meshes; "
                f"waiting {retry_wait} frames for streaming (attempt {attempt + 2}/{retries})."
            )
            for _ in range(retry_wait):
                await omni_app.get_app().next_update_async()

    if applied == 0:
        try:
            from cesium.omniverse.usdUtils import get_tileset_paths

            paths = get_tileset_paths()
            st = omni.usd.get_context().get_stage()
            fallback_cfg = _read_cesium_fallback_ground_config()
            fallback_enabled = bool(fallback_cfg.get("enabled", False))
            if st and paths:
                log_cesium_tileset_usd_mesh_diagnostics(st, paths, as_warning=not fallback_enabled)
            msg = (
                "Cesium tile collision: still 0 UsdGeom.Mesh under tilesets. "
                "PhysX cannot use Fabric-only geometry; ensure cesium:debug:disableGeometryPool is on before "
                "tiles load (Pegasus sets this when cesium_tile_collision.disable_geometry_pool_for_physics is true). "
                "Try Cesium World Terrain (Cesium Globe preset) if photorealistic tiles never expose USD meshes."
            )
            if fallback_enabled:
                carb.log_info(msg + " Fallback ground is enabled, so basic contact remains available.")
            else:
                carb.log_warn(msg + " Enable cesium_fallback_ground to keep a safety collision surface.")
        except Exception:
            pass

    return applied


def ensure_world_layout_hierarchy(stage) -> None:
    """Backward-compatible alias for :func:`ensure_world_environment_hierarchy`."""
    ensure_world_environment_hierarchy(stage)


def reparent_cesium_tilesets_under_world_layout(stage) -> int:
    """Backward-compatible alias for :func:`reparent_cesium_tilesets_under_world_environment`."""
    return reparent_cesium_tilesets_under_world_environment(stage)
