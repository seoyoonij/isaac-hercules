"""
Optional integration with Cesium for Omniverse: place Pegasus vehicles on the WGS84 globe using
CesiumGlobeAnchorAPI so lat/lon/height from Pegasus match the streamed Cesium tileset.

Requires the Cesium for Omniverse and cesium.usd.plugins Kit extensions to be enabled.
"""

from __future__ import annotations

import carb
import os

_CESIUM_IMPORT_ERROR_LOGGED = False
_SETUP_ERROR_LOGGED = False
_CESIUM_DOME_LIGHT_INTENSITY = 600.0

# Kit settings paths used by cesium.omniverse (see extension.toml [settings])
_CARB_ION_TOKEN_PATHS = (
    "/persistent/exts/cesium.omniverse/userAccessToken",
    "/exts/cesium.omniverse/defaultAccessToken",
    "persistent/exts/cesium.omniverse/userAccessToken",
    "exts/cesium.omniverse/defaultAccessToken",
)


def _read_ion_access_token_from_pegasus_config() -> str:
    try:
        from pegasus.simulator.params import CONFIG_FILE
        import yaml

        with open(CONFIG_FILE, "r") as f:
            data = yaml.safe_load(f)
        if not data:
            return ""
        t = data.get("cesium_ion_access_token", "")
        if t is None:
            return ""
        s = str(t).strip()
        return s if s else ""
    except Exception:
        return ""


def _write_ion_access_token_to_pegasus_config(access_token: str) -> None:
    t = str(access_token).strip()
    if not t:
        return
    try:
        from pegasus.simulator.params import CONFIG_FILE
        import yaml

        with open(CONFIG_FILE, "r") as f:
            data = yaml.safe_load(f)
        if data is None:
            data = {}
        if str(data.get("cesium_ion_access_token", "")).strip() == t:
            return
        data["cesium_ion_access_token"] = t
        with open(CONFIG_FILE, "w") as f:
            yaml.dump(data, f)
    except Exception:
        pass


def _read_ion_access_token_from_carb_settings() -> str:
    try:
        settings = carb.settings.get_settings()
        for path in _CARB_ION_TOKEN_PATHS:
            try:
                raw = None
                if hasattr(settings, "get_as_string"):
                    try:
                        raw = settings.get_as_string(path)
                    except Exception:
                        raw = None
                if raw is None or (isinstance(raw, str) and len(raw.strip()) == 0):
                    try:
                        raw = settings.get(path)
                    except Exception:
                        pass
                if raw is None:
                    continue
                s = str(raw).strip()
                if len(s) > 0:
                    return s
            except Exception:
                continue
    except Exception:
        pass
    return ""


def _write_ion_access_token_to_carb_settings(access_token: str) -> None:
    t = str(access_token).strip()
    if not t:
        return
    try:
        settings = carb.settings.get_settings()
        for path in _CARB_ION_TOKEN_PATHS:
            try:
                if hasattr(settings, "set_string"):
                    settings.set_string(path, t)
                else:
                    settings.set(path, t)
            except Exception:
                continue
    except Exception:
        pass


def _read_ion_access_token_from_env() -> str:
    for key in ("CESIUM_ION_ACCESS_TOKEN", "CESIUM_ION_TOKEN", "OMNIVERSE_CESIUM_ION_TOKEN"):
        v = os.environ.get(key)
        if v and len(str(v).strip()) > 0:
            return str(v).strip()
    return ""


def _read_ion_access_token_from_cesium_session() -> str:
    """
    Token from Cesium ion session: default entry from get_tokens(), or OAuth connection string.
    This is often populated after signing in to ion in the Cesium UI, even when USD IonServer prims are still empty.
    """
    try:
        from cesium.omniverse.bindings import acquire_cesium_omniverse_interface

        iface = acquire_cesium_omniverse_interface()
        session = iface.get_session()
        if session is None:
            return ""

        if session.is_token_list_loaded():
            tokens = session.get_tokens()
            if tokens:
                for t in tokens:
                    try:
                        if t.is_default and t.token:
                            s = str(t.token).strip()
                            if s:
                                return s
                    except Exception:
                        continue
                try:
                    t0 = tokens[0]
                    if t0.token:
                        s = str(t0.token).strip()
                        if s:
                            return s
                except Exception:
                    pass

        try:
            conn = session.get_connection()
            if conn is not None:
                at = conn.get_access_token()
                if at and len(str(at).strip()) > 0:
                    return str(at).strip()
        except Exception:
            pass
    except Exception:
        pass
    return ""


def _read_ion_access_token_from_ion_server_prims(stage) -> str:
    try:
        from cesium.usd.plugins.CesiumUsdSchemas import IonServer as CesiumIonServer

        for prim in stage.Traverse():
            if prim.IsA(CesiumIonServer):
                server = CesiumIonServer.Get(stage, prim.GetPath())
                t = server.GetProjectDefaultIonAccessTokenAttr().Get()
                if t is not None and len(str(t).strip()) > 0:
                    return str(t).strip()
    except Exception:
        pass
    return ""


def resolve_cesium_ion_access_token(stage) -> str:
    """
    Token used by add_tileset_ion must be non-empty for ion API requests; empty string causes 401.

    Resolution order: IonServer prim on stage, Cesium ion session (default token / OAuth), Kit settings,
    environment variables, pegasus configs.yaml (cesium_ion_access_token).
    """
    t = _read_ion_access_token_from_ion_server_prims(stage)
    if t:
        return t
    t = _read_ion_access_token_from_cesium_session()
    if t:
        return t
    t = _read_ion_access_token_from_carb_settings()
    if t:
        return t
    t = _read_ion_access_token_from_env()
    if t:
        return t
    t = _read_ion_access_token_from_pegasus_config()
    if t:
        return t
    return ""


def persist_cesium_ion_token_from_available_sources(stage=None) -> str:
    """
    Resolve Cesium token from current sources and mirror it to Kit settings + Pegasus config.
    Call before creating a new stage so Cesium keeps token state and avoids token picker prompts.
    """
    token = resolve_cesium_ion_access_token(stage)
    if not token:
        return ""
    _write_ion_access_token_to_carb_settings(token)
    _write_ion_access_token_to_pegasus_config(token)
    return token


def _ensure_default_cesium_ion_server(stage, access_token: str):
    """
    Ensure an IonServer prim exists and is selected from /Cesium, matching Cesium extension.
    If access_token is set, copy it to the project default token on the server prim so tilesets can use it.
    """
    from cesium.usd.plugins.CesiumUsdSchemas import IonServer as CesiumIonServer
    from cesium.omniverse.usdUtils import set_path_to_current_ion_server

    server_prims = [x for x in stage.Traverse() if x.IsA(CesiumIonServer)]
    if len(server_prims) >= 1:
        path = server_prims[0].GetPath().pathString
        server = CesiumIonServer.Get(stage, path)
    else:
        path = "/CesiumServers/IonOfficial"
        server = CesiumIonServer.Define(stage, path)
        server.GetDisplayNameAttr().Set("ion.cesium.com")
        server.GetIonServerUrlAttr().Set("https://ion.cesium.com/")
        server.GetIonServerApiUrlAttr().Set("https://api.cesium.com/")
        server.GetIonServerApplicationIdAttr().Set(413)

    if access_token:
        server.GetProjectDefaultIonAccessTokenAttr().Set(access_token)

    set_path_to_current_ion_server(path)


def _apply_pegasus_cesium_georeference(latitude: float, longitude: float, height_m: float) -> None:
    """Set /Cesium georeference origin to Pegasus global coordinates."""
    from cesium.omniverse.usdUtils import get_or_create_cesium_data, get_or_create_cesium_georeference

    get_or_create_cesium_data()
    geo = get_or_create_cesium_georeference()
    geo.GetGeoreferenceOriginLatitudeAttr().Set(float(latitude))
    geo.GetGeoreferenceOriginLongitudeAttr().Set(float(longitude))
    geo.GetGeoreferenceOriginHeightAttr().Set(float(height_m))


def ensure_world_dome_light(stage, intensity: float = _CESIUM_DOME_LIGHT_INTENSITY, force_intensity: bool = True) -> bool:
    """
    Ensure a dome light exists at /World/Light/DomeLight.

    The light is created once and reused on later scene loads.
    In Cesium workflows we force a softer default dome intensity for better balance.
    """
    try:
        from pxr import Sdf, UsdGeom, UsdLux

        world_path = Sdf.Path("/World")
        light_parent_path = Sdf.Path("/World/Light")
        dome_light_path = Sdf.Path("/World/Light/DomeLight")

        world_prim = stage.GetPrimAtPath(world_path)
        if not world_prim.IsValid():
            UsdGeom.Xform.Define(stage, world_path)

        light_parent_prim = stage.GetPrimAtPath(light_parent_path)
        if not light_parent_prim.IsValid():
            UsdGeom.Xform.Define(stage, light_parent_path)

        dome_light = UsdLux.DomeLight.Get(stage, dome_light_path)
        if not dome_light or not dome_light.GetPrim().IsValid():
            dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
            dome_light.GetIntensityAttr().Set(float(intensity))
            dome_light.GetColorAttr().Set((1.0, 1.0, 1.0))
            carb.log_info("Added Cesium dome light at /World/Light/DomeLight")
            return True

        intensity_attr = dome_light.GetIntensityAttr()
        if force_intensity or not intensity_attr.HasAuthoredValueOpinion():
            intensity_attr.Set(float(intensity))
        color_attr = dome_light.GetColorAttr()
        if not color_attr.HasAuthoredValueOpinion():
            color_attr.Set((1.0, 1.0, 1.0))
        return True
    except Exception as exc:
        carb.log_warn(f"Cesium dome light setup skipped: {exc}")
        return False


def _setup_cesium_ion_scene_manual_add_tilesets(
    latitude: float, longitude: float, height_m: float, preset_token: str
) -> bool:
    """
    Fallback: call add_tileset_ion with resolved token (used if cesium.omniverse.models is unavailable).
    """
    global _SETUP_ERROR_LOGGED

    from pegasus.simulator.params import PEGASUS_CESIUM_PRESET_TILESETS

    tilesets = PEGASUS_CESIUM_PRESET_TILESETS.get(preset_token)
    if not tilesets:
        carb.log_error(f"Unknown Cesium preset token: {preset_token}")
        return False

    try:
        from cesium.omniverse.usdUtils import add_tileset_ion
    except ImportError:
        if not _SETUP_ERROR_LOGGED:
            carb.log_warn(
                "Cesium for Omniverse is not available: enable cesium.omniverse and cesium.usd.plugins "
                "to use the Cesium Globe world asset."
            )
            _SETUP_ERROR_LOGGED = True
        return False

    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage:
            ensure_world_dome_light(stage, intensity=_CESIUM_DOME_LIGHT_INTENSITY, force_intensity=True)
        try:
            from pegasus.simulator.logic.cesium_physics import apply_cesium_debug_disable_geometry_pool_for_physx

            apply_cesium_debug_disable_geometry_pool_for_physx(stage)
        except Exception:
            pass
        _apply_pegasus_cesium_georeference(latitude, longitude, height_m)

        try:
            from pegasus.simulator.logic.cesium_physics import ensure_cesium_fallback_ground

            ensure_cesium_fallback_ground(stage)
        except Exception as exc:
            carb.log_warn(f"Cesium fallback ground: skipped ({exc})")

        token = resolve_cesium_ion_access_token(stage)
        if not token:
            carb.log_warn(
                "Cesium ion access token is empty. Set a default token in the Cesium ion Token window "
                "(project default), or use cesium_ion_access_token in configs.yaml."
            )

        _ensure_default_cesium_ion_server(stage, token)

        if not token:
            token = resolve_cesium_ion_access_token(stage)

        for tileset_name, ion_asset_id in tilesets:
            add_tileset_ion(tileset_name, ion_asset_id, token)

        carb.log_info(
            f"Cesium ion scene (manual tilesets, {preset_token}): lat={latitude}, lon={longitude}, height={height_m}; "
            f"tilesets={[t[0] for t in tilesets]}"
        )
        return True
    except Exception as exc:
        carb.log_error(f"Cesium globe environment setup failed: {exc}")
        return False


async def setup_cesium_ion_scene_async(latitude: float, longitude: float, height_m: float, preset_token: str) -> bool:
    """
    Match the Cesium for Omniverse UI: push cesium.omniverse.ADD_ION_ASSET for each tileset (see quick_add_widget),
    so the extension runs _add_ion_assets → add_tileset_ion(name, id) with the same token / ion session as the Cesium window.

    Sets georeference origin from Pegasus coordinates first, then queues each AssetToAdd with short frame gaps so
    each tileset is processed like using Quick Add in order.
    """
    global _SETUP_ERROR_LOGGED

    from pegasus.simulator.params import PEGASUS_CESIUM_PRESET_TILESETS

    tilesets = PEGASUS_CESIUM_PRESET_TILESETS.get(preset_token)
    if not tilesets:
        carb.log_error(f"Unknown Cesium preset token: {preset_token}")
        return False

    try:
        from cesium.omniverse.models.asset_to_add import AssetToAdd
    except ImportError:
        carb.log_warn("cesium.omniverse.models not available; falling back to manual add_tileset_ion with token.")
        ok = _setup_cesium_ion_scene_manual_add_tilesets(latitude, longitude, height_m, preset_token)
        if ok:
            try:
                from pegasus.simulator.logic.cesium_physics import (
                    apply_cesium_tile_mesh_collisions_after_streaming_async,
                )

                await apply_cesium_tile_mesh_collisions_after_streaming_async()
            except Exception as exc:
                carb.log_warn(f"Cesium tile collision: skipped ({exc})")
        return ok

    try:
        import carb.events
        import omni.kit.app as omni_app
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage:
            ensure_world_dome_light(stage, intensity=_CESIUM_DOME_LIGHT_INTENSITY, force_intensity=True)
            from pegasus.simulator.logic.cesium_physics import apply_cesium_debug_disable_geometry_pool_for_physx

            apply_cesium_debug_disable_geometry_pool_for_physx(stage)

        _apply_pegasus_cesium_georeference(latitude, longitude, height_m)

        if stage:
            try:
                from pegasus.simulator.logic.cesium_physics import ensure_cesium_fallback_ground

                ensure_cesium_fallback_ground(stage)
            except Exception as exc:
                carb.log_warn(f"Cesium fallback ground: skipped ({exc})")

            # Quick Add path does not pass token in payload. Ensure ion server has a token
            # before queuing multi-tileset presets (e.g., Google Photorealistic + OSM).
            token = resolve_cesium_ion_access_token(stage)
            if token:
                _ensure_default_cesium_ion_server(stage, token)
                persist_cesium_ion_token_from_available_sources(stage)
            else:
                carb.log_warn(
                    "Cesium ion access token is empty before ADD_ION_ASSET. "
                    "Open Cesium ion Token window and set a project default token, "
                    "or set cesium_ion_access_token in configs.yaml."
                )

        _session_connected = False
        _default_token_set = False
        try:
            from cesium.omniverse.bindings import acquire_cesium_omniverse_interface

            _iface = acquire_cesium_omniverse_interface()
            session = _iface.get_session()
            _session_connected = session is not None and session.is_connected()
            _default_token_set = _iface.is_default_token_set()
        except Exception:
            pass

        if _session_connected and _default_token_set:
            bus = omni_app.get_app().get_message_bus_event_stream()
            add_event = carb.events.type_from_string("cesium.omniverse.ADD_ION_ASSET")

            for tileset_name, ion_asset_id in tilesets:
                asset = AssetToAdd(tileset_name, ion_asset_id)
                bus.push(add_event, payload=asset.to_dict())
                for _ in range(4):
                    await omni_app.get_app().next_update_async()
        else:
            carb.log_info(
                "Cesium ion session not connected or default token not set; "
                "using direct add_tileset_ion with resolved token."
            )
            from cesium.omniverse.usdUtils import add_tileset_ion

            if not token:
                token = resolve_cesium_ion_access_token(stage)
            for tileset_name, ion_asset_id in tilesets:
                add_tileset_ion(tileset_name, ion_asset_id, token)
                for _ in range(4):
                    await omni_app.get_app().next_update_async()

        carb.log_info(
            f"Cesium ion scene ({preset_token}): "
            f"georeference lat={latitude}, lon={longitude}, height={height_m}; tilesets={[t[0] for t in tilesets]}"
        )

        try:
            from pegasus.simulator.logic.cesium_physics import (
                apply_cesium_tile_mesh_collisions_after_streaming_async,
            )

            n = await apply_cesium_tile_mesh_collisions_after_streaming_async()
            if n == 0:
                carb.log_info(
                    "Cesium tile collision: no meshes were tagged (see warnings above for diagnostics). "
                    "Increase settle_frames, confirm disable_geometry_pool_for_physics, or use terrain-based presets."
                )
        except Exception as exc:
            carb.log_warn(f"Cesium tile collision: skipped ({exc})")

        return True
    except Exception as exc:
        carb.log_error(f"Cesium globe environment setup failed: {exc}")
        return False


def setup_cesium_default_globe(latitude: float, longitude: float, height_m: float) -> bool:
    """Backward-compatible sync path: manual tilesets only (Cesium World Terrain preset)."""
    from pegasus.simulator.params import PEGASUS_CESIUM_GLOBE_ENV_TOKEN

    return _setup_cesium_ion_scene_manual_add_tilesets(latitude, longitude, height_m, PEGASUS_CESIUM_GLOBE_ENV_TOKEN)


def update_vehicle_globe_anchor(prim_path: str, latitude: float, longitude: float, height_m: float) -> bool:
    """
    Apply or update a Cesium globe anchor on the vehicle root prim so it is placed at the given LLH.

    height_m is passed to cesium:anchor:height (meters above the WGS84 ellipsoid), consistent with
    Pegasus global_coordinates altitude.

    Returns:
        True if Cesium APIs ran successfully, False if extensions are missing or an error occurred.
    """
    global _CESIUM_IMPORT_ERROR_LOGGED

    try:
        from cesium.omniverse.usdUtils import add_globe_anchor_to_prim
    except ImportError:
        if not _CESIUM_IMPORT_ERROR_LOGGED:
            carb.log_warn(
                "Cesium for Omniverse is not available: enable the cesium.omniverse and "
                "cesium.usd.plugins extensions to place vehicles on the globe."
            )
            _CESIUM_IMPORT_ERROR_LOGGED = True
        return False

    try:
        globe_anchor = add_globe_anchor_to_prim(prim_path)
        globe_anchor.GetAnchorLatitudeAttr().Set(float(latitude))
        globe_anchor.GetAnchorLongitudeAttr().Set(float(longitude))
        globe_anchor.GetAnchorHeightAttr().Set(float(height_m))
        carb.log_info(
            f"Cesium globe anchor set for {prim_path}: lat={latitude}, lon={longitude}, height={height_m}"
        )
        return True
    except Exception as exc:
        carb.log_error(f"Cesium globe anchor failed for {prim_path}: {exc}")
        return False
