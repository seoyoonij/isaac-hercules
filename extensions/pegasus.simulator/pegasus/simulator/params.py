"""
| File: params.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: File that defines the base configurations for the Pegasus Simulator.
"""
import os
from pathlib import Path
import yaml

import isaacsim.storage.native as nucleus

# Extension configuration
EXTENSION_NAME = "Pegasus Simulator"
WINDOW_TITLE = "Pegasus Simulator"
MENU_PATH = "Window/" + WINDOW_TITLE
DOC_LINK = "https://docs.omniverse.nvidia.com"
EXTENSION_OVERVIEW = "This extension shows how to incorporate drones into Isaac Sim"

# Get the current directory of where this extension is located
EXTENSION_FOLDER_PATH = Path(os.path.dirname(os.path.realpath(__file__)))
ROOT = str(EXTENSION_FOLDER_PATH.parent.parent.parent.resolve())

# Get the configurations file path
CONFIG_FILE = ROOT + "/pegasus.simulator/config/configs.yaml"

# Pegasus udpin for ArduPilot MAVLink: vehicle n uses (base + n*10). sim_vehicle MAVProxy sends GCS to
# (14550 + instance*10). Same index n: ports always differ (base != 14550). A *cross* clash happens if
# base + 10*m == 14550 + 10*k  =>  k - m == (base - 14550) / 10. With base 15660 that gap is 111 instances,
# so simultaneous vehicles 0..110 stay clear of each other's GCS ports. If you raise the fleet size further,
# increase this base (or set connection_baseport per backend) so (base-14550)/10 stays above max(k-m).
ARDUPILOT_SIM_MAVLINK_BASEPORT = 15660

# Define the Extension Assets Path
ASSET_PATH = ROOT + "/pegasus.simulator/pegasus/simulator/assets"
ROBOTS_ASSETS = ASSET_PATH + "/Robots"

# Define the built in robots of the extension
ROBOTS = {"Iris": ROBOTS_ASSETS + "/Iris/iris.usd",
          "Fixed Wing": ROBOTS_ASSETS + "/fixed_wing/fixed_wing.usd",
          "Flying Cube": ROBOTS_ASSETS + "/Flying Cube/cube.usda",
          # Wrapper so Pegasus FixedWing (expects …/body under spawn root) can load the cube asset
          "Flying Cube FixedWing": ROBOTS_ASSETS + "/Flying Cube/cube_as_fixedwing_root.usda",
        }

# Setup the default simulation environments path
NVIDIA_ASSETS_PATH = str(nucleus.get_assets_root_path())

ISAAC_SIM_ENVIRONMENTS = "/Isaac/Environments"
NVIDIA_SIMULATION_ENVIRONMENTS = {
    "Default Environment": "Grid/default_environment.usd",
    "Black Gridroom": "Grid/gridroom_black.usd",
    "Curved Gridroom": "Grid/gridroom_curved.usd",
    "Hospital": "Hospital/hospital.usd",
    "Office": "Office/office.usd",
    "Simple Room": "Simple_Room/simple_room.usd",
    "Warehouse": "Simple_Warehouse/warehouse.usd",
    "Warehouse with Forklifts": "Simple_Warehouse/warehouse_with_forklifts.usd",
    "Warehouse with Shelves": "Simple_Warehouse/warehouse_multiple_shelves.usd",
    "Full Warehouse": "Simple_Warehouse/full_warehouse.usd",
    "Flat Plane": "Terrains/flat_plane.usd",
    "Rough Plane": "Terrains/rough_plane.usd",
    "Slope Plane": "Terrains/slope.usd",
    "Stairs Plane": "Terrains/stairs.usd",
}

OMNIVERSE_ENVIRONMENTS = {
    "Exhibition Hall": "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/NVIDIA/Assets/Scenes/Templates/Interior/ZetCG_ExhibitionHall.usd"
}


SIMULATION_ENVIRONMENTS = {}

# Add the Isaac Sim assets to the list
for asset in NVIDIA_SIMULATION_ENVIRONMENTS:
    SIMULATION_ENVIRONMENTS[asset] = (
        NVIDIA_ASSETS_PATH + ISAAC_SIM_ENVIRONMENTS + "/" + NVIDIA_SIMULATION_ENVIRONMENTS[asset]
    )

# Add the omniverse assets to the list
for asset in OMNIVERSE_ENVIRONMENTS:
    SIMULATION_ENVIRONMENTS[asset] = OMNIVERSE_ENVIRONMENTS[asset]

# Cesium ion asset IDs (match cesium.omniverse quick_add_widget.py catalog)
CESIUM_ION_ASSET_WORLD_TERRAIN = 1
CESIUM_ION_ASSET_GOOGLE_PHOTOREALISTIC_3D_TILES = 2275207
CESIUM_ION_ASSET_OSM_BUILDINGS = 96188

PEGASUS_CESIUM_GLOBE_ENV_TOKEN = "__pegasus_cesium_globe__"
PEGASUS_CESIUM_GOOGLE_PHOTO_TOKEN = "__pegasus_cesium_google_photo__"
PEGASUS_CESIUM_GOOGLE_PHOTO_OSM_TOKEN = "__pegasus_cesium_google_photo_osm__"

PEGASUS_CESIUM_PRESET_TILESETS = {
    PEGASUS_CESIUM_GLOBE_ENV_TOKEN: [("Cesium World Terrain", CESIUM_ION_ASSET_WORLD_TERRAIN)],
    PEGASUS_CESIUM_GOOGLE_PHOTO_TOKEN: [
        ("Google Photorealistic 3D Tiles", CESIUM_ION_ASSET_GOOGLE_PHOTOREALISTIC_3D_TILES),
    ],
    PEGASUS_CESIUM_GOOGLE_PHOTO_OSM_TOKEN: [
        ("Google Photorealistic 3D Tiles", CESIUM_ION_ASSET_GOOGLE_PHOTOREALISTIC_3D_TILES),
        ("Cesium OSM Buildings", CESIUM_ION_ASSET_OSM_BUILDINGS),
    ],
}


CESIUM_PRESET_CONFIG_MAP = {
    "google_photo": PEGASUS_CESIUM_GOOGLE_PHOTO_TOKEN,
    "google_photo_osm": PEGASUS_CESIUM_GOOGLE_PHOTO_OSM_TOKEN,
    "world_terrain": PEGASUS_CESIUM_GLOBE_ENV_TOKEN,
}


def resolve_cesium_preset_from_config() -> str:
    """Read cesium_tileset_preset from configs.yaml and return the preset token."""
    try:
        with open(CONFIG_FILE, "r") as f:
            data = yaml.safe_load(f) or {}
        key = str(data.get("cesium_tileset_preset", "google_photo")).strip().lower()
        return CESIUM_PRESET_CONFIG_MAP.get(key, PEGASUS_CESIUM_GOOGLE_PHOTO_TOKEN)
    except Exception:
        return PEGASUS_CESIUM_GOOGLE_PHOTO_TOKEN


def is_pegasus_cesium_environment(usd_path: str) -> bool:
    return usd_path in PEGASUS_CESIUM_PRESET_TILESETS


BACKENDS = {
    "px4": "px4",
    "ardupilot": "ardupilot",
    "ros2": "ros2"
}

# Define the default settings for the simulation environment
WORLD_SETTINGS = {
    'px4': {
        "physics_dt": 1.0 / 250.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
        "device": "cpu"
    },
    'ardupilot': {
        "physics_dt": 1.0 / 800.0, # Reach communication of 250hz with ardupilot sitl
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 120.0,
        "device": "cpu"
    },
    'ros2': {
        "physics_dt": 1.0 / 250.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
        "device": "cpu"
    }
}
DEFAULT_WORLD_SETTINGS = WORLD_SETTINGS['px4']

# Define where the thumbnail of the vehicle is located
THUMBNAIL = ROBOTS_ASSETS + "/Iris/iris_thumbnail.png"

# Define where the thumbail of the world is located
WORLD_THUMBNAIL = ASSET_PATH + "/Worlds/Empty_thumbnail.png"

BACKENDS_THUMBMAILS_PATH = ASSET_PATH + "/Backends"
BACKENDS_THUMBMAILS = {
    "px4": BACKENDS_THUMBMAILS_PATH + "/px4_logo.png",
    "ardupilot": BACKENDS_THUMBMAILS_PATH + "/ardupilot_logo.png",
    "ros2": BACKENDS_THUMBMAILS_PATH + "/ros2_logo.png"
}
