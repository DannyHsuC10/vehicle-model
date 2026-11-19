"""Persist the geometry source, steering layout and prescribed motion together."""

import copy
import json
import math
import os
from pathlib import Path
from tempfile import NamedTemporaryFile

from ._geometry_support import project as geometry_project
from ._geometry_support.hardpoints import CORNERS
from .animation_settings import default_animation, validate_animation
from .dynamics_settings import default_dynamics, validate_dynamics

AXLES = ("front", "rear")
DRIVERS = (*CORNERS, *AXLES)
COMPONENTS = ("position_mm", "velocity_mm_s", "acceleration_mm_s2")
STEERING_POINTS = ("outer", "inner")
FORMAT = "MultibodyGeometryLab"


def from_suspension(geometry):
    return {
        "format": FORMAT,
        "schema_version": 1,
        "geometry": copy.deepcopy(geometry_project.validate(geometry)),
        "steering": {
            axle: {"outer": [-0.075, -0.075, -0.02], "inner": [-0.075, -0.34, -0.02]}
            for axle in AXLES
        },
        "motion": {name: dict.fromkeys(COMPONENTS, 0.0) for name in DRIVERS},
        "animation": default_animation(),
        "dynamics": default_dynamics(),
    }


def default_project():
    return from_suspension(geometry_project.default_project())


def finite_number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{label} 必須是有限數值。")
    return float(value)


def validate(data):
    required = {"format", "schema_version", "geometry", "steering", "motion"}
    if (not isinstance(data, dict) or not required <= data.keys()
            or data.keys() - required - {"animation", "dynamics"}
            or data["format"] != FORMAT
            or type(data["schema_version"]) is not int or data["schema_version"] != 1):
        raise ValueError("需要 MultibodyGeometryLab v1 專案或 SuspensionGeometryLab 專案。")
    data = copy.deepcopy(data)
    data["geometry"] = geometry_project.validate(data["geometry"])
    data["animation"] = validate_animation(data.get("animation", default_animation()))
    data["dynamics"] = validate_dynamics(data.get("dynamics", default_dynamics()))
    steering, motion = data["steering"], data["motion"]
    if not isinstance(steering, dict) or set(steering) != set(AXLES):
        raise ValueError("轉向設定必須包含前軸和後軸。")
    for axle, endpoints in steering.items():
        if not isinstance(endpoints, dict) or set(endpoints) != set(STEERING_POINTS):
            raise ValueError(f"{axle} 需要轉向拉桿的內、外側接點。")
        for name, point in endpoints.items():
            if not isinstance(point, list) or len(point) != 3:
                raise ValueError(f"{axle}/{name} 需要三個接點偏移座標。")
            endpoints[name] = [finite_number(x, f"{axle}/{name}") for x in point]
    if not isinstance(motion, dict) or set(motion) != set(DRIVERS):
        raise ValueError("工況必須包含四輪行程和前、後齒條位移。")
    for name, settings in motion.items():
        if not isinstance(settings, dict) or set(settings) != set(COMPONENTS):
            raise ValueError(f"{name} 需要位移、速度和加速度。")
        for key in COMPONENTS:
            settings[key] = finite_number(settings[key], f"{name}/{key}")
    return data


def read_project(path):
    """Return the project and whether it can be saved back to the same path."""
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if isinstance(data, dict) and "format" not in data:
        return from_suspension(data), False
    return validate(data), True


def load_project(path):
    return read_project(path)[0]


def save_project(path, data):
    path = Path(path)
    payload = json.dumps(validate(data), ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    temp_path = None
    try:
        with NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as handle:
            temp_path = Path(handle.name)
            handle.write(payload)
        os.replace(temp_path, path)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)
