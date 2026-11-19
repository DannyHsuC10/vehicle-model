"""Project fields for the four elastic elements, road input and CG loading."""

import math

from SuspensionGeometryLab.suspension_geometry_lab.hardpoints import CORNERS

from .elements import validate_elements


ATTACHMENTS = ("WC", "UBJ", "LBJ", "TOE")
MOUNT_REFERENCES = ("attachment", "wheel_center", "body_coordinates")
PUSHROD_FIELDS = {"z_mm": "車身端 Z", "y_mm": "車身端 Y（左側）"}
LEGACY_MOUNT_FIELDS = ("mount_mm", "attachment", "mount_reference")
TARGETS = (*CORNERS, "all")
CORNER_LIMITS = {
    "mass_kg": ("輪胎與輪端質量 (kg)", 1., 150.),
    "spring_n_m": ("彈簧剛性 (N/m)", 100., 500_000.),
    "damper_ns_m": ("避震器阻尼 (N·s/m)", 0., 5000.),
}
SIMULATION_LIMITS = {
    "bump_mm": ("路面擾動高度 (mm)", -100., 100.),
    "bump_seconds": ("擾動持續時間 (s)", .02, 12.),
    "duration_seconds": ("模擬時間 (s)", .5, 12.),
}
HEAVE_LIMITS = {
    "force_down_n": ("重心向下力 (N)", -5000., 5000.),
    "ramp_seconds": ("施力上升時間 (s)", .02, 12.),
    "duration_seconds": ("模擬時間 (s)", .5, 12.),
}
INERTIA_LIMITS = {
    "ixx": ("Ixx · 側傾", .01, 1_000_000.),
    "iyy": ("Iyy · 俯仰", .01, 1_000_000.),
    "izz": ("Izz · 偏航", .01, 1_000_000.),
}


def default_chassis_inertia():
    return {"ixx": 55., "iyy": 95., "izz": 125.}


def default_heave():
    return {"force_down_n": 600., "ramp_seconds": .25, "duration_seconds": 2.5}


def parallel_pushrod_mount():
    return {"attachment": "UBJ", "mount_reference": "attachment", "mount_mm": [0., 0., 250.]}


def default_dynamics():
    return {
        "corners": {
            name: {"mass_kg": 10., "spring_n_m": 30000., "damper_ns_m": 1300., **parallel_pushrod_mount()}
            for name in CORNERS
        },
        "tire_damping_ns_m": 100.,
        "simulation": {"target": "FL", "bump_mm": 20., "bump_seconds": .25, "duration_seconds": 2.5},
        "heave": default_heave(),
        "chassis_inertia": default_chassis_inertia(),
    }


def require_fields(data, keys, label):
    if not isinstance(data, dict) or data.keys() != set(keys):
        raise ValueError(f"{label}的欄位不完整或含未知欄位。")


def bounded(value, label, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label}須為 {low:g} 到 {high:g}。")
    return float(value)


def validate_heave(data):
    require_fields(data, HEAVE_LIMITS, "重心 Heave 設定")
    settings = {key: bounded(data[key], *limits) for key, limits in HEAVE_LIMITS.items()}
    if settings["ramp_seconds"] > settings["duration_seconds"]:
        raise ValueError("施力上升時間不可超過模擬時間。")
    return settings


def validate_chassis_inertia(data):
    require_fields(data, INERTIA_LIMITS, "車身轉動慣量")
    values = {key: bounded(data[key], f"{label} (kg·m²)", low, high)
              for key, (label, low, high) in INERTIA_LIMITS.items()}
    largest = max(values.values())
    if 2 * largest > sum(values.values()) + 1e-10 * largest:
        raise ValueError("車身轉動慣量中，任一軸的數值不可大於另外兩軸的總和。")
    return values


def validate_pushrod_mounts(data):
    require_fields(data, ("front", "rear"), "Push Rod 車身接點")
    result = {}
    for axle, label in (("front", "前軸"), ("rear", "後軸")):
        values = data[axle]
        require_fields(values, PUSHROD_FIELDS, f"{label} Push Rod 車身接點")
        for key, field in PUSHROD_FIELDS.items():
            if type(values[key]) not in (int, float) or not math.isfinite(values[key]):
                raise ValueError(f"{label} · {field} (mm) 需要有限數值。")
        if values["y_mm"] < 0:
            raise ValueError(f"{label} · 左側車身端 Y 必須大於或等於 0 mm。")
        result[axle] = {key: float(values[key]) for key in PUSHROD_FIELDS}
    return result


def _validate_legacy_mount(corner, values):
    if values["attachment"] not in ATTACHMENTS:
        raise ValueError(f"{corner} · 未知的避震器輪端接點。")
    if values["mount_reference"] not in MOUNT_REFERENCES:
        raise ValueError(f"{corner} · 未知的避震器車身接點基準。")
    mount = values["mount_mm"]
    if not isinstance(mount, list) or len(mount) != 3:
        raise ValueError(f"{corner} · 車身接點需要三個 XYZ 數值。")
    if values["mount_reference"] == "body_coordinates":
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in mount):
            raise ValueError(f"{corner} · 車身接點 XYZ 座標 (mm) 需要有限數值。")
        mount_mm = [float(value) for value in mount]
    else:
        mount_mm = [bounded(value, f"{corner} · 車身接點偏移 (mm)", -2000, 2000) for value in mount]
    return {"mount_mm": mount_mm, "attachment": values["attachment"], "mount_reference": values["mount_reference"]}


def validate_dynamics(data):
    fields = ("corners", "tire_damping_ns_m", "simulation", "heave", "chassis_inertia")
    if isinstance(data, dict):
        data = {"heave": default_heave(), "chassis_inertia": default_chassis_inertia(), **data}
    mounts = None
    if isinstance(data, dict) and "pushrod_mounts" in data:
        fields = (*fields, "pushrod_mounts")
        mounts = validate_pushrod_mounts(data["pushrod_mounts"])
    elements = None
    if isinstance(data, dict) and "elements" in data:
        fields = (*fields, "elements")
        elements = validate_elements(data["elements"])
    require_fields(data, fields, "動力學設定")
    require_fields(data["corners"], CORNERS, "動力學輪位")
    corners = {}
    for corner in CORNERS:
        values = data["corners"][corner]
        corner_fields = tuple(CORNER_LIMITS)
        if mounts is None:
            corner_fields = (*corner_fields, *LEGACY_MOUNT_FIELDS)
            if isinstance(values, dict) and "mount_reference" not in values:
                values = {**values, "mount_reference": "wheel_center"}
        elif isinstance(values, dict):
            values = {key: value for key, value in values.items() if key not in LEGACY_MOUNT_FIELDS}
        require_fields(values, corner_fields, corner)
        corner_data = {key: bounded(values[key], f"{corner} · {label}", low, high)
                       for key, (label, low, high) in CORNER_LIMITS.items()}
        if mounts is None:
            corner_data.update(_validate_legacy_mount(corner, values))
        corners[corner] = corner_data

    tire_damping = bounded(data["tire_damping_ns_m"], "輪胎垂直阻尼 (N·s/m)", 0., 5000.)
    simulation = data["simulation"]
    require_fields(simulation, ("target", *SIMULATION_LIMITS), "模擬設定")
    if simulation["target"] not in TARGETS:
        raise ValueError("未知的路面擾動輪位。")
    inputs = {key: bounded(simulation[key], *limits) for key, limits in SIMULATION_LIMITS.items()}
    if inputs["bump_seconds"] > inputs["duration_seconds"]:
        raise ValueError("擾動持續時間不可超過模擬時間。")
    settings = {"corners": corners, "tire_damping_ns_m": tire_damping,
                "simulation": {"target": simulation["target"], **inputs}, "heave": validate_heave(data["heave"]),
                "chassis_inertia": validate_chassis_inertia(data["chassis_inertia"])}
    if mounts is not None:
        settings["pushrod_mounts"] = mounts
    if elements is not None:
        settings["elements"] = elements
    return settings
