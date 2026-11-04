"""Project schema. All editable design defaults live in default_project.json."""
import copy
import json
import math
import os
from pathlib import Path

import numpy as np

from .static.car import Car, SuspensionF, SuspensionR

DEFAULT_PATH = Path(__file__).with_name("default_project.json")

# key: (label, units). SI throughout; user-facing angles are degrees.
VEHICLE_FIELDS = {
    "wheelbase": ("軸距", "m"), "cg_to_front": ("重心至前軸", "m"),
    "track_front": ("前輪距", "m"), "track_rear": ("後輪距", "m"),
    "cg_height": ("重心高度", "m"), "mass": ("總質量", "kg"),
    "gravity": ("重力加速度", "m/s²"), "ground_clearance": ("車底離地高", "m"),
    "free_radius": ("輪胎自由半徑", "m"), "tire_stiffness": ("輪胎垂直剛性", "N/m"),
    "tire_width": ("胎寬", "m"), "rim_inner_diameter": ("輪圈內徑", "m"),
    "rim_gap": ("輪圈避讓間隙", "m"), "clearance_steer_deg": ("靜態避讓轉角", "deg"),
    "brake_bias_front": ("前煞車比例", "0..1"),
    "drive_force_front": ("前軸加速驅動力比例（0 後驅／1 前驅）", "0..1"),
}
AXLE_FIELDS = {
    "fvsa": ("FVSA", "m"), "rc_height": ("Roll center 高度", "m"),
    "kpi_deg": ("KPI", "deg"), "scrub_radius": ("Scrub radius", "m"),
    "upper_offset": ("Au 輪心上偏距", "m"), "lower_offset": ("Al 輪心下偏距", "m"),
    "lower_mount_x": ("下 A 臂車身 |Y|", "m"), "upper_mount_x": ("上 A 臂車身 |Y|", "m"),
    "caster_deg": ("Caster", "deg"), "mechanical_trail": ("Mechanical trail", "m"),
    "anti_dive": ("前軸 Anti-dive（煞車）", "1=100%"),
    "anti_lift": ("前軸 Anti-lift（加速）", "1=100%"),
    "svsa": ("SVSA 水平長（有號）", "m"),
}
REAR_AXLE_FIELDS = {key: value for key, value in AXLE_FIELDS.items() if key not in ("anti_dive", "anti_lift")}
REAR_AXLE_FIELDS["anti_lift"] = ("後軸 Anti-lift（煞車）", "1=100%")
REAR_AXLE_FIELDS["anti_squat"] = ("後軸 Anti-squat（加速）", "1=100%")
DERIVED_AXLE_FIELDS = {**AXLE_FIELDS,
    "anti_lift": ("Anti-lift（前軸加速／後軸煞車）", "1=100%"),
    "anti_squat": ("後軸 Anti-squat（加速）", "1=100%")}
ANALYSIS_FIELDS = {
    "bump_mm": ("懸吊上壓行程", "mm"), "droop_mm": ("懸吊下垂行程", "mm"),
    "steer_deg": ("轉向範圍 ±", "deg"),
    "caster_min_deg": ("Caster 掃描下限", "deg"), "caster_max_deg": ("Caster 掃描上限", "deg"),
    "samples": ("每軸網格點數", "5..51"),
}
FIELDS = {"vehicle": VEHICLE_FIELDS, "front": AXLE_FIELDS, "rear": REAR_AXLE_FIELDS, "analysis": ANALYSIS_FIELDS}
MOUNT_X_FIELDS = {
    "upper_front_x": "上 A 臂前面接點 X", "upper_rear_x": "上 A 臂後面接點 X",
    "lower_front_x": "下 A 臂前面接點 X", "lower_rear_x": "下 A 臂後面接點 X",
}


def mount_x_settings(project):
    return project.get("arm_mount_x", {axle: {key: None for key in MOUNT_X_FIELDS}
                                        for axle in ("front", "rear")})


def default_project():
    return json.loads(DEFAULT_PATH.read_text(encoding="utf-8"))


def travel_limits(project):
    return -float(project["analysis"]["droop_mm"]), float(project["analysis"]["bump_mm"])


def drive_share(project, axle):
    front = project["vehicle"]["drive_force_front"]
    return front if axle == "front" else 1-front


def drive_anti_key(axle):
    return "anti_lift" if axle == "front" else "anti_squat"


def is_derived_input(project, axle, key):
    return ((key == "svsa" and drive_share(project, axle) > 0)
            or (key == drive_anti_key(axle) and drive_share(project, axle) == 0))


def _seed_drive_targets(project, car):
    """Convert the previous SVSA-based geometry into equivalent drive targets."""
    for index, axle in enumerate(("front", "rear")):
        brake = project["vehicle"]["brake_bias_front"] if axle == "front" else 1-project["vehicle"]["brake_bias_front"]
        target = project[axle]["anti_dive" if axle == "front" else "anti_lift"]
        value = drive_share(project, axle)*(target/brake-car.l/car.h_cog*car.load_Radius[index]/project[axle]["svsa"])
        project[axle][drive_anti_key(axle)] = 0. if abs(value) < 1e-14 else float(value)


def _upgrade_v3(project):
    result = copy.deepcopy(project)
    for axle in ("front", "rear"):
        expected = set(FIELDS[axle])-{drive_anti_key(axle)}
        if not isinstance(result.get(axle), dict) or set(result[axle]) != expected:
            raise ValueError(f"舊專案 {axle} 欄位不完整或含未知欄位。")
        result[axle][drive_anti_key(axle)] = 0.
    result["schema_version"] = 4
    validate(result)
    if any(abs(result[axle]["svsa"]) < 1e-8 for axle in ("front", "rear")):
        raise ValueError("舊版 3 專案的 SVSA 水平長不可為 0。")
    car, _, _ = static_objects(result)
    _seed_drive_targets(result, car)
    return result


def _upgrade_legacy(project):
    """Preserve v1/v2 geometry while separating braking and drive definitions."""
    result = copy.deepcopy(project)
    version = result["schema_version"]
    old_axle_keys = (set(AXLE_FIELDS)-{"svsa"}) | {"anti_lift"}
    legacy = {
        "vehicle": set(VEHICLE_FIELDS)-{"drive_force_front"},
        "front": old_axle_keys | ({"svsa"} if version == 1 else set()),
        "rear": old_axle_keys | ({"svsa"} if version == 1 else set()),
        "analysis": ((set(ANALYSIS_FIELDS)-{"bump_mm", "droop_mm"}) | {"travel_mm", "roll_deg", "pitch_deg"}
                     if version == 1 else set(ANALYSIS_FIELDS)),
    }
    for group, keys in legacy.items():
        values = result.get(group)
        if not isinstance(values, dict) or set(values) != keys:
            raise ValueError(f"舊專案 {group} 欄位不完整或含未知欄位。")
        if any(type(value) not in (float, int) or not math.isfinite(value) for value in values.values()):
            raise ValueError(f"舊專案 {group} 必須是有限數值。")
    lengths = {axle: result[axle].pop("svsa") if version == 1 else 0. for axle in ("front", "rear")}
    if min(lengths.values()) < 0:
        raise ValueError("舊專案 SVSA 不得為負。")
    if version == 1:
        a = result["analysis"]
        a["bump_mm"] = a["droop_mm"] = a.pop("travel_mm")
        a.pop("roll_deg")
        a.pop("pitch_deg")
    old_anti = {axle: (result[axle].pop("anti_dive"), result[axle].pop("anti_lift")) for axle in ("front", "rear")}
    result["vehicle"]["drive_force_front"] = 0.
    result["schema_version"] = 4
    for axle, key in (("front", "anti_dive"), ("rear", "anti_lift")):
        result[axle].update(svsa=1., **{key: old_anti[axle][0]})
        result[axle][drive_anti_key(axle)] = 0.
    # Validate physical inputs before using them in the migration arithmetic.
    validate(result)
    car, _, _ = static_objects(result)
    b = result["vehicle"]["brake_bias_front"]
    for index, axle in enumerate(("front", "rear")):
        dive, lift = old_anti[axle]
        dive_slope = dive*car.h_cog/(b*car.l)
        denominator = dive_slope-lift*car.h_cog/((1-b)*car.l)
        if not lengths[axle] and abs(denominator) < 1e-12:
            raise ValueError(f"{axle}：舊 anti 設定沒有有限瞬心，無法轉成 SVSA。")
        result[axle]["svsa"] = float(lengths[axle] or car.load_Radius[index]/denominator)
        if axle == "rear":
            result[axle]["anti_lift"] = dive*(1-b)/b
    _seed_drive_targets(result, car)
    return result


def validate(project):
    """Strict schema validation; no silent fallback to car.py or ignored keys."""
    required = {"schema_version", "name", *FIELDS}
    if not isinstance(project, dict) or not required <= set(project) or set(project) - required - {"hardpoints", "arm_mount_x"}:
        raise ValueError("需要 Suspension Lab 專案（schema_version、name、vehicle、front、rear、analysis）。")
    if type(project["schema_version"]) is not int or project["schema_version"] not in (1, 2, 3, 4):
        raise ValueError("不支援的專案版本；目前為 schema_version=4，可讀取舊版 1／2／3。")
    if project["schema_version"] < 3:
        project = _upgrade_legacy(project)
    elif project["schema_version"] == 3:
        project = _upgrade_v3(project)
    if not isinstance(project["name"], str) or not project["name"].strip():
        raise ValueError("專案名稱不可空白。")
    for group, fields in FIELDS.items():
        values = project[group]
        if not isinstance(values, dict) or set(values) != set(fields):
            raise ValueError(f"{group} 欄位不完整或含未知欄位。")
        for key, value in values.items():
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError(f"{group}.{key} 必須是有限數值。")
    v, a = project["vehicle"], project["analysis"]
    for key in ("wheelbase", "track_front", "track_rear", "mass", "gravity", "cg_height",
                "free_radius", "tire_stiffness", "tire_width", "rim_inner_diameter"):
        if v[key] <= 0:
            raise ValueError(f"{key} 必須大於 0。")
    if not 0 < v["cg_to_front"] < v["wheelbase"] or not 0 < v["brake_bias_front"] < 1:
        raise ValueError("重心必須在軸距內，前煞車比例必須介於 0 與 1。")
    if not 0 <= v["drive_force_front"] <= 1:
        raise ValueError("前軸加速驅動力比例須為 0..1；0 為後驅、1 為前驅。")
    if min(v["ground_clearance"], v["rim_gap"]) < 0:
        raise ValueError("離地高與間隙不得為負。")
    if not 0 <= v["clearance_steer_deg"] <= 80:
        raise ValueError("靜態避讓轉角須為 0..80 deg。")
    for axle in ("front", "rear"):
        s = project[axle]
        if min(s[k] for k in ("fvsa", "upper_offset", "lower_offset")) <= 0:
            raise ValueError(f"{axle}: FVSA、Au、Al 必須大於 0。")
        if drive_share(project, axle) == 0 and abs(s["svsa"]) < 1e-8:
            raise ValueError(f"{axle}: SVSA 水平長不可為 0；正值朝另一軸，負值朝車外。")
        if min(s[k] for k in ("lower_mount_x", "upper_mount_x")) < 0:
            raise ValueError(f"{axle}: 車身接點 |Y| 不得為負。")
        if abs(s["kpi_deg"]) >= 85 or abs(s["caster_deg"]) >= 85:
            raise ValueError("KPI 與 caster 須介於 -85 與 85 deg。")
        if max(s["lower_mount_x"], s["upper_mount_x"]) >= v["track_" + axle] / 2:
            raise ValueError(f"{axle}: 車身接點必須在半輪距內。")
    if not all(0 <= a[key] <= 200 for key in ("bump_mm", "droop_mm")) or a["bump_mm"]+a["droop_mm"] <= 0:
        raise ValueError("上壓、下垂行程各須為 0..200 mm，且不可同時為 0。")
    if not 0 < a["steer_deg"] <= 80:
        raise ValueError("轉向範圍須大於 0 且不超過 80 deg。")
    if not -85 < a["caster_min_deg"] < a["caster_max_deg"] < 85:
        raise ValueError("Caster 掃描範圍無效。")
    if int(a["samples"]) != a["samples"] or not 5 <= a["samples"] <= 51:
        raise ValueError("網格點數必須為 5..51 的整數。")
    if "hardpoints" in project:
        from .hardpoints import validate_points
        h = project["hardpoints"]
        if (not isinstance(h, dict) or set(h) != {"enabled", "corners"}
                or type(h["enabled"]) is not bool):
            raise ValueError("hardpoints 必須包含 enabled 布林值與 corners 硬點。")
        validate_points(h["corners"])
    if "arm_mount_x" in project:
        mounts = project["arm_mount_x"]
        if not isinstance(mounts, dict) or set(mounts) != {"front", "rear"}:
            raise ValueError("arm_mount_x 需要前後軸各四個 X 設定。")
        for axle, values in mounts.items():
            if not isinstance(values, dict) or set(values) != set(MOUNT_X_FIELDS):
                raise ValueError(f"{axle}：A 臂 X 設定必須包含上下臂各前後兩點。")
            if any(value is not None and (type(value) not in (float, int) or not math.isfinite(value))
                   for value in values.values()):
                raise ValueError(f"{axle}：A 臂 X 必須是有限數值，或 null 表示使用極限。")
    return project


def load_project(path):
    return validate(json.loads(Path(path).read_text(encoding="utf-8-sig")))


def save_project(path, project):
    project = validate(project)
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    try:
        temporary.write_text(json.dumps(project, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def static_objects(project):
    """Use legacy types for isinstance dispatch, without running legacy defaults."""
    project = validate(project)
    v = project["vehicle"]
    car = Car.__new__(Car)
    aliases = {"l": "wheelbase", "lf": "cg_to_front", "tf": "track_front", "tr": "track_rear",
               "h_cog": "cg_height", "m": "mass", "bottom": "ground_clearance", "gravity": "gravity",
               "free_radius": "free_radius", "K_tire": "tire_stiffness", "tire_width": "tire_width",
               "rim_inner_diameter": "rim_inner_diameter", "gap": "rim_gap"}
    for target, source in aliases.items():
        setattr(car, target, v[source])
    car.lr = car.l - car.lf
    car.w = car.m * car.gravity
    car.cg_rate = np.array([car.lr, car.lf]) / car.l
    car.load_Radius = car.free_radius - car.w * car.cg_rate / car.K_tire / 2
    car.steer = v["clearance_steer_deg"]
    if min(car.load_Radius) <= 0:
        raise ValueError("輪胎負載半徑小於等於 0；檢查質量與輪胎剛性。")
    result = []
    for cls, axle in ((SuspensionF, "front"), (SuspensionR, "rear")):
        s = project[axle]
        obj = cls.__new__(cls)
        mapping = {"h_RC": "rc_height", "fvsa": "fvsa", "scrub_radius": "scrub_radius", "Au": "upper_offset",
                   "Al": "lower_offset", "lower_mount_x": "lower_mount_x", "upper_mount_x": "upper_mount_x",
                   "mechanical_trail_sv": "mechanical_trail", "caster_sv_deg": "caster_deg",
                   "svsa": "svsa"}
        for target, source in mapping.items():
            setattr(obj, target, s[source])
        obj.kpi = math.radians(s["kpi_deg"])
        obj.front_brake_bias_sv = v["brake_bias_front"]
        obj.target_braking_anti_sv = s["anti_dive" if axle == "front" else "anti_lift"]
        obj.drive_force_share_sv = drive_share(project, axle)
        obj.target_drive_anti_sv = s[drive_anti_key(axle)]
        result.append(obj)
    return car, *result
