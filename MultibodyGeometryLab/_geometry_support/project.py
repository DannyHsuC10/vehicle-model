"""Project schema. All editable design defaults live in default_project.json."""
import copy
import json
import math
from pathlib import Path

import numpy as np

from .hardpoints import KINEMATICS_KEY, validate_kinematics_hardpoints, validate_points
from .input_limits import FIELD_LIMITS, NAME_MAX_LENGTH
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
    "anti": ("軸端 Anti 比例（朝另一軸為正）", "1=100%"),
    "svsa": ("SVSA 水平長（有號）", "m"),
}
ANALYSIS_FIELDS = {
    "bump_mm": ("懸吊上壓行程", "mm"), "droop_mm": ("懸吊下垂行程", "mm"),
    "steer_deg": ("轉向範圍 ±", "deg"),
    "caster_min_deg": ("Caster 掃描下限", "deg"), "caster_max_deg": ("Caster 掃描上限", "deg"),
    "samples": ("每軸網格點數", "5..51"),
}
FIELDS = {"vehicle": VEHICLE_FIELDS,
          "front": {**AXLE_FIELDS, "anti": ("前軸 Anti 比例", "1=100%")},
          "rear": {**AXLE_FIELDS, "anti": ("後軸 Anti 比例", "1=100%")}, "analysis": ANALYSIS_FIELDS}
# Preserve these later SuspensionGeometryLab fields on import and save. The
# multibody model still has its own wheel alignment and suspension settings.
_CAMBER_FIELD = {"static_camber_deg": ("靜態 Camber", "deg")}
_OPTIONAL_FIELDS = {"vehicle": {"roll_stiffness_ratio": ("前後總側傾剛性比", "前/後")},
                    "front": _CAMBER_FIELD, "rear": _CAMBER_FIELD}
LEGACY_ANTI_KEYS = {"front": ("anti_dive", "anti_lift"), "rear": ("anti_lift", "anti_squat")}
MOUNT_X_FIELDS = {
    "upper_front_x": "上 A 臂前面接點 X", "upper_rear_x": "上 A 臂後面接點 X",
    "lower_front_x": "下 A 臂前面接點 X", "lower_rear_x": "下 A 臂後面接點 X",
}


def parameter_fields(version):
    """Return the parameter keys and labels stored by a project version."""
    if version >= 6:
        return FIELDS
    fields = {group: values.copy() for group, values in FIELDS.items()}
    for axle, legacy_keys in LEGACY_ANTI_KEYS.items():
        del fields[axle]["anti"]
        if version < 3:
            keys = LEGACY_ANTI_KEYS["front"]
        else:
            keys = legacy_keys[:1] if version == 3 else legacy_keys
        fields[axle].update(dict.fromkeys(keys, ("Anti", "1=100%")))
        if version == 2:
            del fields[axle]["svsa"]
    if version < 3:
        del fields["vehicle"]["drive_force_front"]
    if version == 1:
        analysis = fields["analysis"]
        del analysis["bump_mm"], analysis["droop_mm"]
        analysis.update(travel_mm=("對稱懸吊行程", "mm"), roll_deg=("舊版側傾範圍", "deg"),
                        pitch_deg=("舊版俯仰範圍", "deg"))
    return fields


def mount_x_settings(project):
    return project.get("arm_mount_x", {axle: {key: None for key in MOUNT_X_FIELDS}
                                        for axle in ("front", "rear")})


def default_project():
    return json.loads(DEFAULT_PATH.read_text(encoding="utf-8"))


def _legacy_uses_svsa(project, axle):
    version = project["schema_version"]
    if version == 3:
        return True
    if version == 4:
        return project["vehicle"]["drive_force_front"] == (0 if axle == "front" else 1)
    return project.get("svsa_design", {}).get(axle, False)


def _upgrade_legacy(project):
    """Preserve v1/v2 geometry while separating braking and drive definitions."""
    result = copy.deepcopy(project)
    version = result["schema_version"]
    for group, fields in parameter_fields(version).items():
        values = result.get(group)
        optional = _OPTIONAL_FIELDS.get(group, {})
        if not isinstance(values, dict) or values.keys() - optional.keys() != fields.keys():
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
    result["schema_version"] = 3
    for axle, (key, _) in LEGACY_ANTI_KEYS.items():
        result[axle].update(svsa=1., **{key: old_anti[axle][0]})
    # Validate physical inputs before using them in the migration arithmetic.
    _validate_parameters(result)
    car = _static_car(result["vehicle"])
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
    return result


def _upgrade_contact_anti(project):
    """Preserve the old CP line and horizontal IC distance in the new design."""
    version = project["schema_version"]
    result = copy.deepcopy(project)
    vehicle = result["vehicle"]
    car = _static_car(vehicle)
    for index, (axle, (brake_key, drive_key)) in enumerate(LEGACY_ANTI_KEYS.items()):
        design = result[axle]
        brake_anti = design.pop(brake_key)
        drive_anti = design.pop(drive_key, None)
        if version < 5:
            front_brake = vehicle["brake_bias_front"]
            brake_anti /= front_brake if axle == "front" else 1-front_brake
        if not _legacy_uses_svsa(project, axle):
            if version == 4:
                front_drive = vehicle["drive_force_front"]
                drive_anti /= front_drive if axle == "front" else 1-front_drive
            slope_scale = car.h_cog / car.l
            brake_slope = brake_anti * slope_scale
            drive_slope = drive_anti * slope_scale
            delta = brake_slope - drive_slope
            if abs(delta) < 1e-12*max(1., abs(brake_slope), abs(drive_slope)):
                if not project.get("hardpoints", {}).get("enabled"):
                    raise ValueError(f"{axle}：舊專案的 Anti 力線平行，無法換算有限 SVSA。")
                # The unused parameter draft may be degenerate in a valid hardpoint project.
                if not FIELD_LIMITS[axle]["svsa"].accepts(design["svsa"]):
                    design["svsa"] = default_project()[axle]["svsa"]
            else:
                design["svsa"] = float(car.load_Radius[index] / delta)
        design["anti"] = brake_anti
    result.pop("svsa_design", None)
    result["schema_version"] = 7
    return result


def project_version(project):
    version = project.get("schema_version") if isinstance(project, dict) else None
    if type(version) is not int or not 1 <= version <= 7:
        raise ValueError("不支援的專案版本；目前為 schema_version=7，可讀取舊版 1～6。")
    return version


def validate(project):
    """Strict schema validation; no silent fallback to car.py or ignored keys."""
    required = {"schema_version", "name", *FIELDS}
    optional = {"hardpoints", "arm_mount_x", "svsa_design", KINEMATICS_KEY}
    if not isinstance(project, dict) or not required <= project.keys() or project.keys() - required - optional:
        raise ValueError("需要 SuspensionGeometryLab 專案（schema_version、name、vehicle、front、rear、analysis）。")
    version = project_version(project)
    if KINEMATICS_KEY in project:
        validate_kinematics_hardpoints(project[KINEMATICS_KEY])
    if version < 3:
        project = _upgrade_legacy(project)
    if project["schema_version"] < 6:
        _validate_parameters(project)
        project = _upgrade_contact_anti(project)
    elif project["schema_version"] == 6:
        # Only v6 stored a forward-positive design ratio for both axles.
        project = copy.deepcopy(_validate_parameters(project))
        project["front"]["anti"] *= -1
        project["schema_version"] = 7
    return _validate_parameters(project)


def _validate_parameters(project):
    version = project["schema_version"]
    if "svsa_design" in project:
        modes = project["svsa_design"]
        if version != 5 or not isinstance(modes, dict) or set(modes) != {"front", "rear"} or any(type(x) is not bool for x in modes.values()):
            raise ValueError("svsa_design 只適用於版本 5，且需要前後軸各一個布林值。")
    if (not isinstance(project["name"], str) or not project["name"].strip()
            or len(project["name"]) > NAME_MAX_LENGTH):
        raise ValueError(f"專案名稱須為 1..{NAME_MAX_LENGTH} 個字元，且不可全為空白。")
    for group, fields in parameter_fields(version).items():
        legacy_keys = LEGACY_ANTI_KEYS.get(group, ()) if version < 6 else ()
        values = project[group]
        optional = _OPTIONAL_FIELDS.get(group, {})
        if not isinstance(values, dict) or values.keys() - optional.keys() != fields.keys():
            raise ValueError(f"{group} 欄位不完整或含未知欄位。")
        for key, value in values.items():
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError(f"{group}.{key} 必須是有限數值。")
            if legacy_keys:
                uses_length = _legacy_uses_svsa(project, group)
                if (key == "svsa" and not uses_length) or (key == legacy_keys[1] and uses_length):
                    continue
                if version < 5 and key in legacy_keys and not -2 <= value <= 3:
                    raise ValueError(f"舊專案 {group}.{key} 必須介於 -2 與 3。")
            limit = FIELD_LIMITS[group]["anti" if key in legacy_keys else key]
            if not limit.accepts(value):
                label, unit = fields[key] if key in fields else optional[key]
                raise ValueError(f"{group} · {label} 允許範圍：{limit.describe(unit)}。")
    v, a = project["vehicle"], project["analysis"]
    for key in ("wheelbase", "track_front", "track_rear", "mass", "gravity", "cg_height",
                "free_radius", "tire_stiffness", "tire_width", "rim_inner_diameter"):
        if v[key] <= 0:
            raise ValueError(f"{key} 必須大於 0。")
    if not 0 < v["cg_to_front"] < v["wheelbase"]:
        raise ValueError("重心必須在軸距內。")
    if version < 5 and not 0 < v["brake_bias_front"] < 1:
        raise ValueError("舊專案前煞車比例必須大於 0 且小於 1。")
    if not 0 <= v["drive_force_front"] <= 1:
        raise ValueError("前軸加速驅動力比例須為 0..1；0 為後驅、1 為前驅。")
    if min(v["ground_clearance"], v["rim_gap"]) < 0:
        raise ValueError("離地高與間隙不得為負。")
    if v["ground_clearance"] > v["cg_height"]:
        raise ValueError("車底離地高不得高於重心高度。")
    if v["rim_inner_diameter"] >= 2*v["free_radius"]:
        raise ValueError("輪圈內徑必須小於輪胎自由直徑（2 × 輪胎自由半徑）。")
    if 2*v["rim_gap"] >= v["rim_inner_diameter"]:
        raise ValueError("輪圈避讓間隙的兩倍必須小於輪圈內徑。")
    fractions = (1-v["cg_to_front"]/v["wheelbase"], v["cg_to_front"]/v["wheelbase"])
    if any(v["free_radius"]-v["mass"]*v["gravity"]*fraction/(2*v["tire_stiffness"]) <= 0
           for fraction in fractions):
        raise ValueError("輪胎負載半徑須大於 0；請檢查質量、重心位置與輪胎垂直剛性。")
    if not 0 <= v["clearance_steer_deg"] <= 80:
        raise ValueError("靜態避讓轉角須為 0..80 deg。")
    for axle in ("front", "rear"):
        s = project[axle]
        if min(s[k] for k in ("fvsa", "upper_offset", "lower_offset")) <= 0:
            raise ValueError(f"{axle}: FVSA、Au、Al 必須大於 0。")
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


def _static_car(v):
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
    return car


def static_objects(project):
    """Use legacy types for isinstance dispatch, without running legacy defaults."""
    project = validate(project)
    car = _static_car(project["vehicle"])
    result = []
    for cls, axle in ((SuspensionF, "front"), (SuspensionR, "rear")):
        s = project[axle]
        obj = cls.__new__(cls)
        mapping = {"h_RC": "rc_height", "fvsa": "fvsa", "scrub_radius": "scrub_radius", "Au": "upper_offset",
                   "Al": "lower_offset", "lower_mount_x": "lower_mount_x", "upper_mount_x": "upper_mount_x",
                   "mechanical_trail_sv": "mechanical_trail", "caster_sv_deg": "caster_deg",
                   "svsa": "svsa", "target_anti_sv": "anti"}
        for target, source in mapping.items():
            setattr(obj, target, s[source])
        obj.kpi = math.radians(s["kpi_deg"])
        result.append(obj)
    return car, *result
