"""Project schema. All editable design defaults live in default_project.json."""
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
}
AXLE_FIELDS = {
    "fvsa": ("FVSA", "m"), "rc_height": ("Roll center 高度", "m"),
    "kpi_deg": ("KPI", "deg"), "scrub_radius": ("Scrub radius", "m"),
    "upper_offset": ("Au 輪心上偏距", "m"), "lower_offset": ("Al 輪心下偏距", "m"),
    "lower_mount_x": ("FV 下接點 X", "m"), "upper_mount_x": ("FV 上接點 X", "m"),
    "caster_deg": ("Caster", "deg"), "mechanical_trail": ("Mechanical trail", "m"),
    "anti_dive": ("Anti-dive 比例", "1=100%"), "anti_lift": ("Anti-lift 比例", "1=100%"),
    "svsa": ("SVSA 水平長（0=由 anti 解）", "m"),
}
ANALYSIS_FIELDS = {
    "travel_mm": ("FV/SV 行程 ±", "mm"), "roll_deg": ("Roll 範圍 ±", "deg"),
    "pitch_deg": ("Pitch 範圍 ±", "deg"), "steer_deg": ("轉向範圍 ±", "deg"),
    "caster_min_deg": ("Caster 掃描下限", "deg"), "caster_max_deg": ("Caster 掃描上限", "deg"),
    "samples": ("每軸網格點數", "5..51"),
}
FIELDS = {"vehicle": VEHICLE_FIELDS, "front": AXLE_FIELDS, "rear": AXLE_FIELDS, "analysis": ANALYSIS_FIELDS}


def default_project():
    return json.loads(DEFAULT_PATH.read_text(encoding="utf-8"))


def validate(project):
    """Strict schema validation; no silent fallback to car.py or ignored keys."""
    if not isinstance(project, dict) or set(project) != {"schema_version", "name", *FIELDS}:
        raise ValueError("需要 Suspension Lab 專案（schema_version、name、vehicle、front、rear、analysis）。")
    if type(project["schema_version"]) is not int or project["schema_version"] != 1:
        raise ValueError("不支援的專案版本；目前為 schema_version=1。")
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
    if min(v["ground_clearance"], v["rim_gap"]) < 0:
        raise ValueError("離地高與間隙不得為負。")
    if not 0 <= v["clearance_steer_deg"] <= 80:
        raise ValueError("靜態避讓轉角須為 0..80 deg。")
    for axle in ("front", "rear"):
        s = project[axle]
        if min(s[k] for k in ("fvsa", "upper_offset", "lower_offset")) <= 0:
            raise ValueError(f"{axle}: FVSA、Au、Al 必須大於 0。")
        if min(s[k] for k in ("lower_mount_x", "upper_mount_x", "svsa")) < 0:
            raise ValueError(f"{axle}: 接點 X、SVSA 不得為負。")
        if abs(s["kpi_deg"]) >= 85 or abs(s["caster_deg"]) >= 85:
            raise ValueError("KPI 與 caster 須介於 -85 與 85 deg。")
        if max(s["lower_mount_x"], s["upper_mount_x"]) >= v["track_" + axle] / 2:
            raise ValueError(f"{axle}: 車身接點必須在半輪距內。")
    if not 0 < a["travel_mm"] <= 200 or not 0 < a["steer_deg"] <= 80:
        raise ValueError("行程範圍須為 0..200 mm，轉向範圍須為 0..80 deg。")
    if not 0 < a["roll_deg"] <= 15 or not 0 < a["pitch_deg"] <= 15:
        raise ValueError("Roll / pitch 範圍須為 0..15 deg。")
    if not -85 < a["caster_min_deg"] < a["caster_max_deg"] < 85:
        raise ValueError("Caster 掃描範圍無效。")
    if int(a["samples"]) != a["samples"] or not 5 <= a["samples"] <= 51:
        raise ValueError("網格點數必須為 5..51 的整數。")
    return project


def load_project(path):
    return validate(json.loads(Path(path).read_text(encoding="utf-8-sig")))


def save_project(path, project):
    validate(project)
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
    validate(project)
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
                   "target_anti_dive_sv": "anti_dive", "target_anti_lift_sv": "anti_lift"}
        for target, source in mapping.items():
            setattr(obj, target, s[source])
        obj.kpi = math.radians(s["kpi_deg"])
        obj.front_brake_bias_sv = v["brake_bias_front"]
        obj.svsa = s["svsa"]
        result.append(obj)
    return car, *result
