"""Find empty cells in a project before the desktop app validates the repair."""
import copy
from dataclasses import dataclass
import math

from . import hardpoints
from .engine import Engine
from .project import (LEGACY_ANTI_KEYS, MOUNT_X_FIELDS, default_project, parameter_fields,
                      project_version, read_project, static_objects)


GROUP_LABELS = {"vehicle": "車輛／輪胎", "front": "前軸", "rear": "後軸", "analysis": "掃描範圍"}


@dataclass(frozen=True)
class MissingField:
    path: tuple[str | int, ...]
    label: str
    unit: str
    default: str | bool | float | int | None
    scale: int = 1

    @property
    def location(self):
        return ".".join(map(str, self.path))

    def default_text(self):
        if self.default is None:
            return "自動（幾何極限）"
        if isinstance(self.default, bool):
            return "是" if self.default else "否"
        if isinstance(self.default, str):
            return self.default
        return f"{self.default * self.scale:.12g}"

    def parse(self, raw):
        raw = raw.strip()
        if not raw:
            return self.default
        if isinstance(self.default, str):
            return raw
        if isinstance(self.default, bool):
            return raw == "是"
        try:
            value = float(raw) / self.scale
            if math.isfinite(value):
                return value
        except ValueError:
            pass
        raise ValueError(f"{self.label}（{self.location}）需要有限數值。")


def _blank(value):
    return value is None or (isinstance(value, str) and not value.strip())


def _section(parent, key, fields, location):
    value = parent.get(key)
    if _blank(value):
        value = parent[key] = {}
    if not isinstance(value, dict) or value.keys() - fields:
        raise ValueError(f"{location} 必須是 JSON 物件，且只能包含專案定義的欄位。")
    return value


def _sizing_section(parent, key):
    values = parent.get(key)
    if _blank(values):
        return {}
    if not isinstance(values, dict):
        raise ValueError(f"Initial Sizing 的 {key} 必須是 JSON 物件。")
    return values


def _sizing_tire_stiffness(dynamics):
    stiffnesses = []
    for end, corners in (("f", ("FL", "FR")), ("r", ("RL", "RR"))):
        stiffness = dynamics.get(f"tire_{end}_k")
        if not _blank(stiffness):
            stiffnesses.append(stiffness)
        else:
            stiffnesses.extend(dynamics.get(f"tire_{corner}_k") for corner in corners)
    stiffnesses = [value for value in stiffnesses if not _blank(value)]
    if any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0 for value in stiffnesses):
        raise ValueError("Initial Sizing 的輪胎剛性須為大於 0 的有限數值（N/m）。")
    if stiffnesses and all(value == stiffnesses[0] for value in stiffnesses):
        return stiffnesses[0]
    # Geometry uses a single stiffness. Ask for it when the supplied values differ.
    return None


def _sizing_project(source, target_version):
    version = source.get("schema_version")
    if type(version) is not int or version not in (1, 2):
        raise ValueError("Initial Sizing 匯出檔須為 schema_version 1 或 2。")
    geometry = _sizing_section(source, "geometry")
    if version == 1:
        dynamics = vehicle_data = _sizing_section(_sizing_section(source, "parameters"), "full")
        environment = clearance_data = _sizing_section(geometry, "vehicle")
        scan = _sizing_section(geometry, "analysis")
    else:
        dynamics = _sizing_section(source, "dynamics")
        environment = vehicle_data = _sizing_section(source, "vehicle")
        scan = clearance_data = geometry

    vehicle = {key: vehicle_data.get(original) for key, original in (
        ("mass", "m"), ("cg_to_front", "lf"), ("cg_height", "h_cg"),
        ("track_front", "tf"), ("track_rear", "tr"), ("brake_bias_front", "brake_front"),
        ("drive_force_front", "drive_front"),
    )}
    lf, lr = vehicle_data.get("lf"), vehicle_data.get("lr")
    for key, value in (("lf", lf), ("lr", lr)):
        if not _blank(value) and (type(value) not in (int, float) or not math.isfinite(value) or value <= 0):
            raise ValueError(f"Initial Sizing 的 {key} 須為大於 0 的有限數值（m）。")
    if not (_blank(lf) or _blank(lr)):
        vehicle["wheelbase"] = lf + lr
    vehicle.update(gravity=environment.get("gravity"), free_radius=environment.get("free_radius"),
                   clearance_steer_deg=clearance_data.get("clearance_steer_deg"),
                   tire_stiffness=_sizing_tire_stiffness(dynamics))

    project = {"schema_version": target_version, "name": source.get("name"), "vehicle": vehicle,
               "analysis": {key: scan.get(key) for key in ("bump_mm", "droop_mm", "steer_deg")}}
    for axle, end, legacy_anti in (("front", "f", "dive_front"), ("rear", "r", "lift_rear")):
        if version == 1:
            axle_data = _sizing_section(geometry, axle)
            design = {key: axle_data.get(key) for key in ("fvsa", "kpi_deg", "caster_deg")}
            design["rc_height"] = dynamics.get(f"h_rc_{end}")
        else:
            design = {"fvsa": geometry.get(f"fvsa_{end}"), "kpi_deg": geometry.get(f"kpi_{end}_deg"),
                      "caster_deg": geometry.get(f"caster_{end}_deg"), "rc_height": geometry.get(f"h_rc_{end}")}
        design["anti"] = dynamics.get(f"anti_{axle}", dynamics.get(legacy_anti))
        project[axle] = design
    return project


def _parameter_defaults(defaults, version):
    """Express the current baseline in the units and conventions of older files."""
    if version >= 6:
        if version == 6:
            defaults["front"]["anti"] *= -1
        return defaults
    car, _, _ = static_objects(defaults)
    brake = defaults["vehicle"]["brake_bias_front"]
    drive = defaults["vehicle"]["drive_force_front"]
    for index, (axle, (brake_key, drive_key)) in enumerate(LEGACY_ANTI_KEYS.items()):
        design = defaults[axle]
        anti = design["anti"]
        wheel_center_anti = anti - car.load_Radius[index] * car.l / (car.h_cog * design["svsa"])
        if version < 3:
            design.update(anti_dive=anti * brake, anti_lift=float(wheel_center_anti * (1 - brake)))
        elif version < 5:
            brake_share, drive_share = (brake, drive) if axle == "front" else (1 - brake, 1 - drive)
            design.update({brake_key: anti * brake_share, drive_key: float(wheel_center_anti * drive_share)})
        else:
            design.update({brake_key: anti, drive_key: float(wheel_center_anti)})
    analysis = defaults["analysis"]
    analysis.update(travel_mm=analysis["bump_mm"], roll_deg=0., pitch_deg=0.)
    return defaults


def _hardpoint_fields(project):
    data = _section(project, "hardpoints", {"enabled", "corners"}, "hardpoints")
    if _blank(data.get("enabled")):
        yield MissingField(("hardpoints", "enabled"), "使用硬點模式", "", False)
    corners = _section(data, "corners", set(hardpoints.CORNERS), "hardpoints.corners")
    for corner in hardpoints.CORNERS:
        points = _section(corners, corner, hardpoints.POINTS.keys(), f"硬點 {corner}")
        for name in hardpoints.POINTS:
            values = points.get(name)
            if _blank(values):
                values = []
            if not isinstance(values, list) or len(values) > 3:
                raise ValueError(f"{corner}/{name}：XYZ 座標須為最多三格的 JSON 陣列。")
            points[name] = values + [None] * (3 - len(values))

    baseline = None
    for corner in hardpoints.CORNERS:
        opposite = hardpoints.MIRROR[corner]
        for name, label in hardpoints.POINTS.items():
            for index, axis in enumerate("XYZ"):
                if not _blank(corners[corner][name][index]):
                    continue
                mirrored = corners[opposite][name][index]
                if type(mirrored) in (int, float) and math.isfinite(mirrored):
                    default = -mirrored if axis == "Y" else mirrored
                else:
                    if baseline is None:
                        baseline = hardpoints.from_scene(Engine(default_project()).scene)
                    default = baseline[corner][name][index]
                yield MissingField(("hardpoints", "corners", corner, name, index),
                                   f"{corner}/{name} {label} · {axis}", "mm", default, 1000)


def read_import_project(path):
    project = read_project(path)
    defaults = default_project()
    if isinstance(project, dict) and project.get("application") == "VehicleInitialSizingLab-DynamicsExport":
        project = _sizing_project(project, defaults["schema_version"])
    version = project_version(project)
    allowed = {"schema_version", "name", "vehicle", "front", "rear", "analysis", "arm_mount_x", "hardpoints"}
    if version == 5:
        allowed.add("svsa_design")
    if project.keys() - allowed:
        raise ValueError("需要 SuspensionGeometryLab 專案，檔案含未知欄位。")

    missing = []
    defaults = _parameter_defaults(defaults, version)
    if _blank(project.get("name")):
        missing.append(MissingField(("name",), "專案名稱", "", defaults["name"]))
    for group, fields in parameter_fields(version).items():
        values = _section(project, group, fields.keys(), group)
        for key, (label, unit) in fields.items():
            if _blank(values.get(key)):
                missing.append(MissingField((group, key), f"{GROUP_LABELS[group]} · {label}",
                                            unit, defaults[group][key]))
    if "arm_mount_x" in project:
        mounts = _section(project, "arm_mount_x", {"front", "rear"}, "arm_mount_x")
        for axle in ("front", "rear"):
            values = _section(mounts, axle, MOUNT_X_FIELDS.keys(), f"arm_mount_x.{axle}")
            for key, label in MOUNT_X_FIELDS.items():
                # Explicit null already requests the automatic extremum.
                if key not in values or (values[key] is not None and _blank(values[key])):
                    missing.append(MissingField(("arm_mount_x", axle, key), f"{GROUP_LABELS[axle]} · {label}",
                                                "mm", defaults["arm_mount_x"][axle][key], 1000))
    if "svsa_design" in project:
        modes = _section(project, "svsa_design", {"front", "rear"}, "svsa_design")
        for axle in ("front", "rear"):
            if _blank(modes.get(axle)):
                missing.append(MissingField(("svsa_design", axle), f"{GROUP_LABELS[axle]} · 直接指定 SVSA",
                                            "", False))
    if "hardpoints" in project:
        missing.extend(_hardpoint_fields(project))
    return project, missing


def fill_import_project(project, fields, inputs):
    candidate = copy.deepcopy(project)
    for field, raw in zip(fields, inputs, strict=True):
        parent = candidate
        for key in field.path[:-1]:
            parent = parent[key]
        parent[field.path[-1]] = field.parse(raw)
    return candidate
