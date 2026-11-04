"""Six hardpoints per corner and their projected FV/SV analysis geometry.

Coordinates are vehicle X forward, Y left, Z up, in metres. The six points
define the arms and kingpin; wheel centres still come from vehicle settings.
FV uses equivalent pivots at the ball-joint X planes. SV retains the existing
fixed-tangent approximation, seeded by each actual arm hinge's velocity.
Neither projection is a coupled spatial suspension/steering solver.
"""
import copy
import math
from types import SimpleNamespace

import numpy as np

from .static.fv.wireframe_geometry import WireframeLine, WireframeScene
from .anti import axle_anti

CORNERS = ("FL", "FR", "RL", "RR")
POINTS = {
    "UCA_F": "上 A 臂前車身接點", "UCA_R": "上 A 臂後車身接點", "UBJ": "上球接點",
    "LCA_F": "下 A 臂前車身接點", "LCA_R": "下 A 臂後車身接點", "LBJ": "下球接點",
}
MIRROR = {"FL": "FR", "FR": "FL", "RL": "RR", "RR": "RL"}


def enabled(project):
    return project.get("hardpoints", {}).get("enabled", False)


def from_scene(scene):
    """Extract exactly the six physical arm points, without reference points."""
    result = {corner: {key: scene.geometry["kingpins"][corner][key].tolist()
                       for key in ("UBJ", "LBJ")} for corner in CORNERS}
    for group in ("upper_arm_mount_geometry", "lower_arm_mount_geometry",
                  "rear_upper_arm_mount_geometry", "rear_lower_arm_mount_geometry"):
        for corner, arm in scene.geometry[group].items():
            for original, name in (("UFI", "UCA_F"), ("URI", "UCA_R"),
                                   ("LFI", "LCA_F"), ("LRI", "LCA_R")):
                if original in arm["mounts"]:
                    point = arm["mounts"][original]
                    if point is None:
                        raise ValueError(f"{corner}/{name}：目前參數無法產生有限車身接點。")
                    result[corner][name] = np.asarray(point).tolist()
    validate_points(result)
    return result


def set_point(points, corner, name, coordinates):
    """Edit either side and mirror only its corresponding point immediately."""
    if corner not in CORNERS or name not in POINTS:
        raise ValueError("未知的輪角或硬點名稱。")
    p = _coordinate(coordinates, f"{corner}/{name}")
    if p[1] * (1 if corner.endswith("L") else -1) <= 0:
        raise ValueError("左側硬點 Y 必須大於 0；右側硬點 Y 必須小於 0。")
    points[corner][name] = p.tolist()
    points[MIRROR[corner]][name] = (p * [1, -1, 1]).tolist()


def _coordinate(value, label):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(type(x) not in (float, int) or not math.isfinite(x) for x in value)):
        raise ValueError(f"{label}：需要三個有限 XYZ 座標（m）。")
    return np.asarray(value, dtype=float)


def validate_points(points):
    if not isinstance(points, dict) or set(points) != set(CORNERS):
        raise ValueError("硬點必須包含 FL、FR、RL、RR 四個輪角。")
    for corner in CORNERS:
        if not isinstance(points[corner], dict) or set(points[corner]) != set(POINTS):
            raise ValueError(f"{corner}：需要上／下 A 臂各三點，共六個硬點。")
    for corner in CORNERS:
        for name in POINTS:
            p = _coordinate(points[corner][name], f"{corner}/{name}")
            if p[1] * (1 if corner.endswith("L") else -1) <= 0:
                raise ValueError(f"{corner}/{name}：硬點不可越過車身中心線。")
            other = _coordinate(points[MIRROR[corner]][name], f"{MIRROR[corner]}/{name}")
            if not np.allclose(p * [1, -1, 1], other, atol=1e-10, rtol=0):
                raise ValueError(f"{corner}/{name}：左右硬點必須以 Y=0 鏡射對稱。")
    return points


def _intersection(a, b, c, d):
    u, v = b - a, d - c
    determinant = u[0] * v[1] - u[1] * v[0]
    if abs(determinant) < 1e-12 * max(1, np.linalg.norm(u) * np.linalg.norm(v)):
        return None
    w = c - a
    return a + (w[0] * v[1] - w[1] * v[0]) / determinant * u


def _arm(first, last, joint, label):
    hinge = last - first
    if np.linalg.norm(hinge) < 1e-8:
        raise ValueError(f"{label}：前後車身接點重合，無法建立 A 臂旋轉軸。")
    if np.linalg.norm(np.cross(hinge, joint - first)) / np.linalg.norm(hinge) < 1e-8:
        raise ValueError(f"{label}：球接點位於車身旋轉軸上，A 臂三點不可共線。")
    if abs(hinge[0]) < 1e-8:
        raise ValueError(f"{label}：目前 FV 平面模型要求前後車身接點有非零 X 間距。")
    pivot = first + (joint[0] - first[0]) / hinge[0] * hinge
    velocity = np.cross(hinge / np.linalg.norm(hinge), joint - first)
    tangent = np.array([-velocity[0], velocity[2]])
    if np.linalg.norm(tangent) < 1e-10:
        raise ValueError(f"{label}：側視投影沒有可用的運動方向。")
    tangent /= np.linalg.norm(tangent)
    if tangent[1] < 0:
        tangent = -tangent
    return np.array([-pivot[1], pivot[2]]), tangent


def build_scene(project, car, suspension_f, suspension_r):
    """Keep every input XYZ exactly; derive analysis data without rebuilding it."""
    source = validate_points(project["hardpoints"]["corners"])
    points = {corner: {name: np.asarray(p, dtype=float) for name, p in data.items()}
              for corner, data in source.items()}
    lines, geometry, derived, sv_sides = [], {"hardpoints": copy.deepcopy(source)}, {}, {}
    kingpins, wheels = {}, {}
    mount_groups = {name: {} for name in ("upper_arm_mount_geometry", "lower_arm_mount_geometry",
                                         "rear_upper_arm_mount_geometry", "rear_lower_arm_mount_geometry")}

    def add(values, color="#64748b", group="physical", reference=False, dash=()):
        lines.append(WireframeLine(np.asarray(values), color, 1.5, dash, group, reference))

    def project_sv(value):
        p = np.asarray(value)
        return np.stack((-car.l - p[..., 0], np.zeros(p.shape[:-1]), p[..., 1]), axis=-1)

    def local_sv(p):
        return np.array([-car.l - p[0], p[2]])

    for axle, prefix, track, radius, suspension in (
            ("front", "F", car.tf, car.load_Radius[0], suspension_f),
            ("rear", "R", car.tr, car.load_Radius[1], suspension_r)):
        right = points[prefix + "R"]
        upper, lower = right["UBJ"], right["LBJ"]
        axis = upper - lower
        if axis[2] <= 1e-8:
            raise ValueError(f"{axle}：上球接點 Z 必須高於下球接點。")
        ground = lower - lower[2] / axis[2] * axis
        top = upper + .25 / axis[2] * axis
        pivots, directions = {}, []
        for kind, joint in (("UCA", upper), ("LCA", lower)):
            pivots[kind], tangent = _arm(right[kind + "_F"], right[kind + "_R"], joint, f"{axle}/{kind}")
            directions.append(tangent)
        x = 0. if axle == "front" else -car.l
        wc = np.array([x, -track / 2, radius])
        contact = np.array([x, -track / 2, 0.])
        fv_u, fv_l = np.array([-upper[1], upper[2]]), np.array([-lower[1], lower[2]])
        ic = _intersection(pivots["UCA"], fv_u, pivots["LCA"], fv_l)
        fv_contact = np.array([track / 2, 0.])
        rc = None if ic is None else _intersection(fv_contact, ic, fv_contact * [-1, 1], ic * [-1, 1])
        static = {"kua": fv_u, "lak": fv_l, "cpb_ru": pivots["UCA"], "cpb_rl": pivots["LCA"],
                  "kg": np.array([-ground[1], 0.]), "gnd": fv_contact, "wheel_center": np.array([track / 2, radius]),
                  "IC": ic, "RC": rc, "cg": np.array([0., car.h_cog])}
        geometry[axle] = SimpleNamespace(static_pts=static, helper=car, longitudinal_x=x)
        sv_u, sv_l = local_sv(upper), local_sv(lower)
        normals = [np.array([-t[1], t[0]]) for t in directions]
        sv_ic = _intersection(sv_u, sv_u + normals[0], sv_l, sv_l + normals[1])
        sign = 1 if axle == "front" else -1
        sv_sides[axle] = SimpleNamespace(
            kingpin_Au=sv_u, kingpin_Al=sv_l, kingpin_ground=local_sv(ground), kingpin_top=local_sv(top),
            wheel_center=local_sv(wc), contact_point=local_sv(contact), initial_ic=sv_ic,
            direction_sign=sign, suspension=suspension, motion_directions=directions)
        sv_contact = local_sv(contact)
        anti = axle_anti(axle, sv_ic, sv_contact, local_sv(wc), car.l, car.h_cog,
                         project["vehicle"]["brake_bias_front"], project["vehicle"]["drive_force_front"])
        derived[axle] = {
            "kpi_deg": math.degrees(math.atan2(axis[1], axis[2])),
            "caster_deg": math.degrees(math.atan2(-sign * axis[0], axis[2])),
            "scrub_radius": float(ground[1] - contact[1]),
            "mechanical_trail": float(sign * (ground[0] - contact[0])),
            "upper_offset": float(upper[2] - radius), "lower_offset": float(radius - lower[2]),
            "upper_mount_x": float(pivots["UCA"][0]), "lower_mount_x": float(pivots["LCA"][0]),
            "fvsa": None if ic is None else float(fv_contact[0] - ic[0]),
            "rc_height": None if rc is None else float(rc[1]),
            "svsa": None if sv_ic is None else float(sign*(sv_ic[0] - sv_contact[0])),
            **{key: anti.get(key) for key in ("anti_dive", "anti_lift", "anti_squat")},
        }
        for suffix in ("L", "R"):
            corner, p = prefix + suffix, points[prefix + suffix]
            mirror = np.array([1, -1 if suffix == "L" else 1, 1])
            kingpins[corner] = {"UBJ": p["UBJ"].copy(), "LBJ": p["LBJ"].copy(),
                                "ground": ground * mirror, "axis_top": top * mirror}
            center = wc * mirror
            wheels[corner] = {"center": center}
            color = "#0072b2" if prefix == "F" else "#d55e00"
            for kind, joint, group in (("UCA", "UBJ", "upper"), ("LCA", "LBJ", "lower")):
                first, last = p[kind + "_F"], p[kind + "_R"]
                add([first, p[joint], last, first], color)
                key = ("rear_" if axle == "rear" else "") + group + "_arm_mount_geometry"
                aliases = ("UFI", "URI") if kind == "UCA" else ("LFI", "LRI")
                mount_groups[key][corner] = {"mounts": dict(zip(aliases, (first, last)))}
            add([p["LBJ"], p["UBJ"]], color)
            add([ground * mirror, top * mirror], color, reference=True, dash=(7, 4))
            angles = np.linspace(0, 2 * np.pi, 65)
            for offset in (-car.tire_width / 2, car.tire_width / 2):
                add(center + np.column_stack((car.free_radius * np.sin(angles),
                                              np.full(angles.shape, offset), car.free_radius * np.cos(angles))), color)
            for angle in (0, np.pi / 2, np.pi, 3 * np.pi / 2):
                c = center + [car.free_radius * math.sin(angle), 0, car.free_radius * math.cos(angle)]
                add([c - [0, car.tire_width / 2, 0], c + [0, car.tire_width / 2, 0]], color)
        for a, b in (("cpb_ru", "kua"), ("cpb_rl", "lak"), ("lak", "kua")):
            for side in (-1, 1):
                add([[x, -side * static[a][0], static[a][1]], [x, -side * static[b][0], static[b][1]]],
                    color, axle)
        for joint, tangent in zip((sv_u, sv_l), directions):
            add(project_sv([joint - .15 * tangent, joint + .15 * tangent]), "#009e73", "sv", True, (7, 4))
    front, rear = sv_sides["front"], sv_sides["rear"]
    pc = (None if front.initial_ic is None or rear.initial_ic is None else
          _intersection(front.contact_point, front.initial_ic, rear.contact_point, rear.initial_ic))
    geometry.update(mount_groups)
    geometry.update(kingpins=kingpins, wheel_projection_geometry=wheels,
                    side_view=SimpleNamespace(front=front, rear=rear, local_pitch_center=pc, project=project_sv))
    all_points = np.concatenate([line.points for line in lines])
    bounds = np.column_stack((all_points.min(axis=0) - .08, all_points.max(axis=0) + .08))
    scene = WireframeScene(lines, bounds, bounds.copy(), -55., 25., geometry)
    return scene, derived
