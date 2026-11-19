"""Build the reference mechanism from the project's static suspension geometry."""

from dataclasses import dataclass
from itertools import combinations
from typing import TYPE_CHECKING

import numpy as np

from ._geometry_support import hardpoints, project as geometry_project
from ._geometry_support.static.fv.wireframe_geometry import build_wireframe_scene
from .project import validate

if TYPE_CHECKING:
    from .elements import Member


FIXED = ("UCA_F", "UCA_R", "LCA_F", "LCA_R", "RACK")
MOVING = ("UBJ", "LBJ", "WC", "TOE")
POINTS = FIXED + MOVING
ARM_LINKS = (("UBJ", "UCA_F"), ("UBJ", "UCA_R"), ("LBJ", "LCA_F"), ("LBJ", "LCA_R"))
UPRIGHT_LINKS = tuple(combinations(MOVING, 2))
LINKS = ARM_LINKS + UPRIGHT_LINKS + (("TOE", "RACK"),)


@dataclass(frozen=True)
class CornerGeometry:
    name: str
    axle: str
    reference: tuple
    lengths: tuple
    wheel_radius: float
    wheel_width: float
    members: tuple["Member", ...] | None = None
    body_mount: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class VehicleGeometry:
    corners: tuple[CornerGeometry, ...]
    cg: tuple[float, float, float]
    chassis_edges: tuple


def static_geometry(geometry):
    """Return the common hardpoints, car and four static wheel centers."""
    geometry = geometry_project.validate(geometry)
    car, front, rear = geometry_project.static_objects(geometry)
    if hardpoints.enabled(geometry):
        scene, _ = hardpoints.build_scene(geometry, car, front, rear)
        points = scene.geometry["hardpoints"]
    else:
        scene = build_wireframe_scene(car, front, rear, mount_x=geometry.get("arm_mount_x"))
        for axle, checks in scene.geometry["mount_position_checks"].items():
            for name, check in checks.items():
                if check["requested_x"] is not None and check["fallback"]:
                    raise ValueError(f"{axle} · {geometry_project.MOUNT_X_FIELDS[name]}：{check['message']}")
        points = hardpoints.from_scene(scene)
    centers = {key: wheel["center"] for key, wheel in scene.geometry["wheel_projection_geometry"].items()}
    return points, car, centers


def chassis_edges(corners):
    """Construct the fixed chassis envelope from the inboard hinge midpoints."""
    perimeter = ("FL", "FR", "RR", "RL", "FL")
    rings = []
    for kind in ("UCA", "LCA"):
        ring = [tuple(np.mean([corners[name][kind + "_F"], corners[name][kind + "_R"]], axis=0))
                for name in perimeter]
        rings.append(ring)
    upper, lower = rings
    return tuple(zip(upper, upper[1:])) + tuple(zip(lower, lower[1:])) + tuple(zip(upper[:4], lower[:4]))


def build_vehicle(data):
    data = validate(data)
    corners, car, centers = static_geometry(data["geometry"])
    model = []
    for name in hardpoints.CORNERS:
        axle = "front" if name.startswith("F") else "rear"
        points = dict(corners[name])
        points["WC"] = np.asarray(centers[name])
        mirror = np.array([1., 1. if name.endswith("L") else -1., 1.])
        for key, endpoint in (("TOE", "outer"), ("RACK", "inner")):
            points[key] = points["WC"] + mirror * data["steering"][axle][endpoint]
        reference = np.array([points[point] for point in POINTS])
        lengths = tuple(float(np.linalg.norm(np.asarray(points[a]) - points[b])) for a, b in LINKS)
        if not np.isfinite(reference).all() or not np.isfinite(lengths).all() or min(lengths) < 1e-6:
            raise ValueError(f"{name}：接點重合或桿長無效，請檢查硬點及轉向拉桿設定。")
        relative = reference[len(FIXED) + 1:] - reference[len(FIXED)]
        if np.linalg.svd(relative, compute_uv=False)[-1] < 1e-6:
            raise ValueError(f"{name}：羊角、輪心及拉桿外接點共平面，請調整拉桿外接點偏移。")
        model.append(CornerGeometry(name, axle, tuple(map(tuple, reference)), lengths,
                                    float(points["WC"][2]), float(car.tire_width)))
    vehicle = data["geometry"]["vehicle"]
    return VehicleGeometry(tuple(model), (-vehicle["cg_to_front"], 0., vehicle["cg_height"]), chassis_edges(corners))
