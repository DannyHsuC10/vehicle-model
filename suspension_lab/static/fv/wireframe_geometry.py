"""Build the existing static suspension scene as backend-independent 3D lines.

All coordinates remain in metres.  No plotting package is imported here;
the same geometry builders used by the Matplotlib view supply the data.
"""

from dataclasses import dataclass

import numpy as np

from .. import Geometric_Analysis as ga
from .. import geometry as ge
from ..analysis_settings import AnalysisSettings
from ..car import Car, SuspensionF, SuspensionR
from .arm_plane_geometry import build_arm_plane_geometry
from .chassis_geometry import build_chassis_geometry
from .circle_plane_intersections import (
    build_circle_plane_intersections, build_rear_circle_plane_intersections,
)
from .interference_geometry import build_interference_geometry
from .kingpin_geometry import build_kingpin_geometry
from .lower_arm_mount_geometry import build_lower_arm_mount_geometry
from .rear_arm_mount_geometry import (
    build_rear_lower_arm_mount_geometry, build_rear_upper_arm_mount_geometry,
)
from .upper_arm_mount_geometry import build_upper_arm_mount_geometry
from .wheel_projection_geometry import build_axle_wheel_projection_geometry
from ..sv.side_view_geometry import build_side_view_geometry


VIEW_COLORS = {"front": "#0072b2", "rear": "#d55e00", "sv": "#009e73"}
REFERENCE_COLOR = "#666666"


@dataclass
class WireframeLine:
    points: np.ndarray
    color: str
    width: float = 1.0
    dash: tuple = ()
    group: str = "physical"
    reference: bool = False

    def __post_init__(self):
        self.points = np.array(self.points, dtype=float, copy=True)
        if (self.points.ndim != 2 or self.points.shape[1] != 3
                or len(self.points) < 2 or not np.isfinite(self.points).all()):
            raise ValueError("A wireframe line needs at least two finite 3D points.")


@dataclass
class WireframeScene:
    lines: list[WireframeLine]
    bounds: np.ndarray
    vehicle_bounds: np.ndarray
    azimuth_deg: float
    elevation_deg: float
    geometry: dict


def project_fv(points, longitudinal_x):
    """Map the original FV coordinates to vehicle +X forward, +Y left, +Z up."""
    points = np.asarray(points, dtype=float)
    return np.stack((np.full(points.shape[:-1], longitudinal_x),
                     -points[..., 0], points[..., 1]), axis=-1)


def _clip_segment(points, bounds):
    """Clip display-only construction segments, preserving source geometry."""
    start, end = np.asarray(points, dtype=float)
    direction = end - start
    lower_t, upper_t = 0.0, 1.0
    for index, (lower, upper) in enumerate(bounds):
        if abs(direction[index]) < 1e-12:
            if start[index] < lower or start[index] > upper:
                return None
            continue
        first, last = sorted(((lower - start[index]) / direction[index],
                              (upper - start[index]) / direction[index]))
        lower_t = max(lower_t, first)
        upper_t = min(upper_t, last)
        if lower_t > upper_t:
            return None
    return np.array([start + lower_t * direction, start + upper_t * direction])


class FrontViewGeometry:
    """The front/rear FV data interface consumed by the existing builders."""

    def __init__(self, car, suspension, name, longitudinal_x):
        self.name = name
        self.longitudinal_x = longitudinal_x
        self.helper = ga.SusGeometryHelper(car, suspension)
        self.static_pts = self.helper.calc_static_points()
        self.right_sus = ga.SuspensionSide(f"{name} Right")
        self.right_sus.setup_right_side(self.static_pts)
        self.left_sus = ga.SuspensionSide(f"{name} Left", is_left=True)
        self.left_sus.setup_from_right_template(self.right_sus)
        self.RC_position = project_fv(self.static_pts["RC"], longitudinal_x)


class SideViewGeometry:
    """The shared SV data aligned with the original combined 3D scene."""

    def __init__(self, car, suspension_f, suspension_r, settings):
        self.longitudinal_offset = settings.wheel_center_x_sv - car.l
        self.front, self.rear, self.local_pitch_center = build_side_view_geometry(
            vehicle=car, suspension_f=suspension_f, suspension_r=suspension_r,
            settings=settings,
        )
        self.pitch_center = self.project(self.local_pitch_center)
        self.ic_positions = {
            side.name: self.project(side.initial_ic) for side in (self.front, self.rear)
        }

    def project(self, points):
        points = np.asarray(points, dtype=float)
        return np.stack((self.longitudinal_offset - points[..., 0],
                         np.zeros(points.shape[:-1]), points[..., 1]), axis=-1)


class _LineCollector:
    """Consume the shared add_to_scene protocol while discarding all markers."""

    def __init__(self, lines, projector, color, group, bounds=None, reference_links=()):
        self.lines = lines
        self.project = projector
        self.color = color
        self.group = group
        self.bounds = bounds
        self.reference_ids = {id(link) for link in reference_links}

    def add(self, obj, **kwargs):
        if isinstance(obj, ge.Link):
            points = self.project([obj.start.pos, obj.end.pos])
            if self.bounds is not None:
                points = _clip_segment(points, self.bounds)
                if points is None:
                    return
        elif isinstance(obj, ge.Polygon):
            vertices = self.project([point.pos for point in obj.points])
            points = np.vstack((vertices, vertices[0]))
        else:
            return
        dashes = {"-": (), "--": (7, 4), ":": (2, 4), "-.": (8, 3, 2, 3)}
        self.lines.append(WireframeLine(
            points, self.color, kwargs.get("linewidth", 1.3),
            dashes.get(kwargs.get("linestyle", "-"), ()), self.group,
            id(obj) in self.reference_ids,
        ))


def build_wireframe_scene(car=None, suspension_f=None, suspension_r=None, settings=None):
    """Return only structural/construction outlines, with no fills or annotations."""
    car = Car() if car is None else car
    suspension_f = SuspensionF() if suspension_f is None else suspension_f
    suspension_r = SuspensionR() if suspension_r is None else suspension_r
    settings = AnalysisSettings() if settings is None else settings
    front = FrontViewGeometry(car, suspension_f, "Front", 0.0)
    rear = FrontViewGeometry(car, suspension_r, "Rear", -car.l)
    side_view = SideViewGeometry(car, suspension_f, suspension_r, settings)
    projections = {
        **build_axle_wheel_projection_geometry(car, front, prefix="F"),
        **build_axle_wheel_projection_geometry(car, rear, prefix="R"),
    }
    kingpins = build_kingpin_geometry(front, rear, side_view)
    chassis = build_chassis_geometry(front, rear)
    arm_planes = build_arm_plane_geometry(front, rear, side_view, kingpins)
    interference = build_interference_geometry(car, front, rear, kingpins)
    intersections = build_circle_plane_intersections(arm_planes, interference)
    rear_intersections = build_rear_circle_plane_intersections(arm_planes, interference)
    upper_mounts = build_upper_arm_mount_geometry(front, kingpins, intersections)
    lower_mounts = build_lower_arm_mount_geometry(front, kingpins, intersections)
    rear_upper_mounts = build_rear_upper_arm_mount_geometry(rear, kingpins, rear_intersections)
    rear_lower_mounts = build_rear_lower_arm_mount_geometry(rear, kingpins, rear_intersections)
    mount_groups = (upper_mounts, lower_mounts, rear_upper_mounts, rear_lower_mounts)

    lines = []

    def add(points, color=REFERENCE_COLOR, width=1.0, dash=(), group="physical", reference=False):
        lines.append(WireframeLine(points, color, width, dash, group, reference))

    transverse_min, transverse_max = settings.front_view_x_limits
    height_min, height_max = settings.front_view_y_limits
    margin = car.free_radius + 0.05
    longitudinal_limits = (-car.l - margin, margin)
    for group, plane in (("front", front), ("rear", rear)):
        color = VIEW_COLORS[group]
        collector = _LineCollector(lines, lambda p: project_fv(p, plane.longitudinal_x),
                                   color, group, reference_links=[
                                       getattr(side, name)
                                       for side in (plane.left_sus, plane.right_sus)
                                       for name in ("kingpin", "upper_force", "lower_force", "wheel_force")
                                   ])
        plane.right_sus.add_to_scene(collector)
        plane.left_sus.add_to_scene(collector)
        corners = project_fv([
            [transverse_min, height_min], [transverse_max, height_min],
            [transverse_max, height_max], [transverse_min, height_max],
        ], plane.longitudinal_x)
        add(np.vstack((corners, corners[0])), color, 1.0, (7, 4), group)
        for height in (0.0, plane.helper.load_Radius):
            add(project_fv([[transverse_min, height], [transverse_max, height]],
                           plane.longitudinal_x), color, 0.9, (7, 4), group)
        add(project_fv([[0.0, height_min], [0.0, height_max]], plane.longitudinal_x),
            color, 0.8, (2, 4), group)

    angles = np.linspace(0.0, 2.0 * np.pi, 65)
    for corner, wheel in projections.items():
        group = "front" if corner.startswith("F") else "rear"
        add(wheel["tire"]["outline"], VIEW_COLORS[group], 1.8, group=group)
        add(wheel["rim"]["outline"], VIEW_COLORS[group], 1.5, (7, 4), group)
        center = wheel["center"]
        radius, half_width = wheel["tire"]["height"] / 2.0, wheel["tire"]["width"] / 2.0
        circles = [np.column_stack((center[0] + radius * np.sin(angles),
                                    np.full(angles.shape, center[1] + offset),
                                    center[2] + radius * np.cos(angles)))
                   for offset in (-half_width, half_width)]
        for circle in circles:
            add(circle, "#777777", 0.9)
        for index in (0, 16, 32, 48):
            add([circles[0][index], circles[1][index]], "#777777", 0.9)

    sv_bounds = [longitudinal_limits, (-0.01, 0.01), settings.front_view_y_limits]
    collector = _LineCollector(lines, side_view.project, VIEW_COLORS["sv"], "sv", sv_bounds,
                               reference_links=[
                                   getattr(side, name)
                                   for side in (side_view.front, side_view.rear)
                                   for name in ("kingpin_link", "svsa_link", "au_ic_link", "al_ic_link",
                                                "anti_lift_link", "au_tangent", "al_tangent")
                               ])
    side_view.front.add_to_scene(collector)
    side_view.rear.add_to_scene(collector)
    corners = np.array([
        [longitudinal_limits[0], 0.0, height_min],
        [longitudinal_limits[1], 0.0, height_min],
        [longitudinal_limits[1], 0.0, height_max],
        [longitudinal_limits[0], 0.0, height_max],
    ])
    add(np.vstack((corners, corners[0])), VIEW_COLORS["sv"], 1.0, (7, 4), "sv")
    add([[longitudinal_limits[0], 0.0, 0.0], [longitudinal_limits[1], 0.0, 0.0]],
        VIEW_COLORS["sv"], 0.8, (7, 4), "sv")
    add([front.RC_position, rear.RC_position], width=1.2, dash=(7, 4))

    for corner, wheel in kingpins.items():
        color = VIEW_COLORS["front" if corner.startswith("F") else "rear"]
        add([wheel["ground"], wheel["axis_top"]], color, 1.8, (7, 4), reference=True)
        add([wheel["LBJ"], wheel["UBJ"]], color, 3.4)
    for first, last in chassis["edges"]:
        add([chassis["vertices"][first], chassis["vertices"][last]])
    for plane in arm_planes.values():
        color = VIEW_COLORS["front" if plane["corner"].startswith("F") else "rear"]
        add(np.vstack((plane["vertices"], plane["vertices"][0])), color, 0.8, reference=True)
    circle_dashes = {"neutral": (), "positive": (7, 4), "negative": (2, 4)}
    for corner, wheel in interference.items():
        color = VIEW_COLORS["front" if corner.startswith("F") else "rear"]
        for name, circle in wheel["circles"].items():
            add(circle["points"], color, 2.0, circle_dashes[name])
    for geometry in mount_groups:
        for corner, arm in geometry.items():
            joint = arm["UBJ"] if "UBJ" in arm else arm["LBJ"]
            color = VIEW_COLORS["front" if corner.startswith("F") else "rear"]
            for leg in arm["legs"]:
                if leg["endpoint"] is not None:
                    add([joint, leg["endpoint"]], color, 2.6,
                        () if leg["is_extension"] else (7, 4))

    vehicle_bounds = np.array([
        longitudinal_limits, (-transverse_max - 0.08, -transverse_min + 0.08),
        (height_min, height_max + 0.12),
    ])
    circle_points = np.concatenate([
        circle["points"] for wheel in interference.values() for circle in wheel["circles"].values()
    ])
    vehicle_bounds[:, 0] = np.minimum(vehicle_bounds[:, 0], circle_points.min(axis=0) - 0.02)
    vehicle_bounds[:, 1] = np.maximum(vehicle_bounds[:, 1], circle_points.max(axis=0) + 0.02)
    mount_points = [endpoint for geometry in mount_groups for arm in geometry.values()
                    for endpoint in arm["mounts"].values()]
    if mount_points:
        mount_points = np.asarray(mount_points)
        vehicle_bounds[:, 0] = np.minimum(vehicle_bounds[:, 0], mount_points.min(axis=0) - 0.06)
        vehicle_bounds[:, 1] = np.maximum(vehicle_bounds[:, 1], mount_points.max(axis=0) + 0.06)
    vertices = np.concatenate([plane["vertices"] for plane in arm_planes.values()])
    padding = np.array([margin, 0.08, 0.06])
    bounds = vehicle_bounds.copy()
    bounds[:, 0] = np.minimum(bounds[:, 0], vertices.min(axis=0) - padding)
    bounds[:, 1] = np.maximum(bounds[:, 1], vertices.max(axis=0) + padding)

    geometry = {
        "front": front, "rear": rear, "side_view": side_view,
        "wheel_projection_geometry": projections, "kingpins": kingpins,
        "chassis_geometry": chassis, "arm_planes": arm_planes,
        "interference_geometry": interference, "circle_plane_intersections": intersections,
        "rear_circle_plane_intersections": rear_intersections,
        "upper_arm_mount_geometry": upper_mounts, "lower_arm_mount_geometry": lower_mounts,
        "rear_upper_arm_mount_geometry": rear_upper_mounts,
        "rear_lower_arm_mount_geometry": rear_lower_mounts,
    }
    return WireframeScene(lines, bounds, vehicle_bounds,
                          settings.front_view_azimuth_deg, settings.front_view_elevation_deg,
                          geometry)
