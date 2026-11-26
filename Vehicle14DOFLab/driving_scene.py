"""Project the replay's vehicle and road geometry onto the native canvas."""

from dataclasses import dataclass
import json

import numpy as np

from ._vendor.suspension.project import CORNERS

from .lap import Track
from .replay_stl import Assembly, load_assembly


def shade_palette(rgb):
    shades = np.linspace(.76, 1., 32)
    return np.array(["#{:02x}{:02x}{:02x}".format(*color)
                     for color in np.rint(shades[:, None] * np.asarray(rgb)).astype(int)])


BODY_PALETTE = shade_palette((61, 79, 102))
WHEEL_PALETTE = shade_palette((147, 154, 162))


def road_markings(centerline, width):
    steps = np.linalg.norm(np.diff(centerline, axis=0), axis=1)
    points = centerline[np.r_[True, steps > 1e-5]]
    if len(points) < 2:
        return points, points, np.empty((0, 3, 2))

    segments = np.diff(points, axis=0)
    length = np.linalg.norm(segments, axis=1)
    normals = np.column_stack((-segments[:, 1], segments[:, 0])) / length[:, None]
    closed = np.linalg.norm(points[-1] - points[0]) <= 1e-5
    before = np.vstack((normals[-1] if closed else normals[0], normals))
    after = np.vstack((normals, normals[0] if closed else normals[-1]))
    # Join the offset lines at each vertex and limit the miter at sharp turns.
    alignment = np.einsum("ij,ij->i", before, after)
    offset = (width / 2.) * (before + after) / np.maximum(.5, 1. + alignment[:, None])

    station = np.r_[0., length.cumsum()]
    starts = np.arange(0., station[-1], 2.6)
    samples = np.minimum(starts[:, None] + [0., .6, 1.2], station[-1])
    dashes = np.stack([np.interp(samples, station, axis) for axis in points.T], axis=-1)
    return points + offset, points - offset, dashes


@dataclass
class ReplayGeometry:
    axles: np.ndarray
    radius: float
    cg_height: float
    road: tuple
    assembly: Assembly | None = None
    cad_origin: np.ndarray | None = None
    cad_warning: str = ""

    @classmethod
    def from_result(cls, result):
        summary = result.timing[0] if result.timing else {}

        chassis, vehicle = {}, {}
        for key, dest in (("suspension_config", chassis), ("vehicle_config", vehicle)):
            path = result.source(key)
            if path is None:
                continue
            try:
                raw = json.loads(path.read_text(encoding="utf-8-sig"))
                dest.update(raw.get("vehicle", {}) if key == "suspension_config" else raw)
            except (OSError, ValueError, TypeError, AttributeError):
                pass
        def positive(source_data, key, default):
            try:
                measured = float(source_data.get(key, default))
            except (TypeError, ValueError):
                return default
            return measured if np.isfinite(measured) and measured > 0. else default

        lf, lr = positive(chassis, "lf", .765), positive(chassis, "lr", .765)
        tf, tr = positive(chassis, "tf", 1.25) / 2., positive(chassis, "tr", 1.24) / 2.
        axles = np.array([[lf, tf], [lf, -tf], [-lr, tr], [-lr, -tr]])
        centerline = None
        width = 3.
        if summary.get("track_csv"):
            try:
                width = float(summary.get("track_width_m") or 3.)
                track = Track.from_csv(result.source("track_csv"), width=width)
                centerline = np.vstack((track.xy, track.xy[0]))
            except (OSError, ValueError):
                width = 3.
        elif summary.get("distance_m"):
            distance = positive(summary, "distance_m", 0.)
            if distance > 0:
                count = int(min(1000, distance)) + 2
                centerline = np.column_stack((np.linspace(0, distance, count), np.zeros(count)))
        if centerline is None:
            centerline = np.column_stack((result.data["x_m"], result.data["y_m"]))
        cg_height = positive(chassis, "h_cg", .30)
        assembly, warning = None, ""
        try:
            assembly = load_assembly()
        except (OSError, ValueError) as error:
            warning = str(error)
        cad_origin = assembly.reference(axles, cg_height) if assembly is not None else None
        return cls(axles, positive(vehicle, "free_radius_m", positive(vehicle, "rolling_radius_m", .203)), cg_height,
                   road_markings(centerline, width), assembly, cad_origin, warning)


def rotation(yaw, pitch=0., roll=0.):
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    return (np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
            @ np.array([[cp, 0., -sp], [0., 1., 0.], [sp, 0., cp]])
            @ np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]]))


def value(row, key, fallback=0.):
    if key not in row.dtype.names or not np.isfinite(row[key]):
        return fallback
    return float(row[key])


class Camera:
    def __init__(self, center, yaw, width, height, overhead):
        forward = np.array([np.cos(yaw), np.sin(yaw), 0.])
        left = np.array([-forward[1], forward[0], 0.])
        offset = -forward * (4.5 if overhead else 5.5) - left * (1.5 if overhead else 3.4)
        self.eye = center + offset + np.array([0., 0., 12. if overhead else 3.2])
        target = center + forward * (1.9 if overhead else 1.05)
        direction = target - self.eye
        direction /= np.linalg.norm(direction)
        right = np.cross(direction, [0., 0., 1.])
        right /= np.linalg.norm(right)
        self.axes = np.array([right, np.cross(right, direction), direction])
        self.focal = min(width * 1.2, height * 1.18)
        self.origin = np.array([width * .50, height * .55])

    def project(self, xyz):
        local = (np.asarray(xyz) - self.eye) @ self.axes.T
        depth = local[:, 2]
        screen = self.origin + local[:, :2] * np.array([1., -1.]) * self.focal / np.maximum(.35, depth[:, None])
        return screen, depth


def wheel_pose(hub, cad_origin, origin, body_matrix, row, yaw, corner):
    center = (hub - cad_origin) @ body_matrix.T + origin
    center[2] += value(row, f"{corner}_wheel_travel_mm") / 1000.
    steer = np.radians(value(row, f"{corner}_steer_deg"))
    camber = np.radians(value(row, f"{corner}_camber_deg"))
    camber *= -1. if corner.endswith("L") else 1.
    # A forward rotation about the wheel's +Y spin axis lowers its +X tread.
    spin = value(row, f"{corner}_wheel_spin_rad")
    matrix = rotation(yaw + steer, roll=camber) @ rotation(0., pitch=-spin)
    return center, matrix


def draw_assembly(canvas, geometry, camera, row, origin, body_matrix, yaw):
    depths, polygons, colors = [], [], []
    light = np.array([-.28, -.42, .86])

    def mesh_part(mesh, vertices, matrix, center, palette):
        world = vertices @ matrix.T + center
        normals = mesh.normals @ matrix.T
        facing = np.einsum("ij,ij->i", normals, camera.eye - world[mesh.faces[:, 0]]) > 0.
        screen, depth = camera.project(world)
        facing &= (depth[mesh.faces] > .45).all(axis=1)
        indices = mesh.faces[facing]
        if not len(indices):
            return
        shade = np.rint(31. * np.clip(normals[facing] @ light, 0., 1.)).astype(int)
        depths.append(depth[indices].mean(axis=1))
        polygons.append(screen[indices].reshape(-1, 6))
        colors.append(palette[shade])

    assembly, pivot = geometry.assembly, geometry.cad_origin
    mesh_part(assembly.body, assembly.body.vertices - pivot, body_matrix, origin, BODY_PALETTE)
    for corner, hub, mesh in zip(CORNERS, assembly.hubs, assembly.wheels):
        center, matrix = wheel_pose(hub, pivot, origin, body_matrix, row, yaw, corner)
        mesh_part(mesh, mesh.vertices, matrix, center, WHEEL_PALETTE)
    if depths:
        coords, rgb = np.concatenate(polygons), np.concatenate(colors)
        for index in np.argsort(np.concatenate(depths))[::-1]:
            shade = str(rgb[index])
            canvas.create_polygon(*coords[index], fill=shade, outline=shade, width=1, tags="stl_vehicle")


def draw_road(canvas, road, camera, position):
    left, right, dashes = road

    def visible(points):
        delta = points - position
        return np.einsum("ij,ij->i", delta, delta) < 22.**2

    for edge in (left, right):
        nearby = visible(edge)
        cuts = np.flatnonzero(np.diff(np.r_[False, nearby, False]))
        for start, end in zip(cuts[::2], cuts[1::2]):
            points = edge[start:end]
            screen, depth = camera.project(np.column_stack((points, np.zeros(len(points)))))
            in_front = np.flatnonzero(np.diff(np.r_[False, depth > .55, False]))
            for first, last in zip(in_front[::2], in_front[1::2]):
                if last - first > 1:
                    canvas.create_line(*screen[first:last].ravel(), fill="#668088", width=2, tags="road_edge")

    marks = dashes[visible(dashes[:, 1])].reshape(-1, 2)
    screen, depth = camera.project(np.column_stack((marks, np.zeros(len(marks)))))
    for dash in screen.reshape(-1, 3, 2)[(depth.reshape(-1, 3) > .55).all(axis=1)]:
        canvas.create_line(*dash.ravel(), fill="#829ba0", width=1, tags="road_center")


def draw_scene(canvas, geometry, row, *, heading, overhead=False):
    width, height = canvas.winfo_width(), canvas.winfo_height()
    canvas.delete("all")
    if width < 120 or height < 120:
        return
    world = np.array([row["x_m"], row["y_m"], 0.])
    yaw = np.radians(value(row, "yaw_deg", np.degrees(heading)))
    roll, pitch = np.radians([value(row, "roll_deg"), value(row, "pitch_deg")])
    heave = value(row, "heave_mm") / 1000.
    camera = Camera(world, yaw, width, height, overhead)
    draw_road(canvas, geometry.road, camera, world[:2])

    origin = world + [0., 0., geometry.cg_height + heave]
    body_matrix = rotation(yaw, pitch, roll)
    view_label = "俯視追蹤" if overhead else "跟車視角"
    if geometry.assembly is not None:
        draw_assembly(canvas, geometry, camera, row, origin, body_matrix, yaw)
        canvas.create_text(12, 12, anchor="nw", text=f"STL 車輛 · {view_label}", fill="#173845",
                           font=("Microsoft JhengHei UI", 10, "bold"))
        return
    faces = []

    def face(vertices, fill, border=""):
        screen, depth = camera.project(vertices)
        faces.append((float(depth.mean()), screen.ravel(), fill, border))

    def prism(front, rear, half_front, half_rear, floor, roof, color, shade):
        ring = np.array([[front, half_front, floor], [rear, half_rear, floor],
                         [rear, -half_rear, floor], [front, -half_front, floor]])
        bottom = ring @ body_matrix.T + origin
        ring[:, 2] = roof
        top = ring @ body_matrix.T + origin
        face(top, color, "#143742")
        for i in range(4):
            j = (i + 1) % 4
            face([bottom[i], bottom[j], top[j], top[i]], shade, "#143742")

    front, rear = geometry.axles[0, 0], geometry.axles[2, 0]
    body_width = min(geometry.axles[0, 1], geometry.axles[2, 1])
    base = -geometry.cg_height + .095
    prism(front * .40, rear - .13, .50 * body_width, .65 * body_width, base, .16, "#197b82", "#13545e")
    prism(front + .38, front * .40, .18 * body_width, .50 * body_width, base, .06, "#39a2a2", "#186571")
    prism(front + .43, front + .19, .87 * body_width, .87 * body_width, base - .015, base + .02, "#30838a", "#17434c")
    prism(rear - .18, rear - .40, .88 * body_width, .88 * body_width, .19, .23, "#39a2a2", "#13545e")
    prism(front * .09, rear * .44, .28 * body_width, .38 * body_width, .16, .185, "#182a35", "#14232b")
    prism(rear * .10, rear * .30, .18 * body_width, .18 * body_width, .185, .29, "#f3bb67", "#b77e38")

    radius = geometry.radius
    phase = np.linspace(0., 2. * np.pi, 16, endpoint=False)
    for corner, (axle_x, axle_y) in zip(CORNERS, geometry.axles):
        position = np.array([axle_x, axle_y, radius - geometry.cg_height]) @ body_matrix.T + origin
        position[2] += value(row, f"{corner}_wheel_travel_mm") / 1000.
        angle = yaw + np.radians(value(row, f"{corner}_steer_deg"))
        forward, side = np.array([np.cos(angle), np.sin(angle), 0.]), np.array([-np.sin(angle), np.cos(angle), 0.])
        out = side * (1 if axle_y > 0 else -1)
        vertical = np.array([0., 0., 1.])
        radial = np.cos(phase)[:, None] * forward + np.sin(phase)[:, None] * vertical
        outside, inside = position + .09 * out + radius * radial, position - .09 * out + radius * radial
        for index in range(len(phase)):
            j = (index + 1) % len(phase)
            face([outside[index], outside[j], inside[j], inside[index]], "#202e37" if index % 2 else "#293943")
        face(outside, "#17252d")
        face(inside, "#17252d")
        rim = position + .092 * out
        face(rim + .62 * radius * radial, "#778c97", "#182c37")
        spin = value(row, f"{corner}_wheel_spin_rad")
        spoke = np.cos(spin) * forward + np.sin(spin) * vertical
        # Paint the narrow marker on the wheel face in the same depth-sorted pass.
        side_spoke = np.cross(out, spoke) * .012
        tip = rim + .57 * radius * spoke
        face([rim + side_spoke, tip + side_spoke, tip - side_spoke, rim - side_spoke], "#e8e8da")
    for _, points, fill, border in sorted(faces, key=lambda item: item[0], reverse=True):
        canvas.create_polygon(*points, fill=fill, outline=border)
    canvas.create_text(12, 12, anchor="nw", text=view_label,
                       fill="#173845", font=("Microsoft JhengHei UI", 10, "bold"))
    if geometry.cad_warning:
        canvas.create_text(12, 35, anchor="nw", text=geometry.cad_warning, width=width - 24, fill="#935336")
