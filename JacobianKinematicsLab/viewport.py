"""Interactive orthographic view of the actual solved point and rod geometry."""

from itertools import product
from math import cos, hypot, pi, sin
import tkinter as tk

import numpy as np

from .model import CORNERS
from .wheel_frame import TireSetup, WheelFrame


VIEWS = {
    "iso": (.68, .48),
    "front": (pi / 2, 0.),
    "side": (0., 0.),
    "top": (-pi / 2, pi / 2),
}
BACKGROUND = "#102534"
MOVING = "#49d9c0"
FIXED = "#b1c6d7"
ROD = "#85b8d5"
SELECTED = "#ffbc70"
REFERENCE = "#57768c"
FONT = "Microsoft JhengHei UI"
TIRE_ANGLES = np.linspace(0., 2. * pi, 32, endpoint=False)
TIRE_COLOR = "#658597"
SUSPENSION_RODS = (
    ("_RACK-TOE", "轉向拉桿", "#84aeff"),
    ("_PUSH_ROD", "推桿", "#e5a9f5"),
    ("FRONT_RACK_BAR", "齒條", "#bada82"),
)


class MechanismViewport(tk.Canvas):
    def __init__(self, parent, on_pick=None):
        super().__init__(parent, height=360, background=BACKGROUND, highlightthickness=0)
        self.on_pick = on_pick
        self.model = None
        self.position = None
        self.view = "iso"
        self.azimuth, self.elevation = VIEWS[self.view]
        self.zoom = 1.
        self.pan = np.zeros(2)
        self.reference_visible = False
        self.labels_visible = False
        self.selected_point = self.selected_rod = None
        self._orbit_origin = self._drag_at = self._pan_at = None
        self._orbit_moved = False
        self.bind("<Configure>", self.draw)
        self.bind("<ButtonPress-1>", self._start_orbit)
        self.bind("<B1-Motion>", self._orbit)
        self.bind("<ButtonRelease-1>", self._finish_orbit)
        for button in (2, 3):
            self.bind(f"<ButtonPress-{button}>", self._start_pan)
            self.bind(f"<B{button}-Motion>", self._pan)
        self.bind("<MouseWheel>", self._wheel)
        self.bind("<Button-4>", self._wheel)
        self.bind("<Button-5>", self._wheel)

    def set_model(self, model, positions=None, *, tires=None):
        """Set topology and fix camera fitting to the bounds of the entire motion."""
        new_model = self.model is not model
        self.model = model
        if model is None:
            self.position = None
            self.selected_point = self.selected_rod = None
            self.draw()
            return

        self.position = model.reference
        points = {name: i for i, name in enumerate(model.point_names)}
        self.fixed = np.zeros(len(points), dtype=bool)
        self.fixed[[points[name] for name in model.fixed]] = True
        self.visible_points = np.array([name not in model.internal_points for name in model.point_names])
        self.members = np.asarray([(points[link.a], points[link.b]) for link in model.links], dtype=int).reshape(-1, 2)
        self.fixed_members = self.fixed[self.members].all(axis=1)
        self.visible_rods = self.visible_points[self.members].all(axis=1)
        self.tires = {}
        for corner in CORNERS:
            setup = (tires or {}).get(corner, TireSetup())
            frame = WheelFrame.from_model(model, corner, points, setup)
            if frame is None:
                continue
            self.tires[corner] = (frame, setup.width_mm / 1000., setup.radius_mm / 1000.)
        self.rod_styles = [next((style for style in SUSPENSION_RODS if link.name.endswith(style[0])), None)
                           for link in model.links]
        self.rod_legend = tuple(style for style in SUSPENSION_RODS if style in self.rod_styles)

        low, high = model.reference.min(axis=0), model.reference.max(axis=0)
        if positions is not None and len(positions):
            low = np.minimum(low, np.min(positions, axis=(0, 1)))
            high = np.maximum(high, np.max(positions, axis=(0, 1)))
        for frame, width, radius in self.tires.values():
            center = frame.center
            extent = hypot(radius, width / 2.)
            samples = model.reference[None, center] if positions is None else positions[:, center]
            low = np.minimum(low, np.minimum(model.reference[center], samples.min(axis=0)) - extent)
            high = np.maximum(high, np.maximum(model.reference[center], samples.max(axis=0)) + extent)
        self.center = (low + high) / 2.
        self.fit_corners = np.array(tuple(product(*zip(low, high))))
        self.span = max((high - low).max(), .001)
        self.grid = self._make_grid(low, high)
        if new_model:
            self.selected_point = self.selected_rod = None
            self.reset_camera()
        else:
            self.draw()

    def set_frame(self, position):
        self.position = position
        self.draw()

    def set_view(self, view):
        if view not in VIEWS:
            raise ValueError(f"Unknown viewport orientation: {view}")
        self.view = view
        self.reset_camera()

    def reset_camera(self):
        self.azimuth, self.elevation = VIEWS[self.view]
        self.zoom = 1.
        self.pan[:] = 0.
        self.draw()

    def set_reference_visible(self, visible):
        self.reference_visible = bool(visible)
        self.draw()

    def set_labels_visible(self, visible):
        self.labels_visible = bool(visible)
        self.draw()

    def set_selection(self, point, rod):
        if (point, rod) != (self.selected_point, self.selected_rod):
            self.selected_point, self.selected_rod = point, rod
            self.draw()

    def _make_grid(self, low, high):
        half = np.maximum((high - low) / 2., .15 * self.span)
        x0, y0 = (self.center - half)[:2]
        x1, y1 = (self.center + half)[:2]
        z = low[2]
        lines = [((x, y0, z), (x, y1, z)) for x in np.linspace(x0, x1, 9)]
        lines.extend(((x0, y, z), (x1, y, z)) for y in np.linspace(y0, y1, 9))
        return np.array(lines)

    def _camera_axes(self):
        az, el = self.azimuth, self.elevation
        right = np.array((cos(az), sin(az), 0.))
        toward_viewer = np.array((sin(az) * cos(el), -cos(az) * cos(el), sin(el)))
        up = np.cross(toward_viewer, right)
        return np.stack((right, up, toward_viewer))

    def _start_orbit(self, event):
        self._orbit_origin = self._drag_at = (event.x, event.y)
        self._orbit_moved = False

    def _orbit(self, event):
        if self.model is None or self._drag_at is None:
            return
        x, y = self._orbit_origin
        if not self._orbit_moved and (event.x - x) ** 2 + (event.y - y) ** 2 < 16:
            return
        self._orbit_moved = True
        x, y = self._drag_at
        self.azimuth = (self.azimuth + (event.x - x) * .01) % (2 * pi)
        self.elevation = min(pi / 2, max(-pi / 2, self.elevation + (event.y - y) * .01))
        self._drag_at = (event.x, event.y)
        self.draw()

    def _finish_orbit(self, event):
        picking = self._drag_at is not None and not self._orbit_moved
        self._orbit_origin = self._drag_at = None
        if picking:
            self._pick(event.x, event.y)

    def _start_pan(self, event):
        self._pan_at = (event.x, event.y)

    def _pan(self, event):
        if self.model is None or self._pan_at is None:
            return
        x, y = self._pan_at
        self.pan += (event.x - x, event.y - y)
        self._pan_at = (event.x, event.y)
        self.draw()

    def _wheel(self, event):
        if self.model is None:
            return
        if event.num in (4, 5):
            steps = 1 if event.num == 4 else -1
        else:
            steps = event.delta / 120
        zoom = min(10., max(.25, self.zoom * 1.15**steps))
        origin = np.array((self.winfo_width() / 2., self.winfo_height() / 2.)) + self.pan
        self.pan += (np.array((event.x, event.y)) - origin) * (1. - zoom / self.zoom)
        self.zoom = zoom
        self.draw()

    def _pick(self, x, y):
        if self.model is None:
            return
        items = self.find_overlapping(x - 4, y - 4, x + 4, y + 4)
        hits = []
        cursor = np.array((x, y))
        for item in reversed(items):
            tag = next((tag for tag in self.gettags(item) if tag.startswith(("point:", "rod:"))), None)
            if tag is None:
                continue
            kind, index = tag.split(":")
            if self.type(item) == "text":
                priority, distance = 3, 0.
            else:
                a, b = np.asarray(self.coords(item)).reshape(2, 2)
                if kind == "point":
                    distance = np.linalg.norm(cursor - (a + b) / 2.)
                    priority = 0 if distance <= (b[0] - a[0]) / 2. + 1 else 2
                else:
                    segment = b - a
                    length_squared = segment @ segment
                    fraction = np.clip((cursor - a) @ segment / length_squared, 0., 1.) if length_squared else 0.
                    priority, distance = 1, np.linalg.norm(cursor - a - fraction * segment)
            hits.append((priority, distance, kind, int(index)))
        if not hits:
            return
        _, _, kind, index = min(hits, key=lambda hit: hit[:2])
        name = self.model.point_names[index] if kind == "point" else self.model.links[index].name
        if self.on_pick is not None:
            self.on_pick(kind, name)
        elif kind == "point":
            self.set_selection(name, self.selected_rod)
        else:
            self.set_selection(self.selected_point, name)

    def _draw_axes(self, camera, width, height):
        x, y = width - 53, height - 53
        for axis, label, color in zip(camera[:2].T, "XYZ", ("#fd9c98", "#8ed8a2", "#91b7ff")):
            dx, dy = axis * (29., -29.)
            if dx * dx + dy * dy < 7:
                continue
            self.create_line(x, y, x + dx, y + dy, fill=color, width=2, arrow=tk.LAST,
                             arrowshape=(6, 8, 3))
            self.create_text(x + 1.35 * dx, y + 1.35 * dy, text=label, fill=color,
                             font=(FONT, 9, "bold"))
        self.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#d5e4f0", outline="")

    def _draw_reference(self, project):
        reference = project(self.model.reference)
        for a, b in self.members[~self.fixed_members & self.visible_rods]:
            self.create_line(*reference[[a, b]].ravel(), fill=REFERENCE, dash=(4, 4), width=1.25)
        for x, y in reference[~self.fixed & self.visible_points]:
            self.create_oval(x - 3, y - 3, x + 3, y + 3, outline=REFERENCE)

    def _draw_rods(self, screen, depth):
        depths = depth[self.members].mean(axis=1)
        order = np.argsort(depths, kind="stable")
        order = order[self.visible_rods[order]]
        selected_item = None
        for i in order:
            a, b = self.members[i]
            highlight = self.selected_rod == self.model.links[i].name
            style = self.rod_styles[i]
            color = style[2] if style else FIXED if self.fixed_members[i] else ROD
            width = 3 if style else 2
            if highlight:
                color, width = SELECTED, 4
            self.create_line(*screen[[a, b]].ravel(), fill=BACKGROUND, width=width + 3)
            item = self.create_line(*screen[[a, b]].ravel(), fill=color, width=width, tags=(f"rod:{i}",))
            if highlight:
                selected_item = item
        if selected_item is not None:
            self.tag_raise(selected_item)

    def _draw_points(self, screen, depth):
        order = np.argsort(depth, kind="stable")
        order = order[self.visible_points[order]]
        for i in order:
            x, y = screen[i]
            name = self.model.point_names[i]
            fixed = self.fixed[i]
            highlight = self.selected_point == name
            radius = 6 if highlight else 4
            color = SELECTED if highlight else FIXED if fixed else MOVING
            tags = (f"point:{i}",)
            bounds = (x - radius, y - radius, x + radius, y + radius)
            if fixed:
                self.create_rectangle(*bounds, outline=color, fill=BACKGROUND, width=2, tags=tags)
            else:
                self.create_oval(*bounds, fill=color, outline=BACKGROUND, width=1, tags=tags)
        for i in order:
            x, y = screen[i]
            name = self.model.point_names[i]
            highlight = self.selected_point == name
            if not (highlight or self.labels_visible):
                continue
            color = SELECTED if highlight else "#e2edf5"
            self.create_text(x + 8, y - 7, text=name, anchor="sw", fill=color,
                             font=(FONT, 9), tags=(f"point:{i}",))

    def _draw_tires(self, camera, project):
        ordered = sorted(self.tires.items(), key=lambda item: self.position[item[1][0].center] @ camera[2])
        for corner, (frame, width, radius) in ordered:
            center = self.position[frame.center]
            normal = frame.normal(self.position)
            radial = np.cross(normal, np.eye(3)[np.argmin(np.abs(normal))])
            radial /= np.linalg.norm(radial)
            ring = radius * (np.cos(TIRE_ANGLES)[:, None] * radial
                             + np.sin(TIRE_ANGLES)[:, None] * np.cross(normal, radial))
            back, front = project(center + ring - width / 2 * normal), project(center + ring + width / 2 * normal)
            if normal @ camera[2] < 0.:
                back, front = front, back
            tags = ("tire", f"tire:{corner}")
            for i in range(len(ring)):
                j = (i + 1) % len(ring)
                if (ring[i] + ring[j]) @ camera[2] > 0.:
                    self.create_polygon(*back[i], *back[j], *front[j], *front[i], fill=TIRE_COLOR,
                                        stipple="gray50", tags=tags)
            self.create_polygon(*back.ravel(), fill="", outline=TIRE_COLOR, tags=tags)
            self.create_polygon(*front.ravel(), fill=TIRE_COLOR, stipple="gray25", outline="#8faebb", tags=tags)

    def draw(self, _event=None):
        self.delete("all")
        width, height = self.winfo_width(), self.winfo_height()
        if width < 140 or height < 120:
            return
        if self.model is None or self.position is None:
            self.create_text(width / 2., height / 2., text="載入硬點表以顯示機構。",
                             fill=FIXED, font=(FONT, 12))
            return
        camera = self._camera_axes()
        bounds = (self.fit_corners - self.center) @ camera[:2].T
        fit = np.maximum(np.ptp(bounds, axis=0), self.span * .12)
        scale = self.zoom * min((width - 92) / fit[0], (height - 80) / fit[1])
        origin = np.array((width / 2., height / 2.)) + self.pan

        def project(points):
            return origin + ((points - self.center) @ camera[:2].T) * (scale, -scale)

        for segment in project(self.grid):
            self.create_line(*segment.ravel(), fill="#233d50")
        self._draw_tires(camera, project)
        if self.reference_visible:
            self._draw_reference(project)

        screen = project(self.position)
        depth = (self.position - self.center) @ camera[2]
        self._draw_rods(screen, depth)
        self._draw_points(screen, depth)
        self._draw_axes(camera, width, height)
        x = 18
        for _, label, color in self.rod_legend:
            self.create_line(x, 20, x + 16, 20, fill=color, width=3)
            item = self.create_text(x + 22, 20, text=label, fill="#d5e4f0", anchor="w", font=(FONT, 9))
            x = self.bbox(item)[2] + 17
