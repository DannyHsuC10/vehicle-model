"""Interactive orthographic drawing of the original suspension geometry."""

from itertools import product
from math import cos, hypot, pi, sin
import tkinter as tk

import numpy as np

from .model import CHASSIS


BG = "#101d29"
INK = "#d3e2ed"
MUTED = "#92a9bb"
ROD = "#79b9ef"
ACCENT = "#ffc373"
GRID = "#233645"
FONT = "Microsoft JhengHei UI"
VIEWS = {"斜視": (.64, .43), "前視": (pi / 2, 0.), "側視": (0., 0.), "上視": (0., pi / 2)}
ANGLES = np.linspace(0., 2. * pi, 49)


def stress_color(fraction):
    start, end = (121, 185, 239), (255, 61, 65)
    values = (round(a + (b - a) * fraction) for a, b in zip(start, end))
    return "#" + "".join(f"{v:02x}" for v in values)


class Viewport(tk.Canvas):
    def __init__(self, parent, on_pick):
        super().__init__(parent, background=BG, highlightthickness=0, width=600, height=460)
        self.on_pick = on_pick
        self.model = None
        self.results = {}
        self.show_names = False
        self.selected = None
        self.drag = None
        self.zoom = 1.
        self.pan = np.zeros(2)
        self.view = "斜視"
        self.azimuth, self.elevation = VIEWS[self.view]
        self.bind("<Configure>", self.draw)
        for button in (1, 2, 3):
            self.bind(f"<ButtonPress-{button}>", self.press)
            self.bind(f"<B{button}-Motion>", self.move)
            self.bind(f"<ButtonRelease-{button}>", self.release)
        for event in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.bind(event, self.wheel)

    def set_model(self, model, reset=True):
        self.model = model
        self.points = {name: np.array(point) for name, point in model.points.items()}
        limits = list(self.points.values())
        self.tires = []
        for wheel in model.wheels:
            center = self.points[wheel.center]
            contact = self.points[wheel.contact]
            radius = center[2] - contact[2]
            radius = radius if radius > .03 else .196
            heading = np.array([1., 0., 0.])
            if wheel.heading:
                delta = self.points[wheel.heading] - center
                length = hypot(*delta[:2])
                if length > 1e-9:
                    heading[:2] = delta[:2] / length
            lateral = np.array([-heading[1], heading[0], 0.])
            limits.extend((center - radius, center + radius))
            self.tires.append((center, heading, lateral, radius))
        low, high = np.min(limits, axis=0), np.max(limits, axis=0)
        self.center = (low + high) / 2.
        self.bounds = np.array(tuple(product(*zip(low, high))))
        self.span = max(float(max(high - low)), .02)
        z = min((self.points[wheel.contact][2] for wheel in model.wheels), default=low[2])
        self.grid = self.make_grid(low, high, z)
        if reset:
            self.set_view(self.view)
        else:
            self.draw()

    def make_grid(self, low, high, z):
        xy = np.maximum((high - low)[:2] / 2., self.span * .23) * 1.06
        x0, y0 = self.center[:2] - xy
        x1, y1 = self.center[:2] + xy
        lines = [((x, y0, z), (x, y1, z)) for x in np.linspace(x0, x1, 11)]
        lines.extend(((x0, y, z), (x1, y, z)) for y in np.linspace(y0, y1, 11))
        return np.asarray(lines)

    def set_view(self, view):
        self.view = view
        self.azimuth, self.elevation = VIEWS[view]
        self.zoom = 1.
        self.pan[:] = 0.
        self.draw()

    def press(self, event):
        self.drag = (event.x, event.y, event.num, False)

    def move(self, event):
        if self.drag is None:
            return
        x, y, button, moved = self.drag
        dx, dy = event.x - x, event.y - y
        if not moved and abs(dx) + abs(dy) < 4:
            return
        if button == 1:
            self.azimuth += dx * .009
            self.elevation = max(-pi / 2, min(pi / 2, self.elevation + dy * .009))
        else:
            self.pan += (dx, dy)
        self.drag = (event.x, event.y, button, True)
        self.draw()

    def release(self, event):
        if self.drag is not None and not self.drag[3] and event.num == 1:
            items = self.find_overlapping(event.x - 4, event.y - 4, event.x + 4, event.y + 4)
            for item in reversed(items):
                for tag in self.gettags(item):
                    if tag.startswith(("point:", "rod:")):
                        kind, name = tag.split(":", 1)
                        self.on_pick(kind, name)
                        self.drag = None
                        return
        self.drag = None

    def wheel(self, event):
        factor = 1.13 if event.num == 4 or event.delta > 0 else 1. / 1.13
        zoom = max(.25, min(8., self.zoom * factor))
        cursor = np.array([event.x - self.winfo_width() / 2., event.y - self.winfo_height() / 2.])
        self.pan += (cursor - self.pan) * (1. - zoom / self.zoom)
        self.zoom = zoom
        self.draw()

    def draw(self, _event=None):
        self.delete("all")
        if self.model is None:
            return
        width, height = self.winfo_width(), self.winfo_height()
        if width < 100 or height < 120:
            return
        sa, ca = sin(self.azimuth), cos(self.azimuth)
        se, ce = sin(self.elevation), cos(self.elevation)
        basis = np.array([[ca, -sa, 0.], [sa * se, ca * se, -ce]])
        extent = np.maximum(np.ptp(self.bounds @ basis.T, axis=0), .07 * self.span)
        scale = self.zoom * min((width - 72.) / extent[0], (height - 105.) / extent[1])
        offset = np.array([width / 2., height / 2.]) + self.pan

        def project(points):
            return (points - self.center) @ basis.T * scale + offset

        def line(xy, **style):
            self.create_line(*np.asarray(xy).ravel(), **style)

        for segment in project(self.grid):
            line(segment, fill=GRID)
        for center, heading, lateral, radius in self.tires:
            rim = np.outer(np.cos(ANGLES) * radius, heading)
            rim[:, 2] = np.sin(ANGLES) * radius
            left, right = (project(center + rim + side * .09525 * lateral) for side in (-1., 1.))
            for i in range(0, len(ANGLES) - 1, 3):
                surface = np.vstack((left[i:i + 4], right[i:i + 4][::-1]))
                self.create_polygon(*surface.ravel(), fill="#5887a0", stipple="gray12", outline="")
            for ring in (left, right):
                line(ring, fill="#547183", width=1.5)
            for i in range(0, len(ANGLES) - 1, 6):
                line((left[i], right[i]), fill="#38556b")

        pixel = {name: project(point) for name, point in self.points.items()}
        for a, b in self.model.rigid_edges:
            line((pixel[a], pixel[b]), fill="#647e90", width=1, dash=(4, 3))
        maximum = max((abs(result.stress_mpa) for result in self.results.values()), default=0.)
        for rod in self.model.rods:
            selected = self.selected == ("rod", rod.name)
            result = self.results.get(rod.name)
            color = stress_color(abs(result.stress_mpa) / maximum) if result and maximum else ROD
            segment = (pixel[rod.a], pixel[rod.b])
            if selected:
                line(segment, fill=ACCENT, width=7, capstyle="round")
            line(segment, fill=color, width=3, capstyle="round", tags=(f"rod:{rod.name}",))

        for name, (x, y) in pixel.items():
            selected = self.selected == ("point", name)
            body = self.model.owners.get(name)
            color = ACCENT if selected else MUTED if body == CHASSIS else "#72ddc8" if body else "#5e7283"
            marker = self.create_rectangle if body == CHASSIS else self.create_oval
            r = 5 if selected else 3
            marker(x - r, y - r, x + r, y + r, fill=color, outline=BG, tags=(f"point:{name}",))
            if self.show_names or selected:
                self.create_text(x + 7, y - 6, text=name, anchor="sw", fill=color, font=(FONT, 8))

        for axis, direction, color in zip("XYZ", basis.T, ("#ef8d8a", "#79dcb6", "#80bafd")):
            start = np.array([37., height - 42.])
            end = start + 22. * direction
            if np.linalg.norm(end - start) > 2.:
                line((start, end), fill=color, arrow="last", width=2)
                self.create_text(*(end + 8. * direction), text=axis, fill=color, font=(FONT, 8, "bold"))
        self.create_text(18, 17, text="原始硬點幾何", fill=INK, anchor="nw", font=(FONT, 10, "bold"))
        if self.results:
            x, y = width - 140, height - 47
            for i in range(12):
                self.create_rectangle(x + i * 10, y, x + (i + 1) * 10, y + 5, fill=stress_color(i / 11), width=0)
            self.create_text(x, y - 3, text="|σ|  /  MPa", fill=INK, anchor="sw", font=(FONT, 8))
            self.create_text(x, y + 9, text="0", fill=MUTED, anchor="nw", font=(FONT, 8))
            self.create_text(x + 120, y + 9, text=f"{maximum:.3g}", fill=MUTED, anchor="ne", font=(FONT, 8))
