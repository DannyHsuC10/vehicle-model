"""Interactive Tk Canvas preview of the static suspension's real wire geometry.

Run from the SuspensionGeometryLab folder with ``python -m suspension_geometry_lab.static.fv.tk_wireframe``.
Only NumPy and Python's bundled Tkinter are required; no Matplotlib is loaded.
"""

import itertools
import math
import tkinter as tk
from tkinter import ttk

import numpy as np

from .wireframe_geometry import build_wireframe_scene


def project_points(points, azimuth, elevation):
    """Orthographically project vehicle XYZ onto camera right/up (angles in radians)."""
    ca, sa = math.cos(azimuth), math.sin(azimuth)
    ce, se = math.cos(elevation), math.sin(elevation)
    basis = np.array([[-sa, -se * ca], [ca, -se * sa], [0.0, ce]])
    return np.asarray(points, dtype=float) @ basis


class WireframeViewer(ttk.Frame):
    """Keep geometry fixed while rotating, panning and zooming its Canvas lines."""

    def __init__(self, master, scene=None):
        super().__init__(master)
        self.scene = build_wireframe_scene() if scene is None else scene
        self.azimuth = math.radians(self.scene.azimuth_deg)
        self.elevation = math.radians(self.scene.elevation_deg)
        self.center = np.zeros(3)
        self.pan = np.zeros(2)
        self.scale = 1.0
        self.fit_mode = "full"
        self._pending_draw = None
        self._drag = None
        self._last_size = None
        self._lengths = [len(line.points) for line in self.scene.lines]
        self._points = np.concatenate([line.points for line in self.scene.lines])
        self._ends = np.cumsum(self._lengths)
        self._starts = np.r_[0, self._ends[:-1]]

        toolbar = ttk.Frame(self, padding=(10, 8))
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="視角").pack(side="left", padx=(0, 6))
        for name, label in (("iso", "3D"), ("front", "前視"),
                            ("side", "側視"), ("top", "上視")):
            ttk.Button(toolbar, text=label, width=6,
                       command=lambda view=name: self.set_view(view)).pack(side="left", padx=2)
        ttk.Separator(toolbar, orient="vertical").pack(side="left", fill="y", padx=10)
        ttk.Button(toolbar, text="完整構造", command=lambda: self.fit("full")).pack(side="left", padx=2)
        ttk.Button(toolbar, text="車體特寫", command=lambda: self.fit("vehicle")).pack(side="left", padx=2)

        self.groups = {
            "front": tk.BooleanVar(self, True),
            "rear": tk.BooleanVar(self, False),
            "sv": tk.BooleanVar(self, False),
        }
        for group, label in (("front", "前 FV"), ("rear", "後 FV"), ("sv", "SV")):
            ttk.Checkbutton(toolbar, text=label, variable=self.groups[group],
                            command=self.request_draw).pack(side="left", padx=(10, 0))

        self.canvas = tk.Canvas(self, width=1200, height=760, background="white",
                                highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.items = [
            self.canvas.create_line(0, 0, 1, 1, fill=line.color, width=line.width,
                                    dash=line.dash, capstyle="round", joinstyle="round")
            for line in self.scene.lines
        ]
        ttk.Label(self, padding=(10, 7),
                  text="左鍵拖曳：旋轉　｜　右鍵／中鍵／Shift＋左鍵：平移　｜　滾輪：縮放　｜　雙擊：重新置中"
                       "　　前輪藍色・後輪橘色・SV 綠色").pack(fill="x")
        self.canvas.bind("<Configure>", self._resize)
        self.canvas.bind("<ButtonPress-1>", self._begin_rotate)
        self.canvas.bind("<B1-Motion>", self._move)
        for button in (2, 3):
            self.canvas.bind(f"<ButtonPress-{button}>", self._begin_pan)
            self.canvas.bind(f"<B{button}-Motion>", self._move)
        for button in (1, 2, 3):
            self.canvas.bind(f"<ButtonRelease-{button}>", self._end_drag)
        self.canvas.bind("<MouseWheel>", self._wheel)
        self.canvas.bind("<Button-4>", lambda event: self.zoom_at(1.15, event.x, event.y))
        self.canvas.bind("<Button-5>", lambda event: self.zoom_at(1 / 1.15, event.x, event.y))
        self.canvas.bind("<Double-Button-1>", lambda event: self.fit())
        self.bind("<Destroy>", self._destroy)
        self.fit()

    def _size(self):
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        return (width if width > 1 else int(self.canvas["width"]),
                height if height > 1 else int(self.canvas["height"]))

    def fit(self, mode=None):
        """Fit exact construction bounds, or magnify the vehicle region."""
        if mode is not None:
            self.fit_mode = mode
        bounds = self.scene.bounds if self.fit_mode == "full" else self.scene.vehicle_bounds
        self.center = np.asarray(bounds).mean(axis=1)
        corners = np.array(list(itertools.product(*bounds)))
        projected = project_points(corners - self.center, self.azimuth, self.elevation)
        span = np.maximum(np.ptp(projected, axis=0), 1e-6)
        width, height = self._size()
        self.scale = min(max(width - 70, 1) / span[0], max(height - 70, 1) / span[1])
        self.pan[:] = 0.0
        self.request_draw()

    def set_view(self, name):
        views = {"iso": (self.scene.azimuth_deg, self.scene.elevation_deg),
                 "front": (0.0, 0.0), "side": (90.0, 0.0), "top": (-90.0, 90.0)}
        azimuth, elevation = views[name]
        self.azimuth, self.elevation = math.radians(azimuth), math.radians(elevation)
        self.fit()

    def request_draw(self):
        """Coalesce pending mouse events into one update of existing Canvas items."""
        if self._pending_draw is None:
            self._pending_draw = self.after_idle(self._flush_draw)

    def _flush_draw(self):
        self._pending_draw = None
        self.render()

    def render(self):
        """Only project coordinates and update polylines; never solve suspension here."""
        width, height = self._size()
        projected = project_points(self._points - self.center, self.azimuth, self.elevation)
        projected *= [self.scale, -self.scale]
        projected += [width / 2 + self.pan[0], height / 2 + self.pan[1]]
        visible = {group: value.get() for group, value in self.groups.items()}
        for line, item, start, end in zip(self.scene.lines, self.items, self._starts, self._ends):
            if visible.get(line.group, True):
                self.canvas.coords(item, *projected[start:end].ravel().tolist())
                self.canvas.itemconfigure(item, state="normal")
            else:
                self.canvas.itemconfigure(item, state="hidden")

    def _resize(self, event):
        size = (event.width, event.height)
        if self._last_size is None:
            self.fit()
        else:
            old_width, old_height = self._last_size
            self.scale *= max(min(size), 1) / max(min(old_width, old_height), 1)
        self._last_size = size
        self.request_draw()

    def _begin_rotate(self, event):
        self._drag = (event.x, event.y, "pan" if event.state & 1 else "rotate")

    def _begin_pan(self, event):
        self._drag = (event.x, event.y, "pan")

    def _move(self, event):
        if self._drag is None:
            return
        x, y, mode = self._drag
        dx, dy = event.x - x, event.y - y
        self._drag = (event.x, event.y, mode)
        if mode == "pan":
            self.pan += [dx, dy]
        else:
            self.azimuth -= dx * 0.008
            self.elevation = float(np.clip(self.elevation + dy * 0.008,
                                           -math.pi / 2 + 0.001, math.pi / 2 - 0.001))
        self.request_draw()

    def _end_drag(self, _event):
        self._drag = None

    def _wheel(self, event):
        if event.delta:
            self.zoom_at(1.15 ** (event.delta / 120.0), event.x, event.y)

    def zoom_at(self, factor, x, y):
        """Keep the screen position under the pointer fixed while zooming."""
        next_scale = float(np.clip(self.scale * factor, 1e-3, 1e6))
        factor = next_scale / self.scale
        width, height = self._size()
        anchor = np.array([x - width / 2, y - height / 2])
        self.pan = anchor - (anchor - self.pan) * factor
        self.scale = next_scale
        self.request_draw()

    def _destroy(self, event):
        if event.widget is self and self._pending_draw is not None:
            self.after_cancel(self._pending_draw)
            self._pending_draw = None


def main():
    root = tk.Tk()
    root.title("SuspensionGeometryLab · 靜態線框預覽")
    root.geometry("1280x880")
    root.minsize(960, 600)
    viewer = WireframeViewer(root)
    viewer.pack(fill="both", expand=True)
    root.mainloop()


if __name__ == "__main__":
    main()
