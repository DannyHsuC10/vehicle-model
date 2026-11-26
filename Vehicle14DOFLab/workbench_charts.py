"""Native canvas charts for the 14 DOF result viewer."""

import tkinter as tk

import numpy as np

from ._vendor.suspension.engine import G
from ._vendor.suspension.project import CORNERS

from .lap import Track
from .plots import BLUE, GRID, MUTED, ORANGE, SPEED_STOPS, TEAL, TEXT, ticks


WHEEL_COLORS = (BLUE, ORANGE, TEAL, "#805bad")


def wheels(suffix, factor=1.):
    return [(f"{corner}_{suffix}", corner, color, factor) for corner, color in zip(CORNERS, WHEEL_COLORS)]


CHARTS = {
    "速度地圖": None,
    "車速": ("km/h", [("speed_mps", "車速", BLUE, 3.6), ("target_speed_mps", "目標", ORANGE, 3.6)]),
    "加速度": ("g", [("ax_mps2", "縱向", BLUE, 1 / G), ("ay_mps2", "側向", ORANGE, 1 / G)]),
    "輪荷": ("N", wheels("fz_n")),
    "滑移率": ("%", wheels("slip_ratio", 100.)),
    "側滑角": ("°", wheels("slip_angle_deg")),
    "外傾角": ("°", wheels("camber_deg")),
    "Bump Steer": ("°", wheels("bump_steer_deg")),
    "Roll Center 傳力": ("N", wheels("roll_center_force_n")),
    "Anti 傳力": ("N", wheels("anti_force_n")),
    "懸吊行程": ("mm", wheels("wheel_travel_mm")),
    "輪速": ("rpm", wheels("wheel_speed_rad_s", 60. / (2. * np.pi))),
    "滾動半徑": ("mm", wheels("rolling_radius_m", 1000.)),
    "輪端驅動": ("N·m", wheels("drive_nm")),
    "輪端煞車": ("N·m", wheels("brake_nm")),
    "SAE 滾阻力矩": ("N·m", wheels("rolling_resistance_nm")),
    "空力": ("N", [("downforce_n", "下壓力", TEAL, 1.), ("drag_n", "阻力", ORANGE, 1.)]),
    "空力作用點": ("m", [("cop_x_m", "X 前後", BLUE, 1.), ("cop_y_m", "Y 左右", ORANGE, 1.),
                       ("cop_z_m", "Z 離地高度", TEAL, 1.)]),
}


class Chart(tk.Canvas):
    def __init__(self, master):
        super().__init__(master, background="white", highlightthickness=0, height=380)
        self.result = None
        self.centerline = None
        self.mode = "速度地圖"
        self.hover = None
        self.pending = None
        self.bind("<Configure>", self.schedule_redraw)
        self.bind("<Motion>", self.inspect)
        self.bind("<Leave>", lambda event: self.delete("cursor"))

    def set_result(self, result):
        self.result = result
        self.centerline = None
        summary = result.timing[0] if result.timing else {}
        course = result.source("track_csv")
        if course and course.is_file():
            try:
                xy = Track.from_csv(course, width=float(summary.get("track_width_m") or 3.)).xy
                self.centerline = np.vstack((xy, xy[0]))
            except (OSError, ValueError):
                pass
        elif summary.get("distance_m"):
            try:
                self.centerline = np.array([[0., 0.], [float(summary["distance_m"]), 0.]])
            except ValueError:
                pass
        self.schedule_redraw()

    def select(self, mode):
        self.mode = mode
        self.schedule_redraw()

    def schedule_redraw(self, _event=None):
        if self.pending is None:
            self.pending = self.after_idle(self.redraw)

    def redraw(self):
        self.pending = None
        self.delete("all")
        self.hover = None
        width, height = self.winfo_width(), self.winfo_height()
        if width < 200 or height < 180:
            return
        if self.result is None:
            self.create_text(width / 2, height / 2, text="執行模擬或開啟遙測 CSV", fill=MUTED)
        elif self.mode == "速度地圖":
            self.draw_map(width, height)
        else:
            self.draw_time(width, height)

    def axes(self, box, limits, labels):
        left, top, right, bottom = box
        low_x, high_x, low_y, high_y = limits

        def tx(x):
            return left + (x - low_x) * (right - left) / (high_x - low_x)

        def ty(y):
            return bottom - (y - low_y) * (bottom - top) / (high_y - low_y)

        for value in ticks(low_x, high_x):
            x = tx(value)
            self.create_line(x, top, x, bottom, fill=GRID)
            self.create_text(x, bottom + 12, text=f"{value:g}", fill=MUTED)
        for value in ticks(low_y, high_y, 5):
            y = ty(value)
            self.create_line(left, y, right, y, fill=GRID)
            self.create_text(left - 7, y, text=f"{value:g}", fill=MUTED, anchor="e")
        self.create_line(left, top, left, bottom, right, bottom, fill=MUTED)
        self.create_text((left + right) / 2, bottom + 31, text=labels[0], fill=MUTED)
        self.create_text(left, top - 16, text=labels[1], anchor="w", fill=MUTED)
        return tx, ty

    def draw_time(self, width, height):
        data = self.result.data
        unit, specs = CHARTS[self.mode]
        series = [(label, color, data[key] * factor) for key, label, color, factor in specs
                  if key in data.dtype.names and np.isfinite(data[key]).any()]
        if not series:
            self.create_text(width / 2, height / 2, text="此筆結果沒有這個訊號", fill=MUTED)
            return
        time = data["time_s"]
        low = min(0., *(float(np.nanmin(y)) for _, _, y in series))
        high = max(0., *(float(np.nanmax(y)) for _, _, y in series))
        pad = max(.01, (high - low) * .07)
        box = (58, 64, width - 22, height - 48)
        tx, ty = self.axes(box, (time[0], time[-1], low - pad, high + pad), ("模擬時間 (s)", unit))
        self.create_line(box[0], ty(0.), box[2], ty(0.), fill=MUTED)
        for i, (label, color, values) in enumerate(series):
            legend = 65 + i * min(120, (width - 90) / len(series))
            self.create_line(legend, 19, legend + 17, 19, fill=color, width=3)
            self.create_text(legend + 22, 19, text=label, fill=TEXT, anchor="w")
            valid = np.flatnonzero(np.isfinite(values))
            shown = valid[np.linspace(0, len(valid) - 1, min(len(valid), 2200), dtype=int)]
            if len(shown) > 1:
                self.create_line(*np.column_stack((tx(time[shown]), ty(values[shown]))).ravel(), fill=color, width=2)
        self.hover = (box, tx, ty, time, series)

    def draw_map(self, width, height):
        data = self.result.data
        box = (56, 54, width - 75, height - 47)
        left, top, right, bottom = box
        position = np.column_stack((data["x_m"], data["y_m"]))
        reference = position if self.centerline is None else np.vstack((position, self.centerline))
        lo, hi = reference.min(axis=0), reference.max(axis=0)
        midpoint = (hi + lo) / 2
        dimensions = np.array([right - left, bottom - top])
        scale = max(.001, 1.12 * max((hi - lo) / dimensions))
        lo, hi = midpoint - .5 * scale * dimensions, midpoint + .5 * scale * dimensions
        tx, ty = self.axes(box, (lo[0], hi[0], lo[1], hi[1]), ("X (m)", "Y (m)"))
        if self.centerline is not None:
            self.create_line(*np.column_stack((tx(self.centerline[:, 0]), ty(self.centerline[:, 1]))).ravel(),
                             fill="#a6b2be", dash=(4, 4), width=2)
        speed = 3.6 * data["speed_mps"]
        low, high = float(speed.min()), float(speed.max())
        shown = np.linspace(0, len(speed) - 1, min(len(speed), 1600), dtype=int)
        xy = np.column_stack((tx(position[shown, 0]), ty(position[shown, 1])))
        fractions = (speed[shown] - low) / max(.001, high - low)
        bins = np.minimum(31, (16 * (fractions[1:] + fractions[:-1])).astype(int))

        def color(fraction):
            rgb = [round(np.interp(fraction, (0., .5, 1.), component)) for component in zip(*SPEED_STOPS)]
            return "#{:02x}{:02x}{:02x}".format(*rgb)

        edges = np.r_[0, np.flatnonzero(np.diff(bins)) + 1, len(bins)]
        for start, end in zip(edges[:-1], edges[1:]):
            self.create_line(*xy[start:end + 1].ravel(), fill=color((bins[start] + .5) / 32), width=3, capstyle="round")
        for i in range(32):
            y = bottom - i * (bottom - top) / 32
            self.create_rectangle(right + 11, y - (bottom - top) / 32, right + 21, y,
                                  outline="", fill=color((i + .5) / 32))
        self.create_text(right + 9, top - 16, text="km/h", fill=MUTED, anchor="w")
        self.create_text(right + 25, top, text=f"{high:.1f}", fill=MUTED, anchor="w")
        self.create_text(right + 25, bottom, text=f"{low:.1f}", fill=MUTED, anchor="w")

    def inspect(self, event):
        self.delete("cursor")
        if self.hover is None:
            return
        box, tx, ty, time, series = self.hover
        left, top, right, bottom = box
        if not (left <= event.x <= right and top <= event.y <= bottom):
            return
        target = time[0] + (event.x - left) / (right - left) * (time[-1] - time[0])
        index = min(len(time) - 1, int(np.searchsorted(time, target)))
        x = tx(time[index])
        self.create_line(x, top, x, bottom, dash=(3, 4), fill=MUTED, tags="cursor")
        label = [f"{time[index]:.2f} s"]
        for name, color, y in series:
            value = y[index]
            if np.isfinite(value):
                at = ty(value)
                self.create_oval(x - 3, at - 3, x + 3, at + 3, fill=color, outline="white", tags="cursor")
                label.append(f"{name} {value:.2f}")
        self.create_text(left, bottom + 44, anchor="sw", text="  |  ".join(label), fill=TEXT, tags="cursor")
