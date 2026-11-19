"""Time-domain inspection of solved animation samples."""

from dataclasses import dataclass
import tkinter as tk
from tkinter import ttk

import numpy as np

from SuspensionGeometryLab.suspension_geometry_lab.hardpoints import CORNERS

from .elements import PUSH_ROD
from .geometry import LINKS, POINTS


BACKGROUND = "#101d2b"
GRID = "#263b4d"
AXIS = "#688299"
TEXT = "#bdcddd"
CURVE = "#56c6ff"
CURSOR = "#f6b76f"
NODE = "節點"
ROD = "桿件"
HINGE = "A 臂鉸鏈"
DAMPER = "避震器"
SPRING = "彈簧阻尼"
BODY = "車身"
MOUNT = "車身接點力"
ROD_OPTIONS = {f"{a} → {b}": (a, b) for a, b in LINKS}
HINGE_OPTIONS = {"上 A 臂": "UCA", "下 A 臂": "LCA"}
MOUNT_LABELS = {"UCA_F": "上 A 臂前接點", "UCA_R": "上 A 臂後接點",
                "LCA_F": "下 A 臂前接點", "LCA_R": "下 A 臂後接點",
                "RACK": "齒條側接點", "DAMPER": "避震器車身端"}
MOUNT_OPTIONS = {"本輪接點合力": None, **{label: key for key, label in MOUNT_LABELS.items()}}
TARGETS = {NODE: POINTS, ROD: tuple(ROD_OPTIONS), HINGE: tuple(HINGE_OPTIONS), DAMPER: ("直推避震器",),
           BODY: ("重心",), MOUNT: tuple(MOUNT_OPTIONS), SPRING: (PUSH_ROD,)}


@dataclass(frozen=True)
class Signal:
    field: str
    unit: str
    scale: float = 1.
    component: int | str | None = None
    unwrap: bool = False

    def scalar(self, value):
        if value is None:
            return np.nan
        if self.component == "norm":
            value = np.linalg.norm(value)
        elif self.component is not None:
            value = value[self.component]
        return float(value * self.scale)


def vector_signals(label, field, unit, scale=1., *, magnitude=True):
    signals = {f"{label} {axis}": Signal(field, unit, scale, i) for i, axis in enumerate("XYZ")}
    if magnitude:
        signals[f"{label}大小"] = Signal(field, unit, scale, "norm")
    return signals


SIGNALS = {
    NODE: {
        **vector_signals("位置", "position", "mm", 1000., magnitude=False),
        **vector_signals("速度", "velocity", "mm/s", 1000.),
        **vector_signals("加速度", "acceleration", "mm/s²", 1000.),
    },
    ROD: {
        "方位角（連續）": Signal("azimuth_deg", "°", unwrap=True),
        "仰角": Signal("elevation_deg", "°"),
        "相對靜態夾角": Signal("from_static_deg", "°"),
        **vector_signals("角速度", "angular_velocity", "rad/s"),
        **vector_signals("角加速度", "angular_acceleration", "rad/s²"),
    },
    HINGE: {
        "鉸鏈轉角（連續）": Signal("angle_deg", "°", unwrap=True),
        "鉸鏈角速度": Signal("velocity", "rad/s"),
        "鉸鏈角加速度": Signal("acceleration", "rad/s²"),
    },
    DAMPER: {
        "安裝長度": Signal("length", "mm", 1000.),
        "相對靜態伸長": Signal("deflection", "mm", 1000.),
        "傾角（相對車身垂直）": Signal("inclination_deg", "°"),
        "長度速率": Signal("velocity", "mm/s", 1000.),
        "長度加速度": Signal("acceleration", "mm/s²", 1000.),
        "避震器力": Signal("force", "N"),
        "彈簧力": Signal("spring_force", "N"),
        "阻尼力": Signal("damping_force", "N"),
        **vector_signals("車身端力", "force_on_chassis", "N"),
        "輪胎正向力": Signal("tire_force", "N"),
        "路面高度": Signal("road_height", "mm", 1000.),
        "桿軸方位角（連續）": Signal("orientation.azimuth_deg", "°", unwrap=True),
        "桿軸仰角": Signal("orientation.elevation_deg", "°"),
        "桿軸相對靜態夾角": Signal("orientation.from_static_deg", "°"),
        **vector_signals("桿軸角速度", "orientation.angular_velocity", "rad/s"),
        **vector_signals("桿軸角加速度", "orientation.angular_acceleration", "rad/s²"),
    },
    SPRING: {
        "安裝長度": Signal("length", "mm", 1000.),
        "相對靜態伸長": Signal("deflection", "mm", 1000.),
        "長度速率": Signal("velocity", "mm/s", 1000.),
        "軸向力（壓縮為正）": Signal("force", "N"),
    },
    BODY: {
        "Z 向位移": Signal("displacement", "mm", 1000.),
        "Z 向速度": Signal("velocity", "mm/s", 1000.),
        "Z 向加速度": Signal("acceleration", "mm/s²", 1000.),
        "X 向位移": Signal("body.displacement", "mm", 1000., 0),
        "Y 向位移": Signal("body.displacement", "mm", 1000., 1),
        "側傾角 Roll": Signal("body.angles_deg", "°", component=0, unwrap=True),
        "俯仰角 Pitch": Signal("body.angles_deg", "°", component=1, unwrap=True),
        "偏航角 Yaw": Signal("body.angles_deg", "°", component=2, unwrap=True),
        **vector_signals("角速度", "body.angular_velocity", "rad/s"),
        **vector_signals("角加速度", "body.angular_acceleration", "rad/s²"),
        "重心外力 Fz": Signal("force_z", "N"),
        "四輪接點合力 Fz": Signal("mount_force_z", "N"),
    },
    MOUNT: vector_signals("車身受力", "mount_load", "N"),
}


def finite_runs(values):
    """Return contiguous finite index ranges without joining undefined samples."""
    finite = np.isfinite(values)
    edges = np.flatnonzero(np.diff(np.r_[False, finite, False]))
    return zip(edges[::2], edges[1::2])


def signal_values(frames, corner, kind, target, signal, rod_options=None):
    links = ROD_OPTIONS if rod_options is None else rod_options
    values = []
    for frame in frames:
        if kind == BODY:
            if signal.field == "mount_force_z":
                value = (sum(load.total[2] for load in frame.mount_loads.values())
                         if frame.mount_loads else None)
            elif signal.field.startswith("body."):
                value = getattr(frame.body, signal.field.removeprefix("body."), None)
            else:
                value = getattr(frame.heave, signal.field, None)
            values.append(signal.scalar(value))
            continue
        if kind == MOUNT:
            value = None
            if frame.mount_loads:
                item, name = frame.mount_loads[corner], MOUNT_OPTIONS[target]
                value = item.total if name is None else item.by_mount[name]
            values.append(signal.scalar(value))
            continue
        if kind == SPRING:
            element = (frame.element_springs or {}).get(corner, {}).get(target)
            if element is None and target == PUSH_ROD and frame.dampers:
                element = frame.dampers.get(corner)
            values.append(signal.scalar(getattr(element, signal.field, None)))
            continue
        result = frame.result.corners[corner]
        if kind == NODE:
            value = getattr(result, signal.field)[POINTS.index(target)]
        elif kind == ROD:
            value = getattr(result.rod_motion.get(links.get(target)), signal.field, None)
        elif kind == DAMPER:
            element = frame.dampers[corner] if frame.dampers else None
            if element is not None and signal.field.startswith("orientation."):
                value = getattr(element.orientation, signal.field.removeprefix("orientation."))
            else:
                value = getattr(element, signal.field, None)
        else:
            value = getattr(result.arm_motion[HINGE_OPTIONS[target]], signal.field)
        values.append(signal.scalar(value))
    values = np.asarray(values, dtype=float)
    if signal.unwrap:
        for first, last in finite_runs(values):
            values[first:last] = np.rad2deg(np.unwrap(np.deg2rad(values[first:last])))
    return values


def number(value):
    return f"{value:.5g}" if abs(value) >= 5e-10 else "0"


class SignalView(ttk.Frame):
    """Select a solved signal and seek the shared animation timeline."""

    def __init__(self, parent, seek):
        super().__init__(parent, padding=10)
        self.seek = seek
        self.frames = ()
        self.rod_options = dict.fromkeys(CORNERS, ROD_OPTIONS)
        self.spring_options = dict.fromkeys(CORNERS, (PUSH_ROD,))
        self.times = np.array([])
        self.values = np.array([])
        self.time = None
        self.bounds = None
        self.corner = tk.StringVar(self, value="FL")
        self.kind = tk.StringVar(self, value=NODE)
        self.target = tk.StringVar(self, value="WC")
        self.signal_label = tk.StringVar(self, value="位置 Z")
        self.detail = tk.StringVar(self)

        selectors = ttk.Frame(self)
        selectors.pack(fill="x", pady=(0, 8))
        self.corner_box = self.pick(selectors, 0, "輪位", self.corner, CORNERS, 6, self.corner_changed)
        self.pick(selectors, 1, "類別", self.kind, tuple(TARGETS), 10, self.set_kind)
        self.target_box = self.pick(selectors, 2, "對象", self.target, POINTS, 18, self.refresh_signal)
        self.signal_box = self.pick(selectors, 3, "訊號", self.signal_label, tuple(SIGNALS[NODE]), 17,
                                    self.refresh_signal)
        selectors.columnconfigure(2, weight=1)
        selectors.columnconfigure(3, weight=1)

        self.canvas = tk.Canvas(self, background=BACKGROUND, highlightthickness=0, width=760, height=390)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self.draw)
        self.canvas.bind("<Button-1>", self.seek_event)
        self.canvas.bind("<B1-Motion>", self.seek_event)
        ttk.Label(self, textvariable=self.detail, anchor="w").pack(fill="x", pady=(8, 0))
        self.refresh_signal()

    def pick(self, parent, column, label, variable, values, width, command):
        ttk.Label(parent, text=label).grid(row=0, column=column, sticky="w", padx=(0, 8))
        combo = ttk.Combobox(parent, textvariable=variable, values=values, width=width, state="readonly")
        combo.grid(row=1, column=column, sticky="ew", padx=(0, 8))
        combo.bind("<<ComboboxSelected>>", command)
        return combo

    def set_kind(self, _event=None):
        kind = self.kind.get()
        signals = tuple(SIGNALS[kind])
        self.corner_box.configure(state="disabled" if kind == BODY else "readonly")
        self.signal_box.configure(values=signals)
        self.target.set("WC" if kind == NODE else "")
        self.signal_label.set("位置 Z" if kind == NODE else signals[0])
        self.corner_changed()

    def corner_changed(self, _event=None):
        kind, corner = self.kind.get(), self.corner.get()
        if kind == ROD:
            targets = tuple(self.rod_options[corner])
        elif kind == SPRING:
            targets = self.spring_options[corner]
        else:
            targets = TARGETS[kind]
        self.target_box.configure(values=targets, state="readonly" if targets else "disabled")
        if self.target.get() not in targets:
            self.target.set(targets[0] if targets else "")
        self.refresh_signal()

    def set_geometry(self, geometry):
        for corner in geometry.corners:
            if corner.members is None:
                self.rod_options[corner.name] = ROD_OPTIONS
                self.spring_options[corner.name] = (PUSH_ROD,)
            else:
                self.rod_options[corner.name] = {member.name: (member.a, member.b) for member in corner.members}
                self.spring_options[corner.name] = tuple(member.name for member in corner.members
                                                        if member.kind == "spring_damper")
        self.corner_changed()

    def set_frames(self, frames):
        self.frames = tuple(frames)
        self.times = np.array([frame.time for frame in self.frames], dtype=float)
        if not self.frames:
            self.time = None
        self.refresh_signal()

    def set_time(self, time_seconds):
        if time_seconds != self.time:
            self.time = time_seconds
            self.draw_cursor()

    def refresh_signal(self, _event=None):
        kind = self.kind.get()
        self.signal = SIGNALS[kind][self.signal_label.get()]
        corner = self.corner.get()
        self.values = signal_values(self.frames, corner, kind, self.target.get(), self.signal,
                                    self.rod_options[corner])
        self.draw()

    def text(self, x, y, content, *, anchor="center", size=10, color=TEXT, tags=()):
        return self.canvas.create_text(x, y, text=content, fill=color, anchor=anchor,
                                       font=("Microsoft JhengHei UI", size), tags=tags)

    def scope(self):
        kind = self.kind.get()
        if kind == BODY:
            return "地面座標（+Z 向上） · 重心"
        if kind == MOUNT:
            return f"地面座標（+Z 向上） · {self.corner.get()} · {self.target.get()}"
        return f"車身座標 · {self.corner.get()} · {self.target.get()}"

    def draw(self, _event=None):
        canvas = self.canvas
        canvas.delete("all")
        self.bounds = None
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 150 or height < 100:
            return
        if not self.frames:
            self.text(width / 2, height / 2, "請先解算一段動畫", size=13)
            self.detail.set(self.scope())
            return
        finite = self.values[np.isfinite(self.values)]
        if not len(finite):
            self.text(width / 2, height / 2, "選定訊號在此工況未定義", size=13)
            self.detail.set(self.scope())
            return
        start = self.times[0]
        end = max(self.times[-1], start + 1e-9)
        low, high = float(finite.min()), float(finite.max())
        precision = max(abs(low), abs(high), 1.) * 1e-8
        padding = max((high - low) * .09, precision) if high > low else max(abs(low) * .05, .1)
        low, high = low - padding, high + padding
        left, top, right, bottom = 77., 31., width - 27., height - 49.
        self.bounds = (left, top, right, bottom, start, end, low, high)
        for value in np.linspace(low, high, 5):
            y = self.plot_y(value)
            canvas.create_line(left, y, right, y, fill=GRID)
            self.text(left - 8, y, number(value), anchor="e", size=9)
        for time in np.linspace(start, end, 5):
            x = self.plot_x(time)
            canvas.create_line(x, top, x, bottom, fill=GRID)
            self.text(x, bottom + 14, number(time), size=9)
        canvas.create_line(left, top, left, bottom, right, bottom, fill=AXIS)
        label = self.signal_label.get()
        if self.kind.get() == ROD and self.signal.field.startswith("angular_"):
            corner = self.frames[0].result.corners[self.corner.get()]
            link = self.rod_options[self.corner.get()][self.target.get()]
            if corner.rod_motion[link].rotation_model == "axis_only":
                label = f"桿軸{label}"
        self.text(left, 9, f"{label} ({self.signal.unit})", anchor="nw")
        self.text((left + right) / 2, bottom + 32, "時間 (s)")

        for first, last in finite_runs(self.values):
            points = [(self.plot_x(self.times[i]), self.plot_y(self.values[i])) for i in range(first, last)]
            if len(points) > 1:
                canvas.create_line(*points, fill=CURVE, width=2)
            else:
                x, y = points[0]
                canvas.create_oval(x-2, y-2, x+2, y+2, fill=CURVE, outline="")
        self.draw_cursor()

    def plot_x(self, time):
        left, _, right, _, start, end, _, _ = self.bounds
        return left + (time - start) / (end - start) * (right - left)

    def plot_y(self, value):
        _, top, _, bottom, _, _, low, high = self.bounds
        return bottom - (value - low) / (high - low) * (bottom - top)

    def draw_cursor(self):
        self.canvas.delete("cursor")
        scope = self.scope()
        if self.bounds is None or self.time is None:
            self.detail.set(scope)
            return
        index = int(np.searchsorted(self.times, self.time))
        index = min(index, len(self.times) - 1)
        if index and self.time - self.times[index-1] <= self.times[index] - self.time:
            index -= 1
        sample_time, value = self.times[index], self.values[index]
        x = self.plot_x(sample_time)
        _, top, _, bottom, *_ = self.bounds
        self.canvas.create_line(x, top, x, bottom, fill=CURSOR, dash=(4, 3), tags="cursor")
        if np.isfinite(value):
            y = self.plot_y(value)
            self.canvas.create_oval(x-4, y-4, x+4, y+4, fill=CURSOR, outline=BACKGROUND, tags="cursor")
            reading = f"{number(value)} {self.signal.unit}"
        else:
            reading = "未定義"
        self.detail.set(f"{scope}   |   t = {sample_time:.3f} s   ·   {reading}")

    def seek_event(self, event):
        if self.bounds is None:
            return
        left, _, right, _, start, end, _, _ = self.bounds
        fraction = min(max((event.x - left) / (right - left), 0.), 1.)
        self.seek(start + fraction * (end - start))
