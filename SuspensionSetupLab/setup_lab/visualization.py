"""Canvas playback based on the SuspensionDynamicsLab result views."""

import math
import time
import tkinter as tk
from tkinter import ttk

import numpy as np

from .chart_base import BG, COLORS, Chart
from .project import CHANNELS, CORNERS
from .road_profiles import ISO_ROUGHNESS

FONT = "Microsoft JhengHei UI"
TEXT = "#e2e8f0"
MUTED = "#94a3b8"
ROAD = "#fb923c"
BODY = "#34d399"
RADIUS, WIDTH, BODY_HEIGHT = .22, .20, .85
ANGLES = np.linspace(0., math.tau, 33)
CHANNEL_LABELS = ("前升沉", "後升沉", "前側傾", "後側傾", "同向", "扭擺", *CORNERS)
READOUT_SHORT = {"升沉加速度": "升沉 a", "車架扭角": "扭角"}


class ResponseChart(Chart):
    def __init__(self, parent):
        self.playback_time = None
        super().__init__(parent)
        self.canvas.configure(width=500, height=380)

    def draw(self):
        super().draw()
        self.cursor(self.playback_time)

    def cursor(self, timestamp):
        self.playback_time = timestamp
        self.canvas.delete("playback")
        if timestamp is None or self.view is None:
            return
        lo, hi, _, _ = self.view
        left, top, right, bottom = self.bounds
        x = left + (timestamp - lo) * (right - left) / (hi - lo)
        if left <= x <= right:
            self.canvas.create_line(x, top, x, bottom, fill=TEXT, dash=(3, 3), tags="playback")


class MotionView(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.canvas = tk.Canvas(self, bg=BG, width=335, height=380, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.result = None
        self.index = 0
        self.gain = 1.
        self.canvas.bind("<Configure>", lambda event: self.update_frame(self.index))

    def set_result(self, result):
        self.result, self.index = result, 0
        self.canvas.delete("all")
        self.header = self.canvas.create_text(12, 12, text="計算後顯示車輛動畫", anchor="nw", fill=TEXT, font=(FONT, 10))
        if result is None:
            return
        model, p = result.project["model"], result.project["vehicle"]
        self.projection = {"quarter": "側視", "half_fr": "側視", "half_lr": "前視"}.get(model, "等角視圖")
        self.is_quarter = model == "quarter"
        case = result.project["case"]
        self.road_speed = case["speed"] if case["scenario"] in ISO_ROUGHNESS else 1.
        self.positions = []
        for corner in result.corners:
            front = corner.startswith("F")
            x = p["lf"] if front else -p["lr"]
            y = (p["tf"] if front else p["tr"]) / 2 * (1 if corner.endswith("L") else -1)
            self.positions.append((0. if self.is_quarter or model == "half_lr" else x, y if self.projection != "側視" else 0.))
        self.loads = [result.signals[f"{corner} [N]"] for corner in result.corners]
        self.body = self.canvas.create_line(0, 0, 0, 0, fill=BODY, width=4)
        self.items = []
        for corner in result.corners:
            color = COLORS[("FL", "FR", "RL", "RR").index(corner)]
            ground = self.canvas.create_line(0, 0, 0, 0, fill=ROAD, width=3)
            spring = self.canvas.create_line(0, 0, 0, 0, fill=color, width=2)
            damper = self.canvas.create_line(0, 0, 0, 0, fill=MUTED, width=3)
            wheel = self.canvas.create_line(0, 0, 0, 0, fill=TEXT, width=2)
            far_wheel = self.canvas.create_line(0, 0, 0, 0, fill=MUTED, width=1, state="hidden")
            label = self.canvas.create_text(0, 0, fill=color, font=("Segoe UI", 9), anchor="n")
            self.items.append((ground, spring, damper, wheel, far_wheel, label))
        self.hint = self.canvas.create_text(12, 36, text="綠：車身　彩：懸吊　橘：路面", anchor="nw", fill=MUTED, font=(FONT, 8))
        self.canvas.tag_raise(self.header)
        self.canvas.tag_raise(self.hint)
        self.offset = np.array([BODY_HEIGHT, RADIUS, 0.])
        self.set_gain(self.gain)

    def set_gain(self, gain):
        self.gain = gain
        if self.result is not None:
            motion = self.result.motion
            self.positions_z = self.offset + motion[0] + gain * (motion - motion[0])
            hub = self.positions_z[:, :, 1]
            self.floor = min(float(np.min(self.positions_z)), float(np.min(hub)) - RADIUS)
            self.ceiling = max(float(np.max(self.positions_z)), float(np.max(hub)) + RADIUS)
            self.update_frame(self.index)

    def project(self, x, y, z):
        if self.projection == "側視":
            return x, z
        if self.projection == "前視":
            return -y, z
        # Same oblique projection convention as the full-car view in Dynamics Lab.
        return .765 * x + .644 * y, .878 * z + .309 * x - .367 * y

    def update_frame(self, index):
        self.index = index
        if self.result is None:
            return
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        if width < 100 or height < 120:
            return
        margin = .65 if self.is_quarter else .35
        extents = [self.project(x + dx, y + dy, z) for x, y in self.positions for dx in (-margin, margin)
                   for dy in (-.3, .3) for z in (self.floor, self.ceiling)]
        xs, zs = zip(*extents)
        scale = min((width - 30) / (max(xs) - min(xs)), (height - 100) / (max(zs) - min(zs)))
        ox = width / 2 - scale * (min(xs) + max(xs)) / 2
        oy = (height + 28) / 2 + scale * (min(zs) + max(zs)) / 2

        def pixel(x, y, z):
            px, pz = self.project(x, y, z)
            return ox + scale * px, oy - scale * pz

        def path(item, points):
            self.canvas.coords(item, *(coord for point in points for coord in pixel(*point)))

        frame = self.positions_z[index]
        points = [(x, y, z[0]) for (x, y), z in zip(self.positions, frame)]
        if len(points) == 4:
            points = [points[i] for i in (0, 1, 3, 2, 0)]
        elif self.is_quarter:
            x, y, z = points[0]
            points = [(x - .28, y, z), (x + .28, y, z)]
        path(self.body, points)

        for i, (corner, (x, y), (body, hub, road), items) in enumerate(zip(self.result.corners, self.positions, frame, self.items)):
            ground, spring, damper, wheel, far_wheel, label = items
            ax, ay = pixel(x, y, body)
            bx, by = pixel(x, y, hub)
            coil = [(ax, ay)]
            for j in range(1, 12):
                u = j / 12
                coil.append((ax + (bx - ax) * u + (5 if j % 2 else -5), ay + (by - ay) * u))
            coil.append((bx, by))
            self.canvas.coords(spring, *(coord for point in coil for coord in point))
            self.canvas.coords(damper, ax + 11, ay, bx + 11, by)
            if self.projection == "前視":
                path(wheel, [(x, y + side * WIDTH / 2, hub + level * RADIUS)
                             for side, level in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))])
                path(ground, [(x, y - .24, road), (x, y + .24, road)])
            else:
                def tire(y_pos):
                    return [(x + RADIUS * math.cos(theta), y_pos, hub + RADIUS * math.sin(theta)) for theta in ANGLES]
                if self.projection == "等角視圖":
                    path(far_wheel, tire(y - WIDTH / 2))
                    self.canvas.itemconfigure(far_wheel, state="normal")
                    path(wheel, tire(y + WIDTH / 2))
                else:
                    path(wheel, tire(y))
                if self.is_quarter:
                    offsets = np.linspace(-.65, .65, 65)
                    times = self.result.time[index] + offsets / self.road_speed
                    heights = np.interp(times, self.result.time, self.positions_z[:, i, 2])
                    path(ground, [(x + dx, y, z) for dx, z in zip(offsets, heights)])
                else:
                    path(ground, [(x - .3, y, road), (x + .3, y, road)])
            load = self.loads[i][index]
            self.canvas.itemconfigure(wheel, fill=TEXT if load > 1e-8 else "#fb7185")
            self.canvas.coords(label, bx, pixel(x, y, min(hub - RADIUS, road))[1] + 6)
            self.canvas.itemconfigure(label, text=f"{corner} · {load:.0f} N")
        self.canvas.itemconfigure(self.header, text=f"{self.projection} · 位移 {self.gain:g}× · 示意尺寸")


class Dashboard(ttk.Frame):
    def __init__(self, parent, *, full_vehicle=False):
        super().__init__(parent)
        self.full_vehicle = full_vehicle
        self.canvas = tk.Canvas(self, bg=BG, width=430 if full_vehicle else 335, height=380, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.result = None
        self.index = 0
        self.canvas.bind("<Configure>", lambda event: self.draw())

    def set_result(self, result):
        self.result, self.index = result, 0
        self.draw()

    def draw(self):
        c = self.canvas
        c.delete("all")
        self.bars, self.readouts = [], []
        self.time_label = None
        width, height = c.winfo_width(), c.winfo_height()
        title = "全車受力監控" if self.full_vehicle else "動態儀表板"
        c.create_text(12, 8, text=title, anchor="nw", fill=TEXT, font=(FONT, 10, "bold"))
        if self.result is None:
            c.create_text(12, 36, text="計算後顯示輪荷、懸吊力與姿態", anchor="nw", fill=MUTED, font=(FONT, 9))
            return
        min_width, min_height = (245, 400) if self.full_vehicle else (170, 190)
        if width < min_width or height < min_height:
            c.create_text(width / 2, height / 2, text="拖曳分隔線或放大視窗", fill=MUTED, font=(FONT, 9))
            return
        self.time_label = c.create_text(width - 12, 8, anchor="ne", fill=MUTED, font=("Segoe UI", 9))
        signals = self.result.signals
        loads = [(corner, signals[f"{corner} [N]"]) for corner in self.result.corners]
        channels = [(short, signals.get(f"{label} 模態／輪端反力 [N]"))
                    for short, label in zip(CHANNEL_LABELS, CHANNELS.values())]
        panels = [("法向輪荷 (N)", loads)]
        if self.full_vehicle:
            suspension = [(corner, signals[f"{corner} 輪端懸吊力 [N]"]) for corner in self.result.corners]
            panels.extend((("四輪懸吊通道合力 (N)", suspension),
                           ("模態通道力 (N)", channels[:-len(CORNERS)]),
                           ("各輪直推力 (N)", channels[-len(CORNERS):])))
        else:
            panels.append(("模態與直推力 (N)", [(label, values) for label, values in channels if values is not None]))
        names = [name for name in self.result.groups["車身"] if name.endswith(("[deg]", "[mm]", "[m/s²]"))]
        columns = 3 if width >= 340 else 2
        readout_height = 31 + math.ceil(len(names) / columns) * 17
        gap, top = 6, 32
        row_height = (height - top - readout_height - gap * (len(panels) + 1)) / len(panels)
        for title, series in panels:
            self.draw_bars(top, top + row_height, title, series)
            top += row_height + gap
        c.create_rectangle(7, top, width - 7, height - 6, fill="#162237", outline="#293750")
        c.create_text(14, top + 9, text="目前姿態與升沉", fill=TEXT, anchor="nw", font=(FONT, 8, "bold"))
        for i, name in enumerate(names):
            row, column = divmod(i, columns)
            item = c.create_text(14 + column * (width - 20) / columns, top + 25 + row * 17, fill=TEXT, anchor="nw", font=(FONT, 8))
            label, unit = name.split(" [")
            self.readouts.append((item, READOUT_SHORT.get(label, label), "°" if unit == "deg]" else unit[:-1], signals[name]))
        self.update_frame(self.index)

    def draw_bars(self, top, bottom, title, series):
        c = self.canvas
        right = c.winfo_width() - 7
        c.create_rectangle(7, top, right, bottom, fill="#162237", outline="#293750")
        c.create_text(14, top + 4, text=title, fill=TEXT, anchor="nw", font=(FONT, 8, "bold"))
        if not series:
            return
        present = [values for _, values in series if values is not None]
        high = max((float(np.max(np.abs(values))) for values in present), default=0.)
        high = max(1., high) * 1.1
        low = -high if any(np.min(values) < 0 for values in present) else 0.
        scale = max(1., bottom - top - 47) / (high - low)
        zero = top + 32 + high * scale
        c.create_line(13, zero, right - 6, zero, fill="#64748b")
        slot = (right - 18) / len(series)
        for i, (label, values) in enumerate(series):
            x = 12 + (i + .5) * slot
            color = COLORS[i % len(COLORS)]
            c.create_text(x, bottom - 2, text=label, fill=MUTED if values is None else TEXT, anchor="s", font=(FONT, 7))
            if values is None:
                c.create_text(x, top + 18, text="—", fill=MUTED, anchor="n", font=("Segoe UI", 8))
                continue
            number = c.create_text(x, top + 18, fill=color, anchor="n", font=("Segoe UI", 8))
            bar = c.create_rectangle(0, 0, 0, 0, fill=color, outline="")
            self.bars.append((bar, number, values, x - slot * .29, x + slot * .29, zero, scale))

    def update_frame(self, index):
        self.index = index
        if self.time_label is not None:
            self.canvas.itemconfigure(self.time_label, text=f"{self.result.time[index]:.3f} s")
        for bar, number, values, left, right, zero, scale in self.bars:
            value = values[index]
            end = zero - value * scale
            self.canvas.coords(bar, left, min(zero, end), right, max(zero, end))
            self.canvas.itemconfigure(number, text=f"{value:.0f}")
        for item, label, unit, values in self.readouts:
            self.canvas.itemconfigure(item, text=f"{label} {values[index]:.2f} {unit}")


class Playback(ttk.Frame):
    def __init__(self, parent, draw_frame, set_gain=None):
        super().__init__(parent)
        self.draw_frame = draw_frame
        self.result = self.timer = None
        self.index = 0
        self.seconds = tk.DoubleVar(value=0.)
        self.clock_label = ttk.Label(self, text="0.000 s")
        self.clock_label.pack(side="right", padx=(7, 0))
        self.button = ttk.Button(self, text="播放", width=6, command=self.toggle)
        self.button.pack(side="left")
        self.step_buttons = []
        for step, label in ((-1, "‹"), (1, "›")):
            button = ttk.Button(self, text=label, width=2, command=lambda s=step: self.step(s))
            button.pack(side="left", padx=2)
            self.step_buttons.append(button)
        self.rate = ttk.Combobox(self, state="readonly", width=4, values=("0.25×", "0.5×", "1×", "2×"))
        self.rate.current(2)
        self.rate.pack(side="left", padx=(5, 5))
        self.rate.bind("<<ComboboxSelected>>", lambda event: self.reanchor())
        if set_gain is not None:
            ttk.Label(self, text="位移").pack(side="left")
            gain = ttk.Combobox(self, state="readonly", width=4, values=("1×", "3×", "6×", "12×"))
            gain.current(0)
            gain.pack(side="left", padx=(2, 7))
            gain.bind("<<ComboboxSelected>>", lambda event: set_gain(float(gain.get()[:-1])))
        self.slider = ttk.Scale(self, orient="horizontal", variable=self.seconds, command=self.seek)
        self.slider.pack(side="left", fill="x", expand=True)
        self.set_result(None)

    def set_result(self, result):
        self.pause()
        self.result = result
        self.index = 0
        self.seconds.set(0.)
        self.slider.configure(to=float(result.time[-1]) if result else 1.)
        for widget in (self.button, self.slider, *self.step_buttons):
            widget.state(("!disabled",) if result else ("disabled",))
        self.show(0)

    def show(self, index):
        if self.result is None:
            self.clock_label.configure(text="0.000 s")
            return
        self.index = min(len(self.result.time) - 1, max(0, index))
        seconds = self.result.time[self.index]
        self.seconds.set(seconds)
        self.clock_label.configure(text=f"{seconds:.3f} s")
        self.draw_frame(self.index, seconds)

    def seek(self, seconds):
        if self.result is not None:
            self.pause()
            index = int(np.searchsorted(self.result.time, float(seconds), side="right")) - 1
            self.show(index)

    def step(self, direction):
        self.pause()
        self.show(self.index + direction)

    def reanchor(self):
        if self.result is not None:
            self.started = time.monotonic()
            self.start_seconds = self.result.time[self.index]

    def toggle(self):
        if self.result is None:
            return
        if self.timer is not None:
            self.pause()
            return
        if self.index == len(self.result.time) - 1:
            self.show(0)
        self.reanchor()
        self.button.configure(text="暫停")
        self.tick()

    def tick(self):
        seconds = self.start_seconds + (time.monotonic() - self.started) * float(self.rate.get()[:-1])
        index = int(np.searchsorted(self.result.time, seconds, side="right")) - 1
        self.show(index)
        self.timer = None
        if index < len(self.result.time) - 1:
            self.timer = self.after(33, self.tick)
        else:
            self.button.configure(text="播放")

    def pause(self):
        if self.timer is not None:
            self.after_cancel(self.timer)
            self.timer = None
        self.button.configure(text="播放")
