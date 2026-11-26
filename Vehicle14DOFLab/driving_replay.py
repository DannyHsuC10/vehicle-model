"""Synchronized driving replay and instrumentation for the workbench."""

import math
from time import monotonic
import tkinter as tk
from tkinter import ttk

import numpy as np

from ._vendor.suspension.engine import G
from ._vendor.suspension.project import CORNERS

from .driving_scene import ReplayGeometry, draw_scene, value


INK = "#142b41"
MUTED = "#506579"
GRID = "#d7e2e9"
COLORS = ("#245e9b", "#376c94", "#197b82", "#57a5a0")


class DrivingReplay(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent, padding=7)
        self.result = self.geometry = None
        self.headings = None
        self.position = tk.DoubleVar(value=0.)
        self.frame = 0
        self.displayed = -1
        self.timer = None
        self.running = False
        self.anchor_time = self.anchor_position = 0.
        self.rate = tk.StringVar(value="1×")
        self.camera = tk.StringVar(value="跟車")
        self.time_text = tk.StringVar(value="0.00 / 0.00 s")
        self.speed_text = tk.StringVar(value="— km/h")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        controls = ttk.Frame(self)
        controls.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        self.play = ttk.Button(controls, text="播放", command=self.toggle, width=6, state="disabled")
        self.play.pack(side="left")
        self.restart = ttk.Button(controls, text="回起點", command=lambda: self.seek(0.), state="disabled")
        self.restart.pack(side="left", padx=(4, 7))
        speed = ttk.Combobox(controls, textvariable=self.rate, values=("0.25×", "0.5×", "1×", "2×", "4×"),
                             width=4, state="readonly")
        speed.pack(side="left")
        speed.bind("<<ComboboxSelected>>", self.change_rate)
        camera = ttk.Combobox(controls, textvariable=self.camera, values=("跟車", "俯視"), width=5, state="readonly")
        camera.pack(side="left", padx=(6, 0))
        camera.bind("<<ComboboxSelected>>", lambda event: self.render())
        ttk.Label(controls, textvariable=self.speed_text, font=("Segoe UI", 13, "bold")).pack(side="right")

        content = ttk.Frame(self)
        content.grid(row=1, column=0, sticky="nsew")
        content.columnconfigure(0, weight=1)
        content.rowconfigure(0, weight=1)
        self.scene = tk.Canvas(content, background="#dae6e4", highlightthickness=0, width=240, height=235)
        self.scene.grid(row=0, column=0, sticky="nsew", padx=(0, 7))
        self.pedals = tk.Canvas(content, background="white", highlightthickness=0, height=69)
        self.pedals.grid(row=1, column=0, sticky="ew", padx=(0, 7), pady=(6, 0))
        instruments = ttk.Frame(content)
        instruments.grid(row=0, column=1, rowspan=2, sticky="nsew")
        instruments.columnconfigure(0, weight=1)
        instruments.rowconfigure((0, 1), weight=1, uniform="instrument")
        self.accel = tk.Canvas(instruments, background="white", highlightthickness=0, width=181, height=150)
        self.accel.grid(row=0, column=0, sticky="nsew", pady=(0, 5))
        self.loads = tk.Canvas(instruments, background="white", highlightthickness=0, width=181, height=150)
        self.loads.grid(row=1, column=0, sticky="nsew")
        for canvas in (self.scene, self.pedals, self.accel, self.loads):
            canvas.bind("<Configure>", self.resize)
        timeline = ttk.Frame(self)
        timeline.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        timeline.columnconfigure(0, weight=1)
        self.slider = ttk.Scale(timeline, variable=self.position, from_=0, to=1, command=self.seek, state="disabled")
        self.slider.grid(row=0, column=0, sticky="ew")
        ttk.Label(timeline, textvariable=self.time_text, width=18, anchor="e").grid(row=0, column=1, padx=(6, 0))
        self.bind("<Unmap>", lambda event: self.pause())
        self.resize_job = None

    def resize(self, _event=None):
        if self.resize_job is None:
            self.resize_job = self.after_idle(self.redraw)

    def redraw(self):
        self.resize_job = None
        self.render()

    def set_result(self, result):
        self.pause()
        self.result = result
        self.geometry = ReplayGeometry.from_result(result)
        data = result.data
        self.headings = np.arctan2(np.gradient(data["y_m"]), np.gradient(data["x_m"]))
        names = data.dtype.names
        ax = np.nan_to_num(data["ax_mps2"], nan=0., posinf=0., neginf=0.) if "ax_mps2" in names else 0.
        self.g_limit = max(2., math.ceil(2. * np.max(np.hypot(ax, data["ay_mps2"])) / G) / 2.)
        finite_peaks = [np.max(data[field][np.isfinite(data[field])], initial=0.)
                        for corner in CORNERS if (field := f"{corner}_fz_n") in names]
        self.load_limit = max(1000., 250. * math.ceil(max(finite_peaks, default=0.) / 250.))
        self.play.configure(state="normal")
        self.restart.configure(state="normal")
        self.slider.configure(from_=float(data["time_s"][0]), to=float(data["time_s"][-1]), state="normal")
        self.seek(float(data["time_s"][0]))

    def seek(self, seconds):
        if self.result is None:
            return
        time = self.result.data["time_s"]
        position = float(np.clip(float(seconds), time[0], time[-1]))
        self.position.set(position)
        self.frame = int(np.searchsorted(time, position, side="right") - 1)
        self.anchor_position, self.anchor_time = position, monotonic()
        self.render()

    def change_rate(self, _event=None):
        self.anchor_position, self.anchor_time = self.position.get(), monotonic()

    def toggle(self):
        if self.running:
            self.pause()
        elif self.result is not None:
            time = self.result.data["time_s"]
            if self.position.get() >= time[-1]:
                self.seek(float(time[0]))
            self.running = True
            self.play.configure(text="暫停")
            self.change_rate()
            self.tick()

    def pause(self):
        self.running = False
        if self.timer is not None:
            self.after_cancel(self.timer)
            self.timer = None
        self.play.configure(text="播放")

    def tick(self):
        self.timer = None
        if not self.running or self.result is None:
            return
        started = monotonic()
        time = self.result.data["time_s"]
        seconds = self.anchor_position + (started - self.anchor_time) * float(self.rate.get().rstrip("×"))
        seconds = min(seconds, float(time[-1]))
        self.position.set(seconds)
        self.frame = int(np.searchsorted(time, seconds, side="right") - 1)
        if self.frame != self.displayed:
            self.render()
        if seconds >= time[-1]:
            self.pause()
        else:
            delay_ms = max(1, round(1000. * (1. / 30. - (monotonic() - started))))
            self.timer = self.after(delay_ms, self.tick)

    def render(self):
        if self.result is None:
            self.scene.delete("all")
            self.scene.create_text(self.scene.winfo_width() / 2, self.scene.winfo_height() / 2,
                                   text="開啟遙測 CSV 開始回放", fill=MUTED)
            return
        data = self.result.data
        row = data[self.frame]
        self.displayed = self.frame
        self.time_text.set(f"{row['time_s']:.2f} / {data['time_s'][-1]:.2f} s")
        self.speed_text.set(f"{3.6 * row['speed_mps']:.1f} km/h")
        draw_scene(self.scene, self.geometry, row, heading=self.headings[self.frame], overhead=self.camera.get() == "俯視")
        self.draw_pedals(row)
        self.draw_acceleration(row)
        self.draw_loads(row)

    def draw_pedals(self, row):
        canvas = self.pedals
        width = canvas.winfo_width()
        canvas.delete("all")
        for y, title, key, color in ((20, "油門", "throttle", "#197b82"), (49, "煞車", "brake", "#b95031")):
            reading = value(row, key, np.nan)
            canvas.create_text(9, y, text=title, anchor="w", fill=INK)
            canvas.create_rectangle(49, y - 5, width - 54, y + 5, fill="#e4ecf0", outline="")
            if np.isfinite(reading):
                right = 49 + max(0., width - 103.) * float(np.clip(reading, 0., 1.))
                canvas.create_rectangle(49, y - 5, right, y + 5, fill=color, outline="", tags=key)
            canvas.create_text(width - 8, y, text=f"{reading:.0%}" if np.isfinite(reading) else "—", anchor="e", fill=INK)

    def draw_acceleration(self, row):
        canvas = self.accel
        width, height = canvas.winfo_width(), canvas.winfo_height()
        canvas.delete("all")
        canvas.create_text(10, 12, text="加速度圓 (g)", anchor="w", fill=INK, font=("Microsoft JhengHei UI", 10, "bold"))
        x, y = width / 2., (height + 1.) / 2.
        radius = max(2., min(x - 24., y - 31.))
        canvas.create_line(x - radius, y, x + radius, y, fill=GRID)
        canvas.create_line(x, y - radius, x, y + radius, fill=GRID)
        for fraction in (.5, 1.):
            rr = fraction * radius
            canvas.create_oval(x - rr, y - rr, x + rr, y + rr, outline=GRID)
        canvas.create_text(x, y - radius - 9, text="+ax", fill=MUTED, font=("Segoe UI", 8))
        canvas.create_text(x - radius - 11, y, text="+ay", fill=MUTED, font=("Segoe UI", 8))
        canvas.create_text(width - 7, 12, text=f"±{self.g_limit:g}", anchor="e", fill=MUTED)
        ax, ay = value(row, "ax_mps2", np.nan) / G, value(row, "ay_mps2") / G
        if np.isfinite(ax):
            px, py = x - ay * radius / self.g_limit, y - ax * radius / self.g_limit
            canvas.create_line(x, y, px, py, fill="#5b979a", tags="g_vector")
            canvas.create_oval(px - 4, py - 4, px + 4, py + 4, fill="#197b82", outline="white", tags="g_point")
        x_label = f"{ax:+.2f}" if np.isfinite(ax) else "—"
        canvas.create_text(x, height - 13, text=f"ax {x_label}   ay {ay:+.2f}", fill=INK, tags="g_reading")

    def draw_loads(self, row):
        canvas = self.loads
        width, height = canvas.winfo_width(), canvas.winfo_height()
        canvas.delete("all")
        canvas.create_text(10, 12, text="四輪輪荷 (N)", anchor="w", fill=INK, font=("Microsoft JhengHei UI", 10, "bold"))
        canvas.create_text(width - 8, 12, text=f"{self.load_limit:.0f}", anchor="e", fill=MUTED)
        top, bottom = 36., height - 24.
        canvas.create_line(8, top, width - 8, top, fill=GRID, dash=(3, 3))
        canvas.create_line(8, bottom, width - 8, bottom, fill=GRID)
        span = (width - 16.) / 4.
        for i, (corner, color) in enumerate(zip(CORNERS, COLORS)):
            x = 8. + (i + .5) * span
            reading = value(row, f"{corner}_fz_n", np.nan)
            y = bottom
            if np.isfinite(reading):
                y -= (bottom - top) * float(np.clip(reading / self.load_limit, 0., 1.))
                canvas.create_rectangle(x - span * .28, y, x + span * .28, bottom,
                                        fill=color, outline="", tags=f"load_{corner}")
            canvas.create_text(x, y - 8, text=f"{reading:.0f}" if np.isfinite(reading) else "—",
                               fill=INK, font=("Segoe UI", 8), tags=f"value_{corner}")
            canvas.create_text(x, bottom + 12, text=corner, fill=MUTED, font=("Segoe UI", 9))
