"""Tk controls for choosing an animation and playing solved frames."""

from bisect import bisect_left, bisect_right
from time import monotonic
import tkinter as tk
from tkinter import ttk

from ._geometry_support.hardpoints import CORNERS
from .animation_settings import AMPLITUDE_LIMITS, DEFAULT_AMPLITUDES, PRESETS, validate_animation


DESCRIPTIONS = {
    "single_wheel": "車身固定，所選輪心抬升後回到靜態高度。",
    "heave": "車身相對地面上下平移，四輪輪心維持靜態高度。",
    "roll": "車身繞通過重心的縱向軸左右側傾，四輪輪心維持靜態高度。",
    "front_steer": "前齒條左右往復，前輪依轉向拉桿的約束轉動。",
}


class AnimationEditor(ttk.Frame):
    def __init__(self, parent, mark_dirty, start):
        super().__init__(parent, padding=10)
        self.preset = tk.StringVar(self)
        self.corner = tk.StringVar(self)
        self.amplitude = tk.StringVar(self)
        self.duration = tk.StringVar(self)
        self.loop = tk.BooleanVar(self)
        self.amplitude_label = tk.StringVar(self)
        self.description = tk.StringVar(self)
        for variable in (self.preset, self.corner, self.amplitude, self.duration, self.loop):
            variable.trace_add("write", mark_dirty)

        self.columnconfigure(1, weight=1)
        ttk.Label(self, text="標準工況").grid(row=0, column=0, sticky="w", padx=(0, 9), pady=4)
        selector = ttk.Combobox(self, textvariable=self.preset, state="readonly", values=tuple(PRESETS.values()))
        selector.grid(row=0, column=1, sticky="ew")
        selector.bind("<<ComboboxSelected>>", self.select_preset)
        ttk.Label(self, textvariable=self.description, wraplength=305).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(3, 11))
        ttk.Label(self, text="抬升輪位").grid(row=2, column=0, sticky="w", pady=4)
        self.corner_picker = ttk.Combobox(self, textvariable=self.corner, state="readonly", values=CORNERS, width=9)
        self.corner_picker.grid(row=2, column=1, sticky="w")
        ttk.Label(self, textvariable=self.amplitude_label).grid(row=3, column=0, sticky="w", padx=(0, 9), pady=4)
        ttk.Entry(self, textvariable=self.amplitude, width=11).grid(row=3, column=1, sticky="w")
        ttk.Label(self, text="週期 (s)").grid(row=4, column=0, sticky="w", pady=4)
        ttk.Entry(self, textvariable=self.duration, width=11).grid(row=4, column=1, sticky="w")
        ttk.Checkbutton(self, text="循環播放", variable=self.loop).grid(row=5, column=0, columnspan=2, sticky="w", pady=6)
        self.start_button = ttk.Button(self, text="解算並播放", command=start)
        self.start_button.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(5, 0))

    def preset_key(self):
        return next(key for key, label in PRESETS.items() if label == self.preset.get())

    def select_preset(self, _event):
        self.amplitude.set(f"{DEFAULT_AMPLITUDES[self.preset_key()]:g}")
        self.refresh_labels()

    def refresh_labels(self):
        key = self.preset_key()
        unit = "°" if key == "roll" else "mm"
        self.amplitude_label.set(f"幅度 ({unit})")
        self.description.set(DESCRIPTIONS[key] + f" 幅度上限 {AMPLITUDE_LIMITS[key]:g} {unit}。")
        self.corner_picker.configure(state="readonly" if key == "single_wheel" else "disabled")

    def populate(self, settings):
        self.preset.set(PRESETS[settings["preset"]])
        self.corner.set(settings["corner"])
        self.amplitude.set(str(settings["amplitude"]))
        self.duration.set(str(settings["duration"]))
        self.loop.set(settings["loop"])
        self.refresh_labels()

    def draft(self):
        try:
            amplitude, duration = float(self.amplitude.get()), float(self.duration.get())
        except ValueError as error:
            raise ValueError("動畫幅度與週期需要有效數值。") from error
        return validate_animation(dict(preset=self.preset_key(), corner=self.corner.get(),
                                       amplitude=amplitude, duration=duration, loop=self.loop.get()))


class AnimationPlayer(ttk.Frame):
    def __init__(self, parent, refresh):
        super().__init__(parent)
        self.refresh = refresh
        self.frames = self.times = ()
        self.index = 0
        self.active = self.playing = self.loop = False
        self.preset_label = ""
        self.timer = None
        self.started = 0.
        self.cursor = tk.DoubleVar(self)
        self.clock = tk.StringVar(self, value="— / — s")
        self.play_button = ttk.Button(self, text="播放", width=6, command=self.toggle, state="disabled")
        self.play_button.pack(side="left")
        self.stop_button = ttk.Button(self, text="停止", width=6, command=self.stop, state="disabled")
        self.stop_button.pack(side="left", padx=5)
        self.timeline = ttk.Scale(self, from_=0, to=1, variable=self.cursor, command=self.scrub, state="disabled")
        self.timeline.pack(side="left", fill="x", expand=True, padx=4)
        ttk.Label(self, textvariable=self.clock, width=14, anchor="e").pack(side="right")

    @property
    def frame(self):
        return self.frames[self.index] if self.active else None

    def load(self, frames, loop, preset_label):
        self.pause()
        self.frames = frames
        self.times = tuple(frame.time for frame in frames)
        self.loop = loop
        self.preset_label = preset_label
        self.timeline.configure(to=len(frames)-1, state="normal")
        self.play_button.configure(state="normal")
        self.stop_button.configure(state="normal")
        self.seek(0)

    def clear(self):
        self.leave()
        self.frames = self.times = ()
        self.cursor.set(0)
        self.clock.set("— / — s")
        for widget in (self.play_button, self.stop_button, self.timeline):
            widget.configure(state="disabled")

    def leave(self):
        self.pause()
        self.active = False

    def seek(self, index):
        self.index = index
        self.active = True
        self.cursor.set(index)
        self.clock.set(f"{self.times[index]:.2f} / {self.times[-1]:.2f} s")
        self.refresh()

    def scrub(self, value):
        if self.frames:
            self.pause()
            self.seek(min(max(round(float(value)), 0), len(self.frames)-1))

    def seek_time(self, time):
        if not self.times:
            return
        index = min(bisect_left(self.times, time), len(self.times)-1)
        if index > 0 and abs(time - self.times[index-1]) <= abs(self.times[index] - time):
            index -= 1
        self.pause()
        self.seek(index)

    def stop(self):
        self.pause()
        self.seek(0)

    def pause(self):
        if self.timer is not None:
            self.after_cancel(self.timer)
            self.timer = None
        self.playing = False
        self.play_button.configure(text="播放")

    def toggle(self):
        if self.playing:
            self.pause()
            return
        if not self.active or self.index == len(self.frames)-1:
            self.seek(0)
        self.started = monotonic() - self.times[self.index]
        self.playing = True
        self.play_button.configure(text="暫停")
        self.tick()

    def tick(self):
        self.timer = None
        elapsed = monotonic() - self.started
        duration = self.times[-1]
        if elapsed >= duration and not self.loop:
            self.seek(len(self.frames)-1)
            self.pause()
            return
        index = bisect_right(self.times, elapsed % duration) - 1
        if index != self.index:
            self.seek(index)
        self.timer = self.after(15, self.tick)
