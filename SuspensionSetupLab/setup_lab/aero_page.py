"""Edit and preview the constant-coefficient downforce curve."""

import tkinter as tk
from tkinter import ttk

import numpy as np

from .chart_base import COLORS, Chart
from .project import AERO_FIELDS, CASE_FIELDS, MODELS, downforce, validate_aero
from .widgets import entry


class AeroPage(ttk.Frame):
    def __init__(self, app):
        super().__init__(app.pages, padding=14)
        self.app = app
        settings = ttk.LabelFrame(self, text="全車下壓力參考點", padding=12)
        settings.pack(fill="x")
        self.enabled = tk.BooleanVar()
        ttk.Checkbutton(settings, text="啟用空力", variable=self.enabled, command=self.draw).grid(
            row=0, column=0, columnspan=4, sticky="w")
        self.fields = {}
        for index, (key, (label, _, _)) in enumerate(AERO_FIELDS.items()):
            variable = self.fields[key] = tk.StringVar()
            entry(settings, 1, index * 2, label, variable)
            variable.trace_add("write", self.draw)
        ttk.Label(settings, text="下壓力填正值。F(v) = 參考下壓力 ×（車速 ÷ 參考車速）²").grid(
            row=2, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self.readout = ttk.Label(self, padding=(0, 8))
        self.readout.pack(anchor="w")
        self.chart = Chart(self)
        self.chart.pack(fill="both", expand=True)

    def set_settings(self, settings):
        self.enabled.set(settings["enabled"])
        for key, variable in self.fields.items():
            variable.set(settings[key])

    def settings(self):
        try:
            settings = {"enabled": self.enabled.get(), **{key: float(var.get()) for key, var in self.fields.items()}}
        except ValueError as exc:
            raise ValueError("空力參考車速與下壓力須填入數值。") from exc
        validate_aero(settings)
        return settings

    def draw(self, *_):
        try:
            settings = self.settings()
        except ValueError as exc:
            self.chart.set(None)
            self.readout.configure(text=str(exc))
            return
        reference_speed = settings["reference_speed_kmh"]
        speed_kmh = 0.
        operating_point = None
        status = "空力已啟用" if settings["enabled"] else "空力已停用"
        page = self.app.analysis
        if page is not None:
            try:
                speed_mps = float(page.case_vars["speed"].get())
            except ValueError:
                speed_mps = np.nan
            _, low, high = CASE_FIELDS["speed"]
            if low <= speed_mps <= high:
                speed_kmh = speed_mps * 3.6
                force = downforce(settings, speed_kmh)
                operating_point = {"name": "工況車速", "x": [speed_kmh], "y": [force], "marker": True,
                                   "color": COLORS[2]}
                status = (f"{MODELS[page.model_key]}　｜　工況 {speed_kmh:g} km/h　｜　"
                          f"曲線下壓力 {force:,.1f} N　｜　{status}")
            else:
                status += "　｜　請在分析頁設定有效的工況車速"
        samples = np.linspace(0., max(reference_speed, speed_kmh) * 1.25, 121)
        speeds = np.unique(np.r_[samples, reference_speed, speed_kmh])
        series = [{"name": "下壓力曲線", "x": speeds.tolist(), "y": downforce(settings, speeds).tolist()},
                  {"name": "參考點", "x": [reference_speed], "y": [settings["reference_force_n"]],
                   "marker": True, "color": COLORS[1]}]
        if operating_point is not None and speed_kmh != reference_speed:
            series.append(operating_point)
        self.readout.configure(text=status)
        self.chart.set({"title": "全車下壓力與車速", "xlabel": "車速 (km/h)", "ylabel": "下壓力 (N)",
                        "series": series})
