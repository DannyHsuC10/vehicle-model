"""Standalone views of the two quasi-static load transfer calculations."""

import math
import tkinter as tk
from tkinter import ttk

import numpy as np

from .chart_base import COLORS, Chart
from .load_transfer import TransferDistribution, contact_patch, rigid_transfer
from .components import number
from .project import CORNERS, TRANSFER_FIELDS, VEHICLE_FIELDS, default_project, transfer_options
from .widgets import entry


TITLES = {"rigid": "剛體載重轉移", "distribution": "載重轉移分配"}


class LoadTransferPage(ttk.Frame):
    def __init__(self, app, kind):
        super().__init__(app.pages, padding=10)
        self.app, self.kind = app, kind
        self.fields = {}
        setup = ttk.LabelFrame(self, text="工況與輪胎參數", padding=8)
        setup.pack(fill="x")
        for i, (key, (label, *_)) in enumerate(TRANSFER_FIELDS[kind].items()):
            variable = self.fields[key] = tk.StringVar()
            entry(setup, i // 3, 2 * (i % 3), label, variable, width=8)
        row = (len(self.fields) + 2) // 3
        ttk.Button(setup, text="更新計算", command=self.draw).grid(row=row, column=0, sticky="w", pady=(4, 0))
        hint = ("正 ax 為加速，負 ax 為煞車。正 ay 為左轉。"
                if kind == "rigid" else "正 ay 為左轉。空力依彎道半徑與 ay 換算車速。")
        ttk.Label(setup, text=hint).grid(row=row, column=1, columnspan=5, sticky="w")
        self.readout = ttk.Label(self, justify="left", padding=(0, 6))
        self.readout.pack(anchor="w", fill="x")
        self.views = ttk.Notebook(self)
        self.views.pack(fill="both", expand=True, pady=(6, 0))
        labels = (("輪荷", "接地面") if kind == "rigid"
                  else ("輪荷", "側傾", "載重轉移分配", "三路徑力矩", "穩態轉向", "不足轉向", "抓地"))
        tables = ttk.Frame(self.views, padding=8)
        self.views.add(tables, text="計算明細")
        if kind == "rigid":
            headers = ("輪位", "靜態輪荷 (N)", "輪荷 (N)", "接地面積 (cm²)", "接地長 (mm)", "接地寬 (mm)")
        else:
            headers = ("輪位", "基準輪荷 (N)", "輪荷 (N)", "側向力需求 (N)", "滑角 (°)")
        self.wheels = self.table(tables, headers, 4)
        if kind == "distribution":
            self.paths = self.table(tables, ("力矩路徑", "前軸 (N·m)", "後軸 (N·m)"), 3)
            self.balance = ttk.Label(tables, justify="left")
            self.balance.pack(anchor="w", pady=(4, 0))
        self.charts = {}
        for label in labels:
            chart = self.charts[label] = Chart(self.views)
            chart.canvas.configure(width=600, height=260)
            self.views.add(chart, text=label)
        self.views.select(self.charts["輪荷"])

    def table(self, parent, headers, height):
        table = ttk.Treeview(parent, columns=headers, show="headings", height=height)
        for name in headers:
            table.heading(name, text=name)
            table.column(name, width=118, minwidth=64, anchor="center")
        table.pack(fill="x", pady=(0, 4))
        return table

    def set_options(self, values):
        for key, value in transfer_options(self.kind, values).items():
            self.fields[key].set(value)
        self.clear()

    def options(self):
        try:
            values = {key: float(var.get()) for key, var in self.fields.items()}
        except ValueError as exc:
            raise ValueError(f"{TITLES[self.kind]}欄位須填入數值。") from exc
        return transfer_options(self.kind, values)

    def clear(self):
        self.readout.configure(text="")
        self.wheels.delete(*self.wheels.get_children())
        if self.kind == "distribution":
            self.paths.delete(*self.paths.get_children())
            self.balance.configure(text="")
        for chart in self.charts.values():
            chart.set(None)

    def draw(self):
        self.clear()
        try:
            options = self.options()
            if self.kind == "rigid":
                vehicle = self.app.vehicle_settings()
                for key in ("m", "h_cg", "lf", "lr", "tf", "tr"):
                    number(vehicle[key], *VEHICLE_FIELDS[key])
                self.draw_rigid(vehicle, options)
            else:
                case = {**default_project()["case"], "target": "all"}
                project = self.app.collect(self.app.analysis_pages["full"], case=case)
                self.draw_distribution(TransferDistribution(project, options), options["ay"])
        except ValueError as exc:
            self.readout.configure(text=str(exc))

    @staticmethod
    def number(value, digits=1):
        return "—" if value is None else f"{value:.{digits}f}"

    def plot(self, key, x, curves, unit, *, xlabel="側向加速度 (g)", equal=False):
        series = [{"name": name, "color": COLORS[i % len(COLORS)], "x": x, "y": list(y)}
                  for i, (name, y) in enumerate(curves)]
        self.charts[key].set(dict(title=key, xlabel=xlabel, ylabel=unit, series=series, equal=equal))

    def draw_rigid(self, vehicle, options):
        static, axles, longitudinal, loads = rigid_transfer(vehicle, options["ax"], options["ay"])
        patches = []
        for i, corner in enumerate(CORNERS):
            end = "f" if i < 2 else "r"
            patch = contact_patch(loads[i], options[f"pressure_{end}"], options[f"width_{end}"])
            patches.append(patch)
            self.wheels.insert("", "end", values=(corner, *(self.number(value) for value in (static[i], loads[i], *patch))))
        lift = "、".join(corner for corner, load in zip(CORNERS, loads) if load <= 0)
        note = f"　｜　抬輪：{lift}" if lift else ""
        self.readout.configure(text=f"前軸 {axles[0]:.1f} N　｜　後軸 {axles[1]:.1f} N　｜　轉移至前軸 {longitudinal:+.1f} N{note}")
        bound = max(2., abs(options["ay"]))
        ay = np.unique(np.r_[np.linspace(-bound, bound, 81), options["ay"]]).tolist()
        scan = np.array([rigid_transfer(vehicle, options["ax"], value)[3] for value in ay])
        self.plot("輪荷", ay, zip(CORNERS, scan.T), "輪荷 (N)")
        theta = np.linspace(0., 2 * math.pi, 65)
        series = []
        for i, (corner, (_, length, width)) in enumerate(zip(CORNERS, patches)):
            front = i < 2
            x = 0. if front else -1000 * (vehicle["lf"] + vehicle["lr"])
            y = (1 if i % 2 == 0 else -1) * 500 * vehicle["tf" if front else "tr"]
            series.append({"name": corner, "x": (x + length / 2 * np.cos(theta)).tolist() if width else [x],
                           "y": (y + width / 2 * np.sin(theta)).tolist() if width else [y],
                           "marker": "x" if width == 0 else False})
        self.charts["接地面"].set(dict(title="接地面橢圓", xlabel="縱向，向前為正 (mm)", ylabel="橫向，向左為正 (mm)",
                                      equal=True, series=series))

    def draw_distribution(self, model, ay):
        data = model.at(ay)
        for i, corner in enumerate(CORNERS):
            values = (data["baseline"][i], data["loads"][i], data["forces"][i], data["slips"][i // 2])
            self.wheels.insert("", "end", values=(corner, *(self.number(value) for value in values)))
        for label, row in zip(("簧下", "幾何", "彈性"), data["paths"]):
            self.paths.insert("", "end", values=(label, *(self.number(value) for value in row)))
        first_lift = int(np.argmin(model.lift_limits_g))
        lift_limit = model.lift_limits_g[first_lift]
        lift_label = f"{('前軸', '後軸')[first_lift]} {lift_limit:.2f} g" if math.isfinite(lift_limit) else "—"
        moment_share = data["front_moment_share"]
        moment_label = self.number(None if moment_share is None else 100 * moment_share)
        self.balance.configure(text=(
            f"前軸力矩占比 {moment_label}%　｜　力矩平衡誤差 {data['moment_error']:.3g} N·m\n"
            f"前／後側傾剛性 {model.roll_stiffness[0]:.1f}／{model.roll_stiffness[1]:.1f} N·m/rad"
            f"　｜　側傾梯度 {model.roll_gradient:.3f}°/g\n"
            f"前／後靜態 Roll Center {model.rc[0]*1000:.1f}／{model.rc[1]*1000:.1f} mm\n"
            f"無空力首個抬輪門檻 {lift_label}"))
        lift = "、".join(corner for corner, load in zip(CORNERS, data["loads"]) if load <= 0)
        note = f"　｜　抬輪：{lift}" if lift else ""
        front_share = self.number(None if data["front_share"] is None else 100 * data["front_share"])
        self.readout.configure(text=(
            f"前軸 LLTD {front_share}%　｜　側傾 {data['roll']:.3f}°　｜　車速 {data['speed_kmh']:.1f} km/h{note}\n"
            f"前／後抓地 {data['grip'][0]:.1%}／{data['grip'][1]:.1%}　｜　轉向 {self.number(data['steer'], 2)}°"
            f"　｜　不足轉向 {self.number(data['understeer'], 2)}°/g"))
        bound = max(min(2.5, float(lift_limit)), abs(ay), .05)
        x = np.unique(np.r_[np.linspace(-bound, bound, 81), ay]).tolist()
        scan = [model.at(value) for value in x]
        self.plot("輪荷", x, zip(CORNERS, np.array([d["loads"] for d in scan]).T), "輪荷 (N)")
        self.plot("側傾", x, [("側傾", [d["roll"] for d in scan])], "側傾角 (°)")
        self.plot("載重轉移分配", x, zip(("前軸 ΔF", "後軸 ΔF"), np.array([d["lateral"] for d in scan]).T), "內輪移至外輪 (N)")
        path_names = [f"{axle} {path}" for path in ("簧下", "幾何", "彈性") for axle in ("前軸", "後軸")]
        self.plot("三路徑力矩", x, zip(path_names, np.array([d["paths"].flatten() for d in scan]).T), "力矩 (N·m)")
        self.plot("穩態轉向", x, [("穩態轉向", [d["steer"] for d in scan])], "轉向角 (°)")
        self.plot("不足轉向", x, [("不足轉向梯度", [d["understeer"] for d in scan])], "梯度 (°/g)")
        self.plot("抓地", x, zip(("前軸", "後軸"), 100 * np.array([d["grip"] for d in scan]).T), "剩餘抓地 (%)")
