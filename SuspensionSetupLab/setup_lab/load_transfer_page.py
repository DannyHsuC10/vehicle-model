"""Quasi-static load transfer and tire contact patch views."""

import math
import tkinter as tk
from tkinter import filedialog, ttk

import numpy as np

from .chart_base import COLORS, Chart
from .lateral_tire import DEFAULT_TIR, import_tir
from .load_transfer import TransferDistribution, contact_patch
from .project import CONTACT_FIELDS, CORNERS, TRANSFER_FIELDS, default_project, transfer_options
from .widgets import entry


class LoadTransferPage(ttk.Frame):
    def __init__(self, app):
        super().__init__(app.pages, padding=10)
        self.app = app
        self.contact_model = None
        self.fields = {}
        setup = ttk.LabelFrame(self, text="工況與輪胎參數", padding=8)
        setup.pack(fill="x")
        for i, (key, (label, *_)) in enumerate(TRANSFER_FIELDS.items()):
            variable = self.fields[key] = tk.StringVar()
            entry(setup, i // 3, 2 * (i % 3), label, variable, width=8)
        row = (len(self.fields) + 2) // 3
        tire = ttk.Frame(setup)
        tire.grid(row=row, column=0, columnspan=6, sticky="w", pady=(4, 0))
        self.tir_source = None
        self.tir_name = tk.StringVar(value=DEFAULT_TIR)
        ttk.Label(tire, text="前後軸 TIR").pack(side="left", padx=(0, 6))
        ttk.Entry(tire, textvariable=self.tir_name, state="readonly", width=42).pack(side="left", padx=(0, 6))
        ttk.Button(tire, text="匯入 .tir", command=self.choose_tir).pack(side="left", padx=(0, 6))
        self.tir_reset = ttk.Button(tire, text="恢復預設", command=self.reset_tir, state="disabled")
        self.tir_reset.pack(side="left")
        ttk.Button(setup, text="更新計算", command=self.draw).grid(row=row + 1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(setup, text="正 ay 為左轉").grid(row=row + 1, column=1, columnspan=5, sticky="w")
        self.readout = ttk.Label(self, justify="left", padding=(0, 6))
        self.readout.pack(anchor="w", fill="x")
        self.views = ttk.Notebook(self)
        self.views.pack(fill="both", expand=True, pady=(6, 0))
        tables = ttk.Frame(self.views, padding=8)
        self.views.add(tables, text="計算明細")
        headers = ("輪位", "基準輪荷 (N)", "輪荷 (N)", "側向力 (N)", "滑角 (°)")
        self.wheels = self.table(tables, headers, 4)
        self.paths = self.table(tables, ("力矩路徑", "前軸 (N·m)", "後軸 (N·m)"), 3)
        self.balance = ttk.Label(tables, justify="left")
        self.balance.pack(anchor="w", pady=(4, 0))
        self.charts = {}
        for label in ("輪荷", "側傾", "載重轉移分配", "三路徑力矩", "穩態轉向", "不足轉向", "抓地"):
            chart = self.charts[label] = Chart(self.views)
            chart.canvas.configure(width=600, height=260)
            self.views.add(chart, text=label)
        self.build_contact_view()
        self.views.select(self.charts["輪荷"])

    def build_contact_view(self):
        self.contact_view = view = ttk.Frame(self.views, padding=8)
        self.views.add(view, text="接地面")
        controls = ttk.Frame(view)
        controls.pack(fill="x", pady=(0, 6))
        ttk.Label(controls, text="預覽側向加速度 (g)").pack(side="left", padx=(0, 8))
        _, _, low, high = TRANSFER_FIELDS["ay"]
        self.contact_ay = tk.DoubleVar()
        self.contact_slider = ttk.Scale(controls, from_=low, to=high, variable=self.contact_ay,
                                       command=self.update_contact)
        self.contact_slider.pack(side="left", fill="x", expand=True)
        self.contact_label = ttk.Label(controls, width=9, anchor="e")
        self.contact_label.pack(side="left")

        sidebar = ttk.Frame(view)
        sidebar.pack(side="right", fill="y", padx=(8, 0))
        tires = ttk.LabelFrame(sidebar, text="接地面參數", padding=6)
        tires.pack(fill="x", pady=(0, 8))
        for row, (key, (label, *_)) in enumerate(CONTACT_FIELDS.items()):
            variable = self.fields[key] = tk.StringVar()
            entry(tires, row, 0, label, variable, width=8)
        headers = ("輪位", "輪荷 (N)", "面積 (cm²)", "長 (mm)", "寬 (mm)")
        self.contact_wheels = self.table(sidebar, headers, 4, widths=(45, 68, 76, 62, 62))
        self.contact_readout = ttk.Label(sidebar, justify="left")
        self.contact_readout.pack(anchor="w")
        chart = self.charts["接地面"] = Chart(view)
        chart.canvas.configure(width=530, height=260)
        chart.pack(fill="both", expand=True)
        self.contact_angles = np.linspace(0., 2 * math.pi, 65)

    def table(self, parent, headers, height, *, widths=None):
        table = ttk.Treeview(parent, columns=headers, show="headings", height=height)
        for name, width in zip(headers, widths or (118,) * len(headers)):
            table.heading(name, text=name)
            table.column(name, width=width, minwidth=min(width, 64), anchor="center")
        table.pack(fill="x", pady=(0, 4))
        return table

    def set_options(self, values):
        for key, variable in self.fields.items():
            variable.set(values[key])
        self.set_tir(values.get("tire_tir"))
        self.clear()

    def set_tir(self, source):
        self.tir_source = source
        self.tir_name.set(source["name"] if source else DEFAULT_TIR)
        self.tir_reset.state(["!disabled"] if source else ["disabled"])

    def choose_tir(self):
        path = filedialog.askopenfilename(parent=self, title="匯入 MF6.1 TIR", filetypes=[("TIR", "*.tir")])
        if not path:
            return
        try:
            source = import_tir(path)
        except (OSError, ValueError) as exc:
            self.app.error(exc)
            return
        self.set_tir(source)
        self.draw()

    def reset_tir(self):
        self.set_tir(None)
        self.draw()

    def options(self):
        try:
            values = {key: float(var.get()) for key, var in self.fields.items()}
        except ValueError as exc:
            raise ValueError("載重轉移分配欄位須填入數值。") from exc
        if self.tir_source is not None:
            values["tire_tir"] = self.tir_source
        return transfer_options(values)

    def clear(self):
        self.readout.configure(text="")
        for table in (self.wheels, self.paths, self.contact_wheels):
            table.delete(*table.get_children())
        self.balance.configure(text="")
        self.contact_label.configure(text="")
        self.contact_readout.configure(text="")
        self.contact_model = None
        self.contact_slider.state(["disabled"])
        for chart in self.charts.values():
            chart.set(None)

    def draw(self):
        self.clear()
        try:
            options = self.options()
            case = {**default_project()["case"], "target": "all"}
            project = self.app.collect(self.app.analysis_pages["full"], case=case)
            model = TransferDistribution(project, options)
            self.draw_distribution(model, options["ay"])
            self.set_contact_model(model, options["ay"])
        except ValueError as exc:
            self.clear()
            self.readout.configure(text=str(exc))

    @staticmethod
    def number(value, digits=1):
        return "—" if value is None else f"{value:.{digits}f}"

    def plot(self, key, x, curves, unit):
        series = [{"name": name, "color": COLORS[i % len(COLORS)], "x": x, "y": list(y)}
                  for i, (name, y) in enumerate(curves)]
        self.charts[key].set(dict(title=key, xlabel="側向加速度 (g)", ylabel=unit, series=series))

    def set_contact_model(self, model, ay):
        self.contact_model = model
        _, _, low, high = TRANSFER_FIELDS["ay"]
        # With the current aero law, wheel loads are linear on either side of zero g.
        peak_loads = np.max([model.load_state(g)["loads"] for g in (low, 0., high)], axis=0)
        patches = self.contact_patches(peak_loads)
        half_length = max(length / 2 for _, length, _ in patches)
        half_span = max(track * 500 + width / 2 for track, (_, _, width) in zip(np.repeat(model.track, 2), patches))
        self.contact_bounds = (-model.wheelbase * 1000 - half_length, half_length, -half_span, half_span)
        self.contact_ay.set(ay)
        self.contact_slider.state(["!disabled"])
        self.update_contact(ay)

    def contact_patches(self, loads):
        options = self.contact_model.options
        return [contact_patch(load, options[f"pressure_{axle}"], options[f"width_{axle}"])
                for load, axle in zip(loads, ("f", "f", "r", "r"))]

    def update_contact(self, value):
        model = self.contact_model
        if model is None:
            return
        ay = float(value)
        self.contact_label.configure(text=f"{ay:+.2f} g")
        data = model.load_state(ay)
        loads = data["loads"]
        self.contact_wheels.delete(*self.contact_wheels.get_children())
        series = []
        for i, (corner, load, (area, length, width)) in enumerate(zip(CORNERS, loads, self.contact_patches(loads))):
            self.contact_wheels.insert("", "end", values=(corner, *(self.number(v) for v in (load, area, length, width))))
            x = 0. if i < 2 else -1000 * model.wheelbase
            y = (1 if i % 2 == 0 else -1) * 500 * model.track[i // 2]
            series.append({"name": corner, "x": (x + length / 2 * np.cos(self.contact_angles)).tolist() if width else [x],
                           "y": (y + width / 2 * np.sin(self.contact_angles)).tolist() if width else [y],
                           "marker": "x" if width == 0 else False})
        lift = "、".join(corner for corner, load in zip(CORNERS, loads) if load <= 0)
        front, rear = data["lateral"]
        self.contact_readout.configure(text=f"前軸轉移量 {front:+.1f} N\n後軸轉移量 {rear:+.1f} N"
                                           + (f"\n抬輪 {lift}" if lift else ""))
        self.charts["接地面"].set(dict(title="接地面橢圓", xlabel="縱向，向前為正 (mm)", ylabel="橫向，向左為正 (mm)",
                                      equal=True, fit_bounds=self.contact_bounds, series=series), keep_view=True)

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
        grip = "／".join("—" if value is None else f"{value:.1%}" for value in data["grip"])
        self.readout.configure(text=(
            f"前軸 LLTD {front_share}%　｜　側傾 {data['roll']:.3f}°　｜　車速 {data['speed_kmh']:.1f} km/h{note}\n"
            f"前／後相對抓地 {grip}　｜　轉向 {self.number(data['steer'], 2)}°"
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
        grip_percent = [[None if d["grip"][i] is None else 100 * d["grip"][i] for d in scan] for i in range(2)]
        self.plot("抓地", x, zip(("前軸", "後軸"), grip_percent), "相對抓地 (%)")
