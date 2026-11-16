"""Import and inspect the geometry tables used by the suspension model."""

import csv
from copy import deepcopy
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, ttk

from .chart_base import Chart
from .components import number
from .geometry_lookup import FIELDS, MODES, TITLES, csv_header, geometry_modes, read_csv
from .project import VEHICLE_FIELDS
from .widgets import entry, key_for, option


class GeometryPage(ttk.Frame):
    def __init__(self, app):
        super().__init__(app.pages, padding=10)
        self.app = app
        self.tables = {}
        self.fixed_vars, self.selectors, self.modes = {}, {}, {}
        self.fixed_forms, self.csv_actions, self.status, self.charts = {}, {}, {}, {}
        self.columnconfigure((0, 1), weight=1, uniform="geometry")
        self.rowconfigure(2, weight=1)
        ttk.Label(self, text="橫軸為相對無空力、無加速度靜態位置的各軸平均輪端行程，壓縮為正。").grid(
            row=0, columnspan=2, sticky="w", pady=(0, 8))
        for column, (kind, title) in enumerate(TITLES.items()):
            padx = (0, 4) if column == 0 else (4, 0)
            controls = ttk.LabelFrame(self, text=title, padding=8)
            controls.grid(row=1, column=column, sticky="nsew", padx=padx, pady=(0, 8))
            self.build_controls(controls, kind)
            chart = self.charts[kind] = Chart(self, empty_message=f"請完成 {title} 設定")
            chart.canvas.configure(width=380, height=320)
            chart.grid(row=2, column=column, sticky="nsew", padx=padx)

    def build_controls(self, controls, kind):
        self.modes[kind] = tk.StringVar(value=MODES["fixed"])
        selector = self.selectors[kind] = option(controls, MODES, self.modes[kind], width=14)
        selector.pack(anchor="w", pady=(0, 4))
        selector.bind("<<ComboboxSelected>>", lambda event: self.show_mode(kind))
        forms = ttk.Frame(controls)
        forms.pack(fill="x")
        fixed = self.fixed_forms[kind] = ttk.Frame(forms)
        fixed.grid(row=0, column=0, sticky="w")
        for row, (_, _, key, _, _) in enumerate(FIELDS[kind]):
            var = self.fixed_vars[key] = tk.StringVar()
            entry(fixed, row, 0, VEHICLE_FIELDS[key][0], var, width=12)
            var.trace_add("write", lambda *_, group=kind: self.fixed_changed(group))

        actions = self.csv_actions[kind] = ttk.Frame(forms)
        actions.grid(row=0, column=0, sticky="w")
        ttk.Button(actions, text="匯入 CSV", command=lambda: self.import_table(kind)).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="匯出 CSV 範本", command=lambda: self.export_template(kind)).pack(side="left")
        self.status[kind] = tk.StringVar()
        ttk.Label(controls, textvariable=self.status[kind]).pack(anchor="w", pady=(4, 0))

    def set_settings(self, project):
        self.tables = deepcopy(project.get("geometry_tables", {}))
        for key, var in self.fixed_vars.items():
            var.set(project["vehicle"][key])
        for kind, mode in geometry_modes(project).items():
            self.modes[kind].set(MODES[mode])
            self.show_mode(kind)

    def mode(self, kind):
        return key_for(MODES, self.modes[kind].get())

    def show_mode(self, kind):
        fixed, csv_actions = self.fixed_forms[kind], self.csv_actions[kind]
        if self.mode(kind) == "lookup":
            fixed.grid_remove()
            csv_actions.grid()
        else:
            csv_actions.grid_remove()
            fixed.grid()
        self.draw_table(kind)

    def settings(self):
        settings = {"geometry_tables": deepcopy(self.tables)} if self.tables else {}
        modes = {}
        for kind, default in geometry_modes(settings).items():
            mode = self.mode(kind)
            if mode != default:
                modes[kind] = mode
        if modes:
            settings["geometry_modes"] = modes
        return settings

    def fixed_value(self, key):
        return number(float(self.fixed_vars[key].get()), *VEHICLE_FIELDS[key])

    def fixed_values(self, kind, *, use_previous=False):
        values = {}
        for _, label, key, _, _ in FIELDS[kind]:
            try:
                values[key] = self.fixed_value(key)
            except ValueError as exc:
                if not use_previous:
                    raise ValueError(f"請在「幾何查表」輸入有效的 {label} 固定值。") from exc
                values[key] = self.app.project["vehicle"][key]
        return values

    def fixed_settings(self):
        values = {}
        for kind in FIELDS:
            values.update(self.fixed_values(kind, use_previous=self.mode(kind) == "lookup"))
        return values

    def fixed_changed(self, kind):
        if self.app.pages.select() == str(self) and self.mode(kind) == "fixed":
            self.draw_table(kind)

    def draw(self):
        for kind in TITLES:
            self.draw_table(kind)

    def draw_table(self, kind):
        if self.mode(kind) == "lookup":
            rows = self.tables.get(kind)
            if not rows:
                self.status[kind].set(f"請匯入 {TITLES[kind]} CSV。")
                self.charts[kind].set(None)
                return
            x = [row[0] for row in rows]
            columns = [[row[i] for row in rows] for i in range(1, len(rows[0]))]
            self.status[kind].set(f"{len(rows)} 筆｜行程範圍 {x[0]:g} 至 {x[-1]:g} mm")
            if len(x) == 1:
                x = [x[0] - 1., x[0] + 1.]
                columns = [column * 2 for column in columns]
        else:
            x = [-20., 20.]
            try:
                columns = [[self.fixed_value(key) * units] * 2 for _, _, key, _, units in FIELDS[kind]]
            except ValueError:
                self.status[kind].set("請輸入有效的固定值。")
                self.charts[kind].set(None)
                return
            self.status[kind].set("比例輸入範例  0.15 = 15%" if kind == "anti" else "")
        series = [{"name": field[1], "x": x, "y": values} for field, values in zip(FIELDS[kind], columns)]
        unit = "高度 (mm)" if kind == "roll_center" else "比例 (%)"
        self.charts[kind].set(dict(title=TITLES[kind], xlabel="相對靜態輪端行程 (mm)", ylabel=unit, series=series))

    def import_table(self, kind):
        path = filedialog.askopenfilename(parent=self, title=f"匯入 {TITLES[kind]} CSV", filetypes=[("CSV", "*.csv")])
        if path:
            try:
                rows = read_csv(path, kind)
            except (OSError, ValueError) as exc:
                return self.app.error(exc)
            self.tables[kind] = rows
            self.draw_table(kind)
            self.app.status.set(f"已匯入 {Path(path).name}。")

    def export_template(self, kind):
        fixed = self.fixed_values(kind, use_previous=True)
        values = [fixed[key] * units for _, _, key, _, units in FIELDS[kind]]
        path = filedialog.asksaveasfilename(parent=self, defaultextension=".csv", initialfile=f"{kind}.csv",
                                            filetypes=[("CSV", "*.csv")])
        if path:
            try:
                with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(csv_header(kind))
                    writer.writerows([[travel, *values] for travel in (-20., 0., 20.)])
            except OSError as exc:
                self.app.error(exc)
