"""Import and inspect the geometry tables used by the suspension model."""

import csv
from copy import deepcopy
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, ttk

from .chart_base import Chart
from .components import number
from .geometry_lookup import FIELDS, TITLES, csv_header, read_csv


class GeometryPage(ttk.Frame):
    def __init__(self, app):
        super().__init__(app.pages, padding=10)
        self.app = app
        self.set_tables({})
        ttk.Label(self, text="橫軸為相對無空力、無加速度靜態位置的各軸平均輪端行程，壓縮為正。").pack(anchor="w")
        views = ttk.Notebook(self)
        views.pack(fill="both", expand=True, pady=(8, 0))
        self.status, self.charts, self.reset_buttons = {}, {}, {}
        for kind, title in TITLES.items():
            view = ttk.Frame(views, padding=8)
            views.add(view, text=title)
            actions = ttk.Frame(view)
            actions.pack(fill="x")
            for label, handler in (("匯入 CSV", self.import_table), ("匯出 CSV 範本", self.export_template),
                                   ("使用固定值", self.reset_table)):
                button = ttk.Button(actions, text=label, command=lambda k=kind, action=handler: action(k))
                button.pack(side="left", padx=(0, 8))
                if handler == self.reset_table:
                    self.reset_buttons[kind] = button
            self.status[kind] = tk.StringVar()
            ttk.Label(view, textvariable=self.status[kind], padding=(0, 8)).pack(anchor="w")
            chart = self.charts[kind] = Chart(view)
            chart.pack(fill="both", expand=True)

    def set_tables(self, tables):
        self.tables = deepcopy(tables)

    def draw(self):
        for kind in TITLES:
            self.draw_table(kind)

    def draw_table(self, kind):
        rows = self.tables.get(kind)
        self.reset_buttons[kind].configure(state="normal" if rows else "disabled")
        if rows:
            x = [row[0] for row in rows]
            columns = [[row[i] for row in rows] for i in range(1, len(rows[0]))]
            self.status[kind].set(f"使用查表｜{len(rows)} 筆｜行程範圍 {x[0]:g} 至 {x[-1]:g} mm")
            if len(x) == 1:
                x = [x[0] - 1., x[0] + 1.]
                columns = [column * 2 for column in columns]
        else:
            x = [-20., 20.]
            try:
                columns = [[number(float(self.app.vehicle_vars[key].get()), label) * units] * 2
                           for _, label, key, _, units in FIELDS[kind]]
            except ValueError:
                self.status[kind].set("請在「車輛與輪胎」輸入有效的固定值。")
                self.charts[kind].set(None)
                return
            self.status[kind].set("使用「車輛與輪胎」的固定值。")
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
        try:
            values = [number(float(self.app.vehicle_vars[key].get()), label) * units
                      for _, label, key, _, units in FIELDS[kind]]
        except ValueError:
            return self.app.error("請先填寫有效的固定幾何參數。")
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

    def reset_table(self, kind):
        self.tables.pop(kind, None)
        self.draw_table(kind)
