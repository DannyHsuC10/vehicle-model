"""Standalone five-page native workbench; workers never access Tk widgets."""
import copy
import json
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from vehicle_lab.gui import ScrollFrame
from vehicle_lab.export import write_csv
from .project import (FIELDS, default_project, load_project, save_project, validate)
from .engine import Engine
from .analysis import PAGES, PLOTS, NOTES, calculate_mapping, make_plot, static_point_rows
from .charts import LabChart
from .export import point_rows, write_report

FRAME_INTERVAL_MS = 16


def field_entries(parent, project, groups):
    variables = {}
    for title, group, keys in groups:
        ttk.Label(parent, text=title, font=("Microsoft JhengHei UI", 11, "bold"), padding=(0, 12, 0, 6)).pack(anchor="w")
        for key in keys:
            label, unit = FIELDS[group][key]
            row = ttk.Frame(parent)
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=f"{label} ({unit})", wraplength=215).pack(side="left", fill="x", expand=True)
            var = tk.StringVar(value=f"{project[group][key]:.10g}")
            ttk.Entry(row, textvariable=var, width=12).pack(side="right", padx=(6, 0))
            variables[(group, key)] = var
    return variables


def read_entries(project, entries):
    result = copy.deepcopy(project)
    for (group, key), var in entries.items():
        try:
            result[group][key] = float(var.get())
        except ValueError as exc:
            raise ValueError(f"{group} · {FIELDS[group][key][0]} 需要數值。") from exc
    result["analysis"]["samples"] = int(result["analysis"]["samples"]) if result["analysis"]["samples"].is_integer() else result["analysis"]["samples"]
    return validate(result)


class ParameterEditor(tk.Toplevel):
    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("共用車輛參數 · Suspension Lab")
        self.geometry("700x760")
        self.transient(app)
        self.grab_set()
        ttk.Label(self, text="所有頁面共用一份 JSON 設定；長度 m、角度 deg、比例 1=100%。", padding=14).pack(fill="x")
        name_row = ttk.Frame(self, padding=(14, 0, 14, 10))
        name_row.pack(fill="x")
        ttk.Label(name_row, text="專案名稱").pack(side="left")
        self.name = tk.StringVar(value=app.project["name"])
        ttk.Entry(name_row, textvariable=self.name).pack(side="left", fill="x", expand=True, padx=10)
        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True, padx=14)
        self.entries = {}
        for group, label in (("vehicle", "車輛 / 輪胎"), ("front", "前軸懸吊"), ("rear", "後軸懸吊"), ("analysis", "掃描範圍")):
            frame = ScrollFrame(notebook)
            notebook.add(frame, text=label)
            self.entries.update(field_entries(frame.content, app.project, [(label, group, FIELDS[group])]))
        ttk.Label(self, text="SVSA=0：由 anti-dive / anti-lift 解瞬心。SVSA>0：使用指定水平長與 anti-dive，anti-lift 由結果算出。", wraplength=640, padding=14).pack(fill="x")
        buttons = ttk.Frame(self, padding=14)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="取消", command=self.destroy).pack(side="right")
        ttk.Button(buttons, text="套用全部參數", command=self.apply).pack(side="right", padx=8)

    def apply(self):
        try:
            candidate = read_entries(self.app.project, self.entries)
            candidate["name"] = self.name.get().strip()
            self.app.apply_project(candidate)
            self.destroy()
        except (ValueError, ArithmeticError) as exc:
            messagebox.showerror("設定無法套用", str(exc), parent=self)


class Panel(ttk.Frame):
    def __init__(self, app, key):
        super().__init__(app.notebook)
        self.app, self.key = app, key
        self.token = 0
        self.timer = None
        self._request_in_flight = False
        self._slider_pending = False
        self.state = self.mapping = self.error = None
        self._chart_key = None
        self.ready = False
        self.pending_export = None
        self.axle = tk.StringVar(value="front")
        self.mode = tk.StringVar(value="左右行程" if key in ("front", "rear") else "前後行程")
        self.plot = tk.StringVar(value=PLOTS[key][0][1])
        self.controls = [tk.DoubleVar(value=0.), tk.DoubleVar(value=0.)]
        self.groups = {k: tk.BooleanVar(value=True) for k in ("front", "rear", "sv")}
        self.construction = tk.BooleanVar(value=True)
        self.static_references = tk.BooleanVar(value=True)
        pane = ttk.Panedwindow(self, orient="horizontal")
        pane.pack(fill="both", expand=True)
        left = ttk.Frame(pane, width=345)
        pane.add(left, weight=0)
        right = ttk.Frame(pane)
        pane.add(right, weight=1)
        self.scroll = ScrollFrame(left)
        self.scroll.pack(fill="both", expand=True, padx=(10, 4))
        self.entries = field_entries(self.scroll.content, app.project, self.field_groups())
        if key in ("sv", "static"):
            ttk.Label(self.scroll.content, text="SVSA>0 時由 SVSA 與 anti-dive 決定 IC；anti-lift 改為計算結果。", wraplength=295, foreground="#64748b", padding=(0, 12)).pack(fill="x")
        ttk.Button(left, text="套用左側參數並重新解算", command=self.apply).pack(fill="x", padx=10, pady=10)
        toolbar = ttk.Frame(right, padding=(8, 10))
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="圖表").pack(side="left")
        self.plot_select = ttk.Combobox(toolbar, state="readonly", textvariable=self.plot,
                                       values=[name for _, name in PLOTS[key]], width=40)
        self.plot_select.pack(side="left", padx=8)
        self.plot_select.bind("<<ComboboxSelected>>", lambda e: self.request())
        ttk.Button(toolbar, text="重設視角", command=lambda: self.chart.reset()).pack(side="left", padx=4)
        if key == "we":
            ttk.Label(toolbar, text="車軸").pack(side="left", padx=(8, 0))
            combo = ttk.Combobox(toolbar, state="readonly", textvariable=self.axle, values=["front", "rear"], width=8)
            combo.pack(side="left", padx=6)
            combo.bind("<<ComboboxSelected>>", lambda e: self.request())
        if key in ("front", "rear", "sv"):
            row = ttk.Frame(right, padding=(8, 0, 8, 0))
            row.pack(fill="x")
            modes = ["前後行程", "Heave / Pitch"] if key == "sv" else ["左右行程", "Heave / Roll"]
            ttk.Label(row, text="滑桿 / 曲面座標").pack(side="left")
            combo = ttk.Combobox(row, state="readonly", textvariable=self.mode, values=modes, width=18)
            combo.pack(side="left", padx=8)
            combo.bind("<<ComboboxSelected>>", self.change_mode)
        if key == "static":
            row = ttk.Frame(right, padding=(8, 0))
            row.pack(fill="x")
            for group, label in (("front", "前 FV 平面"), ("rear", "後 FV 平面"), ("sv", "SV 平面")):
                ttk.Checkbutton(row, text=label, variable=self.groups[group], command=self.request).pack(side="left", padx=5)
            self.static_references_check = ttk.Checkbutton(
                row, text="瞬心／Kingpin 連線與延長線", variable=self.static_references, command=self.render)
            self.static_references_check.pack(side="left", padx=8)
        else:
            row = ttk.Frame(right, padding=(8, 4))
            row.pack(fill="x")
            ttk.Checkbutton(row, text="顯示 IC / 力線參考", variable=self.construction, command=self.render).pack(side="left")
            ttk.Button(row, text="回到零姿態", command=self.zero).pack(side="left", padx=12)
        self.sliders, self.slider_labels = [], []
        for i in range(0 if key == "static" else 1 if key == "we" else 2):
            row = ttk.Frame(right, padding=(8, 2))
            row.pack(fill="x")
            label = ttk.Label(row, width=23)
            label.pack(side="left")
            self.slider_labels.append(label)
            scale = ttk.Scale(row, variable=self.controls[i], command=lambda value: self.slider_changed())
            scale.pack(side="left", fill="x", expand=True, padx=8)
            self.sliders.append(scale)
        self.configure_sliders()
        self.chart = LabChart(right)
        metric_frame = ttk.Frame(right)
        self.metrics = ttk.Treeview(metric_frame, columns=("metric", "value"), show="headings", height=5)
        self.metrics.heading("metric", text="目前姿態結果 / 單位")
        self.metrics.heading("value", text="數值（N/A：無有限交點）")
        self.metrics.column("metric", width=270)
        self.metrics.column("value", width=240)
        self.metrics.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(metric_frame, command=self.metrics.yview)
        self.metrics.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.note = ttk.Label(right, text=NOTES[key], wraplength=950, foreground="#475569", padding=(8, 7))
        self.note.pack(side="bottom", fill="x")
        right.bind("<Configure>", lambda event: self.note.configure(wraplength=max(300, event.width-24)))
        metric_frame.pack(side="bottom", fill="x", padx=8)
        self.chart.pack(fill="both", expand=True, padx=8, pady=6)
        self.ready = True

    def field_groups(self):
        k = self.key
        v = ["track_"+k, "free_radius", "tire_width"] if k in ("front", "rear") else list(FIELDS["vehicle"]) if k == "static" else ["wheelbase", "cg_height", "ground_clearance", "free_radius", "tire_stiffness", "brake_bias_front"]
        groups = [("共用車輛 / 輪胎", "vehicle", v)]
        if k in ("front", "rear"):
            groups.append((PAGES[k]+" 設計", k, list(FIELDS[k])[:8]))
        elif k in ("sv", "we"):
            keys = ["caster_deg", "mechanical_trail", "anti_dive", "anti_lift", "svsa", "upper_offset", "lower_offset"] if k == "sv" else ["kpi_deg", "caster_deg", "scrub_radius", "mechanical_trail"]
            for axle, title in (("front", "前軸設計"), ("rear", "後軸設計")):
                groups.append((title, axle, keys))
        else:
            for axle in ("front", "rear"):
                groups.append(("前軸" if axle == "front" else "後軸", axle, FIELDS[axle]))
        groups.append(("範圍掃描設定", "analysis", FIELDS["analysis"]))
        return groups

    def sync(self):
        if self.timer:
            self.after_cancel(self.timer)
            self.timer = None
        self._request_in_flight = self._slider_pending = False
        self.ready_for_export = False
        for (group, key), var in self.entries.items():
            var.set(f"{self.app.project[group][key]:.10g}")
        self.state = self.mapping = self.error = None
        self.token += 1
        self.pending_export = None
        self._chart_key = None
        self.configure_sliders()
        for var in self.controls:
            var.set(0.)
        self.chart.set(None)
        self.metrics.delete(*self.metrics.get_children())

    def apply(self):
        try:
            self.app.apply_project(read_entries(self.app.project, self.entries))
        except (ValueError, ArithmeticError) as exc:
            messagebox.showerror("設定無法套用", str(exc), parent=self)

    def configure_sliders(self):
        a = self.app.project["analysis"]
        pose = self.mode.get().startswith("Heave")
        labels = ["Steering (deg)"] if self.key == "we" else (["Heave (mm)", "Pitch (deg)" if self.key == "sv" else "Roll (deg)"] if pose else ["前輪上壓行程 (mm)", "後輪上壓行程 (mm)"] if self.key == "sv" else ["左輪上壓行程 (mm)", "右輪上壓行程 (mm)"])
        for i, scale in enumerate(self.sliders):
            limit = a["steer_deg"] if self.key == "we" else a["pitch_deg" if self.key == "sv" else "roll_deg"] if pose and i == 1 else a["travel_mm"]
            scale.configure(from_=-limit, to=limit)
            self.slider_labels[i].configure(text=f"{labels[i]}  {self.controls[i].get():+.2f}")

    def change_mode(self, event=None):
        self.zero()
        self.configure_sliders()
        self.request()

    def zero(self):
        for var in self.controls:
            var.set(0.)
        self.slider_changed()

    def slider_changed(self):
        self.configure_sliders()
        if not self.ready:
            return
        self.pending_export = None
        self.ready_for_export = False
        self._slider_pending = True
        # Throttle rather than debounce: continuous movement must still solve.
        if self.timer is None:
            self.timer = self.after(FRAME_INTERVAL_MS, self.flush_slider)

    def flush_slider(self):
        self.timer = None
        if self._slider_pending and not self._request_in_flight:
            self.request()

    @property
    def plot_key(self):
        return next(key for key, name in PLOTS[self.key] if name == self.plot.get())

    def request(self, force_mapping=False, pending_export=None):
        if not self.ready:
            return
        if self.timer:
            self.after_cancel(self.timer)
            self.timer = None
        self._slider_pending = False
        self._request_in_flight = True
        self.token += 1
        token, revision = self.token, self.app.revision
        engine, cache = self.app.engine, self.app.mapping_cache
        mode = "pose" if self.mode.get().startswith("Heave") else "travel"
        axle = self.axle.get()
        values = tuple(v.get() for v in self.controls)
        key, plot_key = self.key, self.plot_key
        need_map = force_mapping or (key != "static" and plot_key != "motion")
        self.pending_export = pending_export
        self.ready_for_export = False
        self.app.status.set(f"{PAGES[key]} · 計算中…")
        cancelled = lambda: self.token != token or self.app.revision != revision or self.app.closing
        def work():
            if cancelled():
                return
            state, error, mapping = None, None, None
            try:
                try:
                    if key in ("front", "rear"):
                        state = engine.fv[key].pose(*values) if mode == "pose" else engine.fv[key].solve(*values)
                    elif key == "sv":
                        state = engine.sv_pose(*values) if mode == "pose" else engine.solve_sv(*values)
                    elif key == "we":
                        state = engine.solve_we(values[0], axle)
                    else:
                        state = {"metrics": {"wireframe_lines": len(engine.scene.lines), "named_points": len(static_point_rows(engine))}, "points": {}}
                except ValueError as exc:
                    error = str(exc)
                if need_map and not cancelled():
                    cache_key = (key, mode, axle)
                    mapping = cache.get(cache_key)
                    if mapping is None:
                        mapping = calculate_mapping(engine, key, mode, axle, cancelled)
                        if mapping is not None and not cancelled():
                            cache[cache_key] = mapping
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
            if not cancelled():
                self.app.results.put((self, token, revision, state, mapping, error))
        self.app.jobs.put(work)

    def receive(self, state, mapping, error):
        self._request_in_flight = False
        self.state, self.mapping, self.error = state, mapping, error
        self.ready_for_export = True
        self.metrics.delete(*self.metrics.get_children())
        for key, value in (state or {}).get("metrics", {}).items():
            self.metrics.insert("", "end", values=(key, "N/A" if value is None else f"{value:.7g}"))
        self.render()
        coverage = f" · 掃描 {mapping['valid']}/{mapping['total']} 有效" if mapping else ""
        self.app.status.set((f"{PAGES[self.key]} · {error}" if error else f"{PAGES[self.key]} · 完成")+coverage)
        if self.pending_export:
            kind, path = self.pending_export
            self.pending_export = None
            if mapping:
                self.app.perform_export(kind, path, self)
        if self._slider_pending:
            # Display completed work even while the pointer moves, then solve
            # the latest slider values. Keep at most one pose solve in flight.
            self.ready_for_export = False
            if self.timer is None:
                self.timer = self.after(FRAME_INTERVAL_MS, self.flush_slider)

    def render(self):
        if not self.ready:
            return
        if self.key != "static" and self.plot_key == "motion" and not self.state:
            self.chart.set(None, preserve_view=True)
            return
        spec = make_plot(self.key, self.plot_key, self.state, self.mapping, self.app.engine,
                         {k: v.get() for k, v in self.groups.items()},
                         show_static_references=self.static_references.get())
        if spec and "world_lines" in spec and self.key != "static" and not self.construction.get():
            spec["world_lines"] = [p for p in spec["world_lines"] if not p["construction"]]
        chart_key = (self.plot_key, self.mode.get(), self.axle.get())
        self.chart.set(spec, preserve_view=chart_key == self._chart_key)
        self._chart_key = chart_key


class App(tk.Tk):
    def __init__(self, project=None):
        super().__init__()
        self.title("Suspension Lab · 懸吊幾何工作台")
        self.geometry("1500x960")
        self.minsize(1150, 760)
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background="#f1f5f9")
        style.configure("TLabel", background="#f1f5f9", font=("Microsoft JhengHei UI", 10))
        style.configure("TButton", padding=(9, 5), font=("Microsoft JhengHei UI", 10))
        style.configure("TNotebook.Tab", padding=(22, 9), font=("Microsoft JhengHei UI", 11))
        self.project = copy.deepcopy(project or default_project())
        self.engine = Engine(self.project)
        self.revision = 0
        self.dirty = self.closing = False
        self.path = None
        self.editor = None
        self.mapping_cache = {}
        self.jobs, self.results = queue.Queue(), queue.Queue()
        header = ttk.Frame(self, padding=(16, 12))
        header.pack(fill="x")
        ttk.Label(header, text="SUSPENSION LAB", font=("Segoe UI", 19, "bold")).pack(side="left")
        self.project_label = ttk.Label(header, text=self.project["name"], foreground="#475569")
        self.project_label.pack(side="left", padx=20)
        for text, command in (("共用車輛參數", self.open_parameters), ("開啟專案", self.open_project),
                              ("儲存專案", self.save), ("另存專案", lambda: self.save(True))):
            ttk.Button(header, text=text, command=command).pack(side="right", padx=4)
        self.notebook = ttk.Notebook(self)
        self.panels = {key: Panel(self, key) for key in PAGES}
        for key, panel in self.panels.items():
            self.notebook.add(panel, text=PAGES[key])
        self.notebook.bind("<<NotebookTabChanged>>", lambda event: self.active.request())
        footer = ttk.Frame(self, padding=(12, 8))
        footer.pack(side="bottom", fill="x")
        for kind, label in (("state", "姿態 CSV"), ("points", "硬點 CSV"), ("mapping", "範圍 CSV"),
                            ("png", "PNG 圖片"), ("svg", "SVG 向量圖"), ("report", "HTML 報告 / PDF")):
            ttk.Button(footer, text=label, command=lambda k=kind: self.export(k)).pack(side="left", padx=3)
        self.status = tk.StringVar(value="靜態幾何已建立")
        ttk.Label(self, textvariable=self.status, padding=(16, 4), foreground="#475569").pack(side="bottom", fill="x")
        self.notebook.pack(fill="both", expand=True, padx=12)
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.bind("<Control-s>", lambda e: self.save())
        self.bind("<Control-o>", lambda e: self.open_project())
        self.worker_thread = threading.Thread(target=self.worker, daemon=True)
        self.worker_thread.start()
        self.poll_timer = self.after(FRAME_INTERVAL_MS, self.poll)
        self.after_idle(self.active.request)

    @property
    def active(self):
        return self.nametowidget(self.notebook.select())

    def worker(self):
        while True:
            job = self.jobs.get()
            if job is None:
                return
            job()

    def poll(self):
        try:
            while True:
                panel, token, revision, state, mapping, error = self.results.get_nowait()
                if token == panel.token and revision == self.revision:
                    panel.receive(state, mapping, error)
        except queue.Empty:
            pass
        if not self.closing:
            self.poll_timer = self.after(FRAME_INTERVAL_MS, self.poll)

    def apply_project(self, candidate, mark_dirty=True):
        # Build everything before committing, so invalid geometry is transactional.
        engine = Engine(candidate)
        self.project, self.engine = copy.deepcopy(candidate), engine
        self.revision += 1
        self.mapping_cache = {}
        self.dirty = mark_dirty
        self.project_label.configure(text=self.project["name"]+(" *" if self.dirty else ""))
        for panel in self.panels.values():
            panel.sync()
        self.active.request()

    def open_parameters(self):
        if self.editor is not None and self.editor.winfo_exists():
            self.editor.lift()
        else:
            self.editor = ParameterEditor(self)

    def save(self, save_as=False):
        path = None if save_as else self.path
        if path is None:
            path = filedialog.asksaveasfilename(parent=self, title="儲存 Suspension Lab 專案", defaultextension=".json", filetypes=[("JSON project", "*.json")])
        if not path:
            return False
        try:
            save_project(path, self.project)
            self.path, self.dirty = Path(path), False
            self.project_label.configure(text=self.project["name"])
            self.status.set(f"專案已儲存：{path}")
            return True
        except (OSError, ValueError) as exc:
            messagebox.showerror("儲存失敗", str(exc), parent=self)
            return False

    def confirm_unsaved(self):
        if not self.dirty:
            return True
        answer = messagebox.askyesnocancel("尚未儲存", "要先儲存目前專案嗎？", parent=self)
        return False if answer is None else self.save() if answer else True

    def open_project(self):
        if not self.confirm_unsaved():
            return
        path = filedialog.askopenfilename(parent=self, title="開啟 Suspension Lab 專案", filetypes=[("JSON project", "*.json")])
        if not path:
            return
        try:
            self.apply_project(load_project(path), False)
            self.path = Path(path)
        except (OSError, ValueError, ArithmeticError) as exc:
            messagebox.showerror("無法開啟專案", str(exc), parent=self)

    def export(self, kind):
        panel = self.active
        if kind == "mapping" and panel.key == "static":
            self.status.set("靜態全車請使用硬點 CSV；範圍掃描在 FV / SV / WE3D 頁。")
            return
        if not getattr(panel, "ready_for_export", False):
            self.status.set("正在計算目前姿態，完成後即可匯出。")
            return
        if kind in ("state", "points") and not panel.state:
            self.status.set("目前姿態無解；請調整滑桿後再匯出。")
            return
        if kind in ("png", "svg", "report") and panel.chart.spec is None:
            self.status.set("目前沒有可匯出的圖表。")
            return
        extension = {"state": ".csv", "points": ".csv", "mapping": ".csv", "png": ".png", "svg": ".svg", "report": ".html"}[kind]
        path = filedialog.asksaveasfilename(parent=self, defaultextension=extension,
                    initialfile=f"suspension_{panel.key}_{kind}{extension}", filetypes=[(extension, "*"+extension)])
        if not path:
            return
        if kind == "mapping" and panel.mapping is None:
            panel.request(force_mapping=True, pending_export=(kind, path))
        else:
            self.perform_export(kind, path, panel)

    def perform_export(self, kind, path, panel):
        try:
            if kind == "mapping":
                write_csv(path, panel.mapping["rows"])
            elif kind in ("state", "points"):
                rows = static_point_rows(self.engine) if panel.key == "static" else point_rows(panel.state) if kind == "points" else [panel.state["metrics"]]
                write_csv(path, rows)
            elif kind == "png":
                panel.chart.png(path)
            elif kind == "svg":
                Path(path).write_text(panel.chart.svg(), encoding="utf-8")
            else:
                controls = {"sliders": [v.get() for v in panel.controls], "mode": panel.mode.get(), "axle": panel.axle.get(), "plot": panel.plot_key}
                if panel.key == "static":
                    controls.update(static_references=panel.static_references.get(),
                                    static_groups={k: v.get() for k, v in panel.groups.items()})
                write_report(path, self.project, panel.key, panel.state, panel.mapping, panel.chart.svg(), controls)
            self.status.set(f"已匯出：{path}")
        except (OSError, ValueError) as exc:
            messagebox.showerror("匯出失敗", str(exc), parent=self)

    def close(self):
        if not self.confirm_unsaved():
            return
        self.closing = True
        for panel in self.panels.values():
            panel.token += 1
            if panel.timer:
                panel.after_cancel(panel.timer)
                panel.timer = None
        self.after_cancel(self.poll_timer)
        self.jobs.put(None)
        self.destroy()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Suspension Lab native geometry workbench")
    parser.add_argument("--project", type=Path, help="Open a Suspension Lab JSON project")
    args = parser.parse_args()
    app = App(load_project(args.project) if args.project else None)
    app.path = args.project
    app.mainloop()
