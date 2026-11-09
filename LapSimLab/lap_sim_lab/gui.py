"""Two-pane Tk workbench for configuring and reviewing one LapSim run."""

from pathlib import Path
from queue import Empty, SimpleQueue
import threading
import tkinter as tk
from tkinter import filedialog, ttk

import numpy as np

from .analysis_gui import MotorPage, SweepPage
from .charts import FONT, ResultChart
from .planner import GRAVITY
from .settings import Settings
from .signal_view import SignalView
from .simulation import simulate, write_csv


SIMULATION_FIELDS = (
    ("track_width", "賽道寬度", "m"),
    ("step_m", "取樣間距", "m"),
    ("max_speed", "速度上限", "m/s"),
    ("safety_factor", "側向安全係數", "0–1"),
    ("passes", "最大前後向掃描次數", "次"),
)
VEHICLE_FIELDS = (
    ("mass", "總質量（含車手）", "kg"),
    ("wheelbase", "軸距", "m"),
    ("cg_from_front", "重心距前軸", "m"),
    ("cg_height", "重心高度", "m"),
    ("front_track", "前輪距", "m"),
    ("rear_track", "後輪距", "m"),
    ("tire_radius", "輪胎自由半徑", "m"),
    ("tire_stiffness", "輪胎垂直剛性", "N/m"),
    ("front_heave_stiffness", "前軸 Heave 剛性", "N/m"),
    ("front_roll_stiffness", "前軸 Roll 剛性（每輪）", "N/m"),
    ("rear_stiffness_ratio", "後／前剛性比", "比值"),
)
POWER_FIELDS = (
    ("gear_ratio", "固定減速比", "比值"),
    ("gear_efficiency", "傳動效率", "0–1"),
    ("front_torque_scale", "前輪限扭比例", "0–1"),
    ("rear_torque_scale", "後輪限扭比例", "0–1"),
    ("front_brake_bias", "前軸煞車比例", "0–1"),
    ("brake_leverage", "煞車機械增益", "比值"),
    ("pedal_force", "踏板力", "N"),
)
FILE_FIELDS = (
    ("track_path", "賽道分段", ".csv"),
    ("tire_path", "輪胎模型", ".json"),
    ("motor_path", "馬達曲線", ".csv"),
    ("aero_path", "空力係數", ".json"),
)

SURFACE = "#f1f5f9"
WHITE = "#ffffff"
INK = "#17253a"
MUTED = "#52657b"


class LapSimLab(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("LapSimLab — 圈速模擬")
        self.geometry("1280x790")
        self.minsize(1040, 665)
        self.configure(bg=SURFACE)
        self.default = Settings()
        self.paths = {key: getattr(self.default, key) for key, _, _ in FILE_FIELDS}
        self.file_vars = {key: tk.StringVar(value=path.name) for key, path in self.paths.items()}
        self.entries = {}
        self.aero_enabled = tk.BooleanVar(value=self.default.aero_enabled)
        self.status = tk.StringVar(value="已載入內附資料，可直接計算。")
        self.result_title = tk.StringVar(value="結果預覽")
        self.result = None
        self.result_revision = None
        self.revision = 0
        self.busy = False
        self.events = SimpleQueue()

        self.configure_styles()
        self.build_layout()
        self.aero_enabled.trace_add("write", self.inputs_changed)
        self.bind("<F5>", lambda _event: self.calculate())

    def configure_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", font=(FONT, 10), foreground=INK, background=SURFACE)
        style.configure("TNotebook", borderwidth=0)
        style.configure("TNotebook.Tab", padding=(12, 7))
        style.map("TNotebook.Tab", background=[("selected", WHITE)])
        style.configure("Panel.TFrame", background=WHITE)
        style.configure("Panel.TLabel", background=WHITE)
        style.configure("Panel.TCheckbutton", background=WHITE)
        style.map("Panel.TCheckbutton", background=[("active", WHITE)])
        style.configure("Header.TLabel", font=(FONT, 22, "bold"), foreground=INK)
        style.configure("Section.TLabel", font=(FONT, 12, "bold"), foreground=INK)
        style.configure("Muted.TLabel", foreground=MUTED)
        style.configure("Hint.TLabel", background=WHITE, foreground=MUTED, font=(FONT, 9))
        style.configure("Metric.TLabel", background=WHITE, font=(FONT, 17, "bold"), foreground="#16385e")
        style.configure("Primary.TButton", background="#1d4ed8", foreground=WHITE,
                        bordercolor="#1d4ed8", padding=(10, 7), font=(FONT, 11, "bold"))
        style.map("Primary.TButton", background=[("disabled", "#94a3b8"), ("active", "#1e40af")],
                  foreground=[("disabled", WHITE)])

    def build_layout(self):
        header = ttk.Frame(self, padding=(19, 12, 19, 8))
        header.pack(fill="x")
        ttk.Label(header, text="LapSimLab", style="Header.TLabel").pack(side="left")
        ttk.Label(header, text="FSAE  圈速模擬", style="Muted.TLabel").pack(side="left", padx=14)

        body = ttk.Frame(self, padding=(19, 2, 19, 0))
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        controls = ttk.Frame(body)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 17))
        ttk.Label(controls, text="模擬輸入", style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        tabs = ttk.Notebook(controls)
        tabs.pack(fill="both", expand=True)

        simulation = self.input_tab(tabs, "模擬與資料", SIMULATION_FIELDS)
        row = len(SIMULATION_FIELDS)
        ttk.Checkbutton(simulation, text="啟用空力", variable=self.aero_enabled,
                        style="Panel.TCheckbutton").grid(row=row, columnspan=3, sticky="w", pady=(4, 9))
        ttk.Separator(simulation).grid(row=row + 1, columnspan=3, sticky="ew", pady=(0, 5))
        for offset, (key, label, suffix) in enumerate(FILE_FIELDS, start=row + 2):
            ttk.Label(simulation, text=f"{label}  {suffix}", style="Panel.TLabel").grid(
                row=offset, column=0, sticky="w", pady=3
            )
            ttk.Entry(simulation, textvariable=self.file_vars[key], state="readonly", width=17).grid(
                row=offset, column=1, sticky="ew", padx=(10, 5), pady=3
            )
            ttk.Button(simulation, text="選擇", width=5,
                       command=lambda k=key, s=suffix: self.choose_file(k, s)).grid(
                row=offset, column=2, sticky="ew", pady=3
            )
        ttk.Label(simulation, text="預設資料位於本工具的 input_data 資料夾。",
                  style="Hint.TLabel", wraplength=285).grid(
            row=row + 6, columnspan=3, sticky="w", pady=(9, 0)
        )
        vehicle = self.input_tab(tabs, "車輛與懸吊", VEHICLE_FIELDS)
        ttk.Label(vehicle, text="後軸 Roll 與 Heave 都由各自的前軸剛性乘上同一個剛性比。",
                  style="Hint.TLabel", wraplength=285).grid(
            row=len(VEHICLE_FIELDS), columnspan=3, sticky="w", pady=(8, 0)
        )
        power = self.input_tab(tabs, "傳動與煞車", POWER_FIELDS)
        ttk.Label(power, text="前後軸各用同一張馬達曲線，限扭比例套用到該軸兩輪。",
                  style="Hint.TLabel", wraplength=285).grid(
            row=len(POWER_FIELDS), columnspan=3, sticky="w", pady=(8, 0)
        )

        buttons = ttk.Frame(controls)
        buttons.pack(fill="x", pady=(11, 0))
        self.run_button = ttk.Button(buttons, text="計算  F5", style="Primary.TButton", command=self.calculate)
        self.run_button.pack(side="left", fill="x", expand=True)
        ttk.Button(buttons, text="恢復預設", command=self.reset).pack(side="left", padx=(8, 0))

        self.result_pages = ttk.Notebook(body)
        self.result_pages.grid(row=0, column=1, sticky="nsew")
        results = ttk.Frame(self.result_pages, padding=(0, 7, 0, 0))
        self.result_pages.add(results, text="圈速結果")
        result_bar = ttk.Frame(results)
        result_bar.pack(fill="x", pady=(0, 8))
        ttk.Label(result_bar, textvariable=self.result_title, style="Section.TLabel").pack(side="left")
        self.export_button = ttk.Button(result_bar, text="匯出完整 CSV", command=self.export, state="disabled")
        self.export_button.pack(side="right")

        summary = ttk.Frame(results)
        summary.pack(fill="x", pady=(0, 10))
        self.metrics = []
        for index, label in enumerate(("預估圈速", "取樣路徑長度", "最高車速", "最大橫向加速度")):
            summary.columnconfigure(index, weight=1, uniform="metric")
            card = ttk.Frame(summary, style="Panel.TFrame", padding=(12, 7))
            card.grid(row=0, column=index, sticky="ew", padx=(0, 6 if index < 3 else 0))
            ttk.Label(card, text=label, style="Hint.TLabel").pack(anchor="w")
            value = tk.StringVar(value="—")
            self.metrics.append(value)
            ttk.Label(card, textvariable=value, style="Metric.TLabel").pack(anchor="w")

        views = ttk.Notebook(results)
        views.pack(fill="both", expand=True)
        self.charts = []
        for kind, label in (
            ("map", "賽道速度圖"), ("speed", "速度曲線"), ("accel", "加速度"),
            ("loads", "四輪輪荷"), ("attitude", "車身姿態"), ("aero", "空力"),
        ):
            chart = ResultChart(views, kind)
            views.add(chart, text=label)
            self.charts.append(chart)

        def read_vehicle():
            return self.read_settings(vehicle_only=True)

        self.aero_page = SweepPage(self.result_pages, read_vehicle)
        self.motor_page = MotorPage(self.result_pages, lambda: self.paths["motor_path"])
        self.tire_page = SweepPage(self.result_pages, read_vehicle, tire=True)
        self.signal_page = SignalView(self.result_pages, self.export)
        for page, label in (
            (self.signal_page, "訊號檢視"),
            (self.aero_page, "空力掃描"), (self.motor_page, "馬達曲線"), (self.tire_page, "輪胎力掃描"),
        ):
            self.result_pages.add(page, text=label)
        self.result_pages.bind("<<NotebookTabChanged>>", self.refresh_analysis)

        footer = ttk.Frame(self, padding=(19, 9, 19, 11))
        footer.pack(fill="x")
        self.status_label = ttk.Label(footer, textvariable=self.status, style="Muted.TLabel", wraplength=900)
        self.status_label.pack(side="left", fill="x", expand=True)
        self.progress = ttk.Progressbar(footer, mode="indeterminate", length=140)

    def refresh_analysis(self, _event=None):
        page = self.nametowidget(self.result_pages.select())
        if page in (self.aero_page, self.motor_page, self.tire_page) and page.dirty:
            page.refresh()

    def input_tab(self, notebook, title, parameters):
        panel = ttk.Frame(notebook, style="Panel.TFrame", padding=(12, 9))
        panel.columnconfigure(1, weight=1)
        notebook.add(panel, text=title)
        for row, (key, label, unit) in enumerate(parameters):
            ttk.Label(panel, text=label, style="Panel.TLabel").grid(row=row, column=0, sticky="w", pady=3)
            var = tk.StringVar(value=str(getattr(self.default, key)))
            var.trace_add("write", self.inputs_changed)
            self.entries[key] = var
            ttk.Entry(panel, textvariable=var, width=16).grid(row=row, column=1, sticky="ew", padx=(10, 5), pady=3)
            ttk.Label(panel, text=unit, style="Hint.TLabel").grid(row=row, column=2, sticky="w")
        return panel

    def inputs_changed(self, *_):
        self.revision += 1
        self.aero_page.invalidate()
        self.tire_page.invalidate()
        self.export_button.state(["disabled"])
        self.signal_page.mark_stale()
        if self.result is not None:
            self.result_title.set("上次計算結果")
        if not self.busy:
            self.status.set("設定已變更，按「計算」更新結果。")

    def choose_file(self, key, suffix):
        path = filedialog.askopenfilename(
            parent=self, initialdir=self.paths[key].parent,
            filetypes=[(suffix[1:].upper(), f"*{suffix}"), ("所有檔案", "*.*")],
        )
        if path:
            self.paths[key] = Path(path)
            self.file_vars[key].set(self.paths[key].name)
            self.inputs_changed()
            if key == "motor_path":
                self.motor_page.invalidate()
                if self.result_pages.select() == str(self.motor_page):
                    self.motor_page.refresh()

    def reset(self):
        for key, var in self.entries.items():
            var.set(str(getattr(self.default, key)))
        self.aero_enabled.set(self.default.aero_enabled)
        for key, var in self.file_vars.items():
            self.paths[key] = getattr(self.default, key)
            var.set(self.paths[key].name)
        self.motor_page.invalidate()
        if self.result_pages.select() == str(self.motor_page):
            self.motor_page.refresh()

    def read_settings(self, *, vehicle_only=False):
        values = dict(self.paths, aero_enabled=self.aero_enabled.get())
        fields = VEHICLE_FIELDS if vehicle_only else (*SIMULATION_FIELDS, *VEHICLE_FIELDS, *POWER_FIELDS)
        for key, label, _ in fields:
            try:
                parse = type(getattr(self.default, key))
                values[key] = parse(self.entries[key].get())
            except ValueError as exc:
                raise ValueError(f"請檢查「{label}」的數值。") from exc
        settings = Settings(**values)
        settings.validate()
        return settings

    def calculate(self):
        if self.busy:
            return
        try:
            settings = self.read_settings()
        except ValueError as exc:
            self.status.set(f"輸入有誤。{exc}")
            return
        self.busy = True
        revision = self.revision
        self.run_button.state(["disabled"])
        self.export_button.state(["disabled"])
        self.signal_page.mark_stale()
        if self.result is not None:
            self.result_title.set("上次計算結果")
        self.status.set("正在計算…")
        self.progress.pack(side="right", padx=(8, 0))
        self.progress.start(18)

        def work():
            try:
                result = simulate(settings, progress=lambda text: self.events.put(("progress", text)))
            except Exception as exc:
                self.events.put(("error", exc))
            else:
                self.events.put(("result", (revision, settings.track_path.stem, result)))

        threading.Thread(target=work, daemon=True).start()
        self.after(50, self.poll)

    def poll(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except Empty:
                self.after(50, self.poll)
                return
            if kind == "progress":
                self.status.set(payload + "…")
                continue
            self.busy = False
            self.progress.stop()
            self.progress.pack_forget()
            self.run_button.state(["!disabled"])
            if kind == "error":
                self.result_revision = None
                self.status.set(f"計算失敗。{payload}")
            else:
                self.show_result(*payload)
            if self.result_revision == self.revision:
                self.export_button.state(["!disabled"])
            return

    def show_result(self, revision, track_name, result):
        self.result = result
        self.result_revision = revision
        updated = self.revision == revision
        self.result_title.set("模擬結果" if updated else "上次計算結果")
        for variable, value in zip(self.metrics, (
            f"{result.lap_time:.3f} s",
            f"{result.track.s[-1]:,.1f} m",
            f"{np.max(result.plan.speed) * 3.6:.1f} km/h",
            f"{np.max(np.abs(result.lateral_acceleration)) / GRAVITY:.2f} g",
        )):
            variable.set(value)
        for chart in self.charts:
            chart.set_result(result)
        self.signal_page.set_result(result, stale=not updated)
        text = f"{track_name}  ·  {len(result.time):,} 個取樣點"
        self.status.set(text if updated else f"{text}。輸入已變更，請重新計算。")

    def export(self):
        if self.busy or self.result is None or self.result_revision != self.revision:
            return
        path = filedialog.asksaveasfilename(
            parent=self, initialfile="lap_result.csv", defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
        )
        if path:
            try:
                write_csv(Path(path), self.result)
            except OSError as exc:
                self.status.set(f"無法匯出。{exc}")
            else:
                self.status.set(f"已匯出 {path}")
