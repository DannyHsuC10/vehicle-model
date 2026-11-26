"""Desktop controls and result analysis for the 14 DOF lab."""

from copy import deepcopy
from datetime import datetime
import json
import os
from pathlib import Path
from queue import Empty, Queue
import re
from shutil import copy2
import subprocess
import sys
from threading import Thread
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import webbrowser

import numpy as np

from ._vendor.lapsim.aero import read_aero_coefficients
from ._vendor.lapsim.motor import load_motor_curve
from ._vendor.suspension.engine import G
from ._vendor.suspension.project import CORNERS

from .aero_setup import modal_defaults
from .driving_replay import DrivingReplay
from .geometry_tables import AXES, DELTA_OUTPUTS, OUTPUTS, TITLES, read_table, write_template
from .lap import DEFAULT_LATERAL_LIMIT_MPS2, DEFAULT_MAX_SPEED_MPS, DEFAULT_TRACK
from .model import default_desktop_config
from .resources import RESULT_GROUPS
from .suspension_editor import SuspensionEditor
from .workbench import LAB, RESULTS, Profile, Result, existing_results, number, run_arguments
from .workbench_charts import CHARTS, Chart


BACKGROUND = "#edf2f6"
INK = "#142b41"
MUTED = "#506579"
ACCENT = "#17656a"
MODES = ("圈速", "9.125 m 定圓", "直線加速")
PROGRESS = re.compile(r"\|\s*(\d+(?:\.\d+)?)%")
BODY = (
    ("m", "含駕駛總質量 (kg)"), ("h_cg", "重心高度 (m)"),
    ("lf", "重心至前軸 (m)"), ("lr", "重心至後軸 (m)"),
    ("tf", "前輪距 (m)"), ("tr", "後輪距 (m)"),
    ("Ixx", "簧上側傾慣量 (kg·m²)"), ("Iyy", "簧上俯仰慣量 (kg·m²)"),
)
POWER = (
    ("dc_voltage", "匯流排電壓 (V)"), ("gear_ratio", "減速比"),
    ("gear_efficiency", "傳動效率 (0–1)"),
)
HYDRAULIC_AXLES = (
    ("master_diameter_m", "總泵活塞直徑 (mm)", 1000., False),
    ("caliper_diameter_m", "卡鉗活塞直徑 (mm)", 1000., False),
    ("pistons", "每輪卡鉗活塞數", 1., True),
    ("disc_diameter_m", "碟盤有效摩擦直徑 (mm)", 1000., False),
)
AERO = (("reference_speed_kmh", "參考車速 (km/h)"),
        ("reference_downforce_n", "參考下壓力 (N)"), ("reference_drag_n", "參考阻力 (N)"))
AERO_MODES = {"modal": "姿態空力（LapSimLab）", "quadratic": "固定係數 v²"}
AERO_BASE = (
    ("rho", "空氣密度 (kg/m³)"), ("area", "參考面積 (m²)"),
    ("CL0", "基準升力係數 CL₀"), ("CD0", "基準阻力係數 CD₀"),
)
AERO_BALANCE = (
    ("front_balance", "基準前軸下壓比例 (0–1)"), ("cop_z", "基準作用點離地高度 (m)"),
    ("K_balance_pitch", "前後分配俯仰敏感度 (1/rad)"),
    ("K_balance_roll", "左右分配側傾敏感度 (1/rad)"),
)
AERO_SENSITIVITY = (
    ("Kh", "高度｜升力一次 Kh (1/m)"), ("Khh", "高度｜升力二次 Khh (1/m²)"),
    ("Kp", "俯仰｜升力一次 Kp (1/rad)"), ("Kpp", "俯仰｜升力二次 Kpp (1/rad²)"),
    ("Kr", "側傾｜升力一次 Kr (1/rad)"), ("Krr", "側傾｜升力二次 Krr (1/rad²)"),
    ("Kd_h", "高度｜阻力 Kd_h (1/m)"), ("Kd_p", "俯仰｜阻力 Kd_p (1/rad)"),
    ("Kd_r", "側傾｜阻力 Kd_r (1/rad)"),
)
WHEELS = (
    ("mu", "簧下質量 (kg)"), ("k", "垂向剛性 (N/mm)"), ("c", "垂向阻尼 (N·s/m)"),
    ("wheel_inertia_kg_m2", "自轉慣量 (kg·m²)"), ("tire_pressure_kpa", "胎壓 (kPa)"),
    ("static_camber_deg", "靜態外傾 (°)"), ("drive_scale", "扭矩倍率 (0–1)"),
)
GEOMETRY = (
    ("h_rc_f", "前 Roll Center (mm)", 1000.), ("h_rc_r", "後 Roll Center (mm)", 1000.),
    ("dive_front", "煞車｜前 Anti-dive (%)", 100.), ("lift_rear", "煞車｜後 Anti-lift (%)", 100.),
    ("lift_front", "加速｜前 Anti-lift (%)", 100.), ("squat_rear", "加速｜後 Anti-squat (%)", 100.),
)


def launch_path(path):
    if sys.platform == "win32":
        os.startfile(str(path))
    else:
        webbrowser.open(Path(path).as_uri())


class Workbench:
    def __init__(self, root, config_path=None):
        self.root = root
        self.profile = Profile.load(default_desktop_config() if config_path is None else config_path)
        self.result = None
        self.process = None
        self.cancelled = False
        self.events = Queue()
        self.result_paths = {}
        self.fields = {}
        self.source_vars = {}
        self.geometry_fields = {}
        self.gain_fields = {}
        self.geometry_entries = {}
        self.gain_entries = []
        self.geometry_tables = {}
        self.hydraulic_fields = {}
        self.run_fields = {}
        self.status = tk.StringVar(value="就緒")
        self.progress = tk.DoubleVar(value=0)
        self.profile_label = tk.StringVar()
        self.metric_vars = []
        root.title("Vehicle14DOFLab ｜ 模擬工作台")
        root.geometry("1240x820")
        root.minsize(1060, 700)
        root.configure(background=BACKGROUND)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.style()
        self.layout()
        self.fill_profile()
        self.refresh_results(load_latest=True)
        root.after(80, self.poll_events)

    def style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        self.root.option_add("*Font", ("Microsoft JhengHei UI", 10))
        style.configure(".", background=BACKGROUND, foreground=INK)
        style.configure("TEntry", padding=4, fieldbackground="white")
        style.configure("Compact.TEntry", padding=(4, 2))
        style.configure("TCombobox", padding=4, fieldbackground="white")
        style.configure("TButton", padding=(9, 5))
        style.configure("Run.TButton", background=ACCENT, foreground="white", padding=(14, 8))
        style.map("Run.TButton", background=[("active", "#20777b"), ("disabled", "#859ba6")])
        style.configure("TNotebook", borderwidth=0)
        style.configure("TNotebook.Tab", padding=(11, 7))
        style.map("TNotebook.Tab", background=[("selected", "white")])
        style.configure("Card.TFrame", background="white")
        style.configure("Small.TLabel", background="white", foreground=MUTED)
        style.configure("Value.TLabel", background="white", foreground=INK, font=("Segoe UI", 19, "bold"))
        style.configure("Heading.TLabel", font=("Microsoft JhengHei UI", 15, "bold"))
        style.configure("Horizontal.TProgressbar", background=ACCENT, troughcolor="#d4e0e8")

    def layout(self):
        header = tk.Frame(self.root, bg=INK, padx=20, pady=12)
        header.pack(fill="x")
        tk.Label(header, text="VEHICLE DYNAMICS  /  14 DOF", bg=INK, fg="white",
                 font=("Segoe UI", 17, "bold")).pack(side="left")
        tk.Label(header, textvariable=self.profile_label, bg=INK, fg="#b9d8df",
                 font=("Segoe UI", 10)).pack(side="right")
        workspace = ttk.Frame(self.root, padding=14)
        workspace.pack(fill="both", expand=True)
        workspace.columnconfigure(1, weight=1)
        workspace.rowconfigure(0, weight=1)
        left = ttk.Frame(workspace, width=435)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 12))
        left.grid_propagate(False)
        left.columnconfigure(0, weight=1)
        left.rowconfigure(2, weight=1)
        ttk.Label(left, text="模擬設定", style="Heading.TLabel").grid(row=0, column=0, sticky="w")
        actions = ttk.Frame(left, padding=(0, 6))
        actions.grid(row=1, column=0, sticky="ew")
        for label, command in (("開啟設定", self.open_profile), ("另存設定", self.save_profile),
                               ("載入 LP03", lambda: self.load_profile(default_desktop_config()))):
            ttk.Button(actions, text=label, command=command).pack(side="left", padx=(0, 4))
        tabs = ttk.Notebook(left)
        tabs.grid(row=2, column=0, sticky="nsew")
        pages = {}
        for title in ("工況", "車身", "動力／空力", "四輪", "懸吊", "懸吊幾何", "煞車"):
            page = ttk.Frame(tabs, padding=10)
            tabs.add(page, text=title)
            pages[title] = page
        self.scenario_page(pages["工況"])
        self.scalar_page(pages["車身"], "vehicle", BODY)
        self.scalar_page(pages["車身"], "config", (("yaw_inertia_kg_m2", "偏航等效慣量 (kg·m²)"),), len(BODY))
        self.power_page(pages["動力／空力"])
        self.wheel_page(pages["四輪"])
        self.suspension_editor = SuspensionEditor(pages["懸吊"])
        self.geometry_page(pages["懸吊幾何"])
        self.brake_page(pages["煞車"])
        run = ttk.Frame(left, padding=(0, 8, 0, 0))
        run.grid(row=3, column=0, sticky="ew")
        self.run_button = ttk.Button(run, text="執行模擬", command=self.start, style="Run.TButton")
        self.run_button.pack(side="left", fill="x", expand=True)
        self.stop_button = ttk.Button(run, text="停止", command=self.stop, state="disabled")
        self.stop_button.pack(side="left", padx=(8, 0))

        right = ttk.Frame(workspace)
        right.grid(row=0, column=1, sticky="nsew")
        self.result_layout(right)
        footer = ttk.Frame(self.root, padding=(15, 4, 15, 9))
        footer.pack(fill="x")
        ttk.Label(footer, textvariable=self.status).pack(side="left")
        ttk.Progressbar(footer, variable=self.progress, maximum=100, length=230).pack(side="right")

    @staticmethod
    def entry(parent, row, label, variable, *, column=0, width=15):
        ttk.Label(parent, text=label).grid(row=row, column=column, sticky="w", pady=2, padx=(0, 6))
        item = ttk.Entry(parent, textvariable=variable, width=width)
        item.grid(row=row, column=column + 1, sticky="ew", pady=2)
        return item

    def scenario_page(self, page):
        page.columnconfigure(1, weight=1)
        self.mode = tk.StringVar(value=MODES[0])
        ttk.Label(page, text="分析工況").grid(row=0, column=0, sticky="w")
        modes = ttk.Combobox(page, state="readonly", values=MODES, textvariable=self.mode)
        modes.grid(row=0, column=1, sticky="ew")
        modes.bind("<<ComboboxSelected>>", self.change_mode)
        defaults = (
            ("track", "賽道 CSV", str(DEFAULT_TRACK)),
            ("width", "賽道寬度 (m)", 3), ("laps", "計時圈數", 1), ("warmup", "暖胎圈數", 0),
            ("speed", "最高目標 (km/h)", 3.6 * DEFAULT_MAX_SPEED_MPS),
            ("lateral", "側向目標 (g)", round(DEFAULT_LATERAL_LIMIT_MPS2 / G, 6)),
            ("initial", "起步車速 (km/h)", 0), ("length", "直線距離 (m)", 75),
            ("step", "積分步長 (s)", .005), ("time_limit", "模擬時間上限 (s)", 240),
        )
        self.run_entries = {}
        for row, (key, label, value) in enumerate(defaults, 1):
            var = tk.StringVar(value=value)
            self.run_fields[key] = var
            self.run_entries[key] = self.entry(page, row, label, var)
        self.track_button = ttk.Button(page, text="瀏覽…", command=self.choose_track)
        self.track_button.grid(row=1, column=2, padx=(4, 0))
        self.change_mode()

    def change_mode(self, _event=None):
        mode = self.mode.get()
        for key, item in self.run_entries.items():
            active = key in ("initial", "step", "time_limit") or (
                key == "length" if mode == MODES[2] else key != "length" and (key != "track" or mode == MODES[0]))
            item.configure(state="normal" if active else "disabled")
        self.track_button.configure(state="normal" if mode == MODES[0] else "disabled")

    def scalar_page(self, page, group, specs, start=0):
        page.columnconfigure(1, weight=1)
        for row, (key, label) in enumerate(specs, start):
            var = tk.StringVar()
            self.fields[group, key] = (var, label)
            self.entry(page, row, label, var)

    def power_page(self, page):
        tabs = ttk.Notebook(page)
        tabs.pack(fill="both", expand=True)
        power = ttk.Frame(tabs, padding=8)
        aero = ttk.Frame(tabs, padding=8)
        tabs.add(power, text="動力")
        tabs.add(aero, text="空力")
        self.scalar_page(power, "config", POWER)
        row = len(POWER)
        motor_file = tk.StringVar()
        self.source_vars["motor_envelope"] = motor_file
        self.entry(power, row, "馬達曲線", motor_file)
        ttk.Button(power, text="匯入 CSV…", command=self.import_motor).grid(
            row=row, column=2, padx=(3, 0))
        ttk.Label(power, text="電壓包絡用 vdc_V,rpm,T_shaft_Nm。單條曲線用 rpm,T_shaft_Nm 或 speed,torque。"
                  "轉速用 RPM，減速前的軸端扭矩用 N·m。",
                  wraplength=330).grid(row=row + 1, column=0, columnspan=3, sticky="w", pady=(6, 0))
        self.aero_page(aero)

    def import_motor(self):
        var = self.source_vars["motor_envelope"]
        directory = Path(var.get().strip()).expanduser().parent
        selected = filedialog.askopenfilename(parent=self.root, title="匯入馬達扭矩曲線",
                                               initialdir=str(directory) if directory.is_dir() else None,
                                               filetypes=[("馬達曲線 CSV", "*.csv")])
        if not selected:
            return
        try:
            voltage, label = self.fields["config", "dc_voltage"]
            path = Path(selected)
            load_motor_curve(path, number(voltage.get(), label, minimum=1.))
        except (OSError, ValueError) as exc:
            messagebox.showerror("無法匯入馬達曲線", str(exc), parent=self.root)
            return
        var.set(str(path))
        self.status.set(f"已匯入馬達曲線：{path.name}")

    def aero_page(self, page):
        self.aero_enabled = tk.BooleanVar()
        self.aero_mode = tk.StringVar()
        page.columnconfigure(1, weight=1)
        ttk.Checkbutton(page, text="啟用空力", variable=self.aero_enabled).grid(
            row=0, column=0, sticky="w", padx=(0, 8))
        selector = ttk.Combobox(page, textvariable=self.aero_mode, values=list(AERO_MODES.values()),
                                state="readonly", width=23)
        selector.grid(row=0, column=1, sticky="ew")
        selector.bind("<<ComboboxSelected>>", self.refresh_aero)
        self.aero_fixed = ttk.Frame(page)
        self.aero_fixed.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        self.scalar_page(self.aero_fixed, "aero", AERO)
        self.aero_modal = ttk.Frame(page)
        self.aero_modal.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(self.aero_modal, text="匯入 LapSimLab 空力 JSON…",
                   command=self.import_aero).pack(anchor="w", pady=(0, 4))
        tabs = ttk.Notebook(self.aero_modal)
        tabs.pack(fill="x")
        for title, specs in (("基準", AERO_BASE), ("姿態係數", AERO_SENSITIVITY), ("作用點", AERO_BALANCE)):
            tab = ttk.Frame(tabs, padding=6)
            tabs.add(tab, text=title)
            self.scalar_page(tab, "aero_coeff", specs)
        ttk.Label(self.aero_modal, text="姿態以無空力的靜態車高為基準，CL 為負時產生下壓力。", wraplength=345).pack(
            anchor="w", pady=(5, 0))

    def refresh_aero(self, _event=None):
        if self.aero_mode.get() == AERO_MODES["modal"]:
            self.aero_fixed.grid_remove()
            self.aero_modal.grid()
        else:
            self.aero_modal.grid_remove()
            self.aero_fixed.grid()

    def import_aero(self):
        path = filedialog.askopenfilename(filetypes=[("LapSimLab 空力設定", "*.json")], parent=self.root)
        if path:
            try:
                coefficients = modal_defaults() | read_aero_coefficients(path)
                number(coefficients["rho"], "空氣密度", minimum=1e-9)
                number(coefficients["area"], "參考面積", minimum=1e-9)
                number(coefficients["front_balance"], "前軸下壓比例", minimum=0, maximum=1)
                for (group, key), (var, _) in self.fields.items():
                    if group == "aero_coeff":
                        var.set(coefficients[key])
            except (OSError, ValueError) as exc:
                messagebox.showerror("無法匯入空力設定", str(exc), parent=self.root)

    def wheel_page(self, page):
        tire_file = tk.StringVar()
        self.source_vars["tire_file"] = tire_file
        tire = ttk.LabelFrame(page, text="輪胎模型", padding=(7, 5))
        tire.grid(row=0, column=0, columnspan=5, sticky="ew", pady=(0, 6))
        tire.columnconfigure(0, weight=1)
        path = ttk.Entry(tire, textvariable=tire_file, width=20)
        path.grid(row=0, column=0, sticky="ew")
        ttk.Button(tire, text="選擇 TIR…", command=lambda: self.choose_file(tire_file, "*.tir")).grid(
            row=0, column=1, padx=(5, 0))
        ttk.Label(tire, text="四輪共用此 TIR。").grid(row=1, column=0, columnspan=2, sticky="w", pady=(3, 0))

        for j, corner in enumerate(CORNERS, 1):
            ttk.Label(page, text=corner).grid(row=1, column=j, pady=4)
            page.columnconfigure(j, weight=1)
        for i, (key, label) in enumerate(WHEELS, 2):
            ttk.Label(page, text=label).grid(row=i, column=0, sticky="w", padx=(0, 5))
            for j, corner in enumerate(CORNERS, 1):
                var = tk.StringVar()
                self.fields[corner, key] = (var, f"{corner} {label}")
                ttk.Entry(page, textvariable=var, width=7).grid(row=i, column=j, sticky="ew", padx=2, pady=3)
        radius = tk.StringVar()
        self.fields["config", "free_radius_m"] = (radius, "輪胎自由半徑 (m)")
        self.entry(page, len(WHEELS) + 2, "輪胎自由半徑 (m)", radius, width=7)
        ttk.Label(page, text="負載半徑由輪荷與垂向剛性計算。滾阻使用 SAE 近似式。",
                  wraplength=350).grid(row=len(WHEELS) + 3, column=0, columnspan=5, sticky="w", pady=(7, 0))

    def brake_page(self, page):
        page.columnconfigure(1, weight=1)
        self.brake_bias = tk.StringVar()
        self.brake_pedal_force = tk.StringVar()
        ttk.Label(page, text="液壓煞車").grid(row=0, column=0, sticky="w", pady=(0, 6))
        self.entry(page, 1, "前軸踏板力分配 (%)", self.brake_bias)
        self.entry(page, 2, "最大踏板力 (N)", self.brake_pedal_force)
        for key, label in (("pedal_ratio", "踏板比"), ("pad_mu", "來令片摩擦係數")):
            var = tk.StringVar()
            self.hydraulic_fields[key] = (var, label, 1., False)
            self.entry(page, len(self.hydraulic_fields) + 2, label, var)
        sizes = ttk.Frame(page)
        sizes.grid(row=5, columnspan=2, sticky="ew", pady=8)
        for i, axle in enumerate(("front", "rear"), 1):
            name = "前軸" if axle == "front" else "後軸"
            sizes.columnconfigure(i, weight=1)
            ttk.Label(sizes, text=name).grid(row=0, column=i, pady=(0, 3))
            for row, (key, label, scale, integer) in enumerate(HYDRAULIC_AXLES, 1):
                if i == 1:
                    ttk.Label(sizes, text=label).grid(row=row, column=0, sticky="w", padx=(0, 5))
                var = tk.StringVar()
                self.hydraulic_fields[f"{axle}_{key}"] = (var, f"{name} {label}", scale, integer)
                ttk.Entry(sizes, textvariable=var, width=8).grid(row=row, column=i, sticky="ew", padx=2, pady=2)
        ttk.Label(page, text="前軸分配是平衡桿的踏板力比例。活塞數填單一車輪卡鉗的總數。",
                  wraplength=380).grid(row=6, columnspan=2, sticky="w")

    def geometry_page(self, page):
        page.columnconfigure(1, weight=1)
        self.geometry_enabled = tk.BooleanVar()
        ttk.Checkbutton(page, text="啟用 Roll Center 與 Anti 傳力", variable=self.geometry_enabled).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 7))
        for row, (key, label, _) in enumerate(GEOMETRY, 1):
            var = tk.StringVar()
            self.geometry_fields[key] = var
            item = self.entry(page, row, label, var)
            item.configure(style="Compact.TEntry")
            self.geometry_entries[key] = item
        ttk.Separator(page).grid(row=7, column=0, columnspan=2, sticky="ew", pady=8)
        ttk.Label(page, text="Camber Gain (°／25 mm 壓縮)").grid(row=8, column=0, columnspan=2, sticky="w")
        wheels = ttk.Frame(page)
        wheels.grid(row=9, column=0, columnspan=2, sticky="ew")
        for i, corner in enumerate(CORNERS):
            wheels.columnconfigure(i, weight=1)
            ttk.Label(wheels, text=corner).grid(row=0, column=i)
            var = tk.StringVar()
            self.gain_fields[corner] = var
            item = ttk.Entry(wheels, textvariable=var, width=7, style="Compact.TEntry")
            item.grid(row=1, column=i, padx=3, pady=2, sticky="ew")
            self.gain_entries.append(item)
        ttk.Label(page, text="負值代表輪跳壓縮時輪頂往車內傾。靜態外傾可在「四輪」調整。",
                  wraplength=385).grid(row=10, column=0, columnspan=2, sticky="w", pady=(5, 0))
        self.geometry_status = tk.StringVar()
        ttk.Button(page, text="曲線／曲面 CSV…", command=self.geometry_import_dialog).grid(
            row=11, column=0, sticky="w", pady=(8, 0))
        ttk.Label(page, textvariable=self.geometry_status, wraplength=180).grid(
            row=11, column=1, sticky="w", padx=6, pady=(8, 0))

    def refresh_geometry_tables(self):
        for key, widget in self.geometry_entries.items():
            kind = "roll_center" if key.startswith("h_rc") else "anti"
            table = self.geometry_tables.get(kind)
            widget.configure(state="disabled" if table and not table.get("relative", False) else "normal")
        for widget in self.gain_entries:
            widget.configure(state="disabled" if "camber" in self.geometry_tables else "normal")
        names = "、".join(TITLES[kind] for kind in TITLES if kind in self.geometry_tables)
        self.geometry_status.set(names or "使用固定值／線性 Gain")

    def geometry_import_dialog(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("懸吊幾何查表")
        dialog.transient(self.root)
        dialog.grab_set()
        panel = ttk.Frame(dialog, padding=14)
        panel.pack(fill="both", expand=True)
        panel.columnconfigure(1, weight=1)
        kind_var = tk.StringVar(value=TITLES["roll_center"])
        form = tk.StringVar()
        output_mode = tk.StringVar(value="絕對值")
        source = tk.StringVar()
        example = tk.StringVar()
        explanation = tk.StringVar()
        ttk.Label(panel, text="幾何項目").grid(row=0, column=0, sticky="w", padx=(0, 7))
        kinds = ttk.Combobox(panel, values=list(TITLES.values()), textvariable=kind_var, state="readonly")
        kinds.grid(row=0, column=1, sticky="ew")
        ttk.Label(panel, text="範本查表軸").grid(row=1, column=0, sticky="w", padx=(0, 7))
        forms = ttk.Combobox(panel, textvariable=form, state="readonly")
        forms.grid(row=1, column=1, sticky="ew")
        ttk.Label(panel, text="範本輸出").grid(row=2, column=0, sticky="w", padx=(0, 7))
        output_modes = ttk.Combobox(panel, textvariable=output_mode, values=("絕對值", "相對變化量"), state="readonly")
        output_modes.grid(row=2, column=1, sticky="ew")

        def kind():
            return next(key for key, title in TITLES.items() if title == kind_var.get())

        def relative_template():
            return kind() in DELTA_OUTPUTS and output_mode.get() == "相對變化量"

        def show(*_):
            key = kind()
            options = [", ".join(axes) for axes in AXES[key]]
            forms.configure(values=options)
            if form.get() not in options:
                form.set(options[0])
            if key in DELTA_OUTPUTS:
                output_modes.configure(state="readonly")
                if output_mode.get() not in ("絕對值", "相對變化量"):
                    output_mode.set("絕對值")
            else:
                output_mode.set("角度變化量")
                output_modes.configure(state="disabled")
            columns = DELTA_OUTPUTS[key] if relative_template() else OUTPUTS[key]
            example.set(form.get() + ", " + ", ".join(columns))
            row = self.geometry_tables.get(key)
            source.set(f"作用中：{row.get('source_name', '已儲存表格')} · {len(row['rows'])} 列"
                       + (" · 相對變化量" if row.get("relative") else "")
                       if row else "目前沒有匯入此項目的表格")
            if key in ("camber", "bump_steer"):
                output = ("四輪相對靜態外傾的變化量，向外為正。" if key == "camber" else
                          "四輪相對名義舵角的修正量，向左為正。")
                explanation.set(output + " 輪跳用 mm，壓縮為正。steer_deg 用各輪 Ackermann 名義舵角，"
                                "front_steer_deg 用前軸等效舵角。舵角均向左為正。")
            else:
                explanation.set("行程用 mm，壓縮為正。travel_mm 用各軸平均輪跳。"
                                "roll_deg 正值表示車身左側抬高。steer_deg 用前軸等效舵角，向左為正。"
                                "相對變化量會加到頁面設定的基準值。")

        for row, var in ((3, source), (4, explanation), (5, example)):
            ttk.Label(panel, textvariable=var, wraplength=530, justify="left").grid(
                row=row, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Label(panel, text="第一列為欄名。雙軸資料須填滿矩形網格，不可留空或重複座標。超出表格使用邊界值。",
                  wraplength=530).grid(row=6, column=0, columnspan=2, sticky="w", pady=8)
        actions = ttk.Frame(panel)
        actions.grid(row=7, column=0, columnspan=2, sticky="ew")

        def load():
            path = filedialog.askopenfilename(parent=dialog, filetypes=[("幾何 CSV", "*.csv")])
            if path:
                try:
                    self.geometry_tables[kind()] = read_table(path, kind())
                except (OSError, ValueError) as error:
                    messagebox.showerror("無法匯入幾何", str(error), parent=dialog)
                    return
                self.refresh_geometry_tables()
                show()

        def clear():
            self.geometry_tables.pop(kind(), None)
            self.refresh_geometry_tables()
            show()

        def template():
            path = filedialog.asksaveasfilename(parent=dialog, defaultextension=".csv",
                                                initialfile=kind() + "_template.csv", filetypes=[("CSV", "*.csv")])
            if path:
                try:
                    write_template(path, kind(), tuple(form.get().split(", ")), relative=relative_template())
                except OSError as error:
                    messagebox.showerror("無法儲存範本", str(error), parent=dialog)

        for label, command in (("匯入 CSV", load), ("儲存範本", template), ("移除此項查表", clear)):
            ttk.Button(actions, text=label, command=command).pack(side="left", padx=(0, 4))
        ttk.Button(actions, text="完成", command=dialog.destroy).pack(side="right")
        kinds.bind("<<ComboboxSelected>>", show)
        forms.bind("<<ComboboxSelected>>", show)
        output_modes.bind("<<ComboboxSelected>>", show)
        show()

    def result_layout(self, panel):
        ttk.Label(panel, text="分析結果", style="Heading.TLabel").pack(anchor="w")
        selector = ttk.Frame(panel, padding=(0, 6))
        selector.pack(fill="x")
        self.result_name = tk.StringVar()
        self.result_select = ttk.Combobox(selector, textvariable=self.result_name, state="readonly")
        self.result_select.pack(side="left", fill="x", expand=True)
        self.result_select.bind("<<ComboboxSelected>>", lambda event: self.open_result(self.result_paths[self.result_name.get()]))
        ttk.Button(selector, text="開啟 CSV", command=self.choose_result).pack(side="left", padx=(5, 0))
        ttk.Button(selector, text="結果資料夾", command=self.open_result_folder).pack(side="left", padx=(5, 0))
        cards = ttk.Frame(panel)
        cards.pack(fill="x", pady=(3, 8))
        for i, title in enumerate(("最快圈／耗時", "最高車速", "側向加速度峰值")):
            cards.columnconfigure(i, weight=1, uniform="metrics")
            card = ttk.Frame(cards, style="Card.TFrame", padding=(12, 8))
            card.grid(row=0, column=i, sticky="ew", padx=(0, 5 if i < 2 else 0))
            ttk.Label(card, text=title, style="Small.TLabel").pack(anchor="w")
            var = tk.StringVar(value="—")
            self.metric_vars.append(var)
            ttk.Label(card, textvariable=var, style="Value.TLabel").pack(anchor="w")
        self.result_tabs = ttk.Notebook(panel)
        self.result_tabs.pack(fill="both", expand=True)
        graphs = ttk.Frame(self.result_tabs, style="Card.TFrame")
        self.result_tabs.add(graphs, text="圖表")
        toolbar = ttk.Frame(graphs, style="Card.TFrame", padding=6)
        toolbar.pack(fill="x")
        chart_name = tk.StringVar(value=next(iter(CHARTS)))
        charts = ttk.Combobox(toolbar, textvariable=chart_name, values=list(CHARTS), state="readonly", width=15)
        charts.pack(side="left")
        ttk.Button(toolbar, text="開啟已匯出圖檔", command=lambda: self.open_figure(chart_name.get())).pack(side="right")
        self.chart = Chart(graphs)
        self.chart.pack(fill="both", expand=True)
        charts.bind("<<ComboboxSelected>>", lambda event: self.chart.select(chart_name.get()))

        self.replay = DrivingReplay(self.result_tabs)
        self.result_tabs.add(self.replay, text="駕駛動畫")
        timings = ttk.Frame(self.result_tabs, padding=8)
        self.result_tabs.add(timings, text="計時")
        self.table = ttk.Treeview(timings, show="headings", columns=("lap", "time", "start", "finish"))
        for key, title in zip(("lap", "time", "start", "finish"), ("圈次", "耗時 (s)", "起點 (s)", "終點 (s)")):
            self.table.heading(key, text=title)
            self.table.column(key, width=85, anchor="center")
        scroll = ttk.Scrollbar(timings, command=self.table.yview)
        self.table.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.table.pack(side="left", fill="both", expand=True)
        self.console = ttk.Frame(self.result_tabs)
        self.result_tabs.add(self.console, text="執行紀錄")
        self.log = tk.Text(self.console, wrap="word", state="disabled", bg="#f8fafb", fg=INK,
                           relief="flat", padx=10, pady=10)
        scrollbar = ttk.Scrollbar(self.console, command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.log.pack(fill="both", expand=True)

    def choose_file(self, var, pattern):
        directory = Path(var.get().strip()).expanduser().parent
        path = filedialog.askopenfilename(parent=self.root, initialdir=str(directory) if directory.is_dir() else None,
                                          filetypes=[(pattern, pattern), ("所有檔案", "*.*")])
        if path:
            var.set(path)

    def choose_track(self):
        self.choose_file(self.run_fields["track"], "*.csv")

    def open_profile(self):
        path = filedialog.askopenfilename(filetypes=[("車輛設定", "*.json")])
        if path:
            self.load_profile(path)

    def load_profile(self, path):
        try:
            self.profile = Profile.load(path)
            self.fill_profile()
        except (OSError, ValueError, KeyError, TypeError) as exc:
            messagebox.showerror("無法載入設定", str(exc), parent=self.root)

    def fill_profile(self):
        profile = self.profile
        label = profile.path.parent.name if profile.path.name == "vehicle.json" else profile.path.stem
        self.profile_label.set(f"{label}  ·  {profile.suspension['name']}")
        for (group, key), (var, _) in self.fields.items():
            if group == "config":
                value = profile.vehicle[key]
            elif group == "aero":
                value = profile.vehicle["aero"][key]
            elif group == "aero_coeff":
                value = profile.vehicle["aero"]["coefficients"][key]
            elif group == "vehicle":
                value = profile.suspension["vehicle"][key]
            elif key in ("mu", "k", "c"):
                value = profile.suspension["tires"][group][key]
            else:
                value = profile.vehicle[key][CORNERS.index(group)]
            var.set(value)
        for key, var in self.source_vars.items():
            var.set(profile.vehicle[key])
        self.aero_enabled.set(profile.vehicle["aero"]["enabled"])
        self.aero_mode.set(AERO_MODES[profile.vehicle["aero"]["model"]])
        self.refresh_aero()
        self.brake_bias.set(profile.vehicle["front_brake_bias"] * 100.)
        self.brake_pedal_force.set(profile.vehicle["max_pedal_force_n"])
        for key, (var, _, scale, integer) in self.hydraulic_fields.items():
            value = profile.vehicle["hydraulic_brake"][key]
            var.set(value if integer else value * scale)
        geometry = profile.vehicle.get("suspension_geometry", {})
        self.geometry_enabled.set(geometry.get("load_transfer_enabled", False))
        for key, _, scale in GEOMETRY:
            self.geometry_fields[key].set(profile.suspension["vehicle"][key] * scale)
        gains = geometry.get("camber_gain_deg_per_25mm", [0.] * 4)
        for corner, gain in zip(CORNERS, gains):
            self.gain_fields[corner].set(gain)
        self.geometry_tables = deepcopy(geometry.get("tables", {}))
        self.refresh_geometry_tables()
        self.suspension_editor.fill(profile.suspension["channels"])

    def capture(self):
        snapshot = deepcopy(self.profile)
        snapshot.suspension["channels"] = self.suspension_editor.capture()
        mode = "modal" if self.aero_mode.get() == AERO_MODES["modal"] else "quadratic"
        aero = snapshot.vehicle["aero"]
        aero["model"], aero["enabled"] = mode, self.aero_enabled.get()
        for (group, key), (var, label) in self.fields.items():
            if (group == "aero" and mode == "modal") or (group == "aero_coeff" and mode == "quadratic"):
                continue
            value = number(var.get(), label)
            if group == "config":
                snapshot.vehicle[key] = value
            elif group == "aero":
                aero[key] = value
            elif group == "aero_coeff":
                aero["coefficients"][key] = value
            elif group == "vehicle":
                snapshot.suspension["vehicle"][key] = value
            elif key in ("mu", "k", "c"):
                snapshot.suspension["tires"][group][key] = value
            else:
                snapshot.vehicle[key][CORNERS.index(group)] = value
        for key, var in self.source_vars.items():
            path = Path(var.get().strip()).expanduser().resolve()
            if not path.is_file():
                raise ValueError(f"找不到資料檔：{path}")
            snapshot.vehicle[key] = str(path)
        snapshot.vehicle["front_brake_bias"] = number(self.brake_bias.get(), "前軸踏板力分配", minimum=0., maximum=100.) / 100.
        snapshot.vehicle["max_pedal_force_n"] = number(self.brake_pedal_force.get(), "最大踏板力", minimum=0.)
        for key, (var, label, scale, integer) in self.hydraulic_fields.items():
            minimum = 1. if integer or scale == 1000. else (.001 if key == "pedal_ratio" else 0.)
            value = number(var.get(), label, minimum=minimum, integer=integer)
            snapshot.vehicle["hydraulic_brake"][key] = value if integer else value / scale
        for key, label, scale in GEOMETRY:
            kind = "roll_center" if key.startswith("h_rc") else "anti"
            table = self.geometry_tables.get(kind)
            if not table or table.get("relative", False):
                snapshot.suspension["vehicle"][key] = number(self.geometry_fields[key].get(), label) / scale
        geometry = snapshot.vehicle.setdefault("suspension_geometry", {})
        geometry.update(
            load_transfer_enabled=self.geometry_enabled.get(),
            tables=deepcopy(self.geometry_tables),
        )
        if "camber" not in self.geometry_tables:
            geometry["camber_gain_deg_per_25mm"] = [
                number(self.gain_fields[corner].get(), f"{corner} Camber Gain", minimum=-30., maximum=30.)
                for corner in CORNERS
            ]
        return snapshot

    def save_profile(self):
        parent = filedialog.askdirectory(title="選擇新設定資料夾的存放位置", initialdir=LAB)
        if parent:
            try:
                profile = self.capture()
                path = profile.save(Path(parent) / datetime.now().strftime("LP03_%Y%m%d_%H%M%S_%f"))
                self.profile = Profile.load(path)
                self.fill_profile()
                self.status.set(f"已儲存設定：{path}")
            except (OSError, ValueError, KeyError, TypeError) as exc:
                messagebox.showerror("無法儲存設定", str(exc), parent=self.root)

    def start(self):
        if self.process is not None:
            return
        try:
            values = {key: var.get() for key, var in self.run_fields.items()}
            args = run_arguments(self.mode.get(), values)
            folder = RESULTS / RESULT_GROUPS[self.mode.get()] / datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")
            config = self.capture().save(folder / "inputs")
            if "--track" in args:
                field = args.index("--track") + 1
                source = Path(args[field])
                track = Path("inputs") / "track" / source.name
                (folder / track.parent).mkdir()
                copy2(source, folder / track)
                args[field] = track.as_posix()
            output = folder / "telemetry.csv"
            (folder / "run.json").write_text(json.dumps({"mode": self.mode.get(), "arguments": args},
                                                       ensure_ascii=False, indent=2), encoding="utf-8")
            self.process = subprocess.Popen(
                [sys.executable, "-B", "-X", "utf8", str(LAB / "run.py"), "--config", str(config),
                 "--output", str(output), *args],
                cwd=folder, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                errors="replace", creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        except (OSError, TypeError, ValueError, KeyError) as exc:
            messagebox.showerror("無法啟動模擬", str(exc), parent=self.root)
            return
        self.cancelled = False
        self.run_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.progress.set(0)
        self.status.set("模擬啟動中")
        self.append_log(f"\n{folder.name} · {self.mode.get()}")
        self.result_tabs.select(self.console)
        Thread(target=self.read_process, args=(self.process, output), daemon=True).start()

    def read_process(self, process, output):
        with process.stdout:
            for line in process.stdout:
                self.events.put(("line", line.rstrip()))
        self.events.put(("done", (process.wait(), output)))

    def append_log(self, line):
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def poll_events(self):
        try:
            while True:
                event, payload = self.events.get_nowait()
                if event == "line":
                    self.append_log(payload)
                    if match := PROGRESS.search(payload):
                        self.progress.set(float(match.group(1)))
                        self.status.set(payload)
                else:
                    code, output = payload
                    self.process = None
                    self.run_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
                    if code == 0 and self.open_result(output):
                        self.refresh_results()
                        self.progress.set(100)
                        self.status.set("模擬完成，結果已儲存")
                        self.result_tabs.select(0)
                    else:
                        self.status.set("模擬已停止" if self.cancelled else "模擬失敗，請查看執行紀錄")
        except Empty:
            pass
        self.root.after(80, self.poll_events)

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.cancelled = True
            self.process.terminate()
            self.stop_button.configure(state="disabled")
            self.status.set("停止模擬中")

    def close(self):
        if self.process is not None and self.process.poll() is None:
            if not messagebox.askyesno("結束工作台", "模擬正在執行。停止模擬並關閉工作台？", parent=self.root):
                return
            self.stop()
        self.replay.pause()
        self.root.destroy()

    def refresh_results(self, *, load_latest=False):
        paths = existing_results()
        self.result_paths = {str(path.relative_to(LAB)): path for path in paths}
        self.result_select.configure(values=list(self.result_paths))
        if load_latest and paths:
            self.open_result(paths[0])

    def choose_result(self):
        path = filedialog.askopenfilename(filetypes=[("14 DOF CSV", "*.csv")], initialdir=RESULTS)
        if path:
            self.open_result(path)

    def open_result(self, path):
        try:
            result = Result.load(path)
            timing = []
            for row in result.timing:
                if "lap_time_s" in row:
                    timing.append((row["lap"], float(row["lap_time_s"]), float(row["start_time_s"]),
                                   float(row["finish_time_s"])))
                elif "elapsed_time_s" in row:
                    time = float(row["elapsed_time_s"])
                    timing.append(("直線", time, 0., time))
            data = result.data
            values = (f"{min(row[1] for row in timing):.3f} s" if timing else "—",
                      f"{3.6 * np.max(data['speed_mps']):.2f} km/h",
                      f"{np.max(np.abs(data['ay_mps2'])) / G:.2f} g")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            messagebox.showerror("無法開啟結果", str(exc), parent=self.root)
            return False
        self.result = result
        label = str(result.path.relative_to(LAB)) if result.path.is_relative_to(LAB) else result.path.name
        self.result_name.set(label)
        self.chart.set_result(result)
        self.replay.set_result(result)
        self.table.delete(*self.table.get_children())
        for row in timing:
            self.table.insert("", "end", values=(row[0], *(f"{value:.3f}" for value in row[1:])))
        for var, value in zip(self.metric_vars, values):
            var.set(value)
        return True

    def open_result_folder(self):
        self.open_path(self.result.path.parent if self.result else LAB)

    def open_path(self, path):
        try:
            launch_path(path)
        except OSError as exc:
            messagebox.showerror("無法開啟檔案", str(exc), parent=self.root)

    def open_figure(self, name):
        if self.result is None:
            return
        suffix = {"速度地圖": "speed_map", "車速": "speed_time",
                  "加速度": "acceleration_time", "空力": "aero_time",
                  "空力作用點": "aero_cop_time"}.get(name)
        if suffix:
            file = self.result.path.with_name(f"{self.result.path.stem}_{suffix}.svg")
            if file.is_file():
                self.open_path(file)
                return
        self.status.set("此訊號可在工作台查看。這筆結果沒有對應的 SVG。")


def main(config_path=None):
    root = tk.Tk()
    Workbench(root, config_path)
    root.mainloop()


if __name__ == "__main__":
    main()
