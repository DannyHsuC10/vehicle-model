"""Native project editor and four-corner kinematics viewer."""

import copy
from dataclasses import dataclass, replace
from pathlib import Path
from queue import Empty, SimpleQueue
from threading import Thread
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

from SuspensionGeometryLab.suspension_geometry_lab import hardpoints, project as suspension
from SuspensionGeometryLab.suspension_geometry_lab.widgets import ScrollFrame

from .animation import build_animation
from .animation_controls import AnimationEditor, AnimationPlayer
from .animation_settings import PRESETS
from .deformable import (DamperResult, DynamicsModel, build_dynamics_model, sample_dampers,
                         sample_element_springs, with_axle_pushrod_mounts)
from .dynamics_controls import DynamicsEditor, HeaveEditor, TARGET_LABELS
from .dynamics_settings import INERTIA_LIMITS, PUSHROD_FIELDS
from .element_editor import ElementEditor
from .elements import DAMPER_POINT, PUSH_ROD, default_elements
from .geometry import LINKS, POINTS, VehicleGeometry, build_vehicle, static_geometry
from .heave import simulate_heave
from .project import (AXLES, COMPONENTS, DRIVERS, default_project, read_project,
                      save_project, validate)
from .rigid_body import simulate_rigid_road
from .signal_plot import MOUNT_LABELS, SignalView
from .solver import Unreachable, VehicleResult, solve_vehicle
from .viewport import MechanismView


PROJECTS = Path(__file__).with_name("projects")
AXLE_LABELS = {"front": "前軸", "rear": "後軸"}
DRIVER_LABELS = {"FL": "左前輪", "FR": "右前輪", "RL": "左後輪", "RR": "右後輪",
                 "front": "前齒條", "rear": "後齒條"}
ROD_ROTATION_LABELS = {"upper_arm": "上 A 臂", "lower_arm": "下 A 臂",
                       "upright": "羊角剛體", "axis_only": "桿軸方向"}
HINGE_LABELS = {"UCA": "上 A 臂", "LCA": "下 A 臂"}


def show_number(value):
    return f"{value:.5f}".rstrip("0").rstrip(".") if abs(value) >= .000005 else "0"


def show_vector(vector):
    return ", ".join(map(show_number, vector))


def make_table(parent, headings, widths, height):
    frame = ttk.Frame(parent)
    frame.pack(fill="both", expand=True)
    frame.rowconfigure(0, weight=1)
    frame.columnconfigure(0, weight=1)
    table = ttk.Treeview(frame, columns=tuple(range(len(headings))), show="headings", height=height)
    for index, (label, width) in enumerate(zip(headings, widths)):
        table.heading(index, text=label)
        table.column(index, width=width, minwidth=width//2, anchor="center")
    vertical = ttk.Scrollbar(frame, orient="vertical", command=table.yview)
    horizontal = ttk.Scrollbar(frame, orient="horizontal", command=table.xview)
    table.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
    table.grid(row=0, column=0, sticky="nsew")
    vertical.grid(row=0, column=1, sticky="ns")
    horizontal.grid(row=1, column=0, sticky="ew")
    return table


def fill_table(table, rows):
    items = table.get_children()
    if len(items) == len(rows):
        for item, row in zip(items, rows):
            table.item(item, values=row)
    else:
        table.delete(*items)
        for row in rows:
            table.insert("", "end", values=row)


@dataclass(frozen=True)
class PreparedModel:
    geometry: VehicleGeometry
    dynamics: DynamicsModel
    neutral: VehicleResult
    neutral_dampers: dict[str, DamperResult]
    motion: VehicleResult | None = None
    motion_dampers: dict[str, DamperResult] | None = None
    problem: str | None = None


def solve_run_job(results, model, data, mode):
    try:
        if mode == "heave":
            frames = simulate_heave(model, data["dynamics"]["heave"], data["geometry"]["vehicle"]["mass"])
        elif mode == "road":
            frames = simulate_rigid_road(model, data["dynamics"], data["geometry"]["vehicle"]["mass"])
        else:
            settings = data["animation"]
            frames = build_animation(model.geometry, settings["preset"], settings["amplitude"],
                                     settings["duration"], settings["corner"])
            frames = tuple(replace(frame, dampers=sample_dampers(model, frame.result),
                                   element_springs=sample_element_springs(model, frame.result))
                           for frame in frames)
    except Exception as error:
        results.put((None, error))
    else:
        results.put((frames, None))


class Workbench(ttk.Frame):
    def __init__(self, root, data=None, path=None):
        super().__init__(root, padding=(14, 10))
        self.root = root
        self.path = path
        self.dirty = False
        self.loading = False
        self.revision = 0
        self.animation_job = None
        self.signal_window = self.signal_view = None
        self.data = None
        self.hardpoint_draft = None
        self.pose = self.model = self.neutral = self.motion_pose = None
        self.fields = {}
        self.axle_entries = []
        self.active_corner = tk.StringVar(value="FL")
        self.pose_mode = tk.StringVar(value="static")
        self.hardpoint_mode = tk.BooleanVar(value=False)
        self.name = self.new_variable()
        self.caption = tk.StringVar()
        self.status = tk.StringVar()
        self.result_label = tk.StringVar()
        self.angular_label = tk.StringVar()

        root.title("雙 A 臂多體幾何工作台")
        root.geometry("1360x840")
        root.minsize(960, 660)
        if "clam" in ttk.Style(root).theme_names():
            ttk.Style(root).theme_use("clam")
        root.option_add("*Font", "{Microsoft JhengHei UI} 10")
        self.pack(fill="both", expand=True)
        self.build_toolbar()
        panes = ttk.Panedwindow(self, orient="horizontal")
        panes.pack(fill="both", expand=True, pady=(10, 6))
        editor = ttk.Frame(panes, width=400)
        display = ttk.Frame(panes)
        panes.add(editor, weight=0)
        panes.add(display, weight=1)
        self.build_editor(editor)
        self.build_display(display)
        ttk.Label(self, textvariable=self.status, wraplength=910).pack(anchor="w")
        root.bind("<Control-o>", lambda _event: self.open_file())
        root.bind("<Control-s>", lambda _event: self.save())
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.install(data if data is not None else default_project(), path)

    def new_variable(self):
        variable = tk.StringVar()
        variable.trace_add("write", self.mark_dirty)
        return variable

    def mark_dirty(self, *_):
        if not self.loading:
            self.revision += 1
            self.dirty = True
            self.update_caption()

    def update_caption(self):
        file_name = self.path.name if self.path else "尚未儲存"
        self.caption.set(file_name + ("  *" if self.dirty else ""))

    def build_toolbar(self):
        bar = ttk.Frame(self)
        bar.pack(fill="x")
        ttk.Label(bar, text="雙 A 臂多體幾何", font=("Microsoft JhengHei UI", 17, "bold")).pack(side="left")
        ttk.Label(bar, textvariable=self.caption, foreground="#466079").pack(side="left", padx=20)
        for text, command in (("另存 JSON", lambda: self.save(True)), ("儲存 JSON", self.save),
                              ("開啟 JSON", self.open_file)):
            ttk.Button(bar, text=text, command=command).pack(side="right", padx=(5, 0))

    def field(self, parent, key, label, unit="", *, axle_field=False):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=(8, 10), pady=2)
        ttk.Label(row, text=f"{label} ({unit})" if unit else label, wraplength=215).pack(side="left")
        variable = self.fields[key] = self.new_variable()
        entry = ttk.Entry(row, width=13, textvariable=variable, justify="right")
        entry.pack(side="right", padx=(6, 0))
        if axle_field:
            self.axle_entries.append(entry)

    def build_editor(self, parent):
        name_row = ttk.Frame(parent)
        name_row.pack(fill="x", padx=(0, 7), pady=(2, 8))
        ttk.Label(name_row, text="專案名稱").pack(side="left", padx=(0, 10))
        ttk.Entry(name_row, textvariable=self.name).pack(side="left", fill="x", expand=True)
        notebook = ttk.Notebook(parent)
        notebook.pack(fill="both", expand=True, padx=(0, 7))
        for group, title in (("vehicle", "車輛"), *AXLE_LABELS.items()):
            tab = ScrollFrame(notebook)
            notebook.add(tab, text=title)
            if group == "vehicle":
                ttk.Label(tab.content, text="車身轉動慣量（繞簧上重心）", padding=(8, 8, 0, 4)).pack(anchor="w")
                for key, (label, _, _) in INERTIA_LIMITS.items():
                    self.field(tab.content, ("inertia", "chassis", key), label, "kg·m²")
                ttk.Separator(tab.content).pack(fill="x", padx=8, pady=8)
            else:
                ttk.Label(tab.content, text="Push Rod 車身接點", padding=(8, 8, 0, 4)).pack(anchor="w")
                for key, label in PUSHROD_FIELDS.items():
                    self.field(tab.content, ("pushrod", group, key), label, "mm")
                ttk.Label(tab.content, text="X 取靜態 UBJ。右側 Y 為左側的相反值，Z 共用。",
                          wraplength=320, padding=(8, 3, 8, 3)).pack(anchor="w")
                ttk.Separator(tab.content).pack(fill="x", padx=8, pady=8)
            for key, (label, unit) in suspension.FIELDS[group].items():
                self.field(tab.content, ("geometry", group, key), label, unit,
                           axle_field=group in AXLES)
            if group not in AXLES:
                continue
            ttk.Label(tab.content, text="車身接點 X · 留白自動", padding=(8, 12, 0, 3)).pack(anchor="w")
            for key, label in suspension.MOUNT_X_FIELDS.items():
                self.field(tab.content, ("mount", group, key), label, "mm", axle_field=True)
            ttk.Label(tab.content, text="拉桿接點相對輪心的偏移", padding=(8, 12, 0, 3)).pack(anchor="w")
            for endpoint, label in (("outer", "外側"), ("inner", "內側")):
                for index, axis in enumerate(("X 向前", "Y 向外", "Z 向上")):
                    self.field(tab.content, ("steering", group, endpoint, index), f"{label} · {axis}", "mm")
            ttk.Label(tab.content, text="兩側鏡射。齒條向左為 +Y。", padding=(8, 5)).pack(anchor="w")

        points = ttk.Frame(notebook, padding=6)
        notebook.add(points, text="硬點")
        ttk.Checkbutton(points, text="用硬點 XYZ 定義幾何", variable=self.hardpoint_mode,
                        command=self.toggle_hardpoints).pack(anchor="w")
        ttk.Button(points, text="由目前參數重建 24 點", command=self.generate_hardpoints).pack(anchor="w", pady=4)
        ttk.Label(points, text="雙擊編輯 · 左右鏡射 · 單位 mm").pack(anchor="w", pady=(0, 4))
        self.hardpoint_table = make_table(points, ("輪位", "硬點", "X", "Y", "Z"), (50, 70, 63, 63, 63), 16)
        self.hardpoint_table.bind("<Double-1>", self.edit_hardpoint)

        motion = ttk.Frame(notebook, padding=8)
        notebook.add(motion, text="工況")
        ttk.Label(motion, text="車身固定 · 上壓與齒條向左為正", wraplength=310).pack(anchor="w")
        grid = ttk.Frame(motion)
        grid.pack(fill="x", pady=8)
        for column, label in enumerate(("輸入", "位移 mm", "速度 mm/s", "加速度 mm/s²")):
            ttk.Label(grid, text=label).grid(row=0, column=column, padx=2, pady=3)
        for row, name in enumerate(DRIVERS, 1):
            ttk.Label(grid, text=DRIVER_LABELS[name]).grid(row=row, column=0, padx=2, sticky="w")
            for col, component in enumerate(COMPONENTS, 1):
                variable = self.fields["motion", name, component] = self.new_variable()
                ttk.Entry(grid, textvariable=variable, width=9, justify="right").grid(
                    row=row, column=col, padx=2, pady=4, sticky="ew")
        ttk.Button(motion, text="將工況歸零", command=self.reset_motion).pack(anchor="w")

        self.animation_editor = AnimationEditor(notebook, self.mark_dirty, self.start_animation)
        notebook.add(self.animation_editor, text="動畫")
        dynamics_tabs = ttk.Notebook(notebook)
        notebook.add(dynamics_tabs, text="動力學")
        self.dynamics_editor = DynamicsEditor(dynamics_tabs, self.mark_dirty, self.start_dynamics,
                                              tire_stiffness=self.fields["geometry", "vehicle", "tire_stiffness"])
        dynamics_tabs.add(self.dynamics_editor, text="輪端／路面")
        self.heave_editor = HeaveEditor(dynamics_tabs, self.mark_dirty, self.start_heave)
        dynamics_tabs.add(self.heave_editor, text="重心 Heave")
        self.element_editor = ElementEditor(notebook, self.mark_dirty, lambda: self.apply(False))
        notebook.add(self.element_editor, text="機構元件")

        buttons = ttk.Frame(parent)
        buttons.pack(fill="x", padx=(0, 7), pady=(8, 0))
        ttk.Button(buttons, text="解算靜態", command=lambda: self.apply(False)).pack(side="left", fill="x", expand=True)
        ttk.Button(buttons, text="解算工況", command=lambda: self.apply(True)).pack(side="left", fill="x", expand=True,
                                                                                padx=(6, 0))

    def build_display(self, parent):
        bar = ttk.Frame(parent)
        bar.pack(fill="x", pady=(1, 5))
        ttk.Label(bar, textvariable=self.result_label, font=("Microsoft JhengHei UI", 11, "bold")).pack(side="left")
        ttk.Button(bar, text="重設視角", command=lambda: self.viewport.reset()).pack(side="right", padx=(4, 0))
        projection = ttk.Combobox(bar, values=("3D", "前視", "側視", "上視"), width=6, state="readonly")
        projection.set("3D")
        projection.pack(side="right")
        projection.bind("<<ComboboxSelected>>", lambda _event: self.viewport.change_view(projection.get()))
        ttk.Button(bar, text="2D 訊號圖", command=self.open_signal_view).pack(side="right", padx=(0, 6))
        self.viewport = MechanismView(parent)
        self.viewport.pack(fill="both", expand=True)
        self.player = AnimationPlayer(parent, self.refresh_results)
        self.player.pack(fill="x", pady=(6, 0))

        bar = ttk.Frame(parent)
        bar.pack(fill="x", pady=(8, 4))
        ttk.Label(bar, text="結果輪位").pack(side="left")
        selector = ttk.Combobox(bar, textvariable=self.active_corner, values=hardpoints.CORNERS,
                                width=5, state="readonly")
        selector.pack(side="left", padx=6)
        selector.bind("<<ComboboxSelected>>", lambda _event: self.refresh_results())
        ttk.Radiobutton(bar, variable=self.pose_mode, value="static", text="靜態", command=self.select_pose).pack(
            side="left")
        self.pose_button = ttk.Radiobutton(bar, variable=self.pose_mode, value="motion", text="工況",
                                           command=self.select_pose)
        self.pose_button.pack(side="left")
        tabs = ttk.Notebook(parent)
        tabs.pack(fill="x")
        summary, points, links, angular, hinges, dampers, loads = (ttk.Frame(tabs, padding=3) for _ in range(7))
        for tab, title in ((summary, "四輪摘要"), (points, "節點運動"), (links, "桿長／誤差"),
                           (angular, "桿件角運動"), (hinges, "A 臂鉸鏈"), (dampers, "避震器"),
                           (loads, "車身接點力")):
            tabs.add(tab, text=title)
        self.summary = make_table(summary, ("輪角", "行程 mm", "轉向角 °", "外傾 °", "輪心 Vz mm/s",
                                            "輪心 Az mm/s²", "Jacobian 條件數"), (56, 83, 88, 83, 115, 125, 130), 4)
        self.point_result = make_table(points, ("節點", "X mm", "Y mm", "Z mm", "Vx mm/s", "Vy mm/s",
                                                 "Vz mm/s", "Ax mm/s²", "Ay mm/s²", "Az mm/s²"),
                                       (62, 67, 67, 67, 80, 80, 80, 83, 83, 83), 5)
        self.link_result = make_table(links, ("端點 A", "端點 B", "靜態長度 mm", "目前長度 mm", "誤差 mm"),
                                      (94, 94, 126, 126, 126), 5)
        self.rod_result = make_table(angular, ("A → B", "轉動來源", "方位 °", "仰角 °", "距靜態 °",
                                              "ω X, Y, Z (rad/s)", "α X, Y, Z (rad/s²)"),
                                     (128, 86, 74, 74, 78, 170, 175), 5)
        ttk.Label(angular, text="方位由 +X 朝 +Y，仰角向上為正。轉向拉桿只顯示桿軸方向的轉動。").pack(
            anchor="w", pady=(3, 0))
        self.hinge_result = make_table(hinges, ("A 臂", "鉸鏈軸 X, Y, Z", "轉角 °", "角速度 rad/s", "角加速度 rad/s²"),
                                       (80, 186, 94, 116, 126), 2)
        ttk.Label(hinges, text="轉角相對靜態零位，正向沿車身前接點指向後接點。").pack(anchor="w", pady=(3, 0))
        self.damper_result = make_table(dampers, ("輪位", "長度 mm", "伸長 mm", "傾角 °", "長度速率 mm/s",
                                                "長度加速度 mm/s²", "彈簧阻尼力 N", "車身端 XYZ 力 (N)", "輪胎正向力 N"),
                                       (48, 75, 72, 58, 111, 127, 90, 184, 98), 4)
        ttk.Label(dampers, text="伸長以靜態為零。傾角相對車身垂直軸。XYZ 為車身座標的彈簧阻尼力。").pack(
            anchor="w", pady=(3, 0))
        self.mount_result = make_table(loads, ("車身接點", "Fx (N)", "Fy (N)", "Fz (N)"),
                                      (162, 121, 121, 121), 5)
        ttk.Label(loads, text="接點施加在車身上的力，以地面座標表示，+Z 向上。").pack(anchor="w", pady=(3, 0))
        ttk.Label(parent, textvariable=self.angular_label, padding=(0, 6, 0, 0)).pack(anchor="w")

    def open_signal_view(self):
        if self.signal_window is not None:
            self.signal_window.deiconify()
            self.signal_window.lift()
            return
        window = tk.Toplevel(self.root)
        window.title("多體幾何 · 2D 訊號圖")
        window.geometry("920x505")
        window.minsize(620, 320)
        self.signal_window = window
        self.signal_view = SignalView(window, self.player.seek_time)
        self.signal_view.pack(fill="both", expand=True)
        self.signal_view.set_geometry(self.model)
        self.signal_view.set_frames(self.player.frames)
        frame = self.player.frame
        self.signal_view.set_time(frame.time if frame is not None else None)
        window.protocol("WM_DELETE_WINDOW", self.close_signal_view)

    def close_signal_view(self):
        self.signal_window.destroy()
        self.signal_window = self.signal_view = None

    def reset_motion(self):
        for name in DRIVERS:
            for component in COMPONENTS:
                self.fields["motion", name, component].set("0")

    def set_parameter_states(self):
        for entry in self.axle_entries:
            entry.configure(state="disabled" if self.hardpoint_mode.get() else "normal")

    def toggle_hardpoints(self):
        if self.hardpoint_mode.get() and self.hardpoint_draft is None and not self.generate_hardpoints():
            self.hardpoint_mode.set(False)
            return
        self.set_parameter_states()
        self.mark_dirty()

    def generate_hardpoints(self):
        try:
            points, _, _ = static_geometry(self.read_geometry(direct=False))
        except (ValueError, ArithmeticError) as error:
            self.report_error(error)
            return False
        self.hardpoint_draft = copy.deepcopy(points)
        self.refresh_hardpoints()
        self.mark_dirty()
        return True

    def refresh_hardpoints(self):
        source = self.hardpoint_draft
        if source is None:
            source = {corner.name: {key: corner.reference[POINTS.index(key)] for key in hardpoints.POINTS}
                      for corner in self.model.corners}
        fill_table(self.hardpoint_table, [(corner, key, *(show_number(value * 1000) for value in source[corner][key]))
                                         for corner in hardpoints.CORNERS for key in hardpoints.POINTS])

    def edit_hardpoint(self, event):
        row = self.hardpoint_table.identify_row(event.y)
        if not row or not self.hardpoint_mode.get():
            return
        corner, key, *_ = self.hardpoint_table.item(row, "values")
        current = self.hardpoint_draft[corner][key]
        value = simpledialog.askstring("編輯硬點", f"{corner} · {hardpoints.POINTS[key]}\n輸入 X, Y, Z（mm）",
                                       initialvalue=", ".join(f"{v*1000:.12g}" for v in current), parent=self.root)
        if value is None:
            return
        try:
            hardpoints.set_point(self.hardpoint_draft, corner, key,
                                 [float(item) / 1000 for item in value.replace("，", ",").split(",")])
        except ValueError as error:
            self.report_error(error)
            return
        self.refresh_hardpoints()
        self.mark_dirty()

    def number(self, key, label):
        try:
            return float(self.fields[key].get())
        except ValueError as error:
            raise ValueError(f"{label}需要有效數值。") from error

    def read_geometry(self, *, direct):
        geometry = copy.deepcopy(self.data["geometry"])
        geometry["name"] = self.name.get().strip()
        for group in ("vehicle", *AXLES):
            for key, (label, _unit) in suspension.FIELDS[group].items():
                geometry[group][key] = self.number(("geometry", group, key), label)
        geometry["arm_mount_x"] = {
            axle: {key: (self.number(("mount", axle, key), label) / 1000
                         if self.fields["mount", axle, key].get().strip() else None)
                   for key, label in suspension.MOUNT_X_FIELDS.items()} for axle in AXLES
        }
        if self.hardpoint_draft is not None:
            geometry["hardpoints"] = {"enabled": direct, "corners": copy.deepcopy(self.hardpoint_draft)}
        return geometry

    def draft(self):
        data = copy.deepcopy(self.data)
        data["geometry"] = self.read_geometry(direct=self.hardpoint_mode.get())
        data["animation"] = self.animation_editor.draft()
        inertia = {key: self.number(("inertia", "chassis", key), label)
                   for key, (label, _, _) in INERTIA_LIMITS.items()}
        pushrod_mounts = {
            axle: {key: self.number(("pushrod", axle, key), f"{AXLE_LABELS[axle]} {label}")
                   for key, label in PUSHROD_FIELDS.items()}
            for axle in AXLES
        }
        data["dynamics"] = self.dynamics_editor.draft(self.heave_editor.draft(), inertia, pushrod_mounts)
        elements = self.element_editor.draft()
        if elements != default_elements():
            data["dynamics"]["elements"] = elements
        for axle, endpoints in data["steering"].items():
            for name in endpoints:
                endpoints[name] = [self.number(("steering", axle, name, index), "拉桿接點偏移") / 1000
                                   for index in range(3)]
        for name in DRIVERS:
            for key in COMPONENTS:
                data["motion"][name][key] = self.number(("motion", name, key), DRIVER_LABELS[name])
        return validate(data)

    def prepare(self, data, solve_motion, *, geometry=None):
        model = build_vehicle(data) if geometry is None else geometry
        dynamics = build_dynamics_model(model, data["geometry"]["vehicle"], data["dynamics"])
        model = dynamics.geometry
        zero = {name: dict.fromkeys(COMPONENTS, 0.) for name in DRIVERS}
        neutral = solve_vehicle(model, zero)
        neutral_dampers = sample_dampers(dynamics, neutral)
        if solve_motion:
            try:
                motion = solve_vehicle(model, data["motion"])
                return PreparedModel(model, dynamics, neutral, neutral_dampers,
                                     motion, sample_dampers(dynamics, motion))
            except Unreachable as error:
                return PreparedModel(model, dynamics, neutral, neutral_dampers, problem=str(error))
        return PreparedModel(model, dynamics, neutral, neutral_dampers)

    @staticmethod
    def element_points(prepared):
        points = {}
        for name, pose in prepared.neutral.corners.items():
            points[name] = dict(zip(POINTS, pose.position))
            points[name][DAMPER_POINT] = prepared.neutral_dampers[name].fixed_point
        return points

    def populate(self, prepared):
        geometry = self.data["geometry"]
        self.loading = True
        try:
            self.name.set(geometry["name"])
            self.hardpoint_mode.set(hardpoints.enabled(geometry))
            self.hardpoint_draft = copy.deepcopy(geometry.get("hardpoints", {}).get("corners"))
            self.animation_editor.populate(self.data["animation"])
            self.dynamics_editor.populate(self.data["dynamics"])
            self.heave_editor.populate(self.data["dynamics"]["heave"])
            self.element_editor.populate(self.data["dynamics"].get("elements", default_elements()),
                                         self.element_points(prepared))
            for key, var in self.fields.items():
                group, channel, field, *index = key
                if group == "geometry":
                    value = geometry[channel][field]
                elif group == "inertia":
                    value = self.data["dynamics"]["chassis_inertia"][field]
                elif group == "pushrod":
                    value = self.data["dynamics"]["pushrod_mounts"][channel][field]
                elif group == "mount":
                    value = suspension.mount_x_settings(geometry)[channel][field]
                    value = "" if value is None else value * 1000
                elif group == "steering":
                    value = self.data["steering"][channel][field][index[0]] * 1000
                else:
                    value = self.data["motion"][channel][field]
                var.set(str(value))
        finally:
            self.loading = False
        self.set_parameter_states()

    def show_prepared(self, prepared, show_motion):
        self.revision += 1
        self.player.clear()
        if self.signal_view is not None:
            self.signal_view.set_frames(())
        self.model, self.neutral, self.motion_pose = prepared.geometry, prepared.neutral, prepared.motion
        if self.signal_view is not None:
            self.signal_view.set_geometry(self.model)
        self.neutral_dampers, self.motion_dampers = prepared.neutral_dampers, prepared.motion_dampers
        self.element_editor.set_points(self.element_points(prepared))
        self.pose_mode.set("motion" if show_motion and self.motion_pose is not None else "static")
        self.pose_button.configure(state="normal" if self.motion_pose is not None else "disabled")
        self.refresh_results()
        self.refresh_hardpoints()
        mode = "硬點模式" if hardpoints.enabled(self.data["geometry"]) else "懸吊參數模式"
        self.status.set(f"工況無解，顯示靜態。{prepared.problem}" if prepared.problem else
                        f"{mode} · X 向前、Y 向左、Z 向上 · 拖曳旋轉，滾輪縮放")

    def apply(self, show_motion):
        try:
            data = self.draft()
            prepared = self.prepare(data, show_motion)
        except (ValueError, ArithmeticError) as error:
            self.report_error(error)
            return False
        self.data = data
        self.show_prepared(prepared, show_motion)
        return prepared.problem is None

    def install(self, data, path):
        data = validate(data)
        geometry = build_vehicle(data)
        data["dynamics"] = with_axle_pushrod_mounts(geometry, data["dynamics"])
        prepared = self.prepare(data, True, geometry=geometry)
        self.data = data
        self.path, self.dirty = path, False
        self.populate(prepared)
        self.update_caption()
        has_motion = any(value != 0 for driver in data["motion"].values() for value in driver.values())
        self.show_prepared(prepared, has_motion)

    def start_animation(self):
        self.start_run("animation")

    def start_dynamics(self):
        self.start_run("road")

    def start_heave(self):
        self.start_run("heave")

    def start_run(self, mode):
        if self.animation_job is not None:
            return
        try:
            data = self.draft()
            prepared = self.prepare(data, True)
        except (ValueError, ArithmeticError) as error:
            self.report_error(error)
            return
        self.player.pause()
        self.animation_job = SimpleQueue()
        self.set_run_buttons("disabled")
        messages = {"heave": "正在解算車身與四輪的垂直運動…",
                    "road": "正在解算路面擾動下的車身平移、轉動與輪端運動…",
                    "animation": "正在逐格解算動畫…"}
        self.status.set(messages[mode])
        job, revision = self.animation_job, self.revision
        Thread(target=solve_run_job, args=(job, prepared.dynamics, data, mode), daemon=True).start()
        self.after(25, self.finish_animation, revision, data, prepared, mode)

    def set_run_buttons(self, state):
        self.animation_editor.start_button.configure(state=state)
        self.dynamics_editor.start_button.configure(state=state)
        self.heave_editor.start_button.configure(state=state)

    def finish_animation(self, revision, data, prepared, mode):
        try:
            frames, error = self.animation_job.get_nowait()
        except Empty:
            self.after(25, self.finish_animation, revision, data, prepared, mode)
            return
        self.animation_job = None
        self.set_run_buttons("normal")
        if revision != self.revision:
            if self.status.get().startswith("正在"):
                self.status.set("設定已變更，請重新解算。")
            return
        if error is not None:
            self.report_error(error)
            return
        self.data = data
        self.show_prepared(prepared, False)
        if mode == "heave":
            label, loop = "重心受力 · Heave", False
        elif mode == "road":
            target = TARGET_LABELS[data["dynamics"]["simulation"]["target"]]
            label, loop = f"路面擾動 · {target}", False
        else:
            settings = data["animation"]
            label, loop = PRESETS[settings["preset"]], settings["loop"]
            if settings["preset"] == "single_wheel":
                label += f" · {settings['corner']}"
        self.player.load(frames, loop, label)
        if self.signal_view is not None:
            self.signal_view.set_frames(frames)
            self.signal_view.set_time(frames[0].time)
        self.player.toggle()

    def select_pose(self):
        self.player.leave()
        self.status.set("X 向前、Y 向左、Z 向上 · 拖曳旋轉，滾輪縮放")
        self.refresh_results()

    def refresh_results(self):
        frame = self.player.frame
        use_motion = self.pose_mode.get() == "motion" and self.motion_pose is not None
        if frame is not None:
            if self.pose_mode.get() != "animation":
                self.status.set("3D、車身姿態與接點力採地面座標，節點與桿件運動採車身座標。"
                                if frame.heave is not None else
                                "3D 以地面為基準，輪端結果採車身座標。拖曳時間軸可暫停檢查。")
            self.pose_mode.set("animation")
            self.pose = frame.result
            self.result_label.set(self.player.preset_label)
        else:
            self.pose = self.motion_pose if use_motion else self.neutral
            self.result_label.set("指定工況" if use_motion else "靜態全車")
        if self.pose is None:
            return
        dampers = frame.dampers if frame is not None else (self.motion_dampers if use_motion else self.neutral_dampers)
        self.viewport.set_result(self.model, self.pose, frame, dampers)
        if self.signal_view is not None:
            self.signal_view.set_time(frame.time if frame is not None else None)
        wheel = POINTS.index("WC")
        rows = []
        for corner in self.model.corners:
            p = self.pose.corners[corner.name]
            jounce = (p.position[wheel, 2] - corner.reference[wheel][2]) * 1000
            numbers = (jounce, p.toe_deg, p.camber_deg, p.velocity[wheel, 2] * 1000,
                       p.acceleration[wheel, 2] * 1000, p.condition)
            rows.append((corner.name, *map(show_number, numbers)))
        fill_table(self.summary, rows)
        name = self.active_corner.get()
        corner = next(item for item in self.model.corners if item.name == name)
        pose = self.pose.corners[name]
        rigid_links = pose.rigid_links if corner.members is not None else LINKS
        rigid_lengths = pose.rigid_lengths if corner.members is not None else corner.lengths
        fill_table(self.point_result, [
            (point, *(show_number(component * 1000) for vec in (position, velocity, acceleration) for component in vec))
            for point, position, velocity, acceleration in zip(POINTS, pose.position, pose.velocity, pose.acceleration)
        ])
        fill_table(self.link_result, [
            (a, b, show_number(length * 1000), show_number((length + error) * 1000), f"{error * 1000:.2e}")
            for (a, b), length, error in zip(rigid_links, rigid_lengths, pose.length_error)
        ])
        rod_names = ({link: f"{link[0]} → {link[1]}" for link in LINKS} if corner.members is None
                     else {(member.a, member.b): member.name for member in corner.members})
        fill_table(self.rod_result, [
            (rod_names[link], ROD_ROTATION_LABELS[rod.rotation_model],
             "—" if rod.azimuth_deg is None else show_number(rod.azimuth_deg),
             show_number(rod.elevation_deg), show_number(rod.from_static_deg),
             show_vector(rod.angular_velocity), show_vector(rod.angular_acceleration))
            for link, rod in pose.rod_motion.items() if link in rod_names
        ])
        fill_table(self.hinge_result, [
            (HINGE_LABELS[name], show_vector(hinge.axis), show_number(hinge.angle_deg),
             show_number(hinge.velocity), show_number(hinge.acceleration))
            for name, hinge in pose.arm_motion.items()
        ])
        fill_table(self.damper_result, [
            (name, show_number(1000 * element.length), show_number(1000 * element.deflection),
             show_number(element.inclination_deg), show_number(1000 * element.velocity),
             show_number(1000 * element.acceleration), show_number(element.force),
             show_vector(element.force_on_chassis),
             "—" if element.tire_force is None else show_number(element.tire_force))
            for name, element in (dampers or {}).items()
        ])
        mounts = frame.mount_loads[name] if frame is not None and frame.mount_loads else None
        mount_rows = []
        if mounts is not None:
            mount_rows = [(MOUNT_LABELS[key], *map(show_number, force)) for key, force in mounts.by_mount.items()]
            mount_rows.append(("本輪接點合力", *map(show_number, mounts.total)))
        fill_table(self.mount_result, mount_rows)
        omega = show_vector(pose.angular_velocity)
        alpha = show_vector(pose.angular_acceleration)
        self.angular_label.set(f"{name} 羊角角速度 ({omega}) rad/s    角加速度 ({alpha}) rad/s²")

    def report_error(self, error):
        self.status.set(str(error))
        messagebox.showerror("無法套用設定", str(error), parent=self.root)

    def confirm_discard(self):
        if not self.dirty:
            return True
        answer = messagebox.askyesnocancel("未儲存設定", "是否先儲存目前設定？", parent=self.root)
        return self.save() if answer is True else answer is False

    def open_file(self):
        file = filedialog.askopenfilename(parent=self.root, title="開啟專案", initialdir=PROJECTS,
                                          filetypes=[("JSON 專案", "*.json")])
        if not file or not self.confirm_discard():
            return
        try:
            path = Path(file)
            data, native = read_project(path)
            self.install(data, path if native else None)
        except (ValueError, ArithmeticError, OSError) as error:
            self.report_error(error)

    def save(self, save_as=False):
        path = self.path
        if path is None or save_as:
            file = filedialog.asksaveasfilename(parent=self.root, title="儲存專案", initialdir=PROJECTS,
                                                defaultextension=".json", initialfile=path.name if path else "",
                                                filetypes=[("JSON 專案", "*.json")])
            if not file:
                return False
            path = Path(file)
        try:
            data = self.draft()
            prepared = self.prepare(data, True)
            save_project(path, data)
        except (ValueError, ArithmeticError, OSError) as error:
            self.report_error(error)
            return False
        self.data = data
        self.path, self.dirty = path, False
        self.update_caption()
        self.show_prepared(prepared, self.pose_mode.get() == "motion")
        self.status.set(f"已儲存 {path.name} · {self.status.get()}")
        return True

    def close(self):
        if self.confirm_discard():
            self.root.destroy()
