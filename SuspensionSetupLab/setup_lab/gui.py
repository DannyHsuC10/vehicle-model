"""Offline desktop workspace for vehicle setup, replay and matched A/B runs."""

from pathlib import Path
from queue import Empty, Queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np

from .analysis_page import AnalysisPage
from .aero_page import AeroPage
from .chart_base import Chart
from .components import DAMPER_DIRECTIONS, DAMPER_STAGE_COUNTS, KCSystem, damper_kinds
from .engine import Cancelled, prepare_vehicle, simulate
from .geometry_lookup import VEHICLE_KEYS as GEOMETRY_VEHICLE_KEYS
from .geometry_page import GeometryPage
from .load_transfer_page import LoadTransferPage
from .project import (AERO_DEFAULTS, APP, CHANNELS, COMPONENT_FIELDS, CORNERS, DAMPER_FORCE_LABELS,
                      DEFAULT_DAMPER_SWITCH_SPEED, DEFAULT_SOURCES, MODEL_PAGE, MODEL_SOURCES, PAGE_MODELS, ROLL_FRAME_FIELDS,
                      TIRE_FIELDS, VEHICLE_FIELDS, analysis_page_cases, channel_enabled, compile_project, damping_reference_mass,
                      default_project, load_transfer_options, model_channels, model_corners, open_project, project_roll_frame,
                      read_force_csv, roll_frame_options, save_project, transfer_options,
                      with_static_tender_travel, with_total_limit_rates)
from .roll_center_scan import scan_roll_center
from .simulation_export import export_simulation
from .widgets import entry, key_for

SPRINGS = {"single": "單一彈簧", "tender": "主簧＋Tender", "force": "查表彈簧剛性", "stages": "舊專案彈簧曲線"}
DAMPER_MODES = {"single": "一段式阻尼（1 段）", "split": "壓縮回彈阻尼（2 段）",
                "bilinear": "壓縮回彈高低速阻尼（4 段）", "force": "查表阻尼", "legacy": "舊專案阻尼曲線"}
MR_MODES = {"fixed": "固定運動比", "lookup": "查表運動比"}
DAMPER_FIELDS = {
    "single": {"rate": "阻尼"},
    "split": DAMPER_FORCE_LABELS,
    "bilinear": {
        f"{side}{suffix}": f"{label}{name}"
        for suffix, name in (("", "低速阻尼"), ("_high", "高速阻尼"), ("_switch", "切換速度"))
        for side, label in (("compression", "壓縮"), ("rebound", "回彈"))
    },
}
CURVE_LABELS = {"spring": "彈簧力表", "dampers": "阻尼力表", "mr": "運動比"}
PAGE_TITLES = {"full": "全車", "half_lr": "1/2 左右", "half_fr": "1/2 前後", "quarter": "1/4 車"}


def elastic_curve(component):
    reference = component.static_stroke if component.static_stroke is not None else 0.
    low = reference + 1.1 * min(0., -component.rebound - reference)
    high = reference + 1.1 * component.jounce_travel
    if low == high:
        low, high = reference - 1., reference + 1.
    breaks = (*component.spring_breakpoints(), -component.rebound, reference, component.jounce)
    stroke = np.array(sorted({low, high, *(x for x in breaks if low < x < high)}))
    spring = np.array([component.spring_force(x) for x in stroke])
    stops = np.array([component.stop_force(x) for x in stroke])
    return stroke - reference, spring, stops


def motion_ratio_chart(component):
    curve = component.motion_ratio
    extent = max(50., max(map(abs, curve.x)))
    travel = sorted({-extent, *curve.x, extent})
    return {"title": "運動比曲線", "xlabel": "輪端／模態行程 (mm)", "ylabel": "MR",
            "series": [{"x": travel, "y": [curve.lookup(x) for x in travel]}]}


def damper_mode(settings):
    kind = settings["damper_kind"]
    if isinstance(kind, str) and kind in DAMPER_MODES:
        return kind
    kinds = set(damper_kinds(kind).values())
    if kinds == {"force"}:
        return "force"
    if kinds == {"stages"}:
        compression, rebound = (settings[side] for side in DAMPER_DIRECTIONS)
        if len(compression) == len(rebound) == 1:
            return "single" if compression[0][1] == rebound[0][1] else "split"
        if len(compression) == len(rebound) == 2:
            return "bilinear"
    return "legacy"


class SetupApp:
    def __init__(self, root, project=None, path=None):
        self.root = root
        self.project = default_project() if project is None else project
        self.path = path
        self.job = self.running_page = self.analysis = None
        self.events = Queue()
        root.title(f"{APP}｜懸吊調校")
        root.geometry("1270x830")
        root.minsize(1024, 690)
        root.option_add("*Font", "{Microsoft JhengHei UI} 10")
        root.protocol("WM_DELETE_WINDOW", self.close)

        toolbar = ttk.Frame(root, padding=(12, 8))
        toolbar.pack(fill="x")
        for label, command in (("新專案", self.new), ("開啟／匯入 JSON", self.open), ("儲存 JSON", self.save)):
            ttk.Button(toolbar, text=label, command=command).pack(side="left", padx=(0, 6))
        self.name = tk.StringVar()
        ttk.Entry(toolbar, textvariable=self.name, width=27).pack(side="left", padx=12)
        self.status = tk.StringVar(value="設定工況後按 F5 計算。")
        ttk.Label(root, textvariable=self.status, padding=(12, 5)).pack(side="bottom", fill="x")
        self.pages = ttk.Notebook(root)
        self.pages.pack(fill="both", expand=True, padx=12)
        self.source_vars = {model: tk.StringVar() for model in MODEL_SOURCES}
        self.analysis_pages = {key: AnalysisPage(self, key) for key in PAGE_MODELS}
        for key, page in self.analysis_pages.items():
            self.pages.add(page, text=f"  {PAGE_TITLES[key]}  ")
        self.build_vehicle()
        self.geometry_page = GeometryPage(self)
        self.pages.add(self.geometry_page, text="  幾何查表  ")
        self.aero_page = AeroPage(self)
        self.pages.add(self.aero_page, text="  空力  ")
        self.transfer_page = LoadTransferPage(self)
        self.pages.add(self.transfer_page, text="  載重轉移分配  ")
        self.build_components()
        self.set_project(self.project, path)
        self.pages.bind("<<NotebookTabChanged>>", self.tab_changed)
        for sequence, command in (("<F5>", self.run), ("<Control-o>", self.open), ("<Control-s>", self.save)):
            root.bind(sequence, lambda event, action=command: action())

    def build_vehicle(self):
        page = self.vehicle_page = ttk.Frame(self.pages, padding=14)
        self.pages.add(page, text="  車輛與輪胎  ")
        self.vehicle_vars = {}
        body = ttk.LabelFrame(page, text="整車與車架", padding=12)
        body.pack(fill="x")
        fields = [key for key in VEHICLE_FIELDS if key not in GEOMETRY_VEHICLE_KEYS]
        for index, key in enumerate(fields):
            self.vehicle_vars[key] = tk.StringVar()
            entry(body, index // 2, index % 2 * 2, VEHICLE_FIELDS[key][0], self.vehicle_vars[key], 16)
        ttk.Label(body, text="只有 8 DOF 車架使用扭轉 k、c。").grid(
            row=(len(fields) + 1) // 2, columnspan=4, sticky="w", pady=(8, 0))
        self.build_roll_frame_form(page)
        tires = ttk.LabelFrame(page, text="每輪簧下質量與輪胎垂直特性", padding=12)
        tires.pack(fill="x", pady=10)
        self.tire_vars = {c: {} for c in CORNERS}
        for column, (key, (label, _, _)) in enumerate(TIRE_FIELDS.items(), 1):
            ttk.Label(tires, text=label).grid(row=0, column=column, padx=8, sticky="w")
            for row, corner in enumerate(CORNERS, 1):
                self.tire_vars[corner][key] = tk.StringVar()
                ttk.Entry(tires, textvariable=self.tire_vars[corner][key], width=16).grid(row=row, column=column, padx=8, pady=4)
        for row, corner in enumerate(CORNERS, 1):
            ttk.Label(tires, text=corner).grid(row=row, column=0, sticky="w")

    def build_roll_frame_form(self, page):
        form = ttk.LabelFrame(page, text="1/2 左右｜Roll Center 車架傳力", padding=12)
        form.pack(fill="x", pady=(10, 0))
        self.roll_frame_enabled = tk.BooleanVar()
        ttk.Checkbutton(form, text="啟用車架傳力動態", variable=self.roll_frame_enabled,
                        command=self.roll_frame_mode).grid(row=0, columnspan=4, sticky="w")
        self.roll_frame_vars, self.roll_frame_widgets = {}, {}
        for index, (key, (label, *_)) in enumerate(ROLL_FRAME_FIELDS.items()):
            var = self.roll_frame_vars[key] = tk.StringVar()
            self.roll_frame_widgets[key] = entry(form, 1 + index // 2, 2 * (index % 2), label, var, width=16)

    def roll_frame_mode(self):
        state = "normal" if self.roll_frame_enabled.get() else "disabled"
        for widget in self.roll_frame_widgets.values():
            widget.configure(state=state)

    def build_components(self):
        page = self.component_page = ttk.Frame(self.pages, padding=12)
        self.pages.add(page, text="  彈簧與阻尼  ")
        sidebar = ttk.Frame(page)
        sidebar.pack(side="left", fill="y", padx=(0, 10))
        self.channel_list = ttk.Treeview(sidebar, columns=("channel",), show="headings", height=10, selectmode="browse")
        self.channel_list.heading("channel", text="通道")
        self.channel_list.column("channel", width=140, stretch=False)
        for key, label in CHANNELS.items():
            self.channel_list.insert("", "end", iid=key, values=(label,))
        self.channel_list.pack(fill="x")
        self.channel_list.bind("<<TreeviewSelect>>", lambda event: self.select_channel())
        model_checks = ttk.LabelFrame(sidebar, text="此通道啟用於", padding=8)
        model_checks.pack(fill="x", pady=(8, 0))
        self.channel_model_vars, self.channel_model_checks = {}, {}
        for family, label in PAGE_TITLES.items():
            var = self.channel_model_vars[family] = tk.BooleanVar()
            check = self.channel_model_checks[family] = ttk.Checkbutton(
                model_checks, text=f"{label}模型", variable=var, command=self.change_channel_models)
            check.pack(anchor="w", pady=2)
        edit = ttk.Frame(page)
        edit.pack(side="left", fill="both", expand=True)
        options = ttk.Frame(edit)
        options.pack(fill="x")
        self.spring_kind = tk.StringVar()
        self.damper_kind = tk.StringVar()
        self.spring_selector = ttk.Combobox(options, textvariable=self.spring_kind, state="readonly", width=17,
                                            values=(SPRINGS["single"], SPRINGS["tender"], SPRINGS["force"]))
        self.spring_selector.pack(side="left")
        self.spring_selector.bind("<<ComboboxSelected>>", lambda event: self.component_mode())
        self.damper_selector = ttk.Combobox(options, textvariable=self.damper_kind, state="readonly", width=27,
                                            values=tuple(DAMPER_MODES[key] for key in (*DAMPER_STAGE_COUNTS, "force")))
        self.damper_selector.pack(side="left", padx=(8, 0))
        self.damper_selector.bind("<<ComboboxSelected>>", lambda event: self.change_damper_mode())
        self.damper_ratio = tk.BooleanVar()
        self.damper_ratio_check = ttk.Checkbutton(options, text="使用阻尼比", variable=self.damper_ratio,
                                                  command=self.change_damper_input)
        imports = ttk.Frame(edit)
        imports.pack(fill="x", pady=(7, 0))
        self.curve_import_buttons = {}
        for column, (key, label) in zip((0, 1, 3), CURVE_LABELS.items()):
            button = ttk.Button(imports, text=f"匯入{label} CSV", command=lambda table=key: self.import_force_csv(table))
            button.grid(row=0, column=column, sticky="w", padx=(0, 5))
            self.curve_import_buttons[key] = button
        self.mr_kind, self.mr_var = tk.StringVar(), tk.StringVar()
        self.mr_selector = ttk.Combobox(imports, textvariable=self.mr_kind, state="readonly", width=15,
                                        values=tuple(MR_MODES.values()))
        self.mr_selector.grid(row=0, column=2, sticky="w", padx=(0, 5))
        self.mr_selector.bind("<<ComboboxSelected>>", lambda event: self.change_mr_mode())
        self.component_vars = {key: tk.StringVar() for key in COMPONENT_FIELDS}
        numeric = ttk.LabelFrame(edit, text="彈簧、阻尼、運動比與安裝", padding=8)
        numeric.pack(fill="x", pady=7)
        self.component_inputs = {}
        for index, (key, label) in enumerate(COMPONENT_FIELDS.items()):
            row, column = index // 2, index % 2 * 2
            caption = ttk.Label(numeric, text=label)
            caption.grid(row=row, column=column, sticky="w", padx=(0, 8), pady=4)
            field = ttk.Entry(numeric, textvariable=self.component_vars[key], width=12)
            field.grid(row=row, column=column + 1, sticky="ew", padx=(0, 14), pady=4)
            self.component_inputs[key] = (caption, field)
        self.mr_fixed_form = ttk.Frame(numeric)
        self.mr_fixed_form.grid(row=0, column=4, sticky="w")
        entry(self.mr_fixed_form, 0, 0, "固定 MR", self.mr_var, width=8)
        self.damper_vars = {key: tk.StringVar() for key in ("rate", *DAMPER_FIELDS["bilinear"])}
        self.damper_forms = {}
        self.damper_labels = {}
        for mode, fields in DAMPER_FIELDS.items():
            form = ttk.Frame(numeric)
            form.grid(row=4, columnspan=4, sticky="ew")
            form.grid_remove()
            self.damper_forms[mode] = form
            for index, key in enumerate(fields):
                row, column = index // 2, index % 2 * 2
                label = ttk.Label(form)
                label.grid(row=row, column=column, sticky="w", padx=(0, 8), pady=4)
                ttk.Entry(form, textvariable=self.damper_vars[key], width=12).grid(
                    row=row, column=column + 1, sticky="ew", padx=(0, 14), pady=4)
                self.damper_labels[mode, key] = label
        self.curve_message = ttk.Label(edit)
        charts = ttk.Frame(edit)
        charts.pack(fill="both", expand=True)
        charts.rowconfigure(0, weight=1)
        self.spring_chart, self.damper_chart, self.mr_chart = (Chart(charts, min_width=220) for _ in range(3))
        for column, chart in enumerate((self.spring_chart, self.damper_chart, self.mr_chart)):
            charts.columnconfigure(column, weight=1, uniform="curves")
            chart.canvas.configure(width=230, height=280)
            chart.grid(row=0, column=column, sticky="nsew", padx=(0, 5))
        self.current_channel = None
        self.component_draw_job = None
        edit.bind("<Destroy>", self.cancel_component_draw)
        for variable in (*self.component_vars.values(), *self.damper_vars.values(), self.mr_var):
            variable.trace_add("write", self.schedule_component_draw)
        for key in ("m", "lf", "lr", "Ixx", "tf", "tr"):
            self.vehicle_vars[key].trace_add("write", self.schedule_component_draw)
        for variables in self.tire_vars.values():
            variables["mu"].trace_add("write", self.schedule_component_draw)

    def set_project(self, project, path):
        project = with_static_tender_travel(with_total_limit_rates(project))
        self.project, self.path = project, path
        sources = {**DEFAULT_SOURCES, **project.get("model_sources", {})}
        for key, source in sources.items():
            self.source_vars[key].set(MODEL_SOURCES[key][source])
        self.name.set(project["name"])
        self.aero_page.set_settings(project.get("aero", AERO_DEFAULTS))
        self.geometry_page.set_settings(project)
        self.transfer_page.set_options(load_transfer_options(project.get("load_transfer", {})))
        for key, var in self.vehicle_vars.items():
            var.set(project["vehicle"][key])
        frame = project_roll_frame(project)
        self.roll_frame_enabled.set(frame["enabled"])
        for key, var in self.roll_frame_vars.items():
            var.set(frame[key])
        self.roll_frame_mode()
        for corner, variables in self.tire_vars.items():
            for key, var in variables.items():
                var.set(project["tires"][corner][key])
        self.drafts = {}
        for key, component in project["channels"].items():
            self.drafts[key] = component.copy()
        self.spring_lookups = {key for key, draft in self.drafts.items() if draft["spring_kind"] == "force"}
        self.damper_drafts = {}
        self.mr_drafts = {}
        self.current_channel = None
        self.analysis = None
        active = MODEL_PAGE[project["model"]]
        default_case = {**default_project()["case"], "target": "all"}
        stored = analysis_page_cases(project)
        for key, page in self.analysis_pages.items():
            if key == active:
                page.set_form(project["model"], project["case"])
            elif key in stored:
                page.set_form(stored[key]["model"], stored[key]["case"])
            else:
                page.set_form(PAGE_MODELS[key][0], default_case)
        self.pages.select(self.analysis_pages[active])
        self.tab_changed()
        self.saved = self.editor_state()

    def tab_changed(self, event=None):
        selected = self.pages.select()
        page = next((page for page in self.analysis_pages.values() if str(page) == selected), None)
        if self.analysis is not None and self.analysis is not page:
            self.analysis.playback.pause()
        if page is not None:
            self.analysis = page
            self.refresh_channels()
        elif selected == str(self.aero_page):
            self.aero_page.draw()
        elif selected == str(self.geometry_page):
            self.geometry_page.draw()
        elif selected == str(self.component_page):
            self.schedule_component_draw()
        elif selected == str(self.transfer_page):
            self.transfer_page.draw()

    def refresh_channels(self):
        model = self.analysis.model_key
        self.applicable_channels = model_channels(model, model_corners(model, self.sources()))
        if self.current_channel is None:
            self.channel_list.selection_set(self.applicable_channels[0])
            self.select_channel()
        self.schedule_component_draw()

    def sources(self):
        return {key: key_for(MODEL_SOURCES[key], var.get()) for key, var in self.source_vars.items()}

    def capture_component(self):
        key = self.current_channel
        if key is None:
            return
        draft = self.drafts[key]
        draft["spring_kind"] = key_for(SPRINGS, self.spring_kind.get())
        draft.update({name: var.get() for name, var in self.component_vars.items()})
        mode = self.active_damper_mode
        if mode in DAMPER_STAGE_COUNTS:
            for side in DAMPER_DIRECTIONS:
                rate = self.damper_vars["rate" if mode == "single" else side].get()
                draft[side] = [[0, rate]]
                if mode == "bilinear":
                    draft[side].append([self.damper_vars[f"{side}_switch"].get(),
                                        self.damper_vars[f"{side}_high"].get()])
        if self.active_mr_mode == "fixed":
            draft["mr"] = [[draft["mr"][0][0], self.mr_var.get()]]

    def change_channel_models(self):
        draft = self.drafts[self.current_channel]
        overrides = draft.get("enabled_models", {}).copy()
        for family, var in self.channel_model_vars.items():
            if self.channel_model_checks[family].instate(("disabled",)):
                continue
            if var.get() == draft["enabled"]:
                overrides.pop(family, None)
            else:
                overrides[family] = var.get()
        if overrides:
            draft["enabled_models"] = overrides
        else:
            draft.pop("enabled_models", None)
        self.schedule_component_draw()

    def select_channel(self):
        selection = self.channel_list.selection()
        if not selection or selection[0] == self.current_channel:
            return
        self.capture_component()
        self.current_channel = selection[0]
        draft = self.drafts[self.current_channel]
        for family, var in self.channel_model_vars.items():
            # Use all possible corners so changing a model's selected axle or side leaves the controls stable.
            supported = self.current_channel in model_channels(PAGE_MODELS[family][0], CORNERS)
            var.set(supported and channel_enabled(draft, family))
            self.channel_model_checks[family].state(["!disabled" if supported else "disabled"])
        self.spring_kind.set(SPRINGS[draft["spring_kind"]])
        for key, var in self.component_vars.items():
            var.set(draft[key])
        self.load_curve_controls()
        self.component_mode()

    def component_mode(self):
        spring = key_for(SPRINGS, self.spring_kind.get())
        spring_fields = {"main_rate": spring in ("single", "tender"),
                         "tender_rate": spring == "tender", "tender_after_static_mm": spring == "tender"}
        for key, visible in spring_fields.items():
            for widget in self.component_inputs[key]:
                if visible:
                    widget.grid()
                else:
                    widget.grid_remove()
        fixed_mr = self.active_mr_mode == "fixed"
        if fixed_mr:
            self.mr_fixed_form.grid()
        else:
            self.mr_fixed_form.grid_remove()
        for table, visible in (("spring", spring == "force"), ("dampers", self.active_damper_mode == "force"),
                               ("mr", not fixed_mr)):
            button = self.curve_import_buttons[table]
            if visible:
                button.grid()
            else:
                button.grid_remove()
        self.schedule_component_draw()

    def load_curve_controls(self):
        draft = self.drafts[self.current_channel]
        mode = self.active_damper_mode = damper_mode(draft)
        self.damper_kind.set(DAMPER_MODES[mode])
        use_ratio = draft.get("damper_ratio", False)
        self.damper_ratio.set(use_ratio)
        if mode in DAMPER_STAGE_COUNTS:
            self.damper_ratio_check.pack(side="left", padx=(8, 0))
        else:
            self.damper_ratio_check.pack_forget()
        for key, label in DAMPER_FIELDS.get(mode, {}).items():
            suffix = " (m/s)" if key.endswith("_switch") else ("比 (ζ)" if use_ratio else "係數 (N·s/m)")
            self.damper_labels[mode, key].configure(text=label + suffix)
        for key, form in self.damper_forms.items():
            if key == mode:
                form.grid()
            else:
                form.grid_remove()
        if mode == "single":
            self.damper_vars["rate"].set(draft["compression"][0][1])
        elif mode in ("split", "bilinear"):
            for side in DAMPER_DIRECTIONS:
                self.damper_vars[side].set(draft[side][0][1])
                if mode == "bilinear":
                    speed, rate = draft[side][1]
                    self.damper_vars[f"{side}_switch"].set(speed)
                    self.damper_vars[f"{side}_high"].set(rate)
        lookup_mr = draft.get("mr_mode") == "lookup" or len(draft["mr"]) > 1
        self.active_mr_mode = "lookup" if lookup_mr else "fixed"
        self.mr_kind.set(MR_MODES[self.active_mr_mode])
        if self.active_mr_mode == "fixed":
            self.mr_var.set(draft["mr"][0][1])

    def damping_mass(self):
        vehicle = {key: float(self.vehicle_vars[key].get()) for key in ("m", "lf", "lr", "Ixx", "tf", "tr")}
        unsprung = sum(float(variables["mu"].get()) for variables in self.tire_vars.values())
        return damping_reference_mass(self.current_channel, vehicle, unsprung)

    def change_damper_input(self):
        self.capture_component()
        channel = self.current_channel
        draft = self.drafts[channel]
        was_ratio = draft.get("damper_ratio", False)
        use_ratio = self.damper_ratio.get()
        if use_ratio == was_ratio:
            return
        try:
            settings = self.component(channel)
            mass = self.damping_mass()
            critical = KCSystem(settings, damping_mass=mass).critical_damping
            if use_ratio and critical == 0 and any(value for side in DAMPER_DIRECTIONS for _, value in settings[side]):
                raise ValueError("零行程的彈簧剛性為 0，無法把非零阻尼係數換算為阻尼比。")
            scale = 1 / critical if use_ratio and critical else critical
            converted = {side: [[speed, value * scale] for speed, value in settings[side]] for side in DAMPER_DIRECTIONS}
            # Validate the converted values before replacing the draft.
            KCSystem(dict(settings, **converted, damper_kind=self.active_damper_mode, damper_ratio=use_ratio),
                     damping_mass=mass)
        except ValueError as exc:
            self.damper_ratio.set(was_ratio)
            self.error(exc)
            return
        draft.update(converted, damper_kind=self.active_damper_mode)
        if use_ratio:
            draft["damper_ratio"] = True
        else:
            draft.pop("damper_ratio", None)
        self.load_curve_controls()
        self.component_mode()

    def change_damper_mode(self):
        mode = key_for(DAMPER_MODES, self.damper_kind.get())
        if mode == self.active_damper_mode:
            return
        self.capture_component()
        channel = self.current_channel
        draft = self.drafts[channel]
        old_mode = self.active_damper_mode
        old_kind = old_mode if old_mode in DAMPER_STAGE_COUNTS else draft["damper_kind"]
        old_settings = {side: draft[side] for side in DAMPER_DIRECTIONS}
        old_settings["damper_kind"] = old_kind
        use_ratio = draft.pop("damper_ratio", False)
        if use_ratio:
            old_settings["damper_ratio"] = True
        self.damper_drafts[channel, old_mode] = old_settings
        previous = self.damper_drafts.get((channel, mode))
        if previous is not None:
            draft.update(previous)
        elif mode == "force":
            draft.update(damper_kind="force", compression=[], rebound=[])
        else:
            source = draft if draft["compression"] else self.project["channels"][channel]
            kinds = damper_kinds(source["damper_kind"])
            rows = {}
            for side, kind in kinds.items():
                points = source[side]
                low = points[1][1] / points[1][0] if kind == "force" else points[0][1]
                rows[side] = [[0, low]]
                if mode == "bilinear":
                    high = points[1] if kind == "stages" and len(points) > 1 else [DEFAULT_DAMPER_SWITCH_SPEED, low]
                    rows[side].append(high)
            if mode == "single":
                rows["rebound"] = rows["compression"].copy()
            draft.update(damper_kind=mode, **rows)
            if use_ratio or source.get("damper_ratio", False):
                draft["damper_ratio"] = True
        self.load_curve_controls()
        self.component_mode()

    def change_mr_mode(self):
        mode = key_for(MR_MODES, self.mr_kind.get())
        if mode == self.active_mr_mode:
            return
        self.capture_component()
        channel = self.current_channel
        draft = self.drafts[channel]
        self.mr_drafts[channel, self.active_mr_mode] = draft["mr"]
        previous = self.mr_drafts.get((channel, mode))
        if mode == "lookup":
            draft["mr_mode"] = "lookup"
            draft["mr"] = previous if previous is not None else []
        else:
            draft.pop("mr_mode", None)
            if previous is None:
                travel, ratio = zip(*draft["mr"])
                previous = [[0, float(np.interp(0, travel, ratio))]]
            draft["mr"] = previous
        self.load_curve_controls()
        self.component_mode()

    def import_force_csv(self, table):
        label = CURVE_LABELS[table]
        path = filedialog.askopenfilename(parent=self.root, title=f"匯入{label} CSV", filetypes=[("CSV 檔", "*.csv")])
        if not path:
            return
        try:
            curves = read_force_csv(path, table)
        except (OSError, ValueError) as exc:
            self.error(exc)
            return

        self.capture_component()
        draft = self.drafts[self.current_channel]
        draft.update(curves)
        if table == "spring":
            self.spring_lookups.add(self.current_channel)
            self.spring_kind.set(SPRINGS["force"])
        elif table == "dampers":
            draft["damper_kind"] = "force"
            draft.pop("damper_ratio", None)
        elif table == "mr":
            draft["mr_mode"] = "lookup"
        self.load_curve_controls()
        self.component_mode()
        self.status.set(f"{CHANNELS[self.current_channel]}已匯入{label} CSV。")

    def component(self, channel):
        values = self.drafts[channel].copy()
        try:
            if values["spring_kind"] == "force" and channel not in self.spring_lookups:
                raise ValueError("請匯入彈簧力表 CSV。")
            if values["damper_kind"] == "force" and not all(values[side] for side in DAMPER_DIRECTIONS):
                raise ValueError("請匯入阻尼力表 CSV。")
            if values.get("mr_mode") == "lookup" and not values["mr"]:
                raise ValueError("請匯入運動比 CSV。")
            values.update({key: float(values[key]) for key in COMPONENT_FIELDS})
            for key in ("spring", *DAMPER_DIRECTIONS, "mr"):
                values[key] = [[float(x), float(y)] for x, y in values[key]]
        except ValueError as exc:
            raise ValueError(f"{CHANNELS[channel]} 欄位有誤：{exc}") from exc
        return values

    def schedule_component_draw(self, *_):
        if self.current_channel is not None and self.component_draw_job is None:
            self.component_draw_job = self.root.after_idle(self.draw_component)

    def cancel_component_draw(self, _event):
        if self.component_draw_job is not None:
            self.root.after_cancel(self.component_draw_job)
            self.component_draw_job = None

    def draw_component(self):
        self.component_draw_job = None
        self.capture_component()
        try:
            settings = self.component(self.current_channel)
            mass = self.damping_mass() if settings.get("damper_ratio") else None
            component = KCSystem(settings, damping_mass=mass)
        except ValueError as exc:
            self.curve_message.configure(text=str(exc))
            self.curve_message.pack(fill="x", before=self.spring_chart.master)
            self.spring_chart.set({"series": []})
            self.damper_chart.set({"series": []})
            self.mr_chart.set({"series": []})
            return

        notes = []
        if self.current_channel in self.applicable_channels and channel_enabled(settings, self.analysis.family):
            try:
                vehicle, road = prepare_vehicle(self.collect())
                vehicle.equilibrium(road.at(0.), include_aero=False)
                channel_index = tuple(CHANNELS).index(self.current_channel)
                component = next(kc for i, kc in vehicle.active if i == channel_index)
                notes.append(f"{PAGE_TITLES[self.analysis.family]}無空力靜態軸心 {component.static_stroke:.3g} mm")
            except ValueError:
                component.set_static_reference(0.)
                notes.append("目前無法求得靜態行程，圖為行程示意。")
        else:
            component.set_static_reference(0.)
            notes.append(f"{PAGE_TITLES[self.analysis.family]}的選定輪位或啟用設定未使用此通道，圖為行程示意。")
        if mass is not None:
            notes.append(f"臨界阻尼 {component.critical_damping:.4g} N·s/m（阻尼比 1.0）")
        if notes:
            self.curve_message.configure(text="　｜　".join(notes))
            self.curve_message.pack(fill="x", before=self.spring_chart.master)
        else:
            self.curve_message.pack_forget()
        stroke, spring, stops = elastic_curve(component)
        self.spring_chart.set({"title": "彈簧與限位合力", "xlabel": "相對無空力靜態行程 (mm)", "ylabel": "力 (N)",
                               "x_range": (stroke[0], stroke[-1]),
                               "series": [{"x": stroke.tolist(), "y": (spring + stops).tolist()}]})

        compression = [speed for speed, _ in settings["compression"]]
        rebound = [-speed for speed, _ in settings["rebound"]]
        extent = max(.4, compression[-1], -rebound[-1])
        speeds = sorted(set([-extent, *rebound, *compression, extent]))
        self.damper_chart.set({"title": "阻尼曲線", "xlabel": "軸心速度 (m/s)", "ylabel": "阻尼力 (N)",
                               "series": [{"x": speeds, "y": [component.damper_force(speed) for speed in speeds]}]})
        self.mr_chart.set(motion_ratio_chart(component))

    def collect(self, page=None, *, include_pages=False, case=None):
        self.capture_component()
        page = self.analysis if page is None else page
        run = page.case_input() if case is None else {"model": page.model_key, "case": case}
        project = {**self.project, **run, "name": self.name.get().strip(), "model_sources": self.sources()}
        project.pop("page_cases", None)
        project["vehicle"] = self.vehicle_settings()
        try:
            frame = {key: float(var.get()) for key, var in self.roll_frame_vars.items()}
        except ValueError as exc:
            raise ValueError("車架傳力欄位須填入數值。") from exc
        frame["enabled"] = self.roll_frame_enabled.get()
        if frame != roll_frame_options({}) or "roll_center_frame" in self.project:
            project["roll_center_frame"] = frame
        try:
            project["tires"] = {c: {key: float(var.get()) for key, var in variables.items()}
                                for c, variables in self.tire_vars.items()}
        except ValueError as exc:
            raise ValueError("輪胎的欄位須填入數值。") from exc
        project["channels"] = {key: self.component(key) for key in CHANNELS}
        project["aero"] = self.aero_page.settings()
        project.pop("geometry_tables", None)
        project.pop("geometry_modes", None)
        project.update(self.geometry_page.settings())
        if include_pages:
            project["page_cases"] = {key: other.case_input() for key, other in self.analysis_pages.items() if other is not page}
            transfer = self.transfer_page.options()
            if "load_transfer" in self.project or transfer != transfer_options({}):
                project["load_transfer"] = {"distribution": transfer}
        return project

    def vehicle_settings(self):
        try:
            values = {key: float(var.get()) for key, var in self.vehicle_vars.items()}
        except ValueError as exc:
            raise ValueError("車輛的欄位須填入數值。") from exc
        values.update(self.geometry_page.fixed_settings())
        return values

    def editor_state(self):
        project = self.collect(include_pages=True)
        project["page_cases"][self.analysis.family] = {key: project.pop(key) for key in ("model", "case")}
        return project

    def run_csv(self, page):
        if self.job is not None:
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".csv", initialfile="simulation.csv",
                                            filetypes=[("CSV", "*.csv")])
        if path:
            self.run(page, csv_path=Path(path))

    def run(self, page=None, *, scan=False, csv_path=None):
        if self.job is not None:
            return
        if page is None and self.pages.select() == str(self.transfer_page):
            self.transfer_page.draw()
            return
        page = self.analysis if page is None else page
        try:
            project = self.collect(page)
            compile_project(project)
        except ValueError as exc:
            return self.error(exc)
        self.pages.select(page)
        page.playback.pause()
        self.running_page = page
        self.job = threading.Event()
        page.progress["value"] = 0
        self.set_running(True)
        if scan:
            page.show_scan_running()
            self.status.set("正在掃描 21 個 Roll Center 高度…")
        elif csv_path is not None:
            self.status.set(f"{PAGE_TITLES[page.family]}正在純計算並寫入 CSV…")
        else:
            self.status.set(f"{PAGE_TITLES[page.family]}正在求解靜態平衡及動態反應…")
        # Let the export worker remove its temporary file if the app closes.
        threading.Thread(target=self.worker, args=(project, self.job, scan, csv_path),
                         daemon=csv_path is None).start()
        self.root.after(50, self.poll, scan)

    def worker(self, project, cancel, scan=False, csv_path=None):
        progress = lambda fraction: self.events.put(("progress", fraction))
        try:
            if scan:
                result = scan_roll_center(project, cancel.is_set, progress)
                self.events.put(("scan_done", result))
            elif csv_path is not None:
                count, warnings = export_simulation(project, csv_path, cancel.is_set, progress)
                self.events.put(("csv_done", (csv_path, count, warnings)))
            else:
                result = simulate(project, cancel.is_set, progress)
                self.events.put(("done", result))
        except Cancelled:
            self.events.put(("cancelled", None))
        except Exception as exc:
            self.events.put(("error", exc))

    def stop(self):
        if self.job is not None:
            self.job.set()
            self.status.set("正在停止計算…")

    def set_running(self, running):
        for page in self.analysis_pages.values():
            page.run_button.configure(state="disabled" if running else "normal")
            page.compute_csv_button.configure(state="disabled" if running else "normal")
            page.stop_button.configure(state="normal" if running else "disabled")
            if page.family == "half_lr":
                page.scan_button.configure(state="disabled" if running else "normal")

    def poll(self, scan=False):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except Empty:
                self.root.after(50, self.poll, scan)
                return
            if kind == "progress":
                self.running_page.progress["value"] = 100 * payload
                continue
            cancelled = self.job.is_set()
            self.job = None
            page, self.running_page = self.running_page, None
            self.set_running(False)
            if kind == "done" and not cancelled:
                page.set_result(payload)
                self.status.set(f"{PAGE_TITLES[page.family]}已完成 {len(payload.time):,} 筆輸出。")
            elif kind == "scan_done" and not cancelled:
                page.show_scan_result(payload)
                self.status.set("Roll Center 掃描完成。")
            elif kind == "csv_done":
                path, count, warnings = payload
                self.status.set("　｜　".join([f"已匯出 {path.name}（{count:,} 筆）", *warnings]))
            elif kind == "error" and not cancelled:
                if scan:
                    page.show_scan_stopped("掃描失敗。")
                self.status.set("計算失敗。")
                self.error(payload)
            else:
                if scan:
                    page.show_scan_stopped("已停止掃描。")
                self.status.set("已停止計算。")
            return

    def can_replace(self):
        try:
            dirty = self.editor_state() != self.saved
        except ValueError:
            dirty = True
        if dirty:
            return messagebox.askyesno(APP, "目前設定尚未儲存。要捨棄變更嗎？", parent=self.root)
        return True

    def new(self):
        if self.job is None and self.can_replace():
            self.set_project(default_project(), None)
            self.status.set("已建立新專案。")

    def open(self):
        if self.job is not None:
            return
        path = filedialog.askopenfilename(filetypes=[("專案 JSON", "*.json")])
        if not path:
            return
        try:
            project, destination = open_project(path)
        except (OSError, ValueError) as exc:
            return self.error(exc)
        if self.can_replace():
            self.set_project(project, destination)
            self.status.set(" ".join(project["notes"]) or f"已開啟 {Path(path).name}。")

    def save(self):
        try:
            project = self.collect(include_pages=True)
            compile_project(project)
            path = self.path or filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("調校專案", "*.json")])
            if path:
                save_project(path, project)
                self.path, self.saved = path, self.editor_state()
                self.status.set(f"已儲存 {Path(path).name}。")
        except (OSError, ValueError) as exc:
            self.error(exc)

    def close(self):
        if self.can_replace():
            if self.job:
                self.job.set()
            self.root.destroy()

    def error(self, exc):
        messagebox.showerror(APP, str(exc), parent=self.root)


