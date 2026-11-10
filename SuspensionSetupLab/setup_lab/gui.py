"""Offline desktop workspace for vehicle setup, replay and matched A/B runs."""

from pathlib import Path
from queue import Empty, Queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np

from .analysis_page import AnalysisPage
from .chart_base import Chart
from .components import KCSystem
from .engine import Cancelled, simulate
from .project import (APP, CHANNELS, COMPONENT_FIELDS, CORNERS, DEFAULT_SOURCES, MODEL_PAGE, MODEL_SOURCES,
                      PAGE_MODELS, TIRE_FIELDS, VEHICLE_FIELDS, analysis_page_cases, compile_project, default_project, model_channels,
                      model_corners, open_project, read_force_csv, save_project)
from .widgets import entry, key_for, option

SPRINGS = {"stages": "多段剛性", "tender": "主簧＋Tender", "force": "彈簧力表"}
DAMPERS = {"stages": "多段阻尼係數", "force": "測功機力表"}
TABLES = {"spring": "彈簧", "compression": "壓縮阻尼", "rebound": "回彈阻尼", "mr": "運動比 MR"}
PAGE_TITLES = {"full": "全車", "half_lr": "1/2 左右", "half_fr": "1/2 前後", "quarter": "1/4 車"}


def table_text(rows):
    return "\n".join(f"{x}\t{y}" for x, y in rows)


def parse_table(raw, label):
    rows = []
    for line, text in enumerate(raw.splitlines(), 1):
        columns = text.replace(",", " ").split()
        if not columns:
            continue
        try:
            if len(columns) != 2:
                raise ValueError()
            rows.append([float(value) for value in columns])
        except ValueError as exc:
            raise ValueError(f"{label}第 {line} 列需要兩個數值。") from exc
    return rows


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
        self.build_components()
        self.set_project(self.project, path)
        self.pages.bind("<<NotebookTabChanged>>", self.tab_changed)
        for sequence, command in (("<F5>", self.run), ("<Control-o>", self.open), ("<Control-s>", self.save)):
            root.bind(sequence, lambda event, action=command: action())

    def build_vehicle(self):
        page = ttk.Frame(self.pages, padding=14)
        self.pages.add(page, text="  車輛與輪胎  ")
        self.vehicle_vars = {}
        body = ttk.LabelFrame(page, text="整車、車架與幾何傳力", padding=12)
        body.pack(fill="x")
        for index, (key, (label, _, _)) in enumerate(VEHICLE_FIELDS.items()):
            self.vehicle_vars[key] = tk.StringVar()
            entry(body, index // 2, index % 2 * 2, label, self.vehicle_vars[key], 16)
        ttk.Label(body, text="Anti 採比例輸入，例如 0.15 = 15%。只有 8 DOF 車架使用扭轉 k、c。").grid(
            row=8, columnspan=4, sticky="w", pady=(8, 0))
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

    def build_components(self):
        page = ttk.Frame(self.pages, padding=12)
        self.pages.add(page, text="  彈簧與阻尼  ")
        self.channel_list = ttk.Treeview(page, columns=("channel", "state"), show="headings", height=10, selectmode="browse")
        self.channel_list.heading("channel", text="通道")
        self.channel_list.heading("state", text="目前模型")
        self.channel_list.column("channel", width=120, stretch=False)
        self.channel_list.column("state", width=68, stretch=False, anchor="center")
        for key, label in CHANNELS.items():
            self.channel_list.insert("", "end", iid=key, values=(label, ""))
        self.channel_list.pack(side="left", fill="y", padx=(0, 10))
        self.channel_list.bind("<<TreeviewSelect>>", lambda event: self.select_channel())
        edit = ttk.Frame(page)
        edit.pack(side="left", fill="both", expand=True)
        options = ttk.Frame(edit)
        options.pack(fill="x")
        self.enabled = tk.BooleanVar()
        self.spring_kind, self.damper_kind = tk.StringVar(), tk.StringVar()
        ttk.Checkbutton(options, text="啟用此通道", variable=self.enabled, command=self.capture_component).pack(side="left", padx=(0, 8))
        for choices, variable in ((SPRINGS, self.spring_kind), (DAMPERS, self.damper_kind)):
            combo = option(options, choices, variable, 16)
            combo.pack(side="left", padx=(0, 6))
            combo.bind("<<ComboboxSelected>>", lambda event: self.component_mode())
        ttk.Button(options, text="預覽元件曲線", command=self.preview).pack(side="right")
        self.component_vars = {key: tk.StringVar() for key in COMPONENT_FIELDS}
        numeric = ttk.LabelFrame(edit, text="安裝、串聯彈簧與行程限位", padding=8)
        numeric.pack(fill="x", pady=7)
        self.component_widgets = {}
        for index, (key, label) in enumerate(COMPONENT_FIELDS.items()):
            self.component_widgets[key] = entry(numeric, index // 2, index % 2 * 2, label, self.component_vars[key])
        ttk.Label(edit, text="每行兩欄，可直接貼上試算表。分隔符號可用 Tab、空白或逗號，速度為各方向的正值。",
                  wraplength=725).pack(anchor="w", pady=(0, 5))
        tables = ttk.Frame(edit)
        tables.pack(fill="both", expand=True)
        self.tables, self.table_frames = {}, {}
        self.damper_import_buttons = []
        for index, (key, label) in enumerate(TABLES.items()):
            row, column = divmod(index, 2)
            tables.rowconfigure(row, weight=1)
            tables.columnconfigure(column, weight=1, uniform="curves")
            frame = self.table_frames[key] = ttk.LabelFrame(tables, text=label, padding=5)
            frame.grid(row=row, column=column, sticky="nsew", padx=(0, 5), pady=(0, 5))
            if key != "mr":
                actions = ttk.Frame(frame)
                actions.pack(fill="x", pady=(0, 4))
                button = ttk.Button(actions, text="匯入力表 CSV", command=lambda table=key: self.import_force_csv(table))
                button.pack(side="left")
                if key in ("compression", "rebound"):
                    self.damper_import_buttons.append(button)
                if key == "compression":
                    ttk.Button(actions, text="匯入雙向 CSV", command=lambda: self.import_force_csv("dampers")).pack(side="left", padx=5)
            scroll = ttk.Scrollbar(frame)
            scroll.pack(side="right", fill="y")
            field = self.tables[key] = tk.Text(frame, height=4, width=22, wrap="none", undo=True,
                                                font=("Consolas", 10), yscrollcommand=scroll.set)
            field.pack(fill="both", expand=True)
            scroll.configure(command=field.yview)
        self.current_channel = None

    def set_project(self, project, path):
        self.project, self.path = project, path
        sources = {**DEFAULT_SOURCES, **project.get("model_sources", {})}
        for key, source in sources.items():
            self.source_vars[key].set(MODEL_SOURCES[key][source])
        self.name.set(project["name"])
        for key, var in self.vehicle_vars.items():
            var.set(project["vehicle"][key])
        for corner, variables in self.tire_vars.items():
            for key, var in variables.items():
                var.set(project["tires"][corner][key])
        self.drafts = {}
        for key, component in project["channels"].items():
            self.drafts[key] = {**component, **{name: table_text(component[name]) for name in TABLES},
                                **{name: str(component[name]) for name in COMPONENT_FIELDS}}
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

    def refresh_channels(self):
        model = self.analysis.model_key
        self.applicable_channels = model_channels(model, model_corners(model, self.sources()))
        for key in CHANNELS:
            self.show_channel_state(key)
        if self.current_channel not in self.applicable_channels:
            self.channel_list.selection_set(self.applicable_channels[0])
            self.select_channel()

    def sources(self):
        return {key: key_for(MODEL_SOURCES[key], var.get()) for key, var in self.source_vars.items()}

    def capture_component(self):
        key = self.current_channel
        if key is None:
            return
        self.drafts[key].update(enabled=self.enabled.get(), spring_kind=key_for(SPRINGS, self.spring_kind.get()),
                                damper_kind=key_for(DAMPERS, self.damper_kind.get()))
        self.drafts[key].update({name: var.get() for name, var in self.component_vars.items()})
        self.drafts[key].update({name: widget.get("1.0", "end-1c") for name, widget in self.tables.items()})
        self.show_channel_state(key)

    def show_channel_state(self, key):
        state = "啟用" if self.drafts[key]["enabled"] else "停用"
        self.channel_list.set(key, "state", state if key in self.applicable_channels else "不參與")

    def select_channel(self):
        selection = self.channel_list.selection()
        if not selection or selection[0] == self.current_channel:
            return
        self.capture_component()
        self.current_channel = selection[0]
        draft = self.drafts[self.current_channel]
        self.enabled.set(draft["enabled"])
        self.spring_kind.set(SPRINGS[draft["spring_kind"]])
        self.damper_kind.set(DAMPERS[draft["damper_kind"]])
        for key, var in self.component_vars.items():
            var.set(draft[key])
        for key, widget in self.tables.items():
            widget.configure(state="normal")
            widget.delete("1.0", "end")
            widget.insert("1.0", draft[key])
            widget.edit_reset()
        self.component_mode()

    def component_mode(self):
        spring = key_for(SPRINGS, self.spring_kind.get())
        damper = key_for(DAMPERS, self.damper_kind.get())
        self.tables["spring"].configure(state="disabled" if spring == "tender" else "normal")
        self.table_frames["spring"].configure(text={"stages": "彈簧｜起始行程 mm ／剛性 N/mm",
                                                    "force": "彈簧｜行程 mm ／力 N",
                                                    "tender": "彈簧｜串聯數值由上方欄位設定"}[spring])
        self.table_frames["mr"].configure(text="運動比｜輪端／模態行程 mm ／MR")
        unit = "係數 N·s/m" if damper == "stages" else "力 N"
        for key in ("compression", "rebound"):
            self.table_frames[key].configure(text=f"{TABLES[key]}｜速度 m/s ／{unit}")
        for button in self.damper_import_buttons:
            button.configure(state="normal" if damper == "force" else "disabled")
        for key in ("main_rate", "tender_rate", "tender_travel"):
            self.component_widgets[key].configure(state="normal" if spring == "tender" else "disabled")

    def import_force_csv(self, table):
        label = "雙向阻尼" if table == "dampers" else TABLES[table]
        path = filedialog.askopenfilename(parent=self.root, title=f"匯入{label}力表", filetypes=[("CSV 檔", "*.csv")])
        if not path:
            return
        try:
            curves = read_force_csv(path, table)
        except (OSError, ValueError) as exc:
            self.error(exc)
            return

        if table == "spring":
            self.spring_kind.set(SPRINGS["force"])
        else:
            self.damper_kind.set(DAMPERS["force"])
        self.component_mode()
        for key, rows in curves.items():
            self.tables[key].replace("1.0", "end-1c", table_text(rows))
        self.capture_component()
        self.status.set(f"{CHANNELS[self.current_channel]}已匯入{label}力表。")

    def component(self, channel):
        values = self.drafts[channel].copy()
        try:
            values.update({key: float(values[key]) for key in COMPONENT_FIELDS})
            values.update({key: parse_table(values[key], TABLES[key]) for key in TABLES})
        except ValueError as exc:
            raise ValueError(f"{CHANNELS[channel]} 欄位有誤：{exc}") from exc
        return values

    def collect(self, page=None, *, include_pages=False):
        self.capture_component()
        page = self.analysis if page is None else page
        project = {**self.project, **page.case_input(), "name": self.name.get().strip(), "model_sources": self.sources()}
        project.pop("page_cases", None)
        try:
            project["vehicle"] = {key: float(var.get()) for key, var in self.vehicle_vars.items()}
            project["tires"] = {c: {key: float(var.get()) for key, var in variables.items()}
                                for c, variables in self.tire_vars.items()}
        except ValueError as exc:
            raise ValueError("車輛與輪胎的欄位須填入數值。") from exc
        project["channels"] = {key: self.component(key) for key in CHANNELS}
        if include_pages:
            project["page_cases"] = {key: other.case_input() for key, other in self.analysis_pages.items() if other is not page}
        return project

    def editor_state(self):
        project = self.collect(include_pages=True)
        project["page_cases"][self.analysis.family] = {key: project.pop(key) for key in ("model", "case")}
        return project

    def preview(self):
        self.capture_component()
        key = self.current_channel
        try:
            settings = self.component(key)
            component = KCSystem(settings)
        except ValueError as exc:
            return self.error(exc)
        window = tk.Toplevel(self.root)
        window.title(f"{CHANNELS[key]}｜元件台架曲線")
        window.geometry("825x520")
        mode = ttk.Combobox(window, values=("彈簧與限位", "阻尼測功曲線", "運動比"), state="readonly")
        mode.current(0)
        mode.pack(anchor="w", padx=8, pady=6)
        chart = Chart(window)
        chart.pack(fill="both", expand=True)

        def draw(event=None):
            if mode.current() == 0:
                x = np.unique(np.r_[np.linspace(-80, 80, 481), -settings["rebound_mm"], settings["jounce_mm"],
                                    [row[0] - settings["installed_mm"] for row in settings["spring"]]])
                series = [("彈簧", [component.spring_force(v) for v in x]),
                          ("限位", [component.stop_force(v) for v in x])]
                xlabel, ylabel = "軸心壓縮行程 (mm)", "力 (N)"
            elif mode.current() == 1:
                extent = max(.4, settings["compression"][-1][0], settings["rebound"][-1][0])
                x = np.linspace(-extent, extent, 481)
                series = [("阻尼力", [component.damper_force(v) for v in x])]
                xlabel, ylabel = "軸心速度 (m/s)", "力 (N)"
            else:
                extent = max(50, *(abs(row[0]) for row in settings["mr"]))
                x = np.linspace(-extent, extent, 481)
                series = [("MR", [component.motion_ratio.lookup(v) for v in x])]
                xlabel, ylabel = "輪端／模態行程 (mm)", "運動比"
            chart.set({"title": mode.get(), "xlabel": xlabel, "ylabel": ylabel,
                       "series": [{"name": name, "x": x.tolist(), "y": values} for name, values in series]})

        mode.bind("<<ComboboxSelected>>", draw)
        draw()

    def run(self, page=None):
        if self.job is not None:
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
        self.status.set(f"{PAGE_TITLES[page.family]}正在求解靜態平衡及動態反應…")
        threading.Thread(target=self.worker, args=(project, self.job), daemon=True).start()
        self.root.after(50, self.poll)

    def worker(self, project, cancel):
        try:
            result = simulate(project, cancel.is_set, lambda fraction: self.events.put(("progress", fraction)))
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
            page.stop_button.configure(state="normal" if running else "disabled")

    def poll(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except Empty:
                self.root.after(50, self.poll)
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
            elif kind == "error" and not cancelled:
                self.status.set("計算失敗。")
                self.error(payload)
            else:
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


