"""Editable suspension types and CSV curve import panels."""

from copy import deepcopy
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from ._vendor.suspension.components import damper_kinds
from ._vendor.suspension.project import CHANNELS, channel_enabled

from .suspension_curves import read_curve_csv, save_curve_template
from .workbench import number


SPRINGS = {"single": "單一彈簧", "tender": "主簧＋Tender", "table": "CSV 查表"}
SPRING_FIELDS = {
    "single": {"main_rate"},
    "tender": {"main_rate", "tender_rate", "tender_after_static_mm"},
    "stages": {"spring"}, "force": {"spring"},
}
SPRING_INPUTS = set().union(*SPRING_FIELDS.values(), {"tender_travel", "tender_bind_mm"})
DAMPERS = {"single": "單一係數", "split": "壓縮／回彈各一段", "bilinear": "壓縮／回彈各兩段",
           "table": "CSV 查表"}
BOTTOMING = (("jounce_mm", "觸底行程 (mm)"), ("jounce_rate", "觸底後總剛性 (N/mm)"))
LIMITS = (("installed_mm", "安裝預壓 (mm)"), ("rebound_mm", "伸長限位起點 (mm)"),
          ("rebound_rate", "伸長限位後總剛性 (N/mm)"))
DIRECTIONS = (("compression", "壓縮"), ("rebound", "回彈"))


class SuspensionEditor:
    def __init__(self, parent):
        self.parent = parent
        self.channel = next(iter(CHANNELS))
        self.channels = {}
        self.vars = {}
        self.widgets = {}
        self.csv_tables = {}
        self.table_notes = {}
        self.name = tk.StringVar()
        self.enabled = tk.BooleanVar()
        self.ratio = tk.BooleanVar()
        self.spring = tk.StringVar()
        self.damper = tk.StringVar()
        self.motion = tk.StringVar()
        self.spring_help = tk.StringVar()
        self.damper_help = tk.StringVar()
        selector = ttk.Combobox(parent, textvariable=self.name, values=list(CHANNELS.values()), state="readonly")
        selector.pack(fill="x")
        selector.bind("<<ComboboxSelected>>", self.switch_channel)
        ttk.Checkbutton(parent, text="啟用此元件（全車）", variable=self.enabled).pack(anchor="w", pady=4)
        tabs = ttk.Notebook(parent)
        tabs.pack(fill="both", expand=True)
        spring, damper, motion = (ttk.Frame(tabs, padding=8) for _ in range(3))
        for page, label in ((spring, "彈簧"), (damper, "阻尼"), (motion, "運動比與限位")):
            tabs.add(page, text=label)
            page.columnconfigure(1, weight=1)

        self.choose(spring, 0, "彈簧類型", self.spring, SPRINGS.values())
        self.field(spring, 1, "main_rate", "彈簧／主簧剛性 (N/mm)")
        self.field(spring, 2, "tender_rate", "Tender 剛性 (N/mm)")
        self.field(spring, 3, "tender_after_static_mm", "靜態後併圈行程 (mm)")
        self.table_panel(spring, 4, "spring")
        ttk.Label(spring, textvariable=self.spring_help, wraplength=350, justify="left").grid(
            row=5, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Separator(spring).grid(row=6, columnspan=2, sticky="ew", pady=8)
        for i, (key, label) in enumerate(BOTTOMING, 7):
            self.field(spring, i, key, label)
        ttk.Label(spring, text="觸底行程以無空力靜態平衡的軸心位置為零。觸底後彈簧力連續，斜率改用此總剛性。填 0 N/mm 可停用觸底。",
                  wraplength=350, justify="left").grid(row=9, columnspan=2, sticky="w", pady=(6, 0))

        self.choose(damper, 0, "阻尼類型", self.damper, DAMPERS.values())
        self.widgets["ratio"] = (ttk.Checkbutton(damper, text="以阻尼比輸入（例如 0.7）", variable=self.ratio,
                                                command=self.refresh),)
        self.widgets["ratio"][0].grid(row=1, columnspan=2, sticky="w")
        for i, (key, label) in enumerate(DIRECTIONS, 2):
            self.field(damper, i, key + "_coefficient", label + "係數")
        self.two_stage_panel(damper, 2)
        self.table_panel(damper, 2, "damper")
        ttk.Label(damper, textvariable=self.damper_help, wraplength=350, justify="left").grid(
            row=4, columnspan=2, sticky="w", pady=(6, 0))

        self.choose(motion, 0, "運動比類型", self.motion, ("固定值", "CSV 查表"))
        self.field(motion, 1, "fixed_mr", "固定運動比")
        self.table_panel(motion, 1, "motion")
        for i, (key, label) in enumerate(LIMITS, 2):
            self.field(motion, i, key, label)
        ttk.Label(motion, text="CSV 欄名為 travel_mm,mr。行程用 mm，運動比須為正且無單位。限位剛性填 0 代表停用。",
                  wraplength=350, justify="left").grid(row=7, columnspan=2, sticky="w", pady=(6, 0))

    def choose(self, page, row, label, var, values):
        ttk.Label(page, text=label).grid(row=row, column=0, padx=(0, 7), sticky="w")
        item = ttk.Combobox(page, state="readonly", values=list(values), textvariable=var)
        item.grid(row=row, column=1, sticky="ew", pady=2)
        item.bind("<<ComboboxSelected>>", lambda _event: self.refresh())

    def field(self, page, row, key, label):
        var = tk.StringVar()
        self.vars[key] = var
        caption, item = ttk.Label(page, text=label), ttk.Entry(page, textvariable=var, width=12)
        caption.grid(row=row, column=0, sticky="w", padx=(0, 7), pady=2)
        item.grid(row=row, column=1, sticky="ew", pady=2)
        self.widgets[key] = caption, item

    def two_stage_panel(self, page, row):
        panel = ttk.Frame(page)
        panel.grid(row=row, columnspan=2, rowspan=2, sticky="ew")
        for i, (direction, label) in enumerate(DIRECTIONS, 1):
            panel.columnconfigure(i, weight=1)
            ttk.Label(panel, text=label).grid(row=0, column=i, pady=2)
            for j, (key, caption) in enumerate((("low", "低速係數"), ("high", "高速係數"),
                                                 ("switch", "切換速度 (m/s)")), 1):
                if i == 1:
                    ttk.Label(panel, text=caption).grid(row=j, column=0, sticky="w", padx=(0, 7))
                var = tk.StringVar()
                self.vars[f"{direction}_{key}"] = var
                ttk.Entry(panel, textvariable=var, width=9).grid(row=j, column=i, sticky="ew", padx=3, pady=2)
        self.widgets["bilinear"] = (panel,)

    def table_panel(self, page, row, family):
        panel = ttk.Frame(page)
        panel.grid(row=row, columnspan=2, sticky="ew")
        note = tk.StringVar()
        self.table_notes[family] = note
        ttk.Button(panel, text="匯入 CSV…", command=lambda: self.import_csv(family)).pack(anchor="w")
        actions = ttk.Frame(panel)
        actions.pack(fill="x", pady=3)
        coefficient = "剛性範本" if family == "spring" else "阻尼係數範本"
        templates = (("ratio", "運動比範本"),) if family == "motion" else (("stages", coefficient), ("force", "力曲線範本"))
        for kind, label in templates:
            ttk.Button(actions, text=label, command=lambda k=kind: self.save_template(family, k)).pack(
                side="left", padx=(0, 4))
        ttk.Label(panel, textvariable=note, wraplength=350).pack(anchor="w")
        self.widgets[family + "_table"] = (panel,)

    def import_csv(self, family):
        path = filedialog.askopenfilename(parent=self.parent, filetypes=[("曲線 CSV", "*.csv")])
        if not path:
            return
        try:
            self.csv_tables[family] = read_curve_csv(path, family)
        except (OSError, ValueError) as error:
            messagebox.showerror("無法匯入 CSV", str(error), parent=self.parent)
            return
        self.refresh()

    def save_template(self, family, kind):
        path = filedialog.asksaveasfilename(parent=self.parent, defaultextension=".csv",
                                            initialfile=f"{family}_{kind}.csv", filetypes=[("CSV", "*.csv")])
        if path:
            try:
                save_curve_template(path, family, kind)
            except OSError as error:
                messagebox.showerror("無法儲存範本", str(error), parent=self.parent)

    @property
    def spring_kind(self):
        return next(key for key, label in SPRINGS.items() if label == self.spring.get())

    @property
    def damper_kind(self):
        return next(key for key, label in DAMPERS.items() if label == self.damper.get())

    def refresh(self):
        spring, damper = self.spring_kind, self.damper_kind
        scalar = damper in ("single", "split")
        modes = {
            "main_rate": spring in ("single", "tender"), "tender_rate": spring == "tender",
            "tender_after_static_mm": spring == "tender", "spring_table": spring == "table",
            "ratio": damper in ("single", "split", "bilinear"),
            "compression_coefficient": scalar, "rebound_coefficient": damper == "split",
            "bilinear": damper == "bilinear", "damper_table": damper == "table",
            "fixed_mr": self.motion.get() == "固定值", "motion_table": self.motion.get() == "CSV 查表",
        }
        for key, show in modes.items():
            for widget in self.widgets[key]:
                widget.grid() if show else widget.grid_remove()
        self.widgets["main_rate"][0].configure(text="主簧剛性 (N/mm)" if spring == "tender" else "彈簧剛性 (N/mm)")
        self.widgets["compression_coefficient"][0].configure(text="共用係數" if damper == "single" else "壓縮係數")
        self.spring_help.set({
            "single": "只填一個彈簧剛性，例如 140 N/mm。",
            "tender": "分別填主簧與 Tender 剛性，再填靜態位置之後需要多少軸心壓縮行程才併圈。",
            "table": "CSV 欄名決定使用「行程 × 分段剛性」或「行程 × 彈簧力」。可下載範本。",
        }[spring])
        unit = "阻尼比" if self.ratio.get() and modes["ratio"] else "N·s/m"
        self.damper_help.set({
            "single": f"壓縮、回彈共用同一個係數（{unit}）。",
            "split": f"分別填壓縮與回彈係數（{unit}）。兩者都以正值輸入。",
            "bilinear": f"壓縮與回彈分別填低速、高速係數（{unit}）及切換速度。兩段在切換點連續。",
            "table": "CSV 同時載入壓縮與回彈。速度用 m/s 並從 0 開始，欄名決定數值為係數或力。",
        }[damper])
        for family, note in self.table_notes.items():
            table = self.csv_tables.get(family)
            if table is None:
                note.set("尚未匯入")
                continue
            kind, rows = table
            if family == "spring":
                label = "分段剛性" if kind == "stages" else "彈簧力"
                note.set(f"{label} · {len(rows)} 列")
            elif family == "motion":
                note.set(f"運動比 · {len(rows)} 列")
            else:
                label = "分段係數" if kind == "stages" else ("阻尼力" if kind == "force" else "舊混合表")
                note.set(f"{label} · 壓縮 {len(rows[0])} 列／回彈 {len(rows[1])} 列")

    def fill(self, channels):
        self.channels = deepcopy(channels)
        self.load_channel()

    def load_channel(self):
        current = self.channels[self.channel]
        self.name.set(CHANNELS[self.channel])
        self.enabled.set(channel_enabled(current, "full"))
        self.ratio.set(current.get("damper_ratio", False))
        spring_kind = current["spring_kind"]
        self.spring.set(SPRINGS.get(spring_kind, SPRINGS["table"]))
        kind = current["damper_kind"]
        scalar_kinds = ("single", "split", "bilinear")
        self.damper.set(DAMPERS[kind] if kind in scalar_kinds else DAMPERS["table"])
        self.motion.set("固定值" if len(current["mr"]) == 1 else "CSV 查表")
        for key, var in self.vars.items():
            var.set(current.get(key, ""))
        for key, _ in DIRECTIONS:
            staged = current[key] if damper_kinds(kind)[key] == "stages" else [[0., 0.]]
            low = staged[0][1]
            switch, high = staged[1] if len(staged) > 1 else (.1, low)
            for field, value in (("coefficient", low), ("low", low), ("high", high), ("switch", switch)):
                self.vars[f"{key}_{field}"].set(value)
        self.vars["fixed_mr"].set(current["mr"][0][1])
        self.csv_tables = {}
        if len(current["mr"]) > 1:
            self.csv_tables["motion"] = "ratio", current["mr"]
        if spring_kind in ("stages", "force"):
            self.csv_tables["spring"] = spring_kind, current["spring"]
        if kind not in scalar_kinds:
            self.csv_tables["damper"] = kind, (current["compression"], current["rebound"])
        self.refresh()

    def switch_channel(self, _event):
        target = next(key for key, label in CHANNELS.items() if label == self.name.get())
        try:
            self.channels[self.channel] = self.read_channel()
        except ValueError as error:
            self.name.set(CHANNELS[self.channel])
            messagebox.showerror("請檢查懸吊資料", str(error), parent=self.parent)
            return
        self.channel = target
        self.load_channel()

    def read_channel(self):
        result = deepcopy(self.channels[self.channel])
        result.setdefault("enabled_models", {})["full"] = self.enabled.get()
        for key, label in (*LIMITS, *BOTTOMING):
            result[key] = number(self.vars[key].get(), label, minimum=None if key == "installed_mm" else 0.)
        result["limit_rate_mode"] = "total"
        spring = self.spring_kind
        result["spring_kind"] = spring
        for key in SPRING_INPUTS:
            result.pop(key, None)
        if spring in ("single", "tender"):
            result["main_rate"] = number(self.vars["main_rate"].get(), "彈簧剛性",
                                         minimum=1e-6 if spring == "tender" else 0.)
            if spring == "tender":
                result["tender_rate"] = number(self.vars["tender_rate"].get(), "Tender 剛性", minimum=1e-6)
                result["tender_after_static_mm"] = number(self.vars["tender_after_static_mm"].get(), "併圈行程", minimum=0.)
        else:
            if "spring" not in self.csv_tables:
                raise ValueError("請先匯入彈簧 CSV。")
            result["spring_kind"], result["spring"] = self.csv_tables["spring"]
        damper = self.damper_kind
        result["damper_kind"] = damper
        result["damper_ratio"] = self.ratio.get() if damper in ("single", "split", "bilinear") else False
        if damper == "table":
            if "damper" not in self.csv_tables:
                raise ValueError("請先匯入阻尼 CSV。")
            result["damper_kind"], (result["compression"], result["rebound"]) = self.csv_tables["damper"]
        else:
            for key, label in DIRECTIONS:
                if damper == "bilinear":
                    low = number(self.vars[key + "_low"].get(), label + "低速阻尼", minimum=0.)
                    high = number(self.vars[key + "_high"].get(), label + "高速阻尼", minimum=0.)
                    switch = number(self.vars[key + "_switch"].get(), label + "切換速度", minimum=1e-9)
                    result[key] = [[0., low], [switch, high]]
                else:
                    field = "compression" if damper == "single" else key
                    result[key] = [[0., number(self.vars[field + "_coefficient"].get(), label + "阻尼", minimum=0.)]]
        if self.motion.get() == "固定值":
            result["mr"] = [[0., number(self.vars["fixed_mr"].get(), "固定運動比", minimum=1e-6)]]
        else:
            if "motion" not in self.csv_tables:
                raise ValueError("請先匯入運動比 CSV。")
            _, result["mr"] = self.csv_tables["motion"]
        return result

    def capture(self):
        channels = self.channels | {self.channel: self.read_channel()}
        return {key: {field: value for field, value in settings.items()
                      if field not in SPRING_INPUTS or field in SPRING_FIELDS[settings["spring_kind"]]}
                for key, settings in channels.items()}
