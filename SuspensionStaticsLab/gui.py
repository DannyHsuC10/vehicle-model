"""Desktop interface for editing loads, geometry and axial sections."""

import csv
from math import fsum, hypot, isfinite
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .load_cases import ACCEL, CASES, DEFAULT_ACCELERATION, DESCRIPTIONS, GRAVITY, MASS_KG, STATIC, example_loads
from .model import CHASSIS, EXAMPLE, build_model, number, read_hardpoints, read_topology
from .section_view import SectionSketch
from .solver import Load, Section, member_results, solve
from .viewport import FONT, VIEWS, Viewport


PAPER = "#f4f7fa"
TEXT = "#172839"
SECONDARY = "#496073"
FORCE = "硬點 XYZ 外力"
MOMENT_A = "桿端 A 力矩"
MOMENT_B = "桿端 B 力矩"
LOAD_TYPES = {FORCE: "", MOMENT_A: "A", MOMENT_B: "B"}
END_TYPES = {end: label for label, end in LOAD_TYPES.items()}
SECTION_FIELDS = (
    ("截面積 A", "area_mm2", "mm²"),
    ("截面二次矩 I", "second_moment_mm4", "mm⁴"),
    ("截面係數 S", "modulus_mm3", "mm³"),
    ("截面迴轉半徑 r", "gyration_mm", "mm"),
    ("極慣性矩 J", "polar_moment_mm4", "mm⁴"),
    ("極截面係數 Zₚ", "polar_modulus_mm3", "mm³"),
    ("極迴轉半徑 rₚ", "polar_gyration_mm", "mm"),
)


def numeric(value, digits=5):
    return f"{0. if abs(value) < 1e-11 else value:.{digits}g}"


def input_magnitude(variables):
    try:
        magnitude = hypot(*(number(var.get(), "分量") for var in variables))
    except ValueError:
        return "—"
    return numeric(magnitude) if isfinite(magnitude) else "—"


def row_table(parent, fields, height, *, selectmode="browse"):
    box = ttk.Frame(parent)
    box.columnconfigure(0, weight=1)
    box.rowconfigure(0, weight=1)
    columns = [field for field, _ in fields]
    tree = ttk.Treeview(box, columns=columns, show="headings", height=height, selectmode=selectmode)
    vertical = ttk.Scrollbar(box, orient="vertical", command=tree.yview)
    horizontal = ttk.Scrollbar(box, orient="horizontal", command=tree.xview)
    tree.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
    tree.grid(row=0, column=0, sticky="nsew")
    vertical.grid(row=0, column=1, sticky="ns")
    horizontal.grid(row=1, column=0, sticky="ew")
    for index, (field, width) in enumerate(fields):
        tree.heading(field, text=field, anchor="w" if index == 0 else "center")
        tree.column(field, width=width, minwidth=48, stretch=index == 0, anchor="w" if index == 0 else "e")
    return box, tree


def set_rows(tree, rows):
    tree.delete(*tree.get_children())
    for index, (key, values) in enumerate(rows):
        tree.insert("", "end", iid=key, values=values, tags=("alternate",) if index % 2 else ())
    tree.tag_configure("alternate", background="#edf3f8")


def select(tree, key):
    if tree.exists(key):
        tree.selection_set(key)
        tree.focus(key)
        tree.see(key)


def entry_fields(parent, labels, variables):
    for col, (label, var) in enumerate(zip(labels, variables)):
        parent.columnconfigure(col, weight=1)
        ttk.Label(parent, text=label, style="Muted.TLabel").grid(row=0, column=col, sticky="w", padx=(0, 6))
        ttk.Entry(parent, textvariable=var, width=9).grid(row=1, column=col, sticky="ew", padx=(0, 6), pady=(3, 0))


class App:
    def __init__(self, root):
        self.root = root
        root.title("Suspension Statics Lab · 懸吊靜力分析")
        root.geometry("1380x810")
        root.minsize(1040, 660)
        root.configure(background=PAPER)
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure(".", font=(FONT, 10), background=PAPER, foreground=TEXT)
        style.configure("TLabel", background=PAPER)
        style.configure("Title.TLabel", font=(FONT, 22, "bold"))
        style.configure("Heading.TLabel", font=(FONT, 12, "bold"))
        style.configure("Muted.TLabel", foreground=SECONDARY)
        style.configure("Error.TLabel", foreground="#9e2531")
        style.configure("TButton", padding=(8, 5))
        style.configure("Primary.TButton", background="#125677", foreground="white", font=(FONT, 10, "bold"))
        style.map("Primary.TButton", background=[("active", "#196b93")])
        style.configure("Treeview", background="white", fieldbackground="white", rowheight=27)
        style.configure("Treeview.Heading", font=(FONT, 9, "bold"), background="#e1eaf2")
        style.map("Treeview", background=[("selected", "#1c658b")], foreground=[("selected", "white")])
        style.configure("TNotebook.Tab", padding=(13, 7))
        style.map("TNotebook.Tab", background=[("selected", "#dcecf7")])

        self.model = None
        self.topology = None
        self.loads = []
        self.sections = {}
        self.equilibrium = None
        self.results = {}
        self.selected = None
        self.source = tk.StringVar(root)
        self.metrics = tk.StringVar(root)
        self.status = tk.StringVar(root)
        self.selection = tk.StringVar(root, value="點選硬點或桿件以查看明細")
        self.result_summary = tk.StringVar(root)
        self.case = tk.StringVar(root, value=STATIC)
        self.case_note = tk.StringVar(root, value=DESCRIPTIONS[STATIC])
        self.mass_input = tk.StringVar(root, value=numeric(MASS_KG))
        self.acceleration_inputs = [tk.StringVar(root, value=numeric(value)) for value in DEFAULT_ACCELERATION]
        self.acceleration_magnitude = tk.StringVar(root)
        self.include_gravity = tk.BooleanVar(root, value=True)
        self.load_kind = tk.StringVar(root, value=FORCE)
        self.load_target = tk.StringVar(root)
        self.load_hint = tk.StringVar(root)
        self.load_values = [tk.StringVar(root, value="0") for _ in range(3)]
        self.load_magnitude_label = tk.StringVar(root)
        self.load_magnitude = tk.StringVar(root)
        self.total_force = tk.StringVar(root)
        self.section_name = tk.StringVar(root)
        self.section_values = [tk.StringVar(root) for _ in range(3)]
        self.section_properties = {field: tk.StringVar(root) for _, field, _ in SECTION_FIELDS}

        page = ttk.Frame(root, padding=(20, 15, 20, 13))
        page.pack(fill="both", expand=True)
        header = ttk.Frame(page)
        header.pack(fill="x")
        ttk.Label(header, text="懸吊靜力分析", style="Title.TLabel").pack(side="left")
        ttk.Label(header, textvariable=self.metrics, style="Muted.TLabel").pack(side="right")
        bar = ttk.Frame(page)
        bar.pack(fill="x", pady=(8, 10))
        for label, action in (("匯入硬點 CSV", self.import_csv), ("載入拓撲 JSON", self.import_topology),
                              ("四輪範例幾何", self.load_demo), ("編輯硬點 XYZ", self.edit_points)):
            ttk.Button(bar, text=label, command=action).pack(side="left", padx=(0, 6))
        ttk.Button(bar, text="求解桿件受力", style="Primary.TButton", command=self.solve_model).pack(side="right")
        ttk.Label(bar, textvariable=self.source, style="Muted.TLabel").pack(side="left", padx=(8, 0))

        panes = ttk.Panedwindow(page, orient="horizontal")
        panes.pack(fill="both", expand=True)
        geometry = ttk.Frame(panes, padding=(0, 0, 10, 0))
        panes.add(geometry, weight=3)
        views = ttk.Frame(geometry)
        views.pack(fill="x", pady=(0, 7))
        self.viewport = Viewport(geometry, self.on_pick)
        for view in VIEWS:
            ttk.Button(views, text=view, width=5, command=lambda key=view: self.viewport.set_view(key)).pack(side="left", padx=(0, 4))
        self.labels = tk.BooleanVar(root)
        ttk.Checkbutton(views, text="點名", variable=self.labels, command=self.toggle_labels).pack(side="left", padx=7)
        controls_hint = ttk.Label(views, text="左鍵旋轉  ·  右鍵平移  ·  滾輪縮放", style="Muted.TLabel")

        def place_controls_hint(event):
            if event.width >= 520:
                controls_hint.place(relx=1, rely=.5, anchor="e")
            else:
                controls_hint.place_forget()

        views.bind("<Configure>", place_controls_hint)
        ttk.Label(geometry, textvariable=self.selection, style="Heading.TLabel", wraplength=650).pack(side="bottom", anchor="w")
        ttk.Label(geometry, text="實線 軸力桿件　·　虛線 剛體示意　·　方點 車身硬點", style="Muted.TLabel").pack(side="bottom", anchor="w", pady=(6, 2))
        self.viewport.pack(fill="both", expand=True)

        self.tabs = ttk.Notebook(panes)
        panes.add(self.tabs, weight=2)
        self.load_tab = ttk.Frame(self.tabs, padding=12)
        self.section_tab = ttk.Frame(self.tabs, padding=12)
        self.result_tab = ttk.Frame(self.tabs, padding=12)
        for tab, label in ((self.load_tab, "外載工況"), (self.section_tab, "桿件截面"), (self.result_tab, "求解結果")):
            self.tabs.add(tab, text=label)
        self.create_load_tab()
        self.create_section_tab()
        self.create_result_tab()
        self.status_label = ttk.Label(page, textvariable=self.status, style="Muted.TLabel", wraplength=980)
        self.status_label.pack(before=panes, side="bottom", anchor="w", pady=(11, 0))
        self.load_demo()

    def create_load_tab(self):
        tab = self.load_tab
        ttk.Label(tab, text="範例載入", style="Heading.TLabel").pack(anchor="w")
        cases = ttk.Frame(tab)
        cases.pack(fill="x", pady=(6, 4))
        choice = ttk.Combobox(cases, state="readonly", values=CASES, textvariable=self.case, width=17)
        choice.pack(side="left", padx=(0, 6))
        ttk.Button(cases, text="套用範例", command=self.apply_example).pack(side="left")
        self.case_fields = ttk.Frame(tab)
        entry_fields(self.case_fields, ("重心質量 kg", "ax m/s²", "ay m/s²", "az m/s²"),
                     (self.mass_input, *self.acceleration_inputs))
        gravity_toggle = ttk.Checkbutton(self.case_fields, text=f"計入重力（g = {GRAVITY:g} m/s²，向下）",
                                         variable=self.include_gravity)
        gravity_toggle.grid(row=2, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Label(self.case_fields, textvariable=self.acceleration_magnitude, style="Muted.TLabel").grid(
            row=2, column=2, columnspan=2, sticky="e", padx=(0, 6), pady=(4, 0))
        for var in self.acceleration_inputs:
            var.trace_add("write", self.update_acceleration_magnitude)
        self.update_acceleration_magnitude()
        self.case_caption = ttk.Label(tab, textvariable=self.case_note, style="Muted.TLabel", wraplength=420)
        self.case_caption.pack(anchor="w")
        self.case.trace_add("write", self.update_case_form)
        ttk.Separator(tab).pack(fill="x", pady=10)

        load_form = ttk.Frame(tab)
        load_form.pack(fill="x")
        kind = ttk.Combobox(load_form, values=tuple(LOAD_TYPES), state="readonly", textvariable=self.load_kind, width=13)
        kind.pack(side="left", padx=(0, 7))
        kind.bind("<<ComboboxSelected>>", self.update_targets)
        self.targets = ttk.Combobox(load_form, state="readonly", textvariable=self.load_target, width=19)
        self.targets.pack(side="left", fill="x", expand=True)
        self.targets.bind("<<ComboboxSelected>>", self.update_load_hint)
        ttk.Label(tab, textvariable=self.load_hint, style="Muted.TLabel", wraplength=420).pack(anchor="w", pady=(5, 0))
        xyz = ttk.Frame(tab)
        xyz.pack(fill="x", pady=7)
        entry_fields(xyz, ("X 分量", "Y 分量", "Z 分量"), self.load_values)
        xyz.columnconfigure(3, weight=1)
        ttk.Label(xyz, textvariable=self.load_magnitude_label, style="Muted.TLabel").grid(row=0, column=3, sticky="w")
        ttk.Label(xyz, textvariable=self.load_magnitude).grid(row=1, column=3, sticky="w", pady=(3, 0))
        for var in (self.load_kind, *self.load_values):
            var.trace_add("write", self.update_load_magnitude)
        self.update_load_magnitude()
        buttons = ttk.Frame(tab)
        buttons.pack(fill="x", pady=(0, 8))
        for label, action in (("新增", self.add_load), ("更新選取", self.update_load), ("刪除選取", self.delete_load), ("清空", self.clear_loads)):
            ttk.Button(buttons, text=label, command=action).pack(side="left", padx=(0, 4))
        ttk.Label(tab, textvariable=self.total_force, style="Muted.TLabel").pack(anchor="w", pady=(0, 4))
        fields = (("目標", 145), ("型態", 66), ("X", 62), ("Y", 62), ("Z", 62), ("合量", 68))
        box, self.load_tree = row_table(tab, fields, 7)
        box.pack(fill="both", expand=True)
        self.load_tree.bind("<<TreeviewSelect>>", self.choose_load)

    def create_section_tab(self):
        tab = self.section_tab
        ttk.Label(tab, text="圓形管材截面", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(tab, text="在下方表格用 Ctrl 或 Shift 複選。內徑為 0 時為實心圓桿。", style="Muted.TLabel", wraplength=410).pack(anchor="w", pady=(4, 8))
        self.section_picker = ttk.Combobox(tab, state="readonly", textvariable=self.section_name)
        self.section_picker.pack(fill="x")
        self.section_picker.bind("<<ComboboxSelected>>", self.choose_section)
        dims = ttk.Frame(tab)
        dims.pack(fill="x", pady=8)
        entry_fields(dims, ("外徑 mm", "內徑 mm", "楊氏模數 GPa"), self.section_values)
        actions = ttk.Frame(tab)
        actions.pack(fill="x", pady=(0, 6))
        ttk.Button(actions, text="套用至選取桿件", command=lambda: self.apply_section(False)).pack(side="left", padx=(0, 6))
        ttk.Button(actions, text="套用至全部", command=lambda: self.apply_section(True)).pack(side="left")
        preview = ttk.Frame(tab)
        preview.pack(fill="x", pady=(2, 8))
        self.section_sketch = SectionSketch(preview)
        self.section_sketch.pack(side="left", padx=(0, 12))
        properties = ttk.Frame(preview)
        properties.pack(side="left", fill="x", expand=True)
        properties.columnconfigure(1, weight=1)
        for row, (label, field, unit) in enumerate(SECTION_FIELDS):
            ttk.Label(properties, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Label(properties, textvariable=self.section_properties[field]).grid(row=row, column=1, sticky="e", padx=6)
            ttk.Label(properties, text=unit, style="Muted.TLabel", width=4).grid(row=row, column=2, sticky="w")
        for var in self.section_values[:2]:
            var.trace_add("write", self.refresh_section_preview)
        fields = (("桿件", 145), ("外徑 mm", 77), ("內徑 mm", 77), ("E GPa", 71))
        box, self.section_tree = row_table(tab, fields, 9, selectmode="extended")
        box.pack(fill="both", expand=True)
        self.section_tree.bind("<<TreeviewSelect>>", self.choose_section_row)

    def create_result_tab(self):
        tab = self.result_tab
        ttk.Label(tab, text="桿件軸力與材料結果", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(tab, text="正值為拉伸，負值為壓縮。色階採當前工況的 |σ|。", style="Muted.TLabel", wraplength=430).pack(anchor="w", pady=(4, 5))
        ttk.Label(tab, textvariable=self.result_summary, wraplength=430, justify="left").pack(anchor="w", pady=(0, 7))
        fields = (("桿件", 139), ("狀態", 49), ("P [N]", 76), ("σ [MPa]", 74), ("ε [µε]", 75), ("ΔL [mm]", 80), ("L₀ [mm]", 80), ("A [mm²]", 82))
        box, self.result_tree = row_table(tab, fields, 12)
        box.pack(fill="both", expand=True)
        self.result_tree.bind("<<TreeviewSelect>>", self.choose_result)

    def set_status(self, text, error=False):
        self.status.set(text)
        self.status_label.configure(style="Error.TLabel" if error else "Muted.TLabel")

    def show_error(self, error):
        self.set_status(str(error), error=True)

    def load_demo(self):
        try:
            model = build_model(read_hardpoints(EXAMPLE))
            loads = example_loads(model, STATIC)
            self.topology = None
            self.set_model(model, "內建硬點", reset=True)
            self.case.set(STATIC)
            self.loads = loads
            self.refresh_loads()
            self.solve_model(show=False)
        except (ValueError, OSError, csv.Error) as error:
            self.show_error(error)

    def import_csv(self):
        path = filedialog.askopenfilename(parent=self.root, title="匯入硬點座標", filetypes=(("硬點 CSV", "*.csv"), ("所有檔案", "*.*")))
        if path:
            try:
                model = build_model(read_hardpoints(path))
                self.topology = None
                self.set_model(model, Path(path).name, reset=True)
            except (ValueError, OSError, csv.Error) as error:
                self.show_error(error)

    def import_topology(self):
        if self.model is None:
            return
        path = filedialog.askopenfilename(parent=self.root, title="載入自訂拓撲", filetypes=(("拓撲 JSON", "*.json"),))
        if path:
            try:
                topology = read_topology(path)
                model = build_model(self.model.points, topology)
                self.topology = topology
                self.set_model(model, Path(path).name, reset=True)
            except (ValueError, OSError) as error:
                self.show_error(error)

    def set_model(self, model, source, reset):
        self.model = model
        if reset:
            self.loads = []
            self.sections = {}
            self.selected = None
            self.viewport.selected = None
        self.sections = {rod.name: self.sections.get(rod.name, Section()) for rod in model.rods}
        self.source.set(source)
        self.metrics.set(f"{len(model.points)} 個硬點   ·   {len(model.rods)} 支軸力桿件   ·   {len(model.wheels)} 個車輪")
        self.viewport.set_model(model, reset)
        self.invalidate()
        self.refresh_loads()
        names = list(self.sections)
        self.section_picker.configure(values=names)
        if self.section_name.get() not in self.sections:
            self.section_name.set(names[0] if names else "")
        self.refresh_sections()
        self.choose_section(highlight=False)
        self.update_targets()
        self.update_selection()
        self.set_status(model.notes[0] if model.notes else "幾何已就緒。請設定外載後求解。")

    def invalidate(self):
        self.equilibrium = None
        self.results = {}
        self.refresh_results()

    def update_targets(self, _event=None):
        if self.model is None:
            return
        rods = self.model.rods
        if self.load_kind.get() == FORCE:
            eligible = self.model.owners.keys() | {end for rod in rods for end in (rod.a, rod.b)}
            options = [name for name in self.model.points if name in eligible]
        else:
            options = [rod.name for rod in rods]
        self.targets.configure(values=options)
        if self.load_target.get() not in options:
            self.load_target.set(options[0] if options else "")
        self.update_load_hint()

    def update_load_hint(self, _event=None):
        name = self.load_target.get()
        end = LOAD_TYPES[self.load_kind.get()]
        point = name
        if end:
            rod = next((rod for rod in self.model.rods if rod.name == name), None)
            point = (rod.a if end == "A" else rod.b) if rod else ""
        body = self.model.owners.get(point)
        if end:
            self.load_hint.set(f"N·m／{point} → {body or '鉸接節點，請選剛體桿端'}")
        else:
            self.load_hint.set(f"N／車輛座標　+X 前、+Y 左、+Z 上{('　·　' + body) if body else ''}")

    def refresh_loads(self, active=None):
        rows = ((str(i), (load.target, f"M{load.end} N·m" if load.end else "F N",
                          *map(numeric, load.xyz), numeric(hypot(*load.xyz))))
                for i, load in enumerate(self.loads))
        set_rows(self.load_tree, rows)
        if active is not None:
            select(self.load_tree, str(active))
        forces = [load.xyz for load in self.loads if not load.end]
        try:
            total = tuple(fsum(axis) for axis in zip(*forces)) if forces else (0., 0., 0.)
            magnitude = hypot(*total)
        except OverflowError:
            self.total_force.set("載荷表合力超出數值範圍")
        else:
            xyz = ", ".join(map(numeric, total))
            self.total_force.set(f"載荷表合力 ({xyz}) N　 |ΣF| {numeric(magnitude)} N")

    def update_acceleration_magnitude(self, *_):
        self.acceleration_magnitude.set(f"輸入合加速度 {input_magnitude(self.acceleration_inputs)} m/s²")

    def update_load_magnitude(self, *_):
        self.load_magnitude_label.set("合力 (N)" if self.load_kind.get() == FORCE else "合力矩 (N·m)")
        self.load_magnitude.set(input_magnitude(self.load_values))

    def apply_example(self):
        if self.model is None:
            return
        kind = self.case.get()
        description = kind
        try:
            if kind == ACCEL:
                mass = number(self.mass_input.get(), "重心質量")
                acceleration = tuple(number(var.get(), f"{axis} 軸加速度") for axis, var in zip("XYZ", self.acceleration_inputs))
                gravity = self.include_gravity.get()
                self.loads = example_loads(self.model, kind, mass_kg=mass, acceleration_mps2=acceleration, include_gravity=gravity)
                description += "（含重力）" if gravity else "（僅計輸入加速度）"
            else:
                self.loads = example_loads(self.model, kind)
        except ValueError as error:
            self.show_error(error)
            return
        self.finish_load_edit(message=f"已套用「{description}」。可編輯各列外載後求解。")

    def update_case_form(self, *_):
        kind = self.case.get()
        self.case_note.set(DESCRIPTIONS[kind])
        if kind == ACCEL:
            self.case_fields.pack(fill="x", before=self.case_caption, pady=(2, 6))
        else:
            self.case_fields.pack_forget()

    def choose_load(self, _event=None):
        selected = self.load_tree.selection()
        if not selected:
            return
        load = self.loads[int(selected[0])]
        self.load_kind.set(END_TYPES[load.end])
        self.load_target.set(load.target)
        self.update_targets()
        for var, value in zip(self.load_values, load.xyz):
            var.set(repr(float(value)))

    def form_load(self):
        target = self.load_target.get()
        if not target:
            raise ValueError("請選擇外載目標。")
        xyz = tuple(number(var.get(), f"{axis} 分量") for axis, var in zip("XYZ", self.load_values))
        end = LOAD_TYPES[self.load_kind.get()]
        if end:
            rod = next(rod for rod in self.model.rods if rod.name == target)
            point = rod.a if end == "A" else rod.b
            if point not in self.model.owners:
                raise ValueError(f"{point} 只能傳力。請選擇連到剛體的桿端。")
        return Load(target, xyz, end)

    def add_load(self):
        try:
            self.loads.append(self.form_load())
        except ValueError as error:
            self.show_error(error)
            return
        self.finish_load_edit(len(self.loads) - 1)

    def update_load(self):
        selected = self.load_tree.selection()
        if not selected:
            self.show_error("請先選取要更新的外載列。")
            return
        index = int(selected[0])
        try:
            self.loads[index] = self.form_load()
        except ValueError as error:
            self.show_error(error)
            return
        self.finish_load_edit(index)

    def delete_load(self):
        selected = self.load_tree.selection()
        if selected:
            del self.loads[int(selected[0])]
            self.finish_load_edit()

    def clear_loads(self):
        self.loads.clear()
        self.finish_load_edit()

    def finish_load_edit(self, active=None, message="外載已更新。請重新求解。"):
        self.refresh_loads(active)
        self.invalidate()
        self.set_status(message)

    def section_row(self, name):
        section = self.sections[name]
        return (name, numeric(section.outer_mm), numeric(section.inner_mm), numeric(section.young_gpa))

    def refresh_sections(self):
        set_rows(self.section_tree, ((name, self.section_row(name)) for name in self.sections))

    def choose_section_row(self, _event=None):
        selected = self.section_tree.selection()
        if selected and self.section_name.get() not in selected:
            focused = self.section_tree.focus()
            self.section_name.set(focused if focused in selected else selected[0])
            self.choose_section()

    def choose_section(self, _event=None, *, highlight=True):
        name = self.section_name.get()
        section = self.sections.get(name)
        if section is None:
            for var in self.section_values:
                var.set("")
            return
        for var, value in zip(self.section_values, (section.outer_mm, section.inner_mm, section.young_gpa)):
            var.set(repr(value))
        if name not in self.section_tree.selection():
            select(self.section_tree, name)
        if highlight:
            self.selected = ("rod", name)
            self.update_selection()

    def refresh_section_preview(self, *_):
        section = properties = None
        try:
            outer = number(self.section_values[0].get(), "外徑")
            inner = number(self.section_values[1].get(), "內徑")
            candidate = Section(outer, inner)
            properties = candidate.properties()
            section = candidate
        except ValueError:
            pass
        self.section_sketch.set_section(section)
        for field, value in self.section_properties.items():
            value.set(numeric(getattr(properties, field), digits=6) if properties else "—")

    def apply_section(self, all_rods):
        targets = tuple(self.sections) if all_rods else self.section_tree.selection()
        if not targets:
            self.show_error("請先在截面表格選取桿件。")
            return
        try:
            section = Section(*(number(var.get(), label) for var, label in zip(self.section_values, ("外徑", "內徑", "楊氏模數"))))
            section.properties()
            section.young_mpa()
            sections = dict(self.sections)
            sections.update(dict.fromkeys(targets, section))
            results = member_results(self.equilibrium, sections) if self.equilibrium else {}
        except ValueError as error:
            self.show_error(error)
            return
        self.sections, self.results = sections, results
        for name in targets:
            self.section_tree.item(name, values=self.section_row(name))
        self.refresh_results()
        self.set_status(f"已更新 {len(targets)} 支桿件的截面。")

    def solve_model(self, show=True):
        if self.model is None:
            return
        try:
            equilibrium = solve(self.model, self.loads)
            results = member_results(equilibrium, self.sections)
        except ValueError as error:
            self.invalidate()
            self.show_error(error)
            return
        self.equilibrium, self.results = equilibrium, results
        self.refresh_results()
        if show:
            self.tabs.select(self.result_tab)
        status = f"已求解 {len(results)} 支軸力桿件。平衡矩陣秩 {equilibrium.rank}/{equilibrium.equations}。"
        if equilibrium.rank < equilibrium.equations:
            status += " 目前外載可平衡，模型仍有自由方向。"
        self.set_status(status)

    def refresh_results(self):
        rows = []
        for name, res in sorted(self.results.items(), key=lambda item: -abs(item[1].stress_mpa)):
            state = "拉伸" if res.force_n > 1e-8 else "壓縮" if res.force_n < -1e-8 else "零力"
            values = (res.force_n, res.stress_mpa, res.strain * 1e6, res.change_mm, res.length_mm, res.area_mm2)
            rows.append((name, (name, state, *map(numeric, values))))
        set_rows(self.result_tree, rows)
        eq = self.equilibrium
        if eq:
            force = ", ".join(map(numeric, eq.support_force))
            moment = ", ".join(map(numeric, eq.support_moment))
            self.result_summary.set(f"車身支承合反力 ({force}) N\n關於 {eq.origin or '座標原點'} 的合反力矩 ({moment}) N·m")
        else:
            self.result_summary.set("求解後顯示桿件結果與車身支承反力。")
        self.viewport.results = self.results
        self.update_selection()

    def choose_result(self, _event=None):
        chosen = self.result_tree.selection()
        if chosen:
            self.selected = ("rod", chosen[0])
            self.update_selection()

    def toggle_labels(self):
        self.viewport.show_names = self.labels.get()
        self.viewport.draw()

    def on_pick(self, kind, name):
        self.selected = (kind, name)
        if kind == "rod":
            self.section_name.set(name)
            self.choose_section()
            select(self.result_tree, name)
            if self.load_kind.get() != FORCE:
                self.load_target.set(name)
                self.update_load_hint()
        elif name in self.targets["values"] and self.load_kind.get() == FORCE:
            self.load_target.set(name)
            self.update_load_hint()
        self.update_selection()

    def update_selection(self):
        self.viewport.selected = self.selected
        self.viewport.draw()
        if self.selected is None or self.model is None:
            self.selection.set("點選硬點或桿件以查看明細")
            return
        kind, name = self.selected
        if kind == "point":
            x, y, z = (numeric(v * 1000.) for v in self.model.points[name])
            body = self.model.owners.get(name)
            self.selection.set(f"{name}  ·  ({x}, {y}, {z}) mm{('  ·  ' + body) if body else ''}")
            return
        result = self.results.get(name)
        if result:
            self.selection.set(f"{name}  ·  {numeric(result.force_n)} N  ·  {numeric(result.stress_mpa)} MPa  ·  ΔL {numeric(result.change_mm)} mm")
        else:
            rod = next(rod for rod in self.model.rods if rod.name == name)
            self.selection.set(f"{name}  ·  A {rod.a}   B {rod.b}")

    def edit_points(self):
        if self.model is not None:
            PointEditor(self)


class PointEditor:
    def __init__(self, app):
        self.app = app
        self.points = dict(app.model.points)
        window = self.window = tk.Toplevel(app.root)
        window.title("編輯硬點 XYZ")
        window.transient(app.root)
        window.grab_set()
        window.minsize(530, 470)
        window.geometry("610x535")
        page = ttk.Frame(window, padding=14)
        page.pack(fill="both", expand=True)
        ttk.Label(page, text="硬點座標", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(page, text="+X 前、+Y 左、+Z 上。選取一列後可更新座標。", style="Muted.TLabel").pack(anchor="w", pady=(4, 7))
        box, self.tree = row_table(page, (("硬點", 165), ("X mm", 96), ("Y mm", 96), ("Z mm", 96)), 11)
        box.pack(fill="both", expand=True)
        self.tree.bind("<<TreeviewSelect>>", self.choose)
        self.fields = [tk.StringVar(window) for _ in range(3)]
        xyz = ttk.Frame(page)
        xyz.pack(fill="x", pady=7)
        entry_fields(xyz, ("X mm", "Y mm", "Z mm"), self.fields)
        self.note = tk.StringVar(window)
        ttk.Label(page, textvariable=self.note, style="Muted.TLabel", wraplength=520).pack(anchor="w")
        control = ttk.Frame(page)
        control.pack(fill="x", pady=(7, 0))
        ttk.Button(control, text="更新選取列", command=self.update_row).pack(side="left", padx=(0, 6))
        ttk.Button(control, text="另存已編輯 CSV", command=self.export_csv).pack(side="left")
        ttk.Button(control, text="套用並關閉", style="Primary.TButton", command=self.apply).pack(side="right")
        ttk.Button(control, text="取消", command=window.destroy).pack(side="right", padx=(0, 6))
        self.refresh()
        selected = app.selected
        name = selected[1] if selected and selected[0] == "point" else next(iter(self.points))
        select(self.tree, name)

    def refresh(self):
        set_rows(self.tree, ((name, (name, *(numeric(value * 1000.) for value in point))) for name, point in self.points.items()))

    def choose(self, _event=None):
        selection = self.tree.selection()
        if selection:
            for field, value in zip(self.fields, self.points[selection[0]]):
                field.set(repr(float(value * 1000.)))

    def update_row(self):
        selection = self.tree.selection()
        if not selection:
            return True
        name = selection[0]
        try:
            self.points[name] = tuple(.001 * number(field.get(), axis) for field, axis in zip(self.fields, "XYZ"))
        except ValueError as error:
            self.note.set(str(error))
            return False
        self.tree.item(name, values=(name, *(numeric(value * 1000.) for value in self.points[name])))
        self.note.set(f"已編輯 {name}。")
        return True

    def apply(self):
        if not self.update_row():
            return
        try:
            model = build_model(self.points, self.app.topology)
            self.app.set_model(model, self.app.source.get(), reset=False)
        except ValueError as error:
            self.note.set(str(error))
            return
        self.app.set_status("硬點已更新。求解會使用新的原始桿長。範例輪荷可在外載工況重新套用。")
        self.window.destroy()

    def export_csv(self):
        if not self.update_row():
            return
        path = filedialog.asksaveasfilename(parent=self.window, defaultextension=".csv", filetypes=(("硬點 CSV", "*.csv"),), initialfile="hardpoints-edited.csv")
        if path:
            try:
                with open(path, "w", encoding="utf-8-sig", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(("name", "x_m", "y_m", "z_m"))
                    writer.writerows((name, *point) for name, point in self.points.items())
            except OSError as error:
                messagebox.showerror("無法儲存硬點", str(error), parent=self.window)
            else:
                self.note.set(f"已儲存 {Path(path).name}。")


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()
