"""Tk editor for the members connecting the suspension hardpoints."""

from copy import deepcopy
from math import isfinite
import tkinter as tk
from tkinter import ttk

from SuspensionGeometryLab.suspension_geometry_lab.hardpoints import CORNERS

from .dynamics_settings import CORNER_LIMITS
from .elements import DAMPER_POINT, PUSH_ROD, validate_elements
from .geometry import POINTS


ENDPOINTS = (*POINTS, DAMPER_POINT)
KIND_LABELS = {"rigid": "剛性桿", "spring_damper": "彈簧阻尼"}
KIND_CODES = {value: key for key, value in KIND_LABELS.items()}
KIND_COLORS = {"rigid": "#617688", "spring_damper": "#be6234"}


class ElementEditor(ttk.Frame):
    """Edit per-corner member definitions before applying the mechanism."""

    def __init__(self, parent, mark_dirty, apply):
        super().__init__(parent, padding=8)
        self.mark_dirty = mark_dirty
        self.elements = {name: [] for name in CORNERS}
        self.points = {}
        self.active_corner = "FL"
        self.selected = None
        self.syncing = False

        self.corner = tk.StringVar(self, value=self.active_corner)
        self.kind = tk.StringVar(self)
        self.stiffness = tk.StringVar(self)
        self.damping = tk.StringVar(self)
        self.end_a = tk.StringVar(self, value=ENDPOINTS[0])
        self.end_b = tk.StringVar(self, value=ENDPOINTS[-1])
        self.detail = tk.StringVar(self)
        self.error = tk.StringVar(self)
        for variable in (self.kind, self.stiffness, self.damping):
            variable.trace_add("write", self.changed)

        header = ttk.Frame(self)
        header.pack(fill="x")
        ttk.Label(header, text="輪位").pack(side="left")
        wheel = ttk.Combobox(header, textvariable=self.corner, values=CORNERS, state="readonly", width=5)
        wheel.pack(side="left", padx=(5, 9))
        wheel.bind("<<ComboboxSelected>>", self.change_corner)
        ttk.Label(header, text="點線條或清單選元件").pack(side="left")

        self.preview = tk.Canvas(self, width=250, height=164, background="#f6f8fa", highlightthickness=0)
        self.preview.pack(fill="both", expand=True, pady=(6, 5))
        self.preview.bind("<Configure>", self.draw)
        self.preview.bind("<Button-1>", self.choose_line)

        table = ttk.Frame(self)
        table.pack(fill="x")
        self.tree = ttk.Treeview(table, columns=("name", "kind"), show="headings", height=5, selectmode="browse")
        self.tree.heading("name", text="元件")
        self.tree.heading("kind", text="類型")
        self.tree.column("name", width=162, stretch=True)
        self.tree.column("kind", width=75, stretch=False)
        self.tree.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(table, orient="vertical", command=self.tree.yview)
        scrollbar.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.bind("<<TreeviewSelect>>", self.change_row)

        self.build_member_controls()
        self.build_connection_controls()
        buttons = ttk.Frame(self)
        buttons.pack(fill="x", pady=(6, 0))
        ttk.Button(buttons, text="全車設為剛性", command=self.make_rigid).pack(side="left")
        ttk.Button(buttons, text="套用機構", command=apply).pack(side="right")
        ttk.Label(self, textvariable=self.error, wraplength=280, foreground="#9e3829").pack(fill="x", pady=(3, 0))
        self.show_member(None)

    def build_member_controls(self):
        panel = ttk.Frame(self)
        panel.pack(fill="x", pady=(4, 0))
        ttk.Label(panel, textvariable=self.detail).grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(panel, text="類型").grid(row=1, column=0, sticky="w")
        self.role = ttk.Combobox(panel, width=10, textvariable=self.kind, values=tuple(KIND_CODES), state="readonly")
        self.role.grid(row=1, column=1, sticky="w", padx=(3, 7))
        self.role.bind("<<ComboboxSelected>>", self.change_kind)
        self.remove = ttk.Button(panel, text="移除所選", command=self.remove_member)
        self.remove.grid(row=1, column=2, columnspan=2, sticky="e")
        ttk.Label(panel, text="k (N/m)").grid(row=2, column=0, sticky="w", pady=(2, 0))
        self.spring_entry = ttk.Entry(panel, width=8, textvariable=self.stiffness)
        self.spring_entry.grid(row=2, column=1, sticky="w", padx=(3, 7), pady=(2, 0))
        ttk.Label(panel, text="c (N·s/m)").grid(row=2, column=2, sticky="w", pady=(2, 0))
        self.damper_entry = ttk.Entry(panel, width=8, textvariable=self.damping)
        self.damper_entry.grid(row=2, column=3, sticky="w", padx=(3, 0), pady=(2, 0))
        ttk.Label(panel, text="k、c 留白時沿用輪位設定。").grid(row=3, column=0, columnspan=4, sticky="w")

    def build_connection_controls(self):
        panel = ttk.Frame(self)
        panel.pack(fill="x", pady=(5, 0))
        for variable in (self.end_a, self.end_b):
            ttk.Combobox(panel, textvariable=variable, values=ENDPOINTS, state="readonly", width=7).pack(
                side="left", padx=(0, 4))
        ttk.Button(panel, text="新增連線", command=self.add_member).pack(side="left")

    def populate(self, elements, points):
        self.elements = deepcopy(elements)
        self.points = points
        self.error.set("")
        self.refresh_rows(0)

    def set_points(self, points):
        self.points = points
        self.draw()

    def draft(self):
        self.commit_member()
        return validate_elements(deepcopy(self.elements))

    def changed(self, *_):
        if not self.syncing:
            self.error.set("")
            self.mark_dirty()

    def members(self):
        return self.elements[self.active_corner]

    def refresh_rows(self, selected):
        self.syncing = True
        try:
            self.tree.delete(*self.tree.get_children())
            for index, item in enumerate(self.members()):
                self.tree.insert("", "end", iid=str(index), values=(item["name"], KIND_LABELS[item["kind"]]))
            chosen = selected if self.members() else None
            if chosen is not None:
                self.tree.selection_set(str(chosen))
                self.tree.see(str(chosen))
        finally:
            self.syncing = False
        self.show_member(chosen)

    def show_member(self, index):
        self.selected = index
        item = self.members()[index] if index is not None else None
        self.syncing = True
        try:
            self.kind.set(KIND_LABELS[item["kind"]] if item else "")
            stiffness = item.get("stiffness_n_m") if item else None
            damping = item.get("damping_ns_m") if item else None
            self.stiffness.set("" if stiffness is None else str(stiffness))
            self.damping.set("" if damping is None else str(damping))
            self.detail.set(f"{item['a']} → {item['b']}" if item else "選取元件以編輯")
        finally:
            self.syncing = False
        self.role.configure(state="readonly" if item else "disabled")
        self.remove.configure(state="normal" if item and item["name"] != PUSH_ROD else "disabled")
        self.update_coefficient_state()
        self.draw()

    def update_coefficient_state(self):
        state = "normal" if self.kind.get() == KIND_LABELS["spring_damper"] else "disabled"
        self.spring_entry.configure(state=state)
        self.damper_entry.configure(state=state)

    @staticmethod
    def coefficient(text, setting):
        if not text.strip():
            return None
        name, low, high = CORNER_LIMITS[setting]
        try:
            value = float(text)
        except ValueError as error:
            raise ValueError(f"{name}需要有效數值。") from error
        if not isfinite(value) or not low <= value <= high:
            raise ValueError(f"{name}須在 {low:g} 至 {high:g} 之間，或留空沿用輪位值。")
        return value

    def commit_member(self):
        if self.selected is None:
            return
        item = self.members()[self.selected]
        kind = KIND_CODES[self.kind.get()]
        if kind == "spring_damper":
            stiffness = self.coefficient(self.stiffness.get(), "spring_n_m")
            damping = self.coefficient(self.damping.get(), "damper_ns_m")
            item.update(stiffness_n_m=stiffness, damping_ns_m=damping)
        item["kind"] = kind
        self.tree.item(str(self.selected), values=(item["name"], KIND_LABELS[kind]))

    def try_commit(self):
        try:
            self.commit_member()
        except ValueError as error:
            self.error.set(str(error))
            return False
        self.error.set("")
        return True

    def change_corner(self, _event=None):
        corner = self.corner.get()
        if corner == self.active_corner:
            return
        if not self.try_commit():
            self.corner.set(self.active_corner)
            return
        self.active_corner = corner
        self.refresh_rows(0)

    def change_row(self, _event=None):
        selection = self.tree.selection()
        if self.syncing or not selection:
            return
        index = int(selection[0])
        if index == self.selected:
            return
        if self.try_commit():
            self.show_member(index)
        elif self.selected is not None:
            self.tree.selection_set(str(self.selected))

    def change_kind(self, _event=None):
        self.update_coefficient_state()
        if self.try_commit():
            self.draw()

    def mark_and_refresh(self, selection):
        self.error.set("")
        self.mark_dirty()
        self.refresh_rows(selection)

    def make_rigid(self):
        if not self.try_commit():
            return
        for members in self.elements.values():
            for item in members:
                item["kind"] = "rigid"
        self.mark_and_refresh(self.selected)

    def add_member(self):
        if not self.try_commit():
            return
        a, b = self.end_a.get(), self.end_b.get()
        if a == b:
            self.error.set("請選擇兩個不同的端點。")
            return
        label = f"{a} → {b}"
        name, suffix = label, 2
        names = {item["name"] for item in self.members()}
        while name in names:
            name = f"{label} ({suffix})"
            suffix += 1
        self.members().append({"name": name, "a": a, "b": b, "kind": "rigid",
                               "stiffness_n_m": None, "damping_ns_m": None})
        self.mark_and_refresh(len(self.members()) - 1)

    def remove_member(self):
        index = self.selected
        if index is None or self.members()[index]["name"] == PUSH_ROD:
            return
        self.members().pop(index)
        self.mark_and_refresh(min(index, len(self.members()) - 1))

    def draw(self, _event=None):
        canvas = self.preview
        canvas.delete("all")
        points = self.points.get(self.active_corner)
        if not points:
            return
        width, height = max(canvas.winfo_width(), 120), max(canvas.winfo_height(), 100)
        projection = {
            name: (.78 * x + .62 * y, -.26 * x + .32 * y + .91 * z)
            for name, (x, y, z) in points.items()
        }
        xs, ys = zip(*projection.values())
        x_min, x_max, y_min, y_max = min(xs), max(xs), min(ys), max(ys)
        scale = min((width - 56) / max(x_max - x_min, 1e-9), (height - 32) / max(y_max - y_min, 1e-9))
        mapped = {
            name: (width / 2 + scale * (x - (x_min + x_max) / 2),
                   height / 2 - scale * (y - (y_min + y_max) / 2))
            for name, (x, y) in projection.items()
        }
        for index, item in enumerate(self.members()):
            a, b = item["a"], item["b"]
            if a not in mapped or b not in mapped:
                continue
            selected = index == self.selected
            canvas.create_line(*mapped[a], *mapped[b], width=4 if selected else 2,
                               fill="#2169a0" if selected else KIND_COLORS[item["kind"]],
                               dash=(4, 2) if item["kind"] == "spring_damper" else (),
                               tags=("member", f"line:{index}"))
        if self.selected is not None:
            canvas.tag_raise(f"line:{self.selected}")
        for name, (x, y) in mapped.items():
            canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#263848", outline="")
            canvas.create_text(x + 3, y - 3, text=name, anchor="sw", font=("TkDefaultFont", 7), fill="#354555")

    def choose_line(self, event):
        canvas = self.preview
        nearby = canvas.find_overlapping(event.x - 3, event.y - 3, event.x + 3, event.y + 3)
        for item in reversed(nearby):
            tags = canvas.gettags(item)
            if "member" in tags:
                index = next(tag.split(":")[1] for tag in tags if tag.startswith("line:"))
                self.tree.selection_set(index)
                self.tree.see(index)
                return
