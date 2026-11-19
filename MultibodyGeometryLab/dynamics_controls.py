"""Edit four direct-acting dampers and a road displacement event."""

import tkinter as tk
from tkinter import ttk

from ._geometry_support.hardpoints import CORNERS
from .dynamics_settings import CORNER_LIMITS, HEAVE_LIMITS, SIMULATION_LIMITS, validate_dynamics, validate_heave


TARGET_LABELS = {**dict(zip(CORNERS, CORNERS)), "all": "四輪"}


class DynamicsEditor(ttk.Frame):
    def __init__(self, parent, mark_dirty, start, *, tire_stiffness):
        super().__init__(parent, padding=9)

        def editable():
            variable = tk.StringVar(self)
            variable.trace_add("write", mark_dirty)
            return variable

        self.wheels = {name: {key: editable() for key in CORNER_LIMITS} for name in CORNERS}
        self.tire_damping = editable()
        self.target = editable()
        self.simulation = {key: editable() for key in SIMULATION_LIMITS}

        table = ttk.Frame(self)
        table.pack(fill="x")
        for column, heading in enumerate(("輪位", "質量 kg", "剛性 N/m", "阻尼 N·s/m")):
            ttk.Label(table, text=heading).grid(row=0, column=column, padx=3, pady=2)
        for row, name in enumerate(CORNERS, 1):
            ttk.Label(table, text=name).grid(row=row, column=0, sticky="w", padx=3)
            for column, variable in enumerate(self.wheels[name].values(), 1):
                ttk.Entry(table, textvariable=variable, width=9, justify="right").grid(
                    row=row, column=column, padx=3, pady=2, sticky="ew")

        tire = ttk.LabelFrame(self, text="輪胎垂直接地（四輪共用）", padding=(6, 4))
        tire.pack(fill="x", pady=(8, 0))
        for column, (label, variable) in enumerate((
                ("垂直剛性 (N/m)", tire_stiffness), ("垂直阻尼 (N·s/m)", self.tire_damping))):
            tire.columnconfigure(column, weight=1)
            ttk.Label(tire, text=label).grid(row=0, column=column, sticky="w", padx=(2, 8))
            ttk.Entry(tire, textvariable=variable, width=11, justify="right").grid(
                row=1, column=column, sticky="w", padx=(2, 8), pady=(2, 0))

        inputs = ttk.Frame(self)
        inputs.pack(fill="x", pady=(8, 0))
        ttk.Label(inputs, text="路面擾動輪位").grid(row=0, column=0, sticky="w")
        target = ttk.Combobox(inputs, textvariable=self.target, values=tuple(TARGET_LABELS.values()),
                              width=8, state="readonly")
        target.grid(row=0, column=1, sticky="w")
        for row, (key, (label, _, _)) in enumerate(SIMULATION_LIMITS.items(), 1):
            self.entry(inputs, row, label, self.simulation[key])

        self.start_button = ttk.Button(self, text="模擬並播放", command=start)
        self.start_button.pack(fill="x", pady=(8, 0))

    @staticmethod
    def entry(parent, row, label, variable):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=2)
        ttk.Entry(parent, textvariable=variable, width=10, justify="right").grid(row=row, column=1, sticky="w")

    def populate(self, data):
        for name, values in data["corners"].items():
            for key, variable in self.wheels[name].items():
                variable.set(str(values[key]))
        self.tire_damping.set(str(data["tire_damping_ns_m"]))
        self.target.set(TARGET_LABELS[data["simulation"]["target"]])
        for key, variable in self.simulation.items():
            variable.set(str(data["simulation"][key]))

    def draft(self, heave, chassis_inertia, pushrod_mounts):
        try:
            corners = {name: {key: float(variable.get()) for key, variable in fields.items()}
                       for name, fields in self.wheels.items()}
            parameters = {key: float(variable.get()) for key, variable in self.simulation.items()}
            tire_damping = float(self.tire_damping.get())
        except ValueError as error:
            raise ValueError("輪端、避震器與路面輸入需要有效數值。") from error
        target = next(key for key, label in TARGET_LABELS.items() if label == self.target.get())
        return validate_dynamics(dict(corners=corners, tire_damping_ns_m=tire_damping,
                                      simulation=dict(target=target, **parameters), heave=heave,
                                      chassis_inertia=chassis_inertia, pushrod_mounts=pushrod_mounts))


class HeaveEditor(ttk.Frame):
    def __init__(self, parent, mark_dirty, start):
        super().__init__(parent, padding=10)
        ttk.Label(self, text="平坦地面 · 車身只沿 Z 軸移動").pack(anchor="w")
        inputs = ttk.Frame(self)
        inputs.pack(fill="x", pady=(8, 6))
        self.fields = {}
        for row, (name, (label, _, _)) in enumerate(HEAVE_LIMITS.items()):
            variable = self.fields[name] = tk.StringVar(self)
            variable.trace_add("write", mark_dirty)
            DynamicsEditor.entry(inputs, row, label, variable)
        ttk.Label(self, text="正值向下施力，負值向上。外力在上升時間內平滑增加，之後維持。",
                  wraplength=310).pack(anchor="w", pady=(2, 7))
        ttk.Label(self, text="車身質量採車輛總質量扣除四輪輪端質量。要測試無阻尼，請將四輪避震器與輪胎垂直阻尼都設為 0。",
                  wraplength=310).pack(anchor="w", pady=(0, 8))
        self.start_button = ttk.Button(self, text="模擬重心 Heave", command=start)
        self.start_button.pack(fill="x")

    def populate(self, settings):
        for name, variable in self.fields.items():
            variable.set(str(settings[name]))

    def draft(self):
        try:
            parameters = {name: float(variable.get()) for name, variable in self.fields.items()}
        except ValueError as error:
            raise ValueError("重心外力、施力時間與模擬時間需要有效數值。") from error
        return validate_heave(parameters)
