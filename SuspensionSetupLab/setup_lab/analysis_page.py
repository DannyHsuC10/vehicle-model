"""One model family's case editor, charts and comparison state."""

import csv
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np

from .chart_base import COLORS, Chart
from .project import (AERO_DEFAULTS, AXES, BUMP_CASES, CASE_DEFAULTS, CASE_FIELDS, CHANNELS, CORNERS, MANEUVER_CASES,
                      MODELS, MODEL_SOURCES, PAGE_MODELS, SCENARIOS, TARGET_WHEELS, case_axes, model_corners,
                      read_recording, save_project)
from .road_profiles import ISO_ROUGHNESS
from .vehicle_3d import FullVehicleView
from .visualization import Dashboard, MotionView, Playback, ResponseChart
from .widgets import entry, key_for, option

TARGET_LABELS = {**{c: c for c in CORNERS}, "front": "前軸", "rear": "後軸", "left": "左側", "right": "右側",
                 "all": "模型全部輪位"}
LOAD_TRANSFER_GROUPS = ("載重轉移量", "載重轉移速率")
RECORDING_SIGNAL_LABELS = {
    "z_fl_mm": "左前輪路面高度",
    "z_fr_mm": "右前輪路面高度",
    "z_rl_mm": "左後輪路面高度",
    "z_rr_mm": "右後輪路面高度",
    "ax_mps2": "縱向加速度",
    "ay_mps2": "側向加速度",
}


def curve_extrema(time, values, name, unit, color):
    high, low = np.argmax(values), np.argmin(values)
    if values[high] == values[low]:
        points = (("◇ 最大＝最小", "diamond", high),)
    else:
        points = (("▲ 最大", "triangle_up", high), ("▼ 最小", "triangle_down", low))
    markers = [{"x": [float(time[i])], "y": [float(values[i])], "color": color, "marker": shape, "marker_only": True}
               for _, shape, i in points]
    labels = [f"{label} {values[i]:.6g} {unit}（{time[i]:.5f} s）" for label, _, i in points]
    return markers, f"{name}　{'　'.join(labels)}"


class AnalysisPage(ttk.Frame):
    def __init__(self, app, family):
        super().__init__(app.pages, padding=9)
        self.app, self.family = app, family
        self.models = {model: MODELS[model] for model in PAGE_MODELS[family]}
        self.result = self.baseline = None
        setup = ttk.LabelFrame(self, text="測試工況", padding=10)
        setup.pack(side="left", fill="y", padx=(0, 10))
        self.model, self.scenario, self.target = (tk.StringVar() for _ in range(3))
        self.full_source = tk.StringVar(value="四輪")
        self.rear_delay = tk.BooleanVar()
        self.iso_lateral = tk.BooleanVar()
        fields = (("懸吊模型", self.models, self.model), ("模型輪位", {}, self.full_source),
                  ("輸入工況", SCENARIOS, self.scenario), ("作用輪位", TARGET_LABELS, self.target))
        for row, (label, choices, variable) in enumerate(fields):
            ttk.Label(setup, text=label).grid(row=row, column=0, sticky="w", pady=4)
            if row == 0 and len(self.models) == 1:
                ttk.Label(setup, textvariable=variable).grid(row=row, column=1, sticky="w", pady=4)
                continue
            widget = option(setup, choices, variable, 24)
            widget.grid(row=row, column=1, sticky="ew", pady=4)
            widget.bind("<<ComboboxSelected>>", lambda event: self.case_mode())
            if row == 1:
                self.source_widget = widget
            if row == 2:
                self.scenario_widget = widget
            if row == 3:
                self.target_widget = widget
        self.case_vars, self.case_widgets = {}, {}
        case_fields = CASE_FIELDS
        if family == "quarter":
            case_fields = {key: spec for key, spec in CASE_FIELDS.items() if key not in ("ax", "ay", "ramp")}
        for row, (key, (label, _, _)) in enumerate(case_fields.items(), len(fields)):
            self.case_vars[key] = tk.StringVar()
            self.case_widgets[key] = entry(setup, row, 0, label, self.case_vars[key])
        row = len(fields) + len(case_fields)
        self.delay_widget = ttk.Checkbutton(setup, text="選定後輪延遲（軸距 ÷ 車速）", variable=self.rear_delay,
                                            command=self.case_mode)
        self.delay_widget.grid(row=row, columnspan=2, sticky="w", pady=5)
        row += 1
        self.iso_lateral_widget = ttk.Checkbutton(setup, text="疊加側向加速度 ay", variable=self.iso_lateral,
                                                  command=self.case_mode)
        self.iso_lateral_widget.grid(row=row, columnspan=2, sticky="w", pady=(0, 5))
        ttk.Button(setup, text="載入路面與加速度 CSV", command=self.load_recording).grid(row=row + 1, columnspan=2, sticky="ew")
        self.recording_label = tk.StringVar()
        ttk.Label(setup, textvariable=self.recording_label, wraplength=275).grid(row=row + 2, columnspan=2, sticky="w", pady=4)
        self.run_button = ttk.Button(setup, text="計算 F5", command=lambda: app.run(self))
        self.run_button.grid(row=row + 3, column=0, sticky="ew", pady=(10, 4))
        self.stop_button = ttk.Button(setup, text="停止", command=app.stop, state="disabled")
        self.stop_button.grid(row=row + 3, column=1, sticky="ew", pady=(10, 4))
        panel = ttk.Frame(self)
        panel.pack(side="left", fill="both", expand=True)
        actions = ttk.Frame(panel)
        actions.pack(fill="x", pady=(0, 5))
        for label, command in (("設目前結果為 A", self.pin), ("清除 A", self.clear_baseline),
                               ("匯出 CSV", self.export_csv), ("儲存計算快照", self.export_snapshot)):
            ttk.Button(actions, text=label, command=command).pack(side="left", padx=(0, 4))
        batch_actions = ttk.Frame(panel)
        batch_actions.pack(fill="x", pady=(0, 5))
        self.compute_csv_button = ttk.Button(batch_actions, text="純計算並匯出 CSV", command=lambda: app.run_csv(self))
        self.compute_csv_button.pack(side="left", padx=(0, 8))
        self.progress = ttk.Progressbar(batch_actions, maximum=100)
        self.progress.pack(side="left", fill="x", expand=True)
        self.result_label = tk.StringVar(value="尚無結果")
        ttk.Label(panel, textvariable=self.result_label).pack(anchor="w")
        outputs = ttk.Notebook(panel)
        outputs.pack(fill="both", expand=True, pady=(5, 0))
        plots, metrics = ttk.Frame(outputs, padding=5), ttk.Frame(outputs, padding=8)
        outputs.add(plots, text="視覺化")
        outputs.add(metrics, text="調校指標與警示")
        if family == "half_lr":
            self.scan_result = None
            self.build_scan_tab(outputs)
        panes = ttk.Panedwindow(plots, orient="horizontal")
        panes.pack(fill="both", expand=True)
        if family == "full":
            self.dashboard = Dashboard(panes, full_vehicle=True)
            panes.add(self.dashboard, weight=3)
            visual = ttk.Notebook(panes)
            self.motion = FullVehicleView(visual)
            plot = ttk.Frame(visual)
            visual.add(self.motion, text="LP02 實車 3D")
            visual.add(plot, text="曲線")
            panes.add(visual, weight=2)
        else:
            visual = ttk.Notebook(panes)
            self.motion = MotionView(visual)
            self.dashboard = Dashboard(visual)
            visual.add(self.motion, text="車輛動畫")
            visual.add(self.dashboard, text="輪荷與姿態")
            panes.add(visual, weight=2)
            plot = ttk.Frame(panes)
            panes.add(plot, weight=3)
        selection = ttk.Frame(plot)
        selection.pack(fill="x", pady=(0, 4))
        self.group = ttk.Combobox(selection, state="readonly", width=12)
        self.group.pack(side="left", padx=(0, 6))
        self.signal = ttk.Combobox(selection, state="readonly", width=20)
        self.signal.pack(side="left", fill="x", expand=True)
        self.group.bind("<<ComboboxSelected>>", lambda event: self.select_group())
        self.signal.bind("<<ComboboxSelected>>", lambda event: self.draw_result())
        self.extrema_note = None
        if family in ("quarter", "half_lr"):
            self.extrema_note = ttk.Label(plot, justify="left")
            plot.bind("<Configure>", lambda event: self.extrema_note.configure(wraplength=max(1, event.width - 8)))
        self.tender_note = None
        if family in ("quarter", "half_fr"):
            self.tender_note = ttk.Label(plot, justify="left")
            plot.bind("<Configure>", lambda event: self.tender_note.configure(wraplength=max(1, event.width - 8)), add="+")
        self.chart = ResponseChart(plot)
        if family == "full":
            self.chart.canvas.configure(width=380)
        self.chart.pack(fill="both", expand=True)
        self.playback = Playback(plots, self.draw_frame, None if family == "full" else self.motion.set_gain)
        self.playback.pack(fill="x", pady=(6, 0))

        headers = ("輪位", "靜態荷重 N", "最低 N", "最高 N", "靜態行程 mm", "最低 mm", "最高 mm", "零輪荷 s")
        self.metrics = ttk.Treeview(metrics, columns=headers, show="headings", height=4)
        for name in headers:
            self.metrics.heading(name, text=name)
            self.metrics.column(name, width=76, minwidth=53, anchor="e")
        self.metrics.pack(fill="x")
        self.details = tk.Text(metrics, height=8, wrap="word", relief="flat", padx=4, pady=6, state="disabled")
        self.details.pack(fill="both", expand=True, pady=(8, 0))

    def set_form(self, model, case):
        self.model.set(self.models[model])
        self.case = case
        self.scenario.set(SCENARIOS[case["scenario"]])
        self.target.set(TARGET_LABELS[case["target"]])
        self.rear_delay.set(case["rear_delay"])
        self.iso_lateral.set(case.get("iso_lateral", CASE_DEFAULTS["iso_lateral"]))
        for key, var in self.case_vars.items():
            var.set(case.get(key, CASE_DEFAULTS.get(key)))
        self.recording, self.recording_name = case["recording"], case["recording_name"]
        self.case_mode()
        self.set_result(None)
        if self.family == "half_lr":
            self.scan_result = None
            self.scan_summary.set("")
            self.scan_chart.empty_message = "按「掃描 ±10 mm」開始"
            self.scan_chart.set(None)
            self.scan_best_button.state(["disabled"])
        self.progress["value"] = 0

    def build_scan_tab(self, outputs):
        self.outputs, self.plots = outputs, outputs.tabs()[0]
        self.scan_tab = ttk.Frame(outputs, padding=8)
        outputs.add(self.scan_tab, text="Roll Center 掃描")
        actions = ttk.Frame(self.scan_tab)
        actions.pack(fill="x", pady=(0, 6))
        self.scan_button = ttk.Button(actions, text="掃描 ±10 mm", command=lambda: self.app.run(self, scan=True))
        self.scan_button.pack(side="left", padx=(0, 6))
        self.scan_best_button = ttk.Button(actions, text="查看最佳工況曲線", command=self.show_best_run, state="disabled")
        self.scan_best_button.pack(side="left")
        ttk.Label(self.scan_tab, text="每 1 mm 試算一次，以全時段輪胎端載重轉移速率的最大絕對值評分。").pack(anchor="w")
        self.scan_summary = tk.StringVar()
        summary = ttk.Label(self.scan_tab, textvariable=self.scan_summary, justify="left")
        summary.pack(fill="x", pady=(4, 6))
        self.scan_tab.bind("<Configure>", lambda event: summary.configure(wraplength=max(1, event.width - 16)))
        self.scan_chart = Chart(self.scan_tab, empty_message="按「掃描 ±10 mm」開始")
        self.scan_chart.canvas.configure(width=390, height=260)
        self.scan_chart.pack(fill="both", expand=True)

    def show_scan_running(self):
        self.scan_result = None
        self.scan_chart.empty_message = "正在計算 21 個高度…"
        self.scan_chart.set(None)
        self.scan_best_button.state(["disabled"])
        self.scan_summary.set("")
        self.outputs.select(self.scan_tab)

    def show_scan_stopped(self, message):
        self.scan_chart.empty_message = message
        self.scan_chart.set(None)

    def show_scan_result(self, scan):
        self.scan_result = scan
        best = scan.heights[scan.best_index]
        peak = scan.peaks[scan.best_index]
        axle = MODEL_SOURCES["half_lr"][scan.axle]
        lines = [f"{axle}　基準 {scan.center_mm:g} mm　｜　最低峰值 {peak:,.2f} N/s @ {best:g} mm"]
        baseline = scan.peaks[len(scan.heights) // 2]
        if baseline is not None:
            lines.append(f"基準峰值 {baseline:,.2f} N/s　｜　降低 {baseline - peak:,.2f} N/s")
        if scan.lookup:
            lines.append("採 CSV 零行程高度為基準，整條 Roll Center 曲線等量平移。")
        if not np.any(scan.best_result.signals["側向加速度 [m/s²]"]):
            lines.append("本工況的側向加速度為零。選擇轉彎工況可比較 Roll Center 傳力。")
        if scan.best_index in (0, len(scan.heights) - 1):
            lines.append("最低點位於本次掃描邊界。")
        if scan.failures:
            lines.append(f"{len(scan.failures)} 點求解失敗，最低值取自成功的高度。\n{scan.failures[0]}")
        self.scan_summary.set("\n".join(lines))
        series = [{"name": "輪胎端峰值", "x": scan.heights, "y": scan.peaks, "marker": "+"},
                  {"name": "最低點", "x": [best], "y": [peak], "marker": "x", "color": COLORS[1]}]
        self.scan_chart.set({"title": "Roll Center 高度掃描", "xlabel": "Roll Center 高度 (mm)",
                             "ylabel": "輪胎端轉移速率峰值 (N/s)", "series": series})
        self.scan_best_button.state(["!disabled"])

    def show_best_run(self):
        if self.scan_result is None:
            return
        self.playback.pause()
        self.set_result(self.scan_result.best_result)
        self.group.set("載重轉移速率")
        self.signal.set("輪胎端載重轉移速率 [N/s]")
        self.select_group()
        self.outputs.select(self.plots)
        height = self.scan_result.heights[self.scan_result.best_index]
        self.app.status.set(f"正在查看 Roll Center {height:g} mm 的掃描結果。")

    def case_input(self):
        try:
            values = {key: float(var.get()) for key, var in self.case_vars.items()}
        except ValueError as exc:
            raise ValueError("工況欄位須填入數值。") from exc
        case = {**self.case, **values, "scenario": key_for(SCENARIOS, self.scenario.get()),
                "target": key_for(TARGET_LABELS, self.target.get()), "rear_delay": self.rear_delay.get(),
                "iso_lateral": self.iso_lateral.get(), "recording": self.recording, "recording_name": self.recording_name}
        case.pop("quarter_mr", None)
        case.pop("roll_center_frame", None)
        return {"model": self.model_key, "case": case}

    @property
    def model_key(self):
        return key_for(self.models, self.model.get())

    def case_mode(self):
        model = self.model_key
        choices = MODEL_SOURCES.get(model, {})
        self.source_widget.configure(values=tuple(choices.values()), textvariable=self.app.source_vars.get(model, self.full_source),
                                     state="readonly" if choices else "disabled")
        corners = model_corners(model, self.app.sources())
        if self.app.analysis is self:
            self.app.refresh_channels()

        has_ax, has_ay = AXES[model]
        scenario_options = tuple(label for key, label in SCENARIOS.items()
                                 if (key not in MANEUVER_CASES or has_ax or has_ay) and (key != "cornering" or has_ay))
        self.scenario_widget.configure(values=scenario_options)
        if self.scenario.get() not in scenario_options:
            self.scenario.set(SCENARIOS["bump"])
        scenario = key_for(SCENARIOS, self.scenario.get())
        bump, iso = scenario in BUMP_CASES, scenario in ISO_ROUGHNESS
        ax, ay = case_axes(scenario, AXES[model], self.iso_lateral.get())
        if iso and has_ay:
            self.iso_lateral_widget.grid()
        else:
            self.iso_lateral_widget.grid_remove()
        target_options = tuple(TARGET_LABELS[key] for key in (*corners, "all")) if choices else tuple(TARGET_LABELS.values())
        self.target_widget.configure(values=target_options)
        if self.target.get() not in target_options:
            previous = TARGET_WHEELS[key_for(TARGET_LABELS, self.target.get())]
            matching = [corner for corner in corners if corner in previous]
            self.target.set(TARGET_LABELS[matching[0] if len(matching) == 1 else "all"])
        active = {"dt", "speed"}
        if scenario != "csv":
            active.update(("duration", "start"))
        if bump:
            active.update(("height", "period"))
        if iso:
            active.add("seed")
        if ax or ay:
            active.add("ramp")
            active.update(key for key, enabled in (("ax", ax), ("ay", ay)) if enabled)
        for key, widget in self.case_widgets.items():
            widget.configure(state="normal" if key in active else "disabled")
        self.target_widget.configure(state="readonly" if bump or iso else "disabled")
        delay_label = "同側後輪追蹤前輪（軸距 ÷ 車速）" if iso else "選定後輪延遲（軸距 ÷ 車速）"
        self.delay_widget.configure(text=delay_label, state="normal" if (bump or iso) and has_ax else "disabled")
        span = f"｜{self.recording[-1][0] - self.recording[0][0]:g} s" if self.recording else ""
        self.recording_label.set(f"{self.recording_name}{span}" if self.recording else "CSV 回放採用完整量測時間。")

    def load_recording(self):
        path = filedialog.askopenfilename(filetypes=[("路面與加速度 CSV", "*.csv")])
        if not path:
            return
        try:
            rows = read_recording(path, confirm_missing=self.confirm_recording_fill)
        except (OSError, ValueError) as exc:
            return self.app.error(exc)
        if rows is None:
            return
        self.recording, self.recording_name = rows, Path(path).name
        self.scenario.set(SCENARIOS["csv"])
        self.case_mode()
        self.app.status.set(f"已讀取 {len(rows)} 筆量測資料。")

    def confirm_recording_fill(self, missing):
        lines = [f"• {RECORDING_SIGNAL_LABELS[name]}（{name}）" for name in missing]
        question = "要將這些欄位整欄填為 0 後匯入嗎？"
        if any(name.startswith("z_") for name in missing):
            question += "\n路面高度為 0 表示該輪沿平坦路面回放。"
        return messagebox.askyesno(
            "CSV 缺少欄位",
            "找不到下列回放欄位。\n\n" + "\n".join(lines) + "\n\n" + question,
            parent=self.app.root, default=messagebox.NO,
        )

    def set_result(self, result):
        self.result = result
        self.motion.set_result(result)
        self.dashboard.set_result(result)
        self.playback.set_result(result)
        self.show_result()

    def draw_frame(self, index, timestamp):
        self.motion.update_frame(index)
        self.dashboard.update_frame(index)
        self.chart.cursor(timestamp)

    def pin(self):
        if self.result is not None:
            self.baseline = self.result
            self.show_result()

    def clear_baseline(self):
        self.baseline = None
        self.show_result()

    def comparison(self):
        if self.baseline is not self.result and self.baseline is not None and self.result is not None:
            if self.baseline.signature == self.result.signature:
                return self.baseline
        return None

    def show_result(self):
        self.metrics.delete(*self.metrics.get_children())
        self.details.configure(state="normal")
        self.details.delete("1.0", "end")
        result, baseline = self.result, self.comparison()
        if result is None:
            label = "尚無結果"
            if self.baseline is not None:
                label += f"｜已保留 A：{self.baseline.project['name']}"
            self.result_label.set(label)
            self.group.set("")
            self.signal.set("")
            self.group.configure(values=())
            self.signal.configure(values=())
            self.chart.cursor(None)
            self.chart.set(None)
            if self.extrema_note is not None:
                self.extrema_note.pack_forget()
            if self.tender_note is not None:
                self.tender_note.pack_forget()
        else:
            label = f"目前｜{result.project['name']} · {MODELS[result.project['model']]}"
            if result.project["model"] in MODEL_SOURCES:
                label += f" · {'／'.join(result.corners)}"
            if self.baseline is result:
                label += "｜已設為 A"
            elif baseline:
                label += f"｜A：{baseline.project['name']}"
            elif self.baseline is not None:
                label += "｜A 的模型、輪位或工況不同，暫停疊圖"
            self.result_label.set(label)
            for corner in result.corners:
                values = result.metrics[corner]
                self.metrics.insert("", "end", values=(corner, *(f"{value:.2f}" for value in values[:-1]), f"{values[-1]:.4f}"))
            lines = [f"車身升沉加速度 RMS　{result.metrics['body_rms']:.3f} m/s²",
                     f"靜態輪荷總計　{sum(result.metrics[c][0] for c in result.corners):.1f} N　｜　積分嘗試 {result.steps:,} 步"]
            if result.project.get("aero", AERO_DEFAULTS)["enabled"]:
                force = result.signals["全車下壓力 [N]"][0]
                speed = result.project["case"]["speed"] * 3.6
                line = f"空力　{speed:g} km/h　｜　全車下壓力 {force:,.1f} N"
                if "模型承受下壓力 [N]" in result.signals:
                    line += f"　｜　模型承受 {result.signals['模型承受下壓力 [N]'][0]:,.1f} N"
                lines.append(line)
            if peaks := result.metrics.get("load_transfer_rate"):
                rows = [f"{name}　{rate:+,.1f} N/s　於 {time:.4f} s" for name, (rate, time) in peaks.items()]
                lines.append("載重轉移速率峰值（絕對值最大，保留方向）\n" + "\n".join(rows))
            if baseline:
                lines.append(f"B − A 車身升沉加速度 RMS　{result.metrics['body_rms'] - baseline.metrics['body_rms']:+.3f} m/s²")
                changes = [f"{c} {result.metrics[c][1] - baseline.metrics[c][1]:+.1f}" for c in result.corners]
                lines.append("B − A 最低輪荷 (N)　" + "　".join(changes))
            lines.extend(result.warnings or ["本次輸出未觸發零輪荷、限位或大角度警示。"])
            self.details.insert("1.0", "\n\n".join(lines))
            self.group.configure(values=tuple(result.groups))
            if self.group.get() not in result.groups:
                self.group.set("車身姿態" if "車身姿態" in result.groups else "輪荷")
            self.select_group()
        self.details.configure(state="disabled")

    def select_group(self):
        group = self.group.get()
        if self.result is None or group not in self.result.groups:
            return
        signals = self.result.groups[group]
        if self.family == "full" and group == "車身姿態":
            choices = ["全部"]
        else:
            choices = ["全部", *signals] if len({s.rsplit("[", 1)[1] for s in signals}) == 1 else signals
        self.signal.configure(values=choices)
        if self.signal.get() not in choices:
            self.signal.current(0)
        self.draw_result()

    def draw_result(self):
        result = self.result
        group, selected = self.group.get(), self.signal.get()
        names = result.groups[group] if selected == "全部" else [selected]
        baseline = self.comparison()
        series, extrema, extrema_notes = [], [], []
        unit = names[0].rsplit("[", 1)[1][:-1]
        transfer_peaks = self.family == "half_lr" and group in LOAD_TRANSFER_GROUPS
        for index, name in enumerate(names):
            color = COLORS[index % len(COLORS)]
            signal_label = name.split(" [", 1)[0]
            peak_labels = []
            for run, prefix, dash in ((result, "B " if baseline else "", ()), (baseline, "A ", (5, 3))):
                if run and name in run.signals:
                    y = run.signals[name]
                    label = prefix + signal_label
                    if self.family == "quarter":
                        markers, note = curve_extrema(run.time, y, label, unit, color)
                        extrema.extend(markers)
                        extrema_notes.append(note)
                    elif transfer_peaks:
                        peak = np.argmax(np.abs(y))
                        value, time = float(y[peak]), float(run.time[peak])
                        shape = "+" if run is baseline else "x"
                        if value:
                            extrema.append({"x": [time], "y": [value], "marker": shape, "color": color})
                        peak_labels.append(f"{prefix}{value:+,.4g} {unit}（{time:.4f} s）")
                    # Retain each bin's minimum and maximum when drawing large recordings.
                    if len(y) > 3000:
                        stride = int(np.ceil(len(y) / 1200))
                        full = len(y) // stride * stride
                        bins = y[:full].reshape(-1, stride)
                        starts = np.arange(0, full, stride)
                        indices = np.unique(np.r_[0, starts + bins.argmin(axis=1), starts + bins.argmax(axis=1),
                                                  np.arange(full, len(y)), len(y) - 1])
                    else:
                        indices = slice(None)
                    series.append({"name": label, "color": color, "dash": dash,
                                   "x": run.time[indices].tolist(), "y": y[indices].tolist()})
            if peak_labels:
                extrema_notes.append(f"{signal_label.removesuffix(group)}峰值　" + "　".join(peak_labels))
        if self.extrema_note is not None:
            if extrema_notes:
                if transfer_peaks:
                    extrema_notes.insert(0, "峰值取絕對值最大，保留方向" + ("　× B／＋ A" if baseline else ""))
                self.extrema_note.configure(text="\n".join(extrema_notes))
                self.extrema_note.pack(fill="x", before=self.chart, pady=(0, 4))
            else:
                self.extrema_note.pack_forget()
        if self.tender_note is not None:
            self.tender_note.pack_forget()
            if group == "輪端行程":
                axle_colors = None
                if self.family == "half_fr":
                    by_wheel = {name: COLORS[index] for index, name in enumerate(names)}
                    axle_colors = {key: by_wheel[signal] for key, corner in zip(("heave_f", "heave_r"), result.corners)
                                   if (signal := f"{corner} 輪端行程 [mm]") in by_wheel}
                self.draw_travel_limits(series, result, baseline, axle_colors)
                self.draw_tender_switches(series, result, baseline, axle_colors)
        series.extend(extrema)
        self.chart.set({"title": group, "xlabel": "時間 (s)", "ylabel": unit, "series": series})

    def draw_travel_limits(self, series, result, baseline, axle_colors):
        styles = {"compression": ("壓縮限位", "#fb7185"), "rebound": ("伸長限位", "#67e8f9")}
        for run, prefix, dash in ((result, "B " if baseline else "", (10, 3)), (baseline, "A ", (4, 3, 1, 3))):
            if run is None:
                continue
            for key, limits in run.wheel_travel_limits.items():
                if axle_colors is not None and key not in axle_colors:
                    continue
                for direction, travel in limits.items():
                    label, color = styles[direction]
                    line_dash = dash
                    if axle_colors is not None:
                        color = axle_colors[key]
                        if direction == "rebound":
                            line_dash = (2, 3, 6, 3) if run is baseline else (2, 4)
                    series.append({"name": f"{prefix}{CHANNELS[key]}{label} {travel:.3g} mm", "color": color,
                                   "dash": line_dash, "x": [float(run.time[0]), float(run.time[-1])], "y": [travel, travel]})

    def draw_tender_switches(self, series, result, baseline, axle_colors):
        notes = []
        markers = []
        for run, prefix, colors, dash in ((result, "B " if baseline else "", (COLORS[1], COLORS[2], COLORS[5]), (6, 4)),
                                          (baseline, "A ", (COLORS[3], COLORS[4], "#e2e8f0"), (2, 4))):
            if run is None:
                continue
            if axle_colors is not None:
                for key in axle_colors:
                    limits = run.wheel_travel_limits.get(key, {})
                    info = run.tender_switches.get(key)
                    thresholds = [f"{label} {limits[direction]:.3g}" for direction, label in
                                  (("compression", "壓縮"), ("rebound", "伸長")) if direction in limits]
                    if info is not None:
                        thresholds.append(f"併圈 {info.wheel_threshold_mm:.3g}")
                    if thresholds:
                        notes.append(f"{prefix}{CHANNELS[key]}（mm）　" + "　".join(thresholds))
            for (key, info), color in zip(run.tender_switches.items(), colors):
                if axle_colors is not None:
                    if key not in axle_colors:
                        continue
                    color = axle_colors[key]
                threshold = info.wheel_threshold_mm
                name = prefix + CHANNELS[key]
                series.append({"name": name + "併圈門檻", "color": color, "dash": dash,
                               "x": [float(run.time[0]), float(run.time[-1])], "y": [threshold, threshold]})
                binding, release = info.binding_times, info.release_times
                for times, marker in ((binding, "+"), (release, "x")):
                    if len(times):
                        markers.append({"color": color, "marker": marker, "marker_only": True,
                                        "x": times.tolist(), "y": [threshold] * len(times)})
                if axle_colors is not None:
                    continue
                count = len(binding) + len(release)
                initial = "主簧段" if info.starts_bound else "串聯段"
                span = "起始" if count else "全程"
                note = (f"{name} 門檻 {threshold:.3g} mm（無空力靜態後 {threshold - info.wheel_reference_mm:+.3g} mm）"
                        f"｜{span}{initial}｜{count} 次切換")
                if count:
                    first = release[0] if info.starts_bound else binding[0]
                    note += f"｜首次 {first:.4f} s"
                notes.append(note)
        if notes:
            if markers:
                notes.append("＋併圈 · × 回到串聯段（依輸出取樣估算）")
                series.extend(markers)
            self.tender_note.configure(text="\n".join(notes))
            self.tender_note.pack(fill="x", before=self.chart, pady=(0, 4))

    def export_csv(self):
        if self.result is None:
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if path:
            try:
                write_csv(path, self.result)
                self.app.status.set(f"已匯出 {Path(path).name}。")
            except OSError as exc:
                self.app.error(exc)

    def export_snapshot(self):
        if self.result is None:
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="run-setup.json", filetypes=[("JSON", "*.json")])
        if path:
            try:
                save_project(path, self.result.project)
                self.app.status.set(f"已儲存計算快照 {Path(path).name}。")
            except (OSError, ValueError) as exc:
                self.app.error(exc)


def write_csv(path, result):
    with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", *result.signals))
        writer.writerows(zip(result.time, *result.signals.values()))
