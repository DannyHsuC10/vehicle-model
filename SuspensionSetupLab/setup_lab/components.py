"""Replaceable KC laws adapted from new/dynamics_analyze/model/KC_sys."""

from bisect import bisect_left, bisect_right
from itertools import accumulate
import math

MAX_CURVE_POINTS = 256
DAMPER_DIRECTIONS = ("compression", "rebound")
DAMPER_STAGE_COUNTS = {"single": 1, "split": 1, "bilinear": 2}


def number(value, label, minimum=None, maximum=None):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or (minimum is not None and value < minimum)
            or (maximum is not None and value > maximum)):
        bounds = "" if minimum is None else f"（最小 {minimum:g}"
        if bounds:
            bounds += "）" if maximum is None else f"，最大 {maximum:g}）"
        raise ValueError(f"{label} 必須是有效數值{bounds}。")
    return float(value)


def points(rows, label, minimum_x=None, minimum_y=None):
    if not isinstance(rows, list) or not 1 <= len(rows) <= MAX_CURVE_POINTS:
        raise ValueError(f"{label} 需要 1–{MAX_CURVE_POINTS} 筆資料。")
    table = []
    for index, row in enumerate(rows, 1):
        if not isinstance(row, (list, tuple)) or len(row) != 2:
            raise ValueError(f"{label}第 {index} 列需要兩個數值。")
        x = number(row[0], f"{label}第 {index} 列第一欄", minimum_x)
        y = number(row[1], f"{label}第 {index} 列第二欄", minimum_y)
        if table and x <= table[-1][0]:
            raise ValueError(f"{label}第一欄必須嚴格遞增。")
        table.append((x, y))
    return table


def spring_points(rows, *, stages):
    table = points(rows, "彈簧曲線", minimum_y=0 if stages else None)
    if not stages and (len(table) < 2 or any(b[1] < a[1] for a, b in zip(table, table[1:]))):
        raise ValueError("彈簧力表至少需要兩列，力值須隨壓縮行程非遞減。")
    return table


def damper_points(rows, label, *, stages):
    table = points(rows, label, minimum_x=0, minimum_y=0)
    if table[0][0] != 0 or (not stages and (table[0][1] != 0 or len(table) < 2)):
        raise ValueError(f"{label}係數表須由速度 0 起算，力表至少兩列且從 (0, 0) 起算。")
    return table


def damper_kinds(kind):
    # Old projects store one type for both directions.
    if isinstance(kind, str) and kind in DAMPER_STAGE_COUNTS:
        kind = "stages"
    kinds = kind.copy() if isinstance(kind, dict) else dict.fromkeys(DAMPER_DIRECTIONS, kind)
    if kinds.keys() != set(DAMPER_DIRECTIONS) or any(value not in ("stages", "force") for value in kinds.values()):
        raise ValueError("無法辨識阻尼曲線類型。")
    return kinds


class Curve:
    """Piecewise linear lookup, or a continuous integral of piecewise slopes."""

    def __init__(self, table, *, stages=False, extrapolate=False):
        self.x, self.y = zip(*table)
        self.stages = stages
        self.extrapolate = extrapolate
        intervals = tuple(b - a for a, b in zip(self.x, self.x[1:]))
        if stages:
            areas = (dx * y for dx, y in zip(intervals, self.y))
        else:
            self.slopes = tuple((b - a) / dx for a, b, dx in zip(self.y, self.y[1:], intervals))
            areas = (dx * (a + b) / 2 for dx, a, b in zip(intervals, self.y, self.y[1:]))
        self.area = tuple(accumulate(areas, initial=0.0))
        self.zero = self.integral(0.0)

    def lookup(self, x):
        i = bisect_right(self.x, x) - 1
        if i < 0:
            slope = self.slopes[0] if self.extrapolate and self.slopes else 0.0
            return self.y[0] + slope * (x - self.x[0])
        slope = self.slopes[i] if i < len(self.slopes) else 0.0
        if i == len(self.slopes) and self.extrapolate and self.slopes:
            slope = self.slopes[-1]
        return self.y[i] + slope * (x - self.x[i])

    def integral(self, x):
        i = max(0, bisect_right(self.x, x) - 1)
        delta = x - self.x[i]
        slope = 0.0
        if not self.stages and 0 <= delta and i < len(self.slopes):
            slope = self.slopes[i]
        return self.area[i] + delta * (self.y[i] + delta * slope / 2)

    def integrated(self, x):
        return self.integral(x) - self.zero


class KCSystem:
    """Positive travel and velocity indicate compression, positive reaction resists it."""

    def __init__(self, settings):
        self.enabled = settings["enabled"]
        if type(self.enabled) is not bool:
            raise ValueError("元件啟用欄位必須是布林值。")
        self.offset = number(settings["installed_mm"], "安裝預壓行程")
        self.jounce = number(settings["jounce_mm"], "壓縮限位行程", 0)
        self.rebound = number(settings["rebound_mm"], "伸長限位行程", 0)
        self.jounce_rate = number(settings["jounce_rate"], "壓縮限位剛性", 0)
        self.rebound_rate = number(settings["rebound_rate"], "伸長限位剛性", 0)
        limit_mode = settings.get("limit_rate_mode", "additive")
        if limit_mode not in ("additive", "total"):
            raise ValueError("無法辨識限位剛性類型。")
        self.total_limit_rate = limit_mode == "total"
        self.spring_kind = settings["spring_kind"]
        if self.spring_kind not in ("single", "stages", "tender", "force"):
            raise ValueError("無法辨識彈簧曲線類型。")
        if self.spring_kind in ("single", "tender"):
            main = number(settings["main_rate"], "彈簧／主簧剛性", 1e-6 if self.spring_kind == "tender" else 0.)
            table = [(0., main)]
            if self.spring_kind == "tender":
                tender = number(settings["tender_rate"], "Tender 剛性", 1e-6)
                travel = number(settings["tender_travel"], "Tender 併圈前行程", 1e-6)
                rate = main * (tender / (main + tender))
                bind = travel * (1 + tender / main)
                table = [(0., rate), (bind, main)]
            self.spring_curve = Curve(table, stages=True)
        else:
            stages = self.spring_kind == "stages"
            table = spring_points(settings["spring"], stages=stages)
            self.spring_curve = Curve(table, stages=stages, extrapolate=not stages)
        self.spring = self.spring_curve.integrated if self.spring_curve.stages else self.spring_curve.lookup
        if self.total_limit_rate:
            self.jounce_force = self.spring_force(self.jounce)
            self.rebound_force = self.spring_force(-self.rebound)

        damper_kind = settings["damper_kind"]
        kinds = damper_kinds(damper_kind)
        stage_count = DAMPER_STAGE_COUNTS.get(damper_kind) if isinstance(damper_kind, str) else None
        self.dampers = []
        for key, label in (("compression", "壓縮阻尼"), ("rebound", "回彈阻尼")):
            stages = kinds[key] == "stages"
            table = damper_points(settings[key], label, stages=stages)
            if stage_count is not None and len(table) != stage_count:
                raise ValueError(f"{label}需要 {stage_count} 段係數。")
            curve = Curve(table, stages=stages)
            self.dampers.append(curve.integrated if stages else curve.lookup)
        if damper_kind == "single" and settings["compression"][0][1] != settings["rebound"][0][1]:
            raise ValueError("一段式阻尼的壓縮與回彈須使用同一個係數。")
        self.motion_ratio = Curve(points(settings["mr"], "運動比曲線", minimum_y=1e-6))

    def spring_force(self, stroke_mm):
        return self.spring(stroke_mm + self.offset)

    def spring_breakpoints(self):
        """Return the spring curve's points in installed shaft coordinates."""
        return tuple(x - self.offset for x in self.spring_curve.x)

    def spring_slope(self, stroke_mm, *, from_right):
        curve = self.spring_curve
        bisect = bisect_right if from_right else bisect_left
        index = bisect(curve.x, stroke_mm + self.offset) - 1
        rates = curve.y if curve.stages else curve.slopes
        return rates[min(max(index, 0), len(rates) - 1)]

    def damper_force(self, speed):
        return self.dampers[0](speed) if speed >= 0 else -self.dampers[1](-speed)

    def stop_force(self, stroke_mm):
        if self.total_limit_rate:
            if self.jounce_rate and stroke_mm > self.jounce:
                limited = self.jounce_force + self.jounce_rate * (stroke_mm - self.jounce)
            elif self.rebound_rate and stroke_mm < -self.rebound:
                limited = self.rebound_force + self.rebound_rate * (stroke_mm + self.rebound)
            else:
                return 0.0
            return limited - self.spring_force(stroke_mm)
        return (self.jounce_rate * max(0.0, stroke_mm - self.jounce)
                - self.rebound_rate * max(0.0, -stroke_mm - self.rebound))

    def force(self, travel, velocity, details=False):
        if not self.enabled:
            return (0.0,) * 6 if details else 0.0
        travel_mm = travel * 1000
        ratio = self.motion_ratio.lookup(travel_mm)
        stroke = self.motion_ratio.integrated(travel_mm)
        speed = ratio * velocity
        spring = self.spring_force(stroke)
        damper = self.damper_force(speed)
        stop = self.stop_force(stroke)
        reaction = ratio * (spring + damper + stop)
        return (reaction, stroke, speed, spring, damper, stop) if details else reaction
