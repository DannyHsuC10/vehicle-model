"""Editable input guardrails for this workbench, in stored SI units.

These are application limits, not competition rules or proof of a feasible
suspension. Project relationships and the geometry solver add further checks.
"""
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
import math


@dataclass(frozen=True)
class InputLimit:
    low: float
    high: float
    open_low: bool = False
    open_high: bool = False
    integer: bool = False
    min_abs: float = 0.

    def accepts(self, value):
        return (type(value) in (float, int) and math.isfinite(value)
                and (value > self.low if self.open_low else value >= self.low)
                and (value < self.high if self.open_high else value <= self.high)
                and (not self.integer or float(value).is_integer())
                and abs(value) >= self.min_abs)

    def describe(self, unit=""):
        if self.low == -math.inf and self.high == math.inf:
            text = "有限數值"
        else:
            text = f"{'(' if self.open_low else '['}{self.low:g}, {self.high:g}{')' if self.open_high else ']'}"
        if unit:
            text += " " + unit
        if self.integer:
            text += "，整數"
        if self.min_abs:
            text += f"，|值| ≥ {self.min_abs:g}"
        return text


VEHICLE_LIMITS = {
    "wheelbase": InputLimit(.5, 5), "cg_to_front": InputLimit(0, 5, open_low=True),
    "track_front": InputLimit(.4, 3), "track_rear": InputLimit(.4, 3),
    "cg_height": InputLimit(.05, 1.5), "mass": InputLimit(50, 2000),
    "gravity": InputLimit(9, 10), "ground_clearance": InputLimit(0, .5),
    "free_radius": InputLimit(.1, .6), "tire_stiffness": InputLimit(10000, 1000000),
    "tire_width": InputLimit(.05, .6), "rim_inner_diameter": InputLimit(.1, 1),
    "rim_gap": InputLimit(0, .1), "clearance_steer_deg": InputLimit(0, 80),
    "brake_bias_front": InputLimit(0, 1),
    "drive_force_front": InputLimit(0, 1),
}
AXLE_LIMITS = {
    "fvsa": InputLimit(.05, 20), "rc_height": InputLimit(-.5, 1),
    "kpi_deg": InputLimit(-45, 45), "scrub_radius": InputLimit(-.3, .3),
    "upper_offset": InputLimit(.005, .5), "lower_offset": InputLimit(.005, .5),
    "lower_mount_x": InputLimit(0, 1.5), "upper_mount_x": InputLimit(0, 1.5),
    "caster_deg": InputLimit(-45, 45), "mechanical_trail": InputLimit(-.3, .3),
    "anti": InputLimit(-math.inf, math.inf), "svsa": InputLimit(-math.inf, math.inf, min_abs=1e-8),
}
ANALYSIS_LIMITS = {
    "bump_mm": InputLimit(0, 200), "droop_mm": InputLimit(0, 200),
    "steer_deg": InputLimit(0, 80, open_low=True),
    "caster_min_deg": InputLimit(-45, 45), "caster_max_deg": InputLimit(-45, 45),
    "samples": InputLimit(5, 51, integer=True),
}
FIELD_LIMITS = {"vehicle": VEHICLE_LIMITS, "front": AXLE_LIMITS,
                "rear": AXLE_LIMITS, "analysis": ANALYSIS_LIMITS}
NAME_MAX_LENGTH = 128

INPUT_STEPS = {
    "track_front": .005, "track_rear": .005, "fvsa": .005, "rc_height": .005,
    "kpi_deg": .5, "scrub_radius": .001, "lower_mount_x": .005, "upper_mount_x": .005,
    "caster_deg": .5, "mechanical_trail": .001,
    "anti": .05,
}


def round_to_step(value, step):
    if value is None or not math.isfinite(value):
        return value
    step = Decimal(str(step))
    count = (Decimal(str(value)) / step).to_integral_value(rounding=ROUND_HALF_UP)
    return float(count * step) if count else 0.


def round_design_input(group, key, value):
    step = INPUT_STEPS.get(key)
    if step is not None and FIELD_LIMITS[group][key].accepts(value):
        return round_to_step(value, step)
    return value


# Hardpoint XYZ and arm-mount X requests have no artificial numeric bounds.
# A blank/null arm-mount X selects the geometry's automatically solved extremum.


RELATION_HINTS = {
    ("vehicle", "wheelbase"): "須大於重心至前軸距離",
    ("vehicle", "cg_to_front"): "須小於軸距",
    ("vehicle", "ground_clearance"): "不得高於重心",
    ("vehicle", "rim_inner_diameter"): "須小於輪胎自由直徑",
    ("vehicle", "rim_gap"): "兩倍間隙須小於輪圈內徑",
    ("analysis", "bump_mm"): "上壓、下垂不可同時為 0",
    ("analysis", "droop_mm"): "上壓、下垂不可同時為 0",
    ("analysis", "caster_min_deg"): "須小於掃描上限",
    ("analysis", "caster_max_deg"): "須大於掃描下限",
}


def range_hint(group, key, unit):
    text = "允許範圍：" + FIELD_LIMITS[group][key].describe(unit if unit not in ("0..1", "5..51") else "")
    relation = RELATION_HINTS.get((group, key))
    if group in ("front", "rear") and key in ("lower_mount_x", "upper_mount_x"):
        relation = "須小於該軸半輪距"
    if relation:
        text += "；" + relation
    if key in INPUT_STEPS:
        step = "5%" if key == "anti" else f"{INPUT_STEPS[key]:g} {unit}"
        text += "；級距 " + step
    return text
