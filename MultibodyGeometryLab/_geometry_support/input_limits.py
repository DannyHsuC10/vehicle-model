"""Editable input guardrails for this workbench, in stored SI units.

These are application limits, not competition rules or proof of a feasible
suspension. Project relationships and the geometry solver add further checks.
"""
from dataclasses import dataclass
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
        elif self.high == math.inf:
            text = f"{'>' if self.open_low else '≥'} {self.low:g}，有限數值"
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
    "roll_stiffness_ratio": InputLimit(0, math.inf, open_low=True),
}
AXLE_LIMITS = {
    "fvsa": InputLimit(.05, 20), "rc_height": InputLimit(-.5, 1),
    "kpi_deg": InputLimit(-45, 45), "scrub_radius": InputLimit(-.3, .3),
    "upper_offset": InputLimit(.005, .5), "lower_offset": InputLimit(.005, .5),
    "lower_mount_x": InputLimit(0, 1.5), "upper_mount_x": InputLimit(0, 1.5),
    "caster_deg": InputLimit(-45, 45), "mechanical_trail": InputLimit(-.3, .3),
    "anti": InputLimit(-math.inf, math.inf), "svsa": InputLimit(-math.inf, math.inf, min_abs=1e-8),
    "static_camber_deg": InputLimit(-20, 20),
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
