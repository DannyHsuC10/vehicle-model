"""Serializable shared vehicle, alignment and experiment settings."""
from dataclasses import asdict, dataclass, fields
import json
import math
from pathlib import Path

G = 9.80665
CORNERS = ("FL", "FR", "RL", "RR")


@dataclass
class Vehicle:
    name: str = "FSAE EV - four independent wheel motors (estimated)"
    curb_mass: float = 250.0
    occupants: float = 75.0
    wheelbase: float = 1.55
    track_front: float = 1.20
    track_rear: float = 1.20
    cg_height: float = 0.27
    front_fraction: float = 0.48
    unsprung_front: float = 9.0  # kg per front wheel, left/right symmetric
    unsprung_rear: float = 9.0  # kg per rear wheel, left/right symmetric
    unsprung_height_front: float = 0.23
    unsprung_height_rear: float = 0.23
    wheel_rate_front: float = 26000.0
    wheel_rate_rear: float = 28000.0
    motion_ratio_front: float = 1.20
    motion_ratio_rear: float = 1.20
    bell_ratio_front: float = 1.0
    bell_ratio_rear: float = 1.0
    damping_comp_front: float = 900.0
    damping_reb_front: float = 1200.0
    damping_comp_rear: float = 1000.0
    damping_reb_rear: float = 1300.0
    arb_front: float = 1000.0  # N m / rad
    arb_rear: float = 800.0
    arb_motion_ratio_front: float = 0.85
    arb_motion_ratio_rear: float = 0.80
    camber_front: float = -1.5  # degrees, negative top inward
    camber_rear: float = -1.0
    toe_front: float = 0.0  # degrees per wheel, positive inward
    toe_rear: float = 0.0
    cd: float = 0.9
    frontal_area: float = 1.05
    cla_front: float = 0.5
    cla_rear: float = 0.6
    rho: float = 1.225
    tire_diameter_front: float = 0.46
    tire_diameter_rear: float = 0.46
    tire_width_front: float = 0.18
    tire_width_rear: float = 0.18
    pressure_front: float = 80000.0
    pressure_rear: float = 80000.0
    mu_lat: float = 1.5
    mu_long: float = 1.5
    load_sensitivity: float = 0.15
    camber_sensitivity: float = 0.025  # fractional loss per absolute degree
    reference_tire_load: float = 1000.0
    tire_b: float = 8.0
    tire_c: float = 1.3
    tire_e: float = -1.0
    brake_front: float = 0.58
    motor_curve_csv: str = "input_data/motor_speed_vs_torque.csv"
    reduction_front: float = 12.0
    reduction_rear: float = 12.0
    reduction_efficiency_front: float = 0.97
    reduction_efficiency_rear: float = 0.97
    rolling_coefficient: float = 0.015

    @property
    def mass(self):
        return self.curb_mass + self.occupants

    def axle(self, front):
        return "front" if front else "rear"

    def sprung(self, front):
        fraction = self.front_fraction if front else 1 - self.front_fraction
        unsprung = 2 * (self.unsprung_front if front else self.unsprung_rear)
        return self.mass * fraction - unsprung

    def ratio(self, front):
        suffix = self.axle(front)
        return getattr(self, "motion_ratio_" + suffix) * getattr(self, "bell_ratio_" + suffix)

    def validate(self):
        for key, value in asdict(self).items():
            if key in ("name","motor_curve_csv"):
                if not isinstance(value,str) or not value.strip():
                    raise ValueError(f"{key}: must be a nonempty string")
            elif not isinstance(value,(float,int)) or isinstance(value,bool) or not math.isfinite(value):
                raise ValueError(f"{key}: must be a finite number")
        for key in ("curb_mass", "wheelbase", "track_front", "track_rear", "cg_height",
                    "wheel_rate_front", "wheel_rate_rear", "motion_ratio_front", "motion_ratio_rear",
                    "bell_ratio_front", "bell_ratio_rear", "arb_motion_ratio_front", "arb_motion_ratio_rear", "rho", "tire_diameter_front",
                    "tire_diameter_rear", "pressure_front", "pressure_rear", "reference_tire_load",
                    "reduction_front", "reduction_rear", "mu_lat", "mu_long", "tire_width_front", "tire_width_rear", "tire_b", "tire_c"):
            if getattr(self, key) <= 0:
                raise ValueError(f"{key}: must be > 0")
        for key in ("occupants", "unsprung_front", "unsprung_rear",
                    "arb_front", "arb_rear", "cd", "frontal_area", "cla_front", "cla_rear",
                    "damping_comp_front", "damping_reb_front", "damping_comp_rear", "damping_reb_rear",
                    "unsprung_height_front", "unsprung_height_rear", "load_sensitivity",
                    "camber_sensitivity", "rolling_coefficient"):
            if getattr(self, key) < 0:
                raise ValueError(f"{key}: must be >= 0")
        if not 0 < self.front_fraction < 1 or not 0 <= self.brake_front <= 1:
            raise ValueError("Weight fraction must be 0..1 (exclusive); brake fraction 0..1")
        if not 0 < self.reduction_efficiency_front <= 1 or not 0 < self.reduction_efficiency_rear <= 1:
            raise ValueError("Reduction efficiency must be > 0 and <= 1")
        if min(self.sprung(True), self.sprung(False)) <= 0:
            raise ValueError("Unsprung axle mass must be less than static axle mass")

    @classmethod
    def from_dict(cls, data):
        data = dict(data)
        # Existing schema-2 FSAE projects stored four corners. Preserve each
        # axle's total unsprung mass when converting to symmetric per-wheel kg.
        legacy = {"unsprung_fl", "unsprung_fr", "unsprung_rl", "unsprung_rr"}
        present = legacy.intersection(data)
        if present:
            if present != legacy:
                raise ValueError("Legacy unsprung masses require all four corner fields")
            if {"unsprung_front", "unsprung_rear"}.intersection(data):
                raise ValueError("Do not mix per-axle and legacy corner unsprung fields")
            for key in legacy:
                value = data[key]
                if not isinstance(value,(float,int)) or isinstance(value,bool) or not math.isfinite(value) or value < 0:
                    raise ValueError(f"{key}: legacy unsprung mass must be finite and >= 0")
            data["unsprung_front"] = data.pop("unsprung_fl") / 2 + data.pop("unsprung_fr") / 2
            data["unsprung_rear"] = data.pop("unsprung_rl") / 2 + data.pop("unsprung_rr") / 2
        names = {f.name for f in fields(cls)}
        unknown=set(data)-names
        if unknown:raise ValueError("Unsupported vehicle fields: "+", ".join(sorted(unknown)))
        obj = cls(**data)
        obj.validate()
        return obj


def write_project(path, vehicle, geometry, settings):
    vehicle.validate()
    Path(path).write_text(json.dumps({"schema": 2, "vehicle": asdict(vehicle),
        "geometry": geometry, "settings": settings}, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def read_project(path):
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if data.get("schema") != 2:
        raise ValueError("Use an FSAE four-motor project (schema 2); legacy drivetrain projects are unsupported")
    return Vehicle.from_dict(data["vehicle"]), data["geometry"], data.get("settings", {})
