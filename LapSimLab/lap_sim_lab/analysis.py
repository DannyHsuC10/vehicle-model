"""Fixed-condition attitude sweeps using the LapSim aero and tire models."""

from dataclasses import dataclass
from math import isfinite

import numpy as np

from .car import Car
from .planner import AeroTireLimits, GRAVITY, camber_from_roll, distribute_lateral_force
from .settings import Settings
from .tire import SimplifiedTireModel


@dataclass(frozen=True)
class AttitudeSweep:
    axis: str = "Roll"
    extent_deg: float = 3.0
    fixed_deg: float = 0.0
    speed_kmh: float = 72.0
    heave_mm: float = 0.0
    ax_g: float = 0.0
    ay_g: float = 0.0

    def poses(self):
        if self.axis not in ("Roll", "Pitch"):
            raise ValueError("掃描角度須為 Roll 或 Pitch。")
        if not 0 < self.extent_deg < 90 or not abs(self.fixed_deg) < 90:
            raise ValueError("掃描幅度須介於 0° 與 90°，固定角的絕對值須小於 90°。")
        if not all(map(isfinite, (self.speed_kmh, self.heave_mm, self.ax_g, self.ay_g))) or self.speed_kmh < 0:
            raise ValueError("車速須為非負數，且車速、Heave 和加速度都須為有限值。")
        angles = np.linspace(-self.extent_deg, self.extent_deg, 121)
        roll = angles if self.axis == "Roll" else np.full_like(angles, self.fixed_deg)
        pitch = angles if self.axis == "Pitch" else np.full_like(angles, self.fixed_deg)
        return angles, zip(np.radians(roll), np.radians(pitch))


AERO_SIGNALS = ("F_down", "F_drag", "F_FL", "F_FR", "F_RL", "F_RR", "CL", "CD")


def sweep_aero(settings: Settings, sweep: AttitudeSweep):
    angles, poses = sweep.poses()
    aero = Car(settings, preload_motor=False).aero
    if aero is None:
        return angles, {key: np.zeros_like(angles) for key in (*AERO_SIGNALS, "front", "rear")}
    with np.errstate(over="raise", invalid="raise"):
        rows = [
            aero.calculate_from_modal(sweep.speed_kmh / 3.6, sweep.heave_mm / 1000, pitch, roll)
            for roll, pitch in poses
        ]
    data = {key: np.array([row[key] for row in rows]) for key in AERO_SIGNALS}
    data["front"] = data["F_FL"] + data["F_FR"]
    data["rear"] = data["F_RL"] + data["F_RR"]
    return angles, data


@dataclass(frozen=True)
class TireSweepResult:
    angles: np.ndarray
    fy_peak: np.ndarray
    fx_drive: np.ndarray
    fx_brake: np.ndarray
    feasible: np.ndarray


def sweep_tires(settings: Settings, sweep: AttitudeSweep) -> TireSweepResult:
    angles, poses = sweep.poses()
    car = Car(settings, preload_motor=False)
    limits = AeroTireLimits(SimplifiedTireModel(settings.tire_path), car)
    speed, heave = sweep.speed_kmh / 3.6, sweep.heave_mm / 1000
    ax, ay = sweep.ax_g * GRAVITY, sweep.ay_g * GRAVITY
    fy_peak, fx_drive, fx_brake = (np.empty((len(angles), 4)) for _ in range(3))
    feasible = np.empty(len(angles), dtype=bool)

    with np.errstate(over="raise", invalid="raise"):
        for index, (roll, pitch) in enumerate(poses):
            aero = limits.aero_result_from_attitude(speed, heave, pitch, roll)
            state = limits.suspension_state(float(aero["F_down"]), ax, ay)
            loads = np.maximum(state.loads, 0.0)
            contact = loads > 0
            camber = camber_from_roll(np.array([roll, roll, pitch, 0.0]))
            fy_peak[index] = np.where(contact, limits.max_fy_for_loads(loads, camber), 0.0)
            lateral_demand = distribute_lateral_force(car.m * ay, loads)
            brake, drive = limits.fx_limits_for_loads_and_fy(loads, lateral_demand, camber)
            fx_drive[index] = np.where(contact, drive, 0.0)
            fx_brake[index] = np.where(contact, brake, 0.0)
            feasible[index] = np.all(np.abs(lateral_demand) <= fy_peak[index])

    return TireSweepResult(angles, fy_peak, fx_drive, fx_brake, feasible)
