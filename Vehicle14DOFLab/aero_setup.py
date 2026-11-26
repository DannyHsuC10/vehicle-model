"""LapSimLab attitude aero and its force application to the sprung body."""

from dataclasses import dataclass
from functools import lru_cache
import math

import numpy as np

from ._vendor.lapsim.aero import ModalAeroModel, QuadraticAeroModel, read_aero_coefficients

from .resources import INPUT_DATA, source_path

MODAL_FILE = INPUT_DATA / "modal_aero.json"


@lru_cache(maxsize=1)
def modal_defaults():
    return read_aero_coefficients(MODAL_FILE)


def aero_settings(config, folder):
    source = read_aero_coefficients(source_path(folder, config["aero_file"])) if "aero_file" in config else {}
    raw = config.get("aero", {})
    if not isinstance(raw, dict) or not isinstance(raw.get("coefficients", {}), dict):
        raise ValueError("Aero settings and coefficients must be configuration objects")
    mode = raw.get("model", "quadratic" if "aero" in config or source else "modal")
    if mode not in ("modal", "quadratic") or type(raw.get("enabled", True)) is not bool:
        raise ValueError("Aero model must be modal or quadratic, with a boolean enabled setting")
    quadratic = QuadraticAeroModel.coefficient_defaults | source
    area = quadratic["rho"] * quadratic["area"] * .5 * 20.**2
    return {
        "model": mode, "enabled": raw.get("enabled", True), "reference_speed_kmh": 72.,
        "reference_downforce_n": -area * quadratic.get("CL0", -2.4),
        "reference_drag_n": area * quadratic.get("CD0", 1.8), **raw,
        "coefficients": modal_defaults() | source | raw.get("coefficients", {}),
    }


@dataclass(frozen=True)
class Loads:
    downforce: float
    drag: float
    fx: float
    fy: float
    roll: float
    pitch: float
    yaw: float
    cop_x: float
    cop_y: float
    cop_z: float
    cl: float
    cd: float

    @property
    def sprung_forces(self):
        return (-self.downforce, self.roll, self.pitch)


ZERO = Loads(*(0.,) * 12)


class VehicleAero:
    def __init__(self, settings, front, rear, track, cg_height):
        self.mode = settings["model"]
        self.enabled = settings["enabled"]
        self.cg_height = cg_height
        self.axle_midpoint = (front - rear) / 2.
        if self.mode == "modal":
            coefficients = settings["coefficients"]
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in coefficients.values()):
                raise ValueError("Modal aero coefficients must be finite numbers")
            if coefficients["rho"] <= 0. or coefficients["area"] <= 0. or not 0 <= coefficients["front_balance"] <= 1:
                raise ValueError("Aero density and area must be positive and front balance must be 0–1")
            self.modal = ModalAeroModel.from_coefficients(coefficients, wheelbase=front + rear, track=track)
        else:
            speed = float(settings["reference_speed_kmh"]) / 3.6
            forces = [float(settings[name]) for name in ("reference_downforce_n", "reference_drag_n")]
            if not np.isfinite([speed, *forces]).all() or speed <= 0. or min(forces) < 0.:
                raise ValueError("Quadratic aero requires positive reference speed and nonnegative forces")
            self.downforce_factor, self.drag_factor = (force / speed**2 for force in forces)

    def loads(self, u, v, heave, roll, pitch):
        speed = math.hypot(u, v)
        if not self.enabled or speed == 0.:
            return ZERO
        if self.mode == "quadratic":
            drag = self.drag_factor * speed**2
            return Loads(self.downforce_factor * speed**2, drag, -drag * u / speed, -drag * v / speed,
                         0., 0., 0., 0., 0., self.cg_height + heave, 0., 0.)
        state = self.modal.calculate_from_modal(speed, heave, pitch, roll)
        down = state["F_down"]
        drag = max(0., state["F_drag"])
        fx, fy = -drag * u / speed, -drag * v / speed
        x, y = state["COP_x"] + self.axle_midpoint, state["COP_y"]
        # Use the modeled lateral and longitudinal CoP at the current attitude.
        # Cop_z is the mount height from the reference road, adjusted for pitch/roll.
        z = self.modal.cop_z - self.cg_height + x * math.sin(pitch) + y * math.sin(roll)
        return Loads(down, drag, fx, fy, -y * down - z * fy, -x * down - z * fx, x * fy - y * fx,
                     x, y, self.cg_height + heave + z, state["CL"], state["CD"])
