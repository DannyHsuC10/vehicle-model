"""Prescribed-motion animation with four-corner constraint solves at each frame."""

from dataclasses import dataclass
from math import ceil, cos, pi, sin
from typing import TYPE_CHECKING

import numpy as np

from .animation_settings import PRESETS, validate_animation
from .solver import CornerSolver, Unreachable, VehicleResult, VERTICAL, WHEEL, ZERO_VECTOR

if TYPE_CHECKING:
    from .deformable import DamperResult, ElasticResult
    from .heave import HeaveSample
    from .mount_loads import ChassisMountLoads
    from .rigid_body import RigidBodySample


FRAMES_PER_SECOND = 24


@dataclass(frozen=True)
class AnimationFrame:
    """A time sample, chassis-frame mechanism state and its world transform."""

    time: float
    result: VehicleResult
    rotation: np.ndarray
    pivot: np.ndarray
    translation: np.ndarray
    dampers: "dict[str, DamperResult] | None" = None
    heave: "HeaveSample | None" = None
    mount_loads: "dict[str, ChassisMountLoads] | None" = None
    body: "RigidBodySample | None" = None
    element_springs: "dict[str, dict[str, ElasticResult]] | None" = None


def _driver_sample(phase, amplitude, duration, bipolar):
    """Return a smooth pulse and its time derivatives in the amplitude's units."""
    pulses = 2 if bipolar else 1
    elapsed = phase * pulses
    if elapsed in (0, 1, 2):
        return ZERO_VECTOR
    sign = 1 if elapsed < 1 else -1
    position = elapsed if elapsed < 1 else elapsed - 1
    s, c = sin(pi * position), cos(pi * position)
    frequency = pi * pulses / duration
    scale = sign * amplitude
    return (scale * s**4, 4 * scale * frequency * s**3 * c,
            4 * scale * frequency**2 * (3 * s**2 * c**2 - s**4))


def _body_pose(preset, driver):
    """Return the pose and derivatives of world vertical expressed in the chassis frame."""
    if preset == "roll":
        angle, velocity, acceleration = driver
        s, c = sin(angle), cos(angle)
        rotation = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
        normal = rotation[2]
        derivative = np.array([0., c, -s])
        return (rotation, np.zeros(3), normal, derivative * velocity,
                derivative * acceleration - normal * velocity**2)
    translation = np.array([0., 0., driver[0] if preset == "heave" else 0.])
    return np.eye(3), translation, np.asarray(VERTICAL), np.zeros(3), np.zeros(3)


def _solve_corners(solvers, preset, driver, selected, pivot, normal, normal_rate, normal_acceleration):
    results = {}
    for solver in solvers:
        geometry = solver.geometry
        jounce = rack = ZERO_VECTOR
        if preset == "single_wheel" and geometry.name == selected:
            jounce = driver
        if preset == "front_steer" and geometry.axle == "front":
            rack = driver
        if preset == "heave":
            # Chassis rises while each hub stays at its original world height.
            jounce = tuple(-value for value in driver)
        if preset == "roll":
            # n·(WC - pivot) = world reference height - pivot_z.
            # Subtract the chassis reference hub to match the solver's displacement convention.
            offset = solver.reference[WHEEL] - pivot
            jounce = ((VERTICAL - normal) @ offset, -normal_rate @ offset, -normal_acceleration @ offset)
        results[geometry.name] = solver.solve(jounce, rack, normal=normal, normal_rate=normal_rate,
                                              normal_acceleration=normal_acceleration)
    return VehicleResult(results)


def build_animation(model, preset, amplitude, duration, corner="FL") -> tuple[AnimationFrame, ...]:
    """Solve one complete cycle. Amplitudes are mm, except roll in degrees."""
    settings = validate_animation({"preset": preset, "amplitude": amplitude, "duration": duration, "corner": corner})
    duration = settings["duration"]
    amplitude = settings["amplitude"] * (pi / 180 if preset == "roll" else 0.001)
    # Include both endpoints and land exactly on the pulse peaks.
    steps = 4 * ceil(duration * FRAMES_PER_SECOND / 4)
    pivot = np.array(model.cg, dtype=float)
    pivot.setflags(write=False)
    solvers = tuple(CornerSolver(item) for item in model.corners)
    frames = []
    for index in range(steps + 1):
        phase = index / steps
        time = phase * duration
        driver = _driver_sample(phase, amplitude, duration, preset != "single_wheel")
        rotation, translation, normal, normal_rate, normal_acceleration = _body_pose(preset, driver)
        try:
            result = _solve_corners(solvers, preset, driver, corner, pivot, normal, normal_rate, normal_acceleration)
        except Unreachable as error:
            raise Unreachable(f"{PRESETS[preset]}於 {time:.2f} 秒無法求解。{error}") from error
        rotation.setflags(write=False)
        translation.setflags(write=False)
        frames.append(AnimationFrame(time, result, rotation, pivot, translation))
    return tuple(frames)
