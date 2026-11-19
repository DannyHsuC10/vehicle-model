"""Coupled vertical motion of the sprung mass and four constrained wheel masses."""

from dataclasses import dataclass
from math import isfinite, sqrt

import numpy as np

from .animation import AnimationFrame
from .deformable import DynamicsModel, sample_dampers, sample_element_springs
from .dynamics import (
    MAX_INTEGRATION_STEP, MAX_STEPS, WheelEquation, WheelLoads, rk4_step, road_profile, sample_times,
)
from .dynamics_settings import SIMULATION_LIMITS, bounded, require_fields, validate_heave
from .mount_loads import compute_mount_loads
from .solver import Unreachable, VehicleResult


@dataclass(frozen=True)
class HeaveSample:
    """Sprung body motion and applied force, positive upward in ground coordinates."""

    displacement: float
    velocity: float
    acceleration: float
    force_z: float


@dataclass(frozen=True)
class _HeaveEvaluation:
    loads: tuple[WheelLoads, ...]
    acceleration: np.ndarray
    force_z: float
    step_limit: float


def _force_at(time, force_down, ramp):
    phase = min(max(time / ramp, 0.), 1.)
    return -force_down * phase**3 * (10. - phase * (15. - 6. * phase))


class _HeaveEquation:
    def __init__(self, model, total_mass, force_at, road_at):
        self.model = model
        self.total_mass = total_mass
        self.force_at = force_at
        self.road_at = road_at
        self.wheels = tuple(WheelEquation(corner, model) for corner in model.geometry.corners)
        self.names = tuple(corner.name for corner in model.geometry.corners)
        self.wheel_masses = np.array([wheel.spec.mass for wheel in self.wheels])
        self.body_mass = total_mass - self.wheel_masses.sum()
        if not isfinite(self.body_mass) or self.body_mass <= 0.:
            raise ValueError("車輛總質量須大於四輪輪端質量總和，才能求解簧上車身。")

    def evaluate(self, state, time):
        # Coordinates are chassis Z followed by each hub's Z travel relative
        # to the chassis. Tire forces use motion relative to the local road.
        body_position, body_velocity = state[0]
        road = self.road_at(time)
        loads = tuple(wheel.loads(row, road.get(name, (0., 0.)), body_position, body_velocity)
                      for name, wheel, row in zip(self.names, self.wheels, state[1:]))
        force_z = self.force_at(time)
        body_load = force_z + sum(load.tire_force for load in loads) - self.total_mass * self.model.gravity
        wheel_loads = np.array([load.generalized_force for load in loads])
        effective_mass = np.array([load.effective_mass for load in loads])

        # The hub's Z derivative with respect to its relative travel is one.
        # The mass matrix therefore has wheel masses along the body/wheel
        # cross terms. Eliminate the four wheel equations to solve body Z.
        ratio = self.wheel_masses / effective_mass
        schur_mass = self.body_mass + self.wheel_masses @ (1. - ratio)
        body_acceleration = (body_load - ratio @ wheel_loads) / schur_mass
        wheel_acceleration = (wheel_loads - self.wheel_masses * body_acceleration) / effective_mass
        acceleration = np.r_[body_acceleration, wheel_acceleration]
        if not np.isfinite(acceleration).all():
            raise Unreachable("車身或輪心加速度超出數值範圍。")

        # In absolute vertical coordinates the lateral hub motion adds a
        # positive semidefinite mass term. Bound the stiffness and damping
        # rates using row sums and the uncoupled vertical masses.
        k = 2. * np.array([load.suspension_stiffness for load in loads])
        c = 2. * np.array([load.suspension_damping for load in loads])
        stiffness_rate = max(k.sum() / self.body_mass,
                             np.max((k + self.model.tire_stiffness) / self.wheel_masses))
        damping_rate = max(c.sum() / self.body_mass,
                           np.max((c + self.model.tire_damping) / self.wheel_masses))
        travel_rate = np.max(np.abs(state[1:, 1]) + np.sqrt(2 * .002 * np.abs(wheel_acceleration)))
        step_limit = min(MAX_INTEGRATION_STEP, .3 / sqrt(stiffness_rate),
                         .5 / max(damping_rate, 1e-12), .002 / max(travel_rate, 1e-12))
        return _HeaveEvaluation(loads, acceleration, force_z, step_limit)

    def rates(self, state, time):
        return _state_rates(state, self.evaluate(state, time))


def _state_rates(state, evaluation):
    return np.column_stack((state[:, 1], evaluation.acceleration))


def simulate_heave(model: DynamicsModel, settings: dict, total_mass: float) -> tuple[AnimationFrame, ...]:
    """Start at static ride height and apply a smooth ramp to a held CG force."""
    settings = validate_heave(settings)
    force_down = settings["force_down_n"]
    ramp, duration = settings["ramp_seconds"], settings["duration_seconds"]
    equation = _HeaveEquation(model, total_mass, lambda time: _force_at(time, force_down, ramp), lambda _: {})
    return _integrate(equation, duration, ramp, (ramp,))


def simulate_road_heave(model: DynamicsModel, settings: dict, total_mass: float) -> tuple[AnimationFrame, ...]:
    """Start at static ride height and apply a road pulse beneath the selected wheels."""
    simulation = settings.get("simulation") if isinstance(settings, dict) else None
    require_fields(simulation, ("target", *SIMULATION_LIMITS), "模擬設定")
    target = simulation["target"]
    if target not in (*model.corners, "all"):
        raise ValueError("路面擾動輪位必須是 FL、FR、RL、RR 或四輪。")
    inputs = {key: bounded(simulation[key], *limits) for key, limits in SIMULATION_LIMITS.items()}
    width, duration = inputs["bump_seconds"], inputs["duration_seconds"]
    if width > duration:
        raise ValueError("擾動持續時間不可超過模擬時間。")
    amplitude = inputs["bump_mm"] / 1000
    names = tuple(model.corners) if target == "all" else (target,)

    def road_at(time):
        return dict.fromkeys(names, road_profile(time, amplitude, width))

    equation = _HeaveEquation(model, total_mass, lambda _: 0., road_at)
    return _integrate(equation, duration, width, (width / 2, width))


def _integrate(equation, duration, input_window, checkpoints):
    model = equation.model
    state = np.zeros((len(equation.wheels) + 1, 2))
    rotation = np.eye(3)
    pivot = np.asarray(model.geometry.cg, dtype=float)
    for array in (rotation, pivot):
        array.setflags(write=False)
    frames = []
    current = 0.
    steps = 0
    try:
        evaluation = equation.evaluate(state, current)
        for output in sample_times(duration, *checkpoints):
            while output - current > 1e-12:
                input_step = input_window / 32 if current < input_window else MAX_INTEGRATION_STEP
                step_limit = min(input_step, evaluation.step_limit)
                if steps >= MAX_STEPS or step_limit < 1e-8:
                    raise Unreachable("超過積分步數限制，請縮短模擬時間或調整剛性與阻尼。")
                step = min(output - current, step_limit)
                state = rk4_step(equation.rates, state, current, step, _state_rates(state, evaluation))
                current += step
                steps += 1
                evaluation = equation.evaluate(state, current)
            current = output
            corners = {
                wheel.solver.geometry.name: wheel.solver.wheel_result(load.path, speed, acceleration)
                for wheel, load, speed, acceleration in zip(
                    equation.wheels, evaluation.loads, state[1:, 1], evaluation.acceleration[1:]
                )
            }
            result = VehicleResult(corners)
            body_position, body_velocity = map(float, state[0])
            dampers = sample_dampers(model, result, equation.road_at(output),
                                     body_position=body_position, body_velocity=body_velocity)
            body_acceleration = float(evaluation.acceleration[0])
            heave = HeaveSample(body_position, body_velocity, body_acceleration, evaluation.force_z)
            mount_loads = compute_mount_loads(model, result, dampers, body_acceleration)
            translation = np.array([0., 0., body_position])
            translation.setflags(write=False)
            frames.append(AnimationFrame(output, result, rotation, pivot, translation,
                                         dampers=dampers, heave=heave, mount_loads=mount_loads,
                                         element_springs=sample_element_springs(model, result)))
    except Unreachable as error:
        raise Unreachable(f"車身升沉於 {current:.3f} s 無法繼續。{error}") from error
    return tuple(frames)
