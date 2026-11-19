"""Integrate four fixed-chassis wheel masses along the spatial suspension constraints."""

from dataclasses import dataclass
from math import ceil, cos, isfinite, pi, sin, sqrt

import numpy as np

from .animation import AnimationFrame, FRAMES_PER_SECOND
from .deformable import DynamicsModel, sample_dampers, sample_element_springs, suspension_terms
from .solver import CornerSolver, Unreachable, VehicleResult, WHEEL, WheelTravel


MAX_INTEGRATION_STEP = 1 / 240
MAX_STEPS = 12_000


def road_profile(time, amplitude, width):
    """Height and vertical road velocity of one smooth pulse in SI units."""
    if not 0. < time < width:
        return 0., 0.
    phase = pi * time / width
    sine = sin(phase)
    return amplitude * sine**4, 4 * amplitude * (pi / width) * sine**3 * cos(phase)


@dataclass(frozen=True)
class _WheelEvaluation:
    path: WheelTravel
    acceleration: float
    step_limit: float


@dataclass(frozen=True)
class WheelLoads:
    path: WheelTravel
    effective_mass: float
    generalized_force: float
    tire_force: float
    suspension_stiffness: float
    suspension_damping: float


class WheelEquation:
    def __init__(self, geometry, model):
        self.solver = CornerSolver(geometry)
        self.model = model
        self.spec = model.corners[geometry.name]
        self.path = None

    def loads(self, state, road, body_position=0., body_velocity=0.):
        travel, velocity = state
        path = self.solver.wheel_travel(travel, self.path)
        self.path = path
        spec = self.spec
        tire_force = self.model.tire_force(path.position[WHEEL, 2] + body_position, velocity + body_velocity, *road)
        if self.solver.locked:
            # An infinite dummy generalized mass gives zero relative
            # acceleration while leaving this wheel's physical mass intact.
            return WheelLoads(path, float("inf"), 0., tire_force, 0., 0.)
        suspension_force, stiffness, damping = suspension_terms(self.model, self.solver.geometry.name, path, velocity)

        # Lagrange equation with q = hub Z travel and T = m |x_q|² q̇² / 2.
        tangent, curvature = path.tangent[WHEEL], path.curvature[WHEEL]
        mass = spec.mass * (tangent @ tangent)
        inertial_term = spec.mass * (tangent @ curvature) * velocity**2
        force = tire_force - spec.mass * self.model.gravity + suspension_force - inertial_term
        return WheelLoads(path, mass, force, tire_force, stiffness, damping)

    def evaluate(self, state, road):
        loads = self.loads(state, road)
        acceleration = loads.generalized_force / loads.effective_mass
        if not isfinite(acceleration):
            raise Unreachable(f"{self.solver.geometry.name}：輪心加速度超出數值範圍。")

        # Resolve the local stiffness/damping scales and limit the change in
        # wheel travel per step so Newton continuation stays on the same branch.
        stiffness = self.model.tire_stiffness + loads.suspension_stiffness
        damping = self.model.tire_damping + loads.suspension_damping
        rate = max(sqrt(stiffness / loads.effective_mass), damping / loads.effective_mass)
        travel_rate = abs(state[1]) + sqrt(2 * .002 * abs(acceleration))
        step_limit = min(MAX_INTEGRATION_STEP, .35 / max(rate, 1e-12), .002 / max(travel_rate, 1e-12))
        return _WheelEvaluation(loads.path, float(acceleration), step_limit)


def _evaluations(equations, state, time, amplitude, width):
    road = road_profile(time, amplitude, width)
    return tuple(equation.evaluate(row, road) for equation, row in zip(equations, state))


def _state_rates(state, evaluations):
    return np.column_stack((state[:, 1], [evaluation.acceleration for evaluation in evaluations]))


def _advance(equations, state, time, step, amplitude, width, initial):
    def rates(stage, instant):
        return _state_rates(stage, _evaluations(equations, stage, instant, amplitude, width))

    return rk4_step(rates, state, time, step, _state_rates(state, initial))


def rk4_step(rates, state, time, step, k1):
    """Classic RK4. All intermediate stages use the actual constrained geometry."""
    midpoint = time + step / 2
    k2 = rates(state + (step / 2) * k1, midpoint)
    k3 = rates(state + (step / 2) * k2, midpoint)
    k4 = rates(state + step * k3, time + step)
    return state + (step / 6) * (k1 + 2*k2 + 2*k3 + k4)


def sample_times(duration, *checkpoints):
    """Animation samples with exact times for changes in inputs."""
    steps = ceil(duration * FRAMES_PER_SECOND)
    times = list(np.linspace(0., duration, steps + 1))
    for sample in checkpoints:
        if not any(abs(time - sample) < 1e-10 for time in times):
            times.append(sample)
    return sorted(times)


def simulate_dynamics(model: DynamicsModel, settings) -> tuple[AnimationFrame, ...]:
    """Start at static equilibrium and record the response to one road pulse."""
    simulation = settings["simulation"]
    target = simulation["target"]
    amplitude = float(simulation["bump_mm"]) / 1000
    duration = float(simulation["duration_seconds"])
    width = float(simulation["bump_seconds"])
    if target not in model.corners and target != "all":
        raise ValueError("路面擾動輪位必須是 FL、FR、RL、RR 或四輪。")
    if not (isfinite(amplitude) and abs(amplitude) <= .1 and .5 <= duration <= 12 and .02 <= width <= duration):
        raise ValueError("路面高度須在 ±100 mm；模擬時間為 0.5～12 s，擾動時間為 0.02 s 至模擬時間。")

    corner_names = tuple(item.name for item in model.geometry.corners)
    by_name = {item.name: WheelEquation(item, model) for item in model.geometry.corners}
    active_names = tuple(name for name in corner_names if amplitude and (target == "all" or name == target))
    equations = tuple(by_name[name] for name in active_names)
    static = {}
    for name, equation in by_name.items():
        static[name] = equation.solver.wheel_result(equation.solver.wheel_travel(0.), 0., 0.)
    state = np.zeros((len(equations), 2))
    current = 0.
    steps = 0
    pivot = np.asarray(model.geometry.cg, dtype=float)
    rotation = np.eye(3)
    translation = np.zeros(3)
    for array in (pivot, rotation, translation):
        array.setflags(write=False)
    frames = []

    try:
        evaluations = _evaluations(equations, state, current, amplitude, width)
        if evaluations and duration / min(entry.step_limit for entry in evaluations) > MAX_STEPS:
            raise Unreachable("相對於質量的剛性或阻尼需要過小的積分步長，請降低模擬時間、剛性或阻尼。")
        checkpoints = (width / 2, width) if active_names else ()
        for output in sample_times(duration, *checkpoints):
            while equations and output - current > 1e-12:
                resolution = width / 32 if current < width else MAX_INTEGRATION_STEP
                step_limit = min(resolution, *(entry.step_limit for entry in evaluations))
                if steps >= MAX_STEPS or step_limit < 1e-8:
                    raise Unreachable("超過積分步數限制，請降低模擬時間、剛性、阻尼或路面擾動幅度。")
                step = min(output - current, step_limit)
                state = _advance(equations, state, current, step, amplitude, width, evaluations)
                current += step
                steps += 1
                evaluations = _evaluations(equations, state, current, amplitude, width)

            current = output
            corners = static.copy()
            for name, equation, entry, speed in zip(active_names, equations, evaluations, state[:, 1]):
                corners[name] = equation.solver.wheel_result(entry.path, speed, entry.acceleration)
            result = VehicleResult(corners)
            road = dict.fromkeys(active_names, road_profile(output, amplitude, width))
            dampers = sample_dampers(model, result, road)
            frames.append(AnimationFrame(output, result, rotation, pivot, translation, dampers=dampers,
                                         element_springs=sample_element_springs(model, result)))
    except Unreachable as error:
        raise Unreachable(f"動力學於 {current:.3f} s 無法繼續。{error}") from error
    return tuple(frames)
