"""Closed-loop lap timing on a sampled LINE/ARC centerline."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ._vendor.lapsim.track import read_sections
from ._vendor.suspension.engine import G

from .model import Controls, DOF, SLIP_REFERENCE_SPEED, flat_road
from .resources import DEFAULT_TRACK


DEFAULT_MAX_SPEED_MPS = 100. / 3.6
DEFAULT_LATERAL_LIMIT_MPS2 = 2.0 * G
DRIVER_PERIOD = 0.02
LOW_SPEED_STEP_S = 0.0025


class LapIncomplete(RuntimeError):
    """The simulated trajectory failed to complete a valid lap."""


class Track:
    def __init__(self, sections, width):
        self.half_width = width / 2.
        endpoints = zip(sections, sections[1:] + sections[:1])
        if any(np.hypot(a.x[-1] - b.x[0], a.y[-1] - b.y[0]) > .01 for a, b in endpoints):
            raise ValueError("Lap timing requires connected LINE/ARC sections and a closed centerline")
        self.xy = np.concatenate([np.column_stack((part.x[:-1], part.y[:-1])) for part in sections])
        self.delta = np.roll(self.xy, -1, axis=0) - self.xy
        self.ds = np.linalg.norm(self.delta, axis=1)
        if len(self.ds) < 5 or np.any(self.ds <= 1e-6):
            raise ValueError("Track requires at least five distinct centerline segments")
        self.s = np.r_[0., np.cumsum(self.ds)]
        self.length = self.s[-1]
        self.curvature = np.concatenate([np.full(len(part.x) - 1, part.curvature) for part in sections])
        self.heading = np.arctan2(self.delta[0, 1], self.delta[0, 0])
        start_tangent = self.delta[0] / self.ds[0]
        finish_tangent = self.delta[-1] / self.ds[-1] + start_tangent
        norm = np.linalg.norm(finish_tangent)
        if norm < .1:
            raise ValueError("Start/finish requires a consistent forward direction")
        self.finish_direction = finish_tangent / norm
        self._search_s = np.r_[self.s[:-1] - self.length, self.s[:-1],
                               self.s[:-1] + self.length, 2. * self.length]

    @classmethod
    def from_csv(cls, path, *, width=3., step=.35):
        if not np.isfinite([width, step]).all() or min(width, step) <= 0:
            raise ValueError("Track width and sampling distance must be finite and positive")
        return cls(read_sections(Path(path), width, step), width)

    def point(self, distance):
        distance %= self.length
        i = np.searchsorted(self.s, distance, side="right") - 1
        return self.xy[i] + (distance - self.s[i]) / self.ds[i] * self.delta[i]

    def project(self, point, previous):
        # Search along the same branch near the last position, even on a crossing track.
        lap = np.floor(previous / self.length)
        local = previous - lap * self.length
        window = min(3., self.length / 4.)
        lo = max(0, np.searchsorted(self._search_s, local - window) - 1)
        hi = np.searchsorted(self._search_s, local + window, side="right")
        candidates = np.arange(lo, hi) % len(self.ds)
        delta, ds = self.delta[candidates], self.ds[candidates]
        diff = point - self.xy[candidates]
        fractions = np.clip(np.einsum("ij,ij->i", diff, delta) / ds**2, 0., 1.)
        offsets = diff - fractions[:, None] * delta
        distances = np.linalg.norm(offsets, axis=1)
        i = int(np.argmin(distances))
        distance = lap * self.length + self._search_s[lo + i] + fractions[i] * ds[i]
        cross = delta[i, 0] * offsets[i, 1] - delta[i, 1] * offsets[i, 0]
        return distance, np.copysign(distances[i], cross)


class Driver:
    """Periodic speed envelope with braking preview and pure-pursuit steering."""

    def __init__(self, track, wheelbase, *, max_speed=DEFAULT_MAX_SPEED_MPS,
                 lateral_limit=DEFAULT_LATERAL_LIMIT_MPS2):
        if not np.isfinite([max_speed, lateral_limit]).all() or min(max_speed, lateral_limit) <= 0:
            raise ValueError("Driver limits must be finite and positive")
        self.track, self.wheelbase = track, wheelbase
        speed_sq = np.minimum(max_speed**2, lateral_limit / np.maximum(1e-6, np.abs(track.curvature)))
        n = len(speed_sq)
        # Two passes in each direction propagate constraints across the start/finish.
        for i in range(2 * n):
            a, b = i % n, (i + 1) % n
            speed_sq[b] = min(speed_sq[b], speed_sq[a] + 2. * 3. * track.ds[a])
        for i in reversed(range(2 * n)):
            a, b = i % n, (i + 1) % n
            speed_sq[a] = min(speed_sq[a], speed_sq[b] + 2. * 4.5 * track.ds[a])
        self.speed_profile = np.sqrt(np.r_[speed_sq, speed_sq[0]])

    def command(self, state, distance):
        u, v = state[DOF:DOF + 2]
        speed = np.hypot(u, v)
        lookahead = np.clip(1.35 + .14 * speed, 1.5, 4.)
        goal = self.track.point(distance + lookahead) - state[:2]
        yaw = state[2]
        goal_lateral = -np.sin(yaw) * goal[0] + np.cos(yaw) * goal[1]
        steer = np.arctan2(2. * self.wheelbase * goal_lateral, max(.25, goal @ goal))
        # Preview braking slightly ahead of the longitudinal feedback point.
        speed_here = np.interp(distance % self.track.length, self.track.s, self.speed_profile)
        speed_ahead = np.interp((distance + .12 * speed) % self.track.length, self.track.s, self.speed_profile)
        target = min(speed_here, speed_ahead)
        demand = 2.8 * (target - speed)
        return Controls(float(np.clip(steer, -.60, .60)), float(np.clip(demand / 6., 0., 1.)),
                        float(np.clip(-demand / 9., 0., 1.))), target


@dataclass
class LapResult:
    lap_times: list[float]
    telemetry: list[dict]
    max_centerline_error_m: float
    total_time_s: float
    timed_boundaries_s: tuple[float, ...] = ()


def simulate_laps(vehicle, track, driver, *, laps=1, warmup_laps=0, initial_speed=0.,
                  step=.005, time_limit=240., road=flat_road, progress=None, on_position=None):
    """Integrate laps and optionally report (lap number, time, total distance) at each driver step."""
    if type(laps) is not int or type(warmup_laps) is not int or laps < 1 or warmup_laps < 0:
        raise ValueError("At least one timed lap and a nonnegative number of warmup laps are required")
    if not np.isfinite([step, time_limit]).all() or not .0001 <= step <= DRIVER_PERIOD or time_limit <= 0.:
        raise ValueError("Integration step must be 0.0001–0.02 s and the time limit must be positive")
    state = vehicle.initial_state(position=track.xy[0], yaw=track.heading, speed=initial_speed, road=road)
    time, distance, error, max_error = 0., 0., 0., 0.
    records, crossings = [], [0.]
    substeps = int(np.ceil(DRIVER_PERIOD / step))
    total_laps = laps + warmup_laps

    def sample(controls, target):
        records.append({
            "time_s": time, "distance_m": distance, "centerline_error_m": error,
            "target_speed_mps": target, "throttle": controls.throttle, "brake": controls.brake,
            "steer_command_deg": np.degrees(controls.steer_rad),
            **vehicle.evaluate(state, controls, road(time), telemetry=True),
        })

    while time < time_limit:
        if on_position is not None:
            on_position(len(crossings), time, distance)
        controls, target = driver.command(state, distance)
        sample(controls, target)

        previous_time, previous_distance, previous_state = time, distance, state
        time = min(time + DRIVER_PERIOD, time_limit)
        state = advance_state(vehicle, state, controls, road, previous_time, time, substeps)
        if not np.isfinite(state).all():
            raise LapIncomplete(f"Non-finite vehicle state at {time:.3f} s. Reduce the integration step.")
        distance, error = track.project(state[:2], previous_distance)
        max_error = max(max_error, abs(error))
        if abs(error) > track.half_width:
            raise LapIncomplete(f"Vehicle left the track at {time:.2f} s, s = {distance:.1f} m, "
                                f"centerline error = {error:+.2f} m")
        finish = len(crossings) * track.length
        before = (previous_state[:2] - track.xy[0]) @ track.finish_direction
        after = (state[:2] - track.xy[0]) @ track.finish_direction
        if abs(distance - finish) <= track.half_width + 1. and before < 0. <= after:
            fraction = -before / (after - before)
            crossing = previous_time + fraction * (time - previous_time)
            crossings.append(crossing)
            if progress is not None:
                progress(len(crossings) - 1, crossing)
            if len(crossings) - 1 == total_laps:
                time, distance = crossing, finish
                state = previous_state + fraction * (state - previous_state)
                _, error = track.project(state[:2], distance)
                sample(controls, target)
                timed_boundaries = tuple(crossings[warmup_laps:])
                return LapResult(np.diff(timed_boundaries).tolist(), records, max_error, time, timed_boundaries)
    raise LapIncomplete(f"No finish before {time_limit:g} s of simulation, s = {distance:.1f} m")


def advance_state(vehicle, state, controls, road, start, end, substeps):
    def rates(current, time):
        return vehicle.evaluate(current, controls, road(time))

    # The regularized slip model can give wheel spin a shorter time scale at launch.
    if np.hypot(*state[DOF:DOF + 2]) < 2. * SLIP_REFERENCE_SPEED:
        substeps = max(substeps, int(np.ceil(DRIVER_PERIOD / LOW_SPEED_STEP_S)))
    dt = (end - start) / substeps
    for i in range(substeps):
        time = start + i * dt
        midpoint = time + dt / 2
        k1 = rates(state, time)
        k2 = rates(state + dt / 2 * k1, midpoint)
        k3 = rates(state + dt / 2 * k2, midpoint)
        k4 = rates(state + dt * k3, time + dt)
        state = state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    return state
