"""Spatial distance constraints and their analytic time derivatives."""

from dataclasses import dataclass, field
from math import ceil

import numpy as np

from .elements import DAMPER_POINT
from .geometry import FIXED, LINKS, MOVING, POINTS, VehicleGeometry
from .project import COMPONENTS
from .rod_kinematics import HingeMotion, RodMotion, axis_motion, compute_rod_motion


POSITION_TOLERANCE = 1e-10  # m, for the length-normalized equations
CONTINUATION_STEP = 0.002  # m
CONDITION_LIMIT = 1e8
RACK = POINTS.index("RACK")
WHEEL = POINTS.index("WC")
N_FIXED = len(FIXED)
LINK_START, LINK_END = np.array([(POINTS.index(a), POINTS.index(b)) for a, b in LINKS]).T
LINK_ROWS = np.arange(len(LINKS))
VERTICAL = (0., 0., 1.)
ZERO_VECTOR = (0., 0., 0.)
N_MOVING_COORDINATES = 3 * len(MOVING)


class Unreachable(ValueError):
    """The requested assembly branch cannot be solved reliably."""


@dataclass(frozen=True)
class CornerResult:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    wheel_axis: np.ndarray
    toe_deg: float
    camber_deg: float
    angular_velocity: np.ndarray
    angular_acceleration: np.ndarray
    length_error: np.ndarray
    jacobian: np.ndarray
    condition: float
    rod_motion: dict[tuple[str, str], RodMotion] = field(default_factory=dict)
    arm_motion: dict[str, HingeMotion] = field(default_factory=dict)
    rigid_links: tuple[tuple[str, str], ...] = LINKS
    rigid_lengths: np.ndarray | None = None


@dataclass(frozen=True)
class VehicleResult:
    corners: dict[str, CornerResult]


@dataclass(frozen=True)
class WheelTravel:
    """Position and derivatives with respect to hub travel, at zero rack displacement."""
    travel: float
    position: np.ndarray
    tangent: np.ndarray
    curvature: np.ndarray
    jacobian: np.ndarray
    condition: float


class CornerSolver:
    def __init__(self, geometry):
        self.geometry = geometry
        self.reference = np.array(geometry.reference)
        self.lengths = np.array(geometry.lengths)
        self.body_mount = None
        self.locked = False
        self.rigid_rank = N_MOVING_COORDINATES - 1
        if geometry.members is None:
            self.rigid_links = LINKS
            self._link_start, self._link_end = LINK_START, LINK_END
            self._link_rows = LINK_ROWS
            self.rigid_lengths = self.lengths
        else:
            self._configure_members()
        self.rigid_lengths.setflags(write=False)

    def _configure_members(self):
        geometry = self.geometry
        self.rigid_links = tuple((member.a, member.b) for member in geometry.members if member.kind == "rigid")
        names = POINTS
        if geometry.body_mount is not None:
            self.body_mount = np.asarray(geometry.body_mount, dtype=float)
            names += (DAMPER_POINT,)
        self._point_index = dict(zip(names, range(len(names))))
        try:
            indices = np.array([(self._point_index[a], self._point_index[b]) for a, b in self.rigid_links],
                               dtype=int).reshape(-1, 2)
        except KeyError as error:
            raise Unreachable(f"{geometry.name} · 元件接點 {error.args[0]} 無法定位。") from error
        self._link_start, self._link_end = indices.T
        self._link_rows = np.arange(len(indices))
        points = self._with_mount(self.reference)
        self.rigid_lengths = np.linalg.norm(points[self._link_start] - points[self._link_end], axis=1)
        if not np.isfinite(self.rigid_lengths).all() or np.any(self.rigid_lengths < 1e-6):
            raise Unreachable(f"{geometry.name} · 有固定桿長元件的接點重合或桿長無效。")
        _, _, jacobian, _ = self.equations(self.reference[N_FIXED:].ravel(), 0., 0.)
        singular = np.linalg.svd(jacobian[:-1], compute_uv=False)
        self.rigid_rank = int(np.count_nonzero(singular > singular[0] / CONDITION_LIMIT)) if singular.size else 0
        free = N_MOVING_COORDINATES - self.rigid_rank
        if free > 1:
            raise Unreachable(f"{geometry.name} · 目前機構有 {free} 個自由度；輪端求解需要一個輪心垂直自由度或全鎖定。"
                              "請調整固定桿長元件。")
        self.locked = free == 0
        try:
            self._condition(jacobian[:-1] if self.locked else jacobian)
        except Unreachable as error:
            raise Unreachable(f"{geometry.name} · 目前的輪心垂直行程無法作為獨立輸入；請調整固定桿長元件。") from error

    def _with_mount(self, points, *, derivative=False):
        if self.body_mount is None:
            return points
        mount = ZERO_VECTOR if derivative else self.body_mount
        return np.vstack((points, mount))

    def _rigid_relative_rates(self, rates):
        all_rates = self._with_mount(rates, derivative=True)
        return all_rates[self._link_start] - all_rates[self._link_end]

    def equations(self, q, jounce, rack, normal=VERTICAL):
        points = self.reference.copy()
        points[RACK, 1] += rack
        points[N_FIXED:] = q.reshape(len(MOVING), 3)
        all_points = self._with_mount(points)
        delta = all_points[self._link_start] - all_points[self._link_end]
        residual = np.empty(len(self.rigid_links) + 1)
        residual[:-1] = (np.sum(delta**2, axis=1) - self.rigid_lengths**2) / (2 * self.rigid_lengths)
        residual[-1] = normal @ (points[WHEEL] - self.reference[WHEEL]) - jounce
        rows = np.zeros((len(residual), len(all_points), 3))
        rows[self._link_rows, self._link_start] = delta / self.rigid_lengths[:, None]
        rows[self._link_rows, self._link_end] = -rows[self._link_rows, self._link_start]
        rows[-1, WHEEL] = normal
        jacobian = rows[:, N_FIXED:len(POINTS)].reshape(len(residual), -1)
        return points, residual, jacobian, rows[:, RACK, 1]

    @staticmethod
    def _linear_solve(jacobian, rhs, *, exact=True):
        if jacobian.shape[0] == jacobian.shape[1]:
            return np.linalg.solve(jacobian, rhs)
        solution = np.linalg.lstsq(jacobian, rhs, rcond=None)[0]
        if exact and np.linalg.norm(jacobian @ solution - rhs, np.inf) > 1e-8 * max(1., np.linalg.norm(rhs, np.inf)):
            raise Unreachable("指定運動與固定桿長約束不相容。")
        return solution

    def _position_step(self, q, jounce, rack, normal):
        for _ in range(30):
            _, residual, jacobian, _ = self.equations(q, jounce, rack, normal)
            error = np.linalg.norm(residual, np.inf)
            if error < POSITION_TOLERANCE:
                self._condition(jacobian)
                return q
            correction = self._linear_solve(jacobian, -residual, exact=False)
            for exponent in range(10):
                trial = q + correction * (0.5**exponent)
                _, trial_residual, _, _ = self.equations(trial, jounce, rack, normal)
                if np.linalg.norm(trial_residual, np.inf) < error:
                    q = trial
                    break
            else:
                break
        raise Unreachable("位置迭代未收斂。")

    @staticmethod
    def _condition(jacobian):
        condition = np.linalg.cond(jacobian)
        if not np.isfinite(condition) or condition > CONDITION_LIMIT:
            raise Unreachable("Jacobian 接近奇異，請減少輸入或檢查硬點。")
        return float(condition)

    def wheel_travel(self, travel, previous=None):
        """Trace the same rigid constraints and differentiate along the free wheel coordinate."""
        if not np.isfinite(travel) or abs(travel) > .5:
            raise Unreachable(f"{self.geometry.name}：輪心行程超出 ±500 mm 的求解範圍。")
        if self.locked:
            if abs(travel) > POSITION_TOLERANCE:
                raise Unreachable(f"{self.geometry.name} · 固定桿長元件已鎖定輪端，輪心相對車身的行程須為 0。")
            return previous if previous is not None else self._locked_travel()
        if previous is not None and previous.travel == travel:
            return previous
        start = previous.travel if previous is not None else 0.
        reference = previous.position if previous is not None else self.reference
        q = reference[N_FIXED:].ravel()
        steps = max(1, ceil(abs(travel - start) / CONTINUATION_STEP))
        try:
            for step in range(1, steps + 1):
                q = self._position_step(q, start + (travel - start) * step / steps, 0., VERTICAL)
            points, _, jacobian, _ = self.equations(q, travel, 0.)
            condition = self._condition(jacobian)
            tangent = np.zeros_like(points)
            rhs = np.zeros(len(jacobian))
            rhs[-1] = 1.
            tangent[N_FIXED:] = self._linear_solve(jacobian, rhs).reshape(-1, 3)

            relative = self._rigid_relative_rates(tangent)
            rhs[:-1] = -np.einsum("ij,ij->i", relative, relative) / self.rigid_lengths
            rhs[-1] = 0.
            curvature = np.zeros_like(points)
            curvature[N_FIXED:] = self._linear_solve(jacobian, rhs).reshape(-1, 3)
        except (Unreachable, np.linalg.LinAlgError) as error:
            raise Unreachable(f"{self.geometry.name} · 無法求解輪心行程 {travel * 1000:.2f} mm。{error}") from error
        for array in (points, tangent, curvature, jacobian):
            array.setflags(write=False)
        return WheelTravel(travel, points, tangent, curvature, jacobian, condition)

    def _locked_travel(self):
        points, _, jacobian, _ = self.equations(self.reference[N_FIXED:].ravel(), 0., 0.)
        condition = self._condition(jacobian[:-1])
        zeros = np.zeros_like(points)
        for array in (points, zeros, jacobian):
            array.setflags(write=False)
        return WheelTravel(0., points, zeros, zeros, jacobian, condition)

    def wheel_result(self, sample, travel_rate, travel_acceleration):
        """Reconstruct the full node and rod state using the solved path derivatives."""
        velocity = sample.tangent * travel_rate
        acceleration = sample.tangent * travel_acceleration + sample.curvature * travel_rate**2
        return self._make_result(sample.position, velocity, acceleration, sample.jacobian, sample.condition)

    def solve(self, jounce, rack, *, normal=VERTICAL, normal_rate=ZERO_VECTOR, normal_acceleration=ZERO_VECTOR):
        """Constrain the wheel-center displacement projected along a moving normal.

        jounce and rack each contain position, velocity and acceleration.
        The default projection gives the original chassis-frame Z constraint.
        """
        if np.shape(jounce) != (3,) or np.shape(rack) != (3,):
            raise ValueError("每個輸入需要依序指定位置、速度、加速度。")
        if not np.isfinite(jounce).all() or not np.isfinite(rack).all():
            raise ValueError("運動輸入必須是有限數值。")
        normal = np.asarray(normal, dtype=float)
        normal_rate = np.asarray(normal_rate, dtype=float)
        normal_acceleration = np.asarray(normal_acceleration, dtype=float)
        if any(v.shape != (3,) or not np.isfinite(v).all() for v in (normal, normal_rate, normal_acceleration)):
            raise ValueError("輪心投影方向及其導數必須各有三個有限分量。")
        if not np.isclose(np.linalg.norm(normal), 1.) or normal[2] <= 0:
            raise ValueError("輪心投影方向須為向上的單位向量。")
        if self.locked:
            if np.max(np.abs(jounce)) > POSITION_TOLERANCE or np.max(np.abs(rack)) > POSITION_TOLERANCE:
                raise Unreachable(f"{self.geometry.name} · 固定桿長元件已鎖定輪端，請使用零行程及零齒條輸入。")
            return self.wheel_result(self._locked_travel(), 0., 0.)
        travel, travel_rate, travel_acc = jounce
        rack_position, rack_rate, rack_acc = rack
        span = max(abs(travel), abs(rack_position))
        if span > 0.5:
            raise Unreachable("行程或齒條位移超出 ±500 mm 的求解範圍。")
        normal_delta = normal - VERTICAL
        # Take small changes in both the prescribed distances and the projection direction.
        steps = max(1, ceil(span / CONTINUATION_STEP), ceil(np.linalg.norm(normal_delta) / 0.025))
        q = self.reference[N_FIXED:].ravel().copy()
        try:
            # Follow the assembly branch connected to the static geometry.
            for step in range(1, steps + 1):
                fraction = step / steps
                q = self._position_step(q, travel * fraction, rack_position * fraction,
                                        VERTICAL + normal_delta * fraction)
            points, _, jacobian, rack_column = self.equations(q, travel, rack_position, normal)
            condition = self._condition(jacobian)
            wheel_displacement = points[WHEEL] - self.reference[WHEEL]
            velocity = np.zeros_like(points)
            velocity[RACK, 1] = rack_rate
            rhs = -rack_column * rack_rate
            rhs[-1] = travel_rate - normal_rate @ wheel_displacement
            velocity[N_FIXED:] = self._linear_solve(jacobian, rhs).reshape(-1, 3)

            acceleration = np.zeros_like(points)
            acceleration[RACK, 1] = rack_acc
            relative_v = self._rigid_relative_rates(velocity)
            rhs = -rack_column * rack_acc
            rhs[:-1] -= np.sum(relative_v**2, axis=1) / self.rigid_lengths
            rhs[-1] = (travel_acc - 2 * (normal_rate @ velocity[WHEEL])
                       - normal_acceleration @ wheel_displacement)
            acceleration[N_FIXED:] = self._linear_solve(jacobian, rhs).reshape(-1, 3)
        except (Unreachable, np.linalg.LinAlgError) as error:
            raise Unreachable(f"{self.geometry.name} · 無法求解目前工況。{error}") from error

        return self._make_result(points, velocity, acceleration, jacobian, condition)

    def _make_result(self, points, velocity, acceleration, jacobian, condition):
        # Track the wheel axis with its four reference points. For a rigid
        # upright the fitted angular rates recover the exact rigid motion.
        relative = points[N_FIXED + 1:] - points[N_FIXED]
        reference = self.reference[N_FIXED + 1:] - self.reference[N_FIXED]
        wheel_axis = relative.T @ np.linalg.solve(reference.T, [0., 1., 0.])
        wheel_axis /= np.linalg.norm(wheel_axis)
        side = 1 if self.geometry.name.endswith("L") else -1
        toe = np.degrees(np.arctan2(-wheel_axis[0], wheel_axis[1]))
        camber = np.degrees(np.arcsin(np.clip(-side * wheel_axis[2], -1., 1.)))
        cross_matrix = np.cross(np.eye(3), relative[:, None, :]).transpose(0, 2, 1).reshape(-1, 3)
        v = velocity[N_FIXED + 1:] - velocity[N_FIXED]
        a = acceleration[N_FIXED + 1:] - acceleration[N_FIXED]
        omega = np.linalg.lstsq(cross_matrix, v.ravel(), rcond=None)[0]
        angular_rhs = a - np.cross(omega, np.cross(omega, relative))
        alpha = np.linalg.lstsq(cross_matrix, angular_rhs.ravel(), rcond=None)[0]
        all_points = self._with_mount(points)
        lengths = np.linalg.norm(all_points[self._link_start] - all_points[self._link_end], axis=1)
        for array in (points, velocity, acceleration, wheel_axis, omega, alpha, lengths, jacobian):
            if not np.isfinite(array).all():
                raise Unreachable(f"{self.geometry.name}：求解結果超出數值範圍。")
            array.setflags(write=False)
        length_error = lengths - self.rigid_lengths
        length_error.setflags(write=False)
        rods, arms = compute_rod_motion(self.reference, points, velocity, acceleration, omega, alpha,
                                        None if self.geometry.members is None else self.rigid_links)
        if self.geometry.members is not None:
            reference = self._with_mount(self.reference)
            all_velocities = self._with_mount(velocity, derivative=True)
            all_accelerations = self._with_mount(acceleration, derivative=True)
            for member in self.geometry.members:
                link = member.a, member.b
                if link not in rods:
                    start, end = (self._point_index[name] for name in link)
                    axis = all_points[end] - all_points[start]
                    if np.linalg.norm(axis) < 1e-8:
                        raise Unreachable(f"{self.geometry.name} · {member.name} 的接點重合。")
                    rods[link] = axis_motion(reference[end] - reference[start], axis,
                                             all_velocities[end] - all_velocities[start],
                                             all_accelerations[end] - all_accelerations[start])
        return CornerResult(points, velocity, acceleration, wheel_axis, float(toe), float(camber),
                            omega, alpha, length_error, jacobian, condition, rod_motion=rods, arm_motion=arms,
                            rigid_links=self.rigid_links, rigid_lengths=self.rigid_lengths)


def solve_vehicle(geometry: VehicleGeometry, motion):
    def driver(name):
        return tuple(motion[name][component] / 1000 for component in COMPONENTS)

    return VehicleResult({corner.name: CornerSolver(corner).solve(driver(corner.name), driver(corner.axle))
                          for corner in geometry.corners})
