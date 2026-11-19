"""Six chassis coordinates coupled to the four constrained suspension paths."""

from dataclasses import dataclass, replace
from math import atan2, hypot, isfinite, sqrt

import numpy as np

from .animation import AnimationFrame
from .deformable import DynamicsModel, sample_dampers, sample_element_springs, suspension_terms
from .dynamics import MAX_INTEGRATION_STEP, MAX_STEPS, rk4_step, road_profile, sample_times
from .dynamics_settings import SIMULATION_LIMITS, bounded, require_fields, validate_chassis_inertia
from .heave import HeaveSample
from .mount_loads import compute_mount_loads
from .solver import CornerSolver, Unreachable, VehicleResult, WHEEL, WheelTravel


DISPLACEMENT = slice(0, 3)
ORIENTATION = slice(3, 7)
VELOCITY = slice(7, 10)
OMEGA = slice(10, 13)
WHEELS = slice(13, None)


@dataclass(frozen=True)
class RigidBodySample:
    """Sprung-CG motion in ground axes, with roll, pitch and yaw in degrees."""

    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    angles_deg: np.ndarray
    angular_velocity: np.ndarray
    angular_acceleration: np.ndarray


@dataclass(frozen=True)
class _Evaluation:
    paths: tuple[WheelTravel, ...]
    rotation: np.ndarray
    acceleration: np.ndarray
    hub_accelerations: np.ndarray
    tire_forces: np.ndarray
    road_heights: np.ndarray
    step_limit: float


def _rotation_matrix(quaternion):
    # Unit quaternion (w, x, y, z) maps chassis coordinates to ground coordinates.
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def _angles_deg(rotation):
    """Euler angles of Rz(yaw) Ry(pitch) Rx(roll)."""
    projected_x = hypot(rotation[0, 0], rotation[1, 0])
    pitch = atan2(-rotation[2, 0], projected_x)
    if projected_x <= 1e-12:
        return np.degrees([atan2(-rotation[1, 2], rotation[1, 1]), pitch, 0.])
    roll = atan2(rotation[2, 1], rotation[2, 2])
    yaw = atan2(rotation[1, 0], rotation[0, 0])
    return np.degrees([roll, pitch, yaw])


def _rate_bound(response, coefficients):
    weighted = np.sqrt(coefficients)
    return np.linalg.norm(weighted[:, None] * response * weighted, ord=np.inf)


class _RigidChassisEquation:
    def __init__(self, model, total_mass, inertia, target, amplitude, width):
        self.model = model
        self.solvers = tuple(CornerSolver(item) for item in model.geometry.corners)
        self.names = tuple(solver.geometry.name for solver in self.solvers)
        self.specs = tuple(model.corners[name] for name in self.names)
        self.locked_columns = 6 + np.flatnonzero([solver.locked for solver in self.solvers])
        self.masses = np.array([spec.mass for spec in self.specs])
        self.body_mass = total_mass - self.masses.sum()
        if not isfinite(self.body_mass) or self.body_mass <= 0.:
            raise ValueError("車輛總質量須大於四輪輪端質量總和，才能求解剛體車身。")
        self.inertia = inertia
        self.paths = tuple(solver.wheel_travel(0.) for solver in self.solvers)
        hubs = np.array([path.position[WHEEL] for path in self.paths])
        # The measured CG and static tire loads refer to the complete vehicle.
        # Removing the four wheel masses gives the CG of the sprung body.
        self.pivot = (total_mass * np.asarray(model.geometry.cg) - self.masses @ hubs) / self.body_mass
        self.pivot.setflags(write=False)
        self.amplitudes = np.array([amplitude if target in (name, "all") else 0. for name in self.names])
        self.width = width
        self.wheel_columns = 6 + np.arange(len(self.names))
        self.force_rows = np.eye(6 + len(self.names))[6:]
        self.tire_stiffnesses = np.full(len(self.names), model.tire_stiffness)
        self.tire_dampings = np.full(len(self.names), model.tire_damping)

    def evaluate(self, state, time):
        wheel_states = state[WHEELS].reshape(-1, 2)
        speeds = wheel_states[:, 1]
        self.paths = tuple(solver.wheel_travel(travel, previous)
                           for solver, (travel, _), previous in zip(self.solvers, wheel_states, self.paths))
        rotation = _rotation_matrix(state[ORIENTATION])
        omega = state[OMEGA]
        offsets = (np.array([path.position[WHEEL] for path in self.paths]) - self.pivot) @ rotation.T
        tangents = np.array([path.tangent[WHEEL] for path in self.paths]) @ rotation.T
        curvatures = np.array([path.curvature[WHEEL] for path in self.paths]) @ rotation.T
        relative_velocity = tangents * speeds[:, None]
        rotational_velocity = np.cross(omega, offsets)
        hub_velocity = state[VELOCITY] + rotational_velocity + relative_velocity
        bias = (np.cross(omega, rotational_velocity + 2 * relative_velocity)
                + curvatures * speeds[:, None]**2)
        hub_heights = self.pivot[2] + state[DISPLACEMENT][2] + offsets[:, 2]
        road_height, road_velocity = road_profile(time, 1., self.width)
        road_heights = self.amplitudes * road_height
        road_velocities = self.amplitudes * road_velocity
        tire_forces = np.array([self.model.tire_force(z, v, h, rate)
                                for z, v, h, rate in zip(hub_heights, hub_velocity[:, 2], road_heights, road_velocities)])

        jacobian = self.acceleration_jacobians(offsets, tangents)
        ground_inertia = (rotation * self.inertia) @ rotation.T
        mass_matrix = np.einsum("ijk,ijl,i->kl", jacobian, jacobian, self.masses)
        mass_matrix[:3, :3] += self.body_mass * np.eye(3)
        mass_matrix[3:6, 3:6] += ground_inertia
        # A locked suspension path has a zero tangent. A positive dummy
        # diagonal leaves its zero relative motion intact and makes the
        # acceleration solve invertible with one slot per wheel.
        mass_matrix[self.locked_columns, self.locked_columns] = 1.

        wheel_forces = -self.masses[:, None] * bias
        wheel_forces[:, 2] += tire_forces - self.masses * self.model.gravity
        rhs = np.einsum("ijk,ij->k", jacobian, wheel_forces)
        rhs[2] -= self.body_mass * self.model.gravity
        rhs[3:6] -= np.cross(omega, ground_inertia @ omega)
        spring_rates, damping_rates = self.apply_suspension(rhs, speeds)

        # The mass-normalized stiffness and damping eigenvalues are bounded
        # by the absolute row sums of their matrices in force coordinates.
        contact_and_spring = np.vstack((jacobian[:, 2, :], self.force_rows))
        try:
            solutions = np.linalg.solve(mass_matrix, np.column_stack((rhs, contact_and_spring.T)))
        except np.linalg.LinAlgError as error:
            raise Unreachable("車身與懸吊的質量矩陣無法求解。") from error
        acceleration = solutions[:, 0]
        if not np.isfinite(acceleration).all():
            raise Unreachable("車身或輪心加速度超出數值範圍。")
        hub_accelerations = bias + jacobian @ acceleration
        response = contact_and_spring @ solutions[:, 1:]
        stiffness_rate = _rate_bound(response, np.concatenate((self.tire_stiffnesses, spring_rates)))
        damping_rate = _rate_bound(response, np.concatenate((self.tire_dampings, damping_rates)))
        travel_rate = np.max(np.abs(speeds) + np.sqrt(.004 * np.abs(acceleration[6:])))
        rotation_rate = np.linalg.norm(omega) + sqrt(.05 * np.linalg.norm(acceleration[3:6]))
        step_limit = min(MAX_INTEGRATION_STEP, .35 / sqrt(stiffness_rate), .7 / max(damping_rate, 1e-12),
                         .002 / max(travel_rate, 1e-12), .025 / max(rotation_rate, 1e-12))
        return _Evaluation(self.paths, rotation, acceleration, hub_accelerations, tire_forces, road_heights,
                           step_limit)

    def acceleration_jacobians(self, offsets, tangents):
        """Map chassis linear/angular acceleration and wheel travel acceleration to each hub."""
        count = len(self.solvers)
        matrix = np.zeros((count, 3, 6 + count))
        matrix[:, :, :3] = np.eye(3)
        # alpha × r = -skew(r) alpha.
        x, y, z = offsets.T
        matrix[:, 0, 4], matrix[:, 0, 5] = z, -y
        matrix[:, 1, 3], matrix[:, 1, 5] = -z, x
        matrix[:, 2, 3], matrix[:, 2, 4] = y, -x
        matrix[np.arange(count), :, self.wheel_columns] = tangents
        return matrix

    def apply_suspension(self, rhs, speeds):
        stiffness_rates = []
        damping_rates = []
        for i, (name, path, speed) in enumerate(zip(self.names, self.paths, speeds)):
            force, stiffness, damping = suspension_terms(self.model, name, path, speed)
            # The spring/damper is internal to the vehicle, so it does virtual
            # work only along relative suspension travel, never whole-body motion.
            rhs[6+i] += force
            stiffness_rates.append(stiffness)
            damping_rates.append(damping)
        return np.asarray(stiffness_rates), np.asarray(damping_rates)

    def rates(self, state, time):
        return _rates(state, self.evaluate(state, time))

    def frame(self, state, time, evaluation):
        acceleration = evaluation.acceleration
        speeds = state[WHEELS].reshape(-1, 2)[:, 1]
        corners = {name: solver.wheel_result(path, speed, relative_acceleration)
                   for name, solver, path, speed, relative_acceleration in zip(
                       self.names, self.solvers, evaluation.paths, speeds, acceleration[6:])}
        result = VehicleResult(corners)
        dampers = sample_dampers(self.model, result)
        dampers = {name: replace(dampers[name], tire_force=float(force), road_height=float(height))
                   for name, force, height in zip(self.names, evaluation.tire_forces, evaluation.road_heights)}
        rotation = evaluation.rotation
        mount_loads = compute_mount_loads(self.model, result, dampers, 0., rotation=rotation,
                                          wheel_accelerations=dict(zip(self.names, evaluation.hub_accelerations)))
        body = RigidBodySample(
            state[DISPLACEMENT].copy(), state[VELOCITY].copy(), acceleration[:3].copy(),
            _angles_deg(rotation), state[OMEGA].copy(), acceleration[3:6].copy(),
        )
        for array in (rotation, body.displacement, body.velocity, body.acceleration, body.angles_deg,
                      body.angular_velocity, body.angular_acceleration):
            array.setflags(write=False)
        heave = HeaveSample(float(body.displacement[2]), float(body.velocity[2]), float(body.acceleration[2]), 0.)
        return AnimationFrame(time, result, rotation, self.pivot, body.displacement, dampers=dampers,
                              heave=heave, mount_loads=mount_loads, body=body,
                              element_springs=sample_element_springs(self.model, result))


def _rates(state, evaluation):
    derivative = np.empty_like(state)
    derivative[DISPLACEMENT] = state[VELOCITY]
    derivative[VELOCITY] = evaluation.acceleration[:3]
    derivative[OMEGA] = evaluation.acceleration[3:6]
    real, vector = state[3], state[4:7]
    omega = state[OMEGA]
    # Ground angular velocity premultiplies the chassis-to-ground quaternion.
    derivative[3] = -.5 * (omega @ vector)
    derivative[4:7] = .5 * (real * omega + np.cross(omega, vector))
    derivative[WHEELS] = np.column_stack((state[WHEELS].reshape(-1, 2)[:, 1],
                                         evaluation.acceleration[6:])).ravel()
    return derivative


def simulate_rigid_road(model: DynamicsModel, settings: dict, total_mass: float) -> tuple[AnimationFrame, ...]:
    """Integrate a free rigid chassis and the four wheel travels over a road pulse."""
    simulation = settings.get("simulation") if isinstance(settings, dict) else None
    require_fields(simulation, ("target", *SIMULATION_LIMITS), "模擬設定")
    target = simulation["target"]
    if target not in (*model.corners, "all"):
        raise ValueError("路面擾動輪位必須是 FL、FR、RL、RR 或四輪。")
    limits = {key: bounded(simulation[key], *limit) for key, limit in SIMULATION_LIMITS.items()}
    duration, width = limits["duration_seconds"], limits["bump_seconds"]
    if width > duration:
        raise ValueError("擾動持續時間不可超過模擬時間。")
    inertia = validate_chassis_inertia(settings.get("chassis_inertia"))
    inertia = np.array([inertia[key] for key in ("ixx", "iyy", "izz")])
    equation = _RigidChassisEquation(model, total_mass, inertia, target, limits["bump_mm"] / 1000, width)
    state = np.zeros(13 + 2 * len(equation.solvers))
    state[3] = 1.
    frames = []
    current = 0.
    steps = 0
    try:
        evaluation = equation.evaluate(state, current)
        for output in sample_times(duration, width / 2, width):
            while output - current > 1e-12:
                input_step = width / 32 if current < width else MAX_INTEGRATION_STEP
                step_limit = min(input_step, evaluation.step_limit)
                if steps >= MAX_STEPS or step_limit < 1e-8:
                    raise Unreachable("超過積分步數限制，請縮短模擬時間或調整剛性、阻尼與轉動慣量。")
                step = min(output - current, step_limit)
                state = rk4_step(equation.rates, state, current, step, _rates(state, evaluation))
                state[ORIENTATION] /= np.linalg.norm(state[ORIENTATION])
                current += step
                steps += 1
                evaluation = equation.evaluate(state, current)
            current = output
            frames.append(equation.frame(state, output, evaluation))
    except Unreachable as error:
        raise Unreachable(f"剛體車身於 {current:.3f} s 無法繼續。{error}") from error
    return tuple(frames)
