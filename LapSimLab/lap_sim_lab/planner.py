"""Steady-state combined Fx/Fy speed planning from new/lapsim/main.py."""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from .brake import brake as brake_torque
from .car import Car
from .motor import Throttle_command
from .suspension import wheel_load_stiffness
from .tire import SimplifiedTireModel
from .track import SectionGeometry


DEFAULT_MAX_STRAIGHT_SPEED = 30.0
LATERAL_SPEED_MAX_ITERATIONS = 12
LATERAL_SPEED_TOLERANCE = 0.02
ACCELERATION_MAX_ITERATIONS = 10
ACCELERATION_TOLERANCE = 0.1
BRAKING_SPEED_MAX_ITERATIONS = 10
BRAKING_SPEED_TOLERANCE = 0.06
SPEED_PLAN_CONVERGENCE_TOLERANCE = 0.05
GRAVITY = 9.81

class PowerBrakeSystemLimit:
    """Full-throttle and full-brake force limits for the vehicle."""

    def __init__(self, car: Car):
        self.car = car
        self.accel_cache: dict[tuple[str, float, float], float] = {}
        self.max_brake_torque = -np.asarray(
            brake_torque(1.0, self.car),
            dtype=float,
        )

    def max_drive_torque_distribution(
        self,
        speed: float,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        omega = np.full(4, max(speed, 0.0) / self.car.R_tire, dtype=float)
        return np.maximum(
            np.asarray(Throttle_command(omega, self.car, cmd=self.car.T_rate), dtype=float),
            0.0,
        )

    def max_drive_force_distribution(
        self,
        speed: float,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        if loaded_radius is None:
            loaded_radius = np.full(4, self.car.R_tire, dtype=float)
        loaded_radius = np.maximum(np.asarray(loaded_radius, dtype=float), 1e-9)
        wheel_torque = self.max_drive_torque_distribution(speed, loaded_radius)
        return wheel_torque / loaded_radius

    def scale_limited_drive_force(
        self,
        speed: float,
        tire_drive_fx_limit: np.ndarray,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        base_force = self.max_drive_force_distribution(speed, loaded_radius)
        active = base_force > 1e-9

        if not np.any(active):
            return np.zeros(4, dtype=float)

        scale = float(
            np.min(
                np.divide(
                    tire_drive_fx_limit[active],
                    base_force[active],
                    out=np.zeros_like(base_force[active]),
                    where=base_force[active] > 1e-9,
                )
            )
        )
        scale = float(np.clip(scale, 0.0, 1.0))
        return base_force * scale

    def max_brake_torque_distribution(self) -> np.ndarray:
        return self.max_brake_torque.copy()

    def max_brake_force_distribution(
        self,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        if loaded_radius is None:
            loaded_radius = np.full(4, self.car.R_tire, dtype=float)
        loaded_radius = np.maximum(np.asarray(loaded_radius, dtype=float), 1e-9)
        return -self.max_brake_torque_distribution() / loaded_radius

    def scale_limited_brake_force(
        self,
        tire_brake_fx_limit: np.ndarray,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        base_force = self.max_brake_force_distribution(loaded_radius)
        base_capacity = -base_force
        tire_capacity = np.maximum(-np.asarray(tire_brake_fx_limit, dtype=float), 0.0)
        active = base_capacity > 1e-9

        if not np.any(active):
            return np.zeros(4, dtype=float)

        scale = float(
            np.min(
                np.divide(
                    tire_capacity[active],
                    base_capacity[active],
                    out=np.zeros_like(base_capacity[active]),
                    where=base_capacity[active] > 1e-9,
                )
            )
        )
        scale = float(np.clip(scale, 0.0, 1.0))
        return base_force * scale

@dataclass(frozen=True)
class WheelLoadState:
    loads: np.ndarray
    theta: np.ndarray
    heave: float
    aero_result: dict[str, float]


def camber_from_roll(theta_rad: np.ndarray) -> np.ndarray:
    """Return wheel camber in FL, FR, RL, RR order with zero static camber and gain."""
    camber_deg = np.degrees(theta_rad[..., [0, 0, 1, 1]])
    # Positive chassis roll lifts the left side; positive camber tips outward.
    camber_deg[..., ::2] *= -1
    return camber_deg


@dataclass(frozen=True)
class SegmentSpeedPath:
    x: np.ndarray
    y: np.ndarray
    s: np.ndarray
    inner_x: np.ndarray
    inner_y: np.ndarray
    outer_x: np.ndarray
    outer_y: np.ndarray
    curvature: np.ndarray
    length: float


@dataclass(frozen=True)
class AeroTireLimits:
    tire_model: SimplifiedTireModel
    car: Car
    gravity: float = GRAVITY
    attitude_iterations: int = 3

    def wheel_load_state(
        self,
        speed: float,
        ax: float,
        ay: float,
        yaw: float = 0.0,
    ) -> WheelLoadState:
        aero_result = self.aero_result_from_attitude(speed, 0.0, 0.0, 0.0)
        load_state = None

        for _ in range(max(int(self.attitude_iterations), 1)):
            load_state = self.suspension_state(float(aero_result["F_down"]), ax, ay, yaw)
            aero_result = self.aero_result_from_attitude(
                speed,
                load_state.qh,
                load_state.qp,
                load_state.qr,
            )

        return WheelLoadState(
            loads=np.maximum(load_state.loads, 1.0),
            theta=load_state.theta,
            heave=float(load_state.heave),
            aero_result=aero_result,
        )

    def suspension_state(self, downforce: float, ax: float, ay: float, yaw: float = 0.0):
        car = self.car
        return wheel_load_stiffness(
            mass=car.m,
            g=self.gravity,
            aero=downforce,
            ax=ax,
            ay=ay,
            cg_height=car.h_cg,
            wheelbase=car.l,
            front_ratio=car.lr / car.l,
            track_front=car.tf,
            track_rear=car.tr,
            k_heave_front=car.k_heave_front,
            k_heave_rear=car.k_heave_rear,
            k_roll_front=car.k_roll_front,
            k_roll_rear=car.k_roll_rear,
            yaw=yaw,
            return_state=True,
            tire_stiffness=car.K_tire,
        )

    def aero_result_from_attitude(
        self,
        speed: float,
        qh: float,
        qp: float,
        qr: float,
    ) -> dict[str, float]:
        if self.car.aero is None:
            return {
                "F_down": 0.0,
                "F_drag": 0.0,
                "F_FL": 0.0,
                "F_FR": 0.0,
                "F_RL": 0.0,
                "F_RR": 0.0,
                "COP_x": 0.0,
                "COP_y": 0.0,
                "CL": 0.0,
                "CD": 0.0,
                "qh": qh,
                "qp": qp,
                "qr": qr,
            }

        return self.car.aero.calculate_from_modal(speed, qh, qp, qr)

    def max_fy_for_loads(
        self,
        loads: np.ndarray,
        camber_deg: float | np.ndarray = 0.0,
    ) -> np.ndarray:
        clipped_loads = np.clip(loads, *self.tire_model.fz_range)
        clipped_cambers = np.broadcast_to(
            np.clip(camber_deg, *self.tire_model.camber_range), clipped_loads.shape
        )
        return np.array(
            [
                self.tire_model.peak_forces(float(fz), float(camber))[1]
                for fz, camber in zip(clipped_loads, clipped_cambers)
            ],
            dtype=float,
        )

    def fx_limits_at_fz_and_fy(
        self,
        fz: float,
        fy: float,
        camber_deg: float = 0.0,
    ) -> tuple[float, float]:
        fz = float(np.clip(fz, *self.tire_model.fz_range))
        camber_deg = float(np.clip(camber_deg, *self.tire_model.camber_range))
        limits = self.tire_model.fx_limits(fy, fz=fz, camber_deg=camber_deg)
        if len(limits) < 2:
            return 0.0, 0.0
        return float(limits[0]), float(limits[1])

    def fx_limits_for_loads_and_fy(
        self,
        loads: np.ndarray,
        fy_per_wheel: np.ndarray,
        camber_deg: float | np.ndarray = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        brake = np.empty(4, dtype=float)
        drive = np.empty(4, dtype=float)
        cambers = np.broadcast_to(camber_deg, np.shape(loads))
        for index, (fz, fy, camber) in enumerate(zip(loads, fy_per_wheel, cambers)):
            brake[index], drive[index] = self.fx_limits_at_fz_and_fy(fz, fy, camber)
        return brake, drive


@dataclass(frozen=True)
class SpeedPlan:
    lateral_speed_limit: np.ndarray
    speed: np.ndarray
    acceleration: np.ndarray
    phase: np.ndarray
    wheel_loads: np.ndarray
    theta: np.ndarray
    heave: np.ndarray
    downforce: np.ndarray
    drag: np.ndarray
    cop: np.ndarray
    cl: np.ndarray
    cd: np.ndarray


def build_aero_lateral_speed_profile(
    track: SegmentSpeedPath,
    tire_limit_map: AeroTireLimits,
    max_straight_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
    curvature_floor: float = 1e-5,
    safety_factor: float = 0.98,
) -> np.ndarray:
    curvature = np.asarray(track.curvature, dtype=float)
    speed = np.full_like(curvature, max_straight_speed, dtype=float)

    for index, kappa in enumerate(curvature):
        if abs(kappa) <= curvature_floor:
            continue

        low = 0.0
        high = max_straight_speed
        for _ in range(LATERAL_SPEED_MAX_ITERATIONS):
            if high - low <= LATERAL_SPEED_TOLERANCE:
                break
            mid = 0.5 * (low + high)
            ay = mid * mid * kappa
            load_state = tire_limit_map.wheel_load_state(speed=mid, ax=0.0, ay=ay)
            fy_need_total = tire_limit_map.car.m * abs(ay)
            fy_cap = tire_limit_map.max_fy_for_loads(
                load_state.loads, camber_from_roll(load_state.theta)
            )
            fy_cap_total = float(np.sum(fy_cap))
            if fy_need_total <= fy_cap_total * safety_factor:
                low = mid
            else:
                high = mid

        speed[index] = low

    return speed


def build_friction_limited_speed_plan(
    track: SegmentSpeedPath,
    tire_limit_map: AeroTireLimits,
    power_brake_limit: PowerBrakeSystemLimit,
    max_straight_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
    curvature_floor: float = 1e-5,
    safety_factor: float = 0.98,
    passes: int = 3,
    progress: Callable[[str], None] | None = None,
) -> SpeedPlan:
    """
    Build a speed plan with lateral, acceleration, and braking tire limits.

    Lateral demand is converted to per-tire FY, then the tire model estimates
    remaining positive/negative FX. A forward pass handles exit acceleration,
    and a backward pass pulls braking zones back from lower-speed corners.
    """

    report = progress or (lambda message: None)
    report("計算側向抓地速度上限")
    lateral_speed_limit = build_aero_lateral_speed_profile(
        track=track,
        tire_limit_map=tire_limit_map,
        max_straight_speed=max_straight_speed,
        curvature_floor=curvature_floor,
        safety_factor=safety_factor,
    )
    speed = lateral_speed_limit.copy()

    total_passes = max(int(passes), 1)
    for pass_index in range(total_passes):
        speed_before_pass = speed.copy()
        report(f"第 {pass_index + 1}/{total_passes} 輪，向前掃描加速限制")
        for index in range(len(speed) - 1):
            ds = max(float(track.s[index + 1] - track.s[index]), 1e-6)
            accel = drive_accel_at_speed(
                speed[index],
                track.curvature[index],
                tire_limit_map,
                power_brake_limit,
            )
            reachable = np.sqrt(max(speed[index] ** 2 + 2.0 * accel * ds, 0.0))
            speed[index + 1] = min(speed[index + 1], reachable)

        report(f"第 {pass_index + 1}/{total_passes} 輪，向後掃描煞車限制")
        for index in range(len(speed) - 2, -1, -1):
            ds = max(float(track.s[index + 1] - track.s[index]), 1e-6)
            speed[index] = min(
                speed[index],
                max_entry_speed_for_braking(
                    exit_speed=speed[index + 1],
                    ds=ds,
                    curvature=track.curvature[index],
                    upper_speed=speed[index],
                    tire_limit_map=tire_limit_map,
                    power_brake_limit=power_brake_limit,
                ),
            )

        pass_speed_change = float(np.max(np.abs(speed_before_pass - speed)))
        if pass_speed_change <= SPEED_PLAN_CONVERGENCE_TOLERANCE:
            break

    acceleration = speed_to_acceleration(track.s, speed)
    phase = np.full(len(speed), "hold", dtype=object)
    phase[acceleration > 0.05] = "accelerate"
    phase[acceleration < -0.05] = "brake"
    report("整理四輪輪荷與車身姿態")
    wheel_states = [
        tire_limit_map.wheel_load_state(
            speed=float(v),
            ax=float(ax),
            ay=float(v * v * kappa),
        )
        for v, ax, kappa in zip(speed, acceleration, track.curvature)
    ]
    wheel_loads = np.array([state.loads for state in wheel_states], dtype=float)
    theta = np.array([state.theta for state in wheel_states], dtype=float)
    heave = np.array([state.heave for state in wheel_states], dtype=float)
    downforce = np.array(
        [state.aero_result["F_down"] for state in wheel_states],
        dtype=float,
    )
    drag = np.array(
        [state.aero_result["F_drag"] for state in wheel_states],
        dtype=float,
    )
    cop = np.array(
        [
            [
                state.aero_result["COP_x"],
                state.aero_result["COP_y"],
                tire_limit_map.car.aero.cop_z if tire_limit_map.car.aero else 0.0,
            ]
            for state in wheel_states
        ],
        dtype=float,
    )
    cl = np.array([state.aero_result["CL"] for state in wheel_states], dtype=float)
    cd = np.array([state.aero_result["CD"] for state in wheel_states], dtype=float)

    return SpeedPlan(
        lateral_speed_limit=lateral_speed_limit,
        speed=speed,
        acceleration=acceleration,
        phase=phase,
        wheel_loads=wheel_loads,
        theta=theta,
        heave=heave,
        downforce=downforce,
        drag=drag,
        cop=cop,
        cl=cl,
        cd=cd,
    )


def drive_accel_at_speed(
    speed: float,
    curvature: float,
    tire_limit_map: AeroTireLimits,
    power_brake_limit: PowerBrakeSystemLimit,
) -> float:
    key = (
        "drive",
        round(float(speed), 2),
        round(float(curvature), 5),
    )
    cache = power_brake_limit.accel_cache
    if key in cache:
        return cache[key]

    ay = speed * speed * curvature
    vehicle_mass = tire_limit_map.car.m
    low = 0.0
    high = 35.0

    for _ in range(ACCELERATION_MAX_ITERATIONS):
        if high - low <= ACCELERATION_TOLERANCE:
            break
        candidate_ax = 0.5 * (low + high)
        load_state = tire_limit_map.wheel_load_state(speed=speed, ax=candidate_ax, ay=ay)
        loads = load_state.loads
        fy_wheels = distribute_lateral_force(vehicle_mass * ay, loads)
        _, drive_fx = tire_limit_map.fx_limits_for_loads_and_fy(
            loads, fy_wheels, camber_from_roll(load_state.theta)
        )
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        aero_drag = float(load_state.aero_result["F_drag"])
        drive_force = power_brake_limit.scale_limited_drive_force(
            speed,
            drive_fx,
            loaded_radius=loaded_radius,
        )
        capacity_ax = max(
            (float(np.sum(drive_force)) - aero_drag)
            / vehicle_mass,
            0.0,
        )

        if candidate_ax <= capacity_ax:
            low = candidate_ax
        else:
            high = candidate_ax

    cache[key] = low
    return low


def brake_decel_at_speed(
    speed: float,
    curvature: float,
    tire_limit_map: AeroTireLimits,
    power_brake_limit: PowerBrakeSystemLimit,
) -> float:
    key = (
        "brake",
        round(float(speed), 2),
        round(float(curvature), 5),
    )
    cache = power_brake_limit.accel_cache
    if key in cache:
        return cache[key]

    ay = speed * speed * curvature
    vehicle_mass = tire_limit_map.car.m
    low = 0.0
    high = 35.0

    for _ in range(ACCELERATION_MAX_ITERATIONS):
        if high - low <= ACCELERATION_TOLERANCE:
            break
        candidate_decel = 0.5 * (low + high)
        load_state = tire_limit_map.wheel_load_state(
            speed=speed,
            ax=-candidate_decel,
            ay=ay,
        )
        loads = load_state.loads
        fy_wheels = distribute_lateral_force(vehicle_mass * ay, loads)
        brake_fx, _ = tire_limit_map.fx_limits_for_loads_and_fy(
            loads, fy_wheels, camber_from_roll(load_state.theta)
        )
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        aero_drag = float(load_state.aero_result["F_drag"])
        brake_force = power_brake_limit.scale_limited_brake_force(
            brake_fx,
            loaded_radius=loaded_radius,
        )
        capacity_decel = max(
            (float(-np.sum(brake_force)) + aero_drag)
            / vehicle_mass,
            0.0,
        )

        if candidate_decel <= capacity_decel:
            low = candidate_decel
        else:
            high = candidate_decel

    cache[key] = low
    return low


def distribute_lateral_force(total_fy: float, loads: np.ndarray) -> np.ndarray:
    load_sum = float(np.sum(loads))
    if load_sum <= 1e-9:
        return np.full(4, total_fy / 4.0, dtype=float)
    return total_fy * loads / load_sum


def max_entry_speed_for_braking(
    exit_speed: float,
    ds: float,
    curvature: float,
    upper_speed: float,
    tire_limit_map: AeroTireLimits,
    power_brake_limit: PowerBrakeSystemLimit,
) -> float:
    """Solve the highest current speed that can brake to exit_speed over ds."""

    low = float(exit_speed)
    high = float(max(upper_speed, exit_speed))

    for _ in range(BRAKING_SPEED_MAX_ITERATIONS):
        if high - low <= BRAKING_SPEED_TOLERANCE:
            break
        mid = 0.5 * (low + high)
        decel = brake_decel_at_speed(
            mid,
            curvature,
            tire_limit_map,
            power_brake_limit,
        )
        reachable_exit_square = mid * mid - 2.0 * decel * ds

        if reachable_exit_square <= exit_speed * exit_speed:
            low = mid
        else:
            high = mid

    return low


def speed_to_acceleration(s: np.ndarray, speed: np.ndarray) -> np.ndarray:
    accel = np.zeros_like(speed)
    ds = np.diff(s)
    valid = ds > 1e-9
    segment_accel = np.zeros(len(speed) - 1, dtype=float)
    segment_accel[valid] = (
        speed[1:][valid] ** 2 - speed[:-1][valid] ** 2
    ) / (2.0 * ds[valid])

    accel[:-1] += segment_accel
    accel[1:] += segment_accel
    accel[1:-1] *= 0.5
    return accel


def lap_time_vector(s: np.ndarray, speed: np.ndarray) -> np.ndarray:
    time = np.zeros_like(speed, dtype=float)
    ds = np.diff(s)
    segment_speed = 0.5 * (speed[:-1] + speed[1:])
    dt = np.zeros_like(ds, dtype=float)
    valid = (ds > 1e-9) & (segment_speed > 1e-9)
    dt[valid] = ds[valid] / segment_speed[valid]
    time[1:] = np.cumsum(dt)
    return time


def build_segment_speed_path(sections: list[SectionGeometry]) -> SegmentSpeedPath:
    """Expand track geometry using each section's original curvature."""

    x_parts = []
    y_parts = []
    inner_x_parts = []
    inner_y_parts = []
    outer_x_parts = []
    outer_y_parts = []
    curvature_parts = []

    for index, geometry in enumerate(sections):
        start = 0 if index == 0 else 1
        x_parts.append(geometry.x[start:])
        y_parts.append(geometry.y[start:])
        inner_x_parts.append(geometry.inner_x[start:])
        inner_y_parts.append(geometry.inner_y[start:])
        outer_x_parts.append(geometry.outer_x[start:])
        outer_y_parts.append(geometry.outer_y[start:])
        curvature_parts.append(
            np.full(len(geometry.x[start:]), geometry.curvature, dtype=float)
        )

    x = np.concatenate(x_parts)
    y = np.concatenate(y_parts)
    ds = np.hypot(
        np.diff(x, prepend=x[0]),
        np.diff(y, prepend=y[0]),
    )

    return SegmentSpeedPath(
        x=x,
        y=y,
        s=np.cumsum(ds),
        inner_x=np.concatenate(inner_x_parts),
        inner_y=np.concatenate(inner_y_parts),
        outer_x=np.concatenate(outer_x_parts),
        outer_y=np.concatenate(outer_y_parts),
        curvature=np.concatenate(curvature_parts),
        length=float(sum(section.length for section in sections)),
    )
