"""Seven vertical coordinates, three planar coordinates and four wheel spins."""

from dataclasses import dataclass
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from ._vendor.pacejka.model import api
from ._vendor.pacejka.model.RRT_models import MF612_RRT_SAE
from ._vendor.lapsim.brake import hydraulic_brake
from ._vendor.lapsim.motor import load_motor_curve
from ._vendor.lapsim.settings import Settings
from ._vendor.suspension.engine import Vehicle, tire_working_radius
from ._vendor.suspension.project import compile_project, open_project

from .aero_setup import VehicleAero, aero_settings
from .geometry_tables import GeometryTable, TITLES
from .resources import LAB, source_path


DEFAULT_CONFIG = LAB / "lp03.json"
DOF = 14
STATE_SIZE = 2 * DOF
VERTICAL = slice(3, 10)
SPIN = slice(10, 14)
WHEEL_SIDE = np.array([1., -1., 1., -1.])
ZERO_WHEELS = np.zeros(4)
SLIP_REFERENCE_SPEED = 3.0
BRAKE_LOCK_SPEED = 8.0
ROLLING_DIRECTION_RAD_S = .5


def default_desktop_config():
    local_config = LAB / "lp03.local.json"
    return local_config if local_config.is_file() else DEFAULT_CONFIG


@dataclass(frozen=True)
class Controls:
    steer_rad: float = 0.
    throttle: float = 0.
    brake: float = 0.


def flat_road(time):
    """Road height and vertical velocity in FL, FR, RL, RR order."""
    return ZERO_WHEELS, ZERO_WHEELS


def hydraulic_defaults():
    defaults = Settings()
    return {
        "pedal_ratio": defaults.pedal_ratio,
        "front_master_diameter_m": defaults.master_diameter,
        "rear_master_diameter_m": defaults.master_diameter,
        "front_caliper_diameter_m": defaults.front_caliper_diameter,
        "rear_caliper_diameter_m": defaults.rear_caliper_diameter,
        "front_pistons": defaults.front_pistons,
        "rear_pistons": defaults.rear_pistons,
        "front_disc_diameter_m": defaults.disc_diameter,
        "rear_disc_diameter_m": defaults.disc_diameter,
        "pad_mu": defaults.pad_mu,
    }


class Tires:
    """Combined-slip Magic Formula with unilateral contact supplied by the suspension."""

    def __init__(self, path, pressure_kpa):
        self.params = api.load_tir(path)
        self.pressure = pressure_kpa * 1000
        self.unloaded_radius = self.params["UNLOADED_RADIUS"]
        side = str(self.params.get("TYRESIDE", "left")).lower()
        self.mirror = {"left": WHEEL_SIDE, "right": -WHEEL_SIDE, "symmetric": np.ones(4)}[side]

    def fit_inclination(self, inclination):
        return np.clip(inclination * self.mirror, self.params["CAMMIN"], self.params["CAMMAX"])

    def rolling_resistance(self, fz, forward, omega, free_radius=None):
        radius = self.unloaded_radius if free_radius is None else free_radius
        moment = MF612_RRT_SAE.rolling_moment(fz, self.pressure, forward, radius)
        return moment * np.clip(omega / ROLLING_DIRECTION_RAD_S, -1., 1.)

    def forces(self, fz, forward, lateral, tread_speed, inclination):
        speed = np.maximum(np.abs(forward), SLIP_REFERENCE_SPEED)
        slip = (tread_speed - forward) / speed
        angle = np.arctan2(lateral, speed)
        # Blend out steady-state offsets at standstill while preserving traction at launch.
        blend = np.minimum(1., np.abs(forward) / SLIP_REFERENCE_SPEED)
        blend = blend**2 * (3. - 2. * blend)
        low_speed_demand = np.array([12. * slip, -10. * np.tan(angle)])
        low_speed_force = low_speed_demand * fz / np.maximum(1., np.linalg.norm(low_speed_demand, axis=0))
        if not np.any(blend * fz):
            return low_speed_force[0], low_speed_force[1], np.zeros(4), slip, angle

        p = self.params
        fit_load = np.clip(fz, p["FZMIN"], p["FZMAX"])
        fit = api.create_calculation_input_from_config(
            FZ=fit_load,
            SA=np.degrees(np.clip(angle * self.mirror, p["ALPMIN"], p["ALPMAX"])),
            SL=np.clip(slip, p["KPUMIN"], p["KPUMAX"]),
            IA=np.degrees(self.fit_inclination(inclination)),
            V=speed, pressure=self.pressure, tir_params=p,
        )
        fx, fy = api.calculate_fx(fit), api.calculate_fy(fit)
        mz = api.calculate_mz(fit.with_inputs(Fx=fx, Fy=fy))
        # Fade the fit to zero below the minimum calibrated load, including in flight.
        scale = np.minimum(1., fz / fit_load) * blend
        return (fx * scale + low_speed_force[0] * (1. - blend),
                fy * self.mirror * scale + low_speed_force[1] * (1. - blend),
                mz * self.mirror * scale, slip, angle)


class Vehicle14DOF:
    """Body x/y/yaw plus suspension heave/roll/pitch/4 hubs and four spin angles.

    The velocity half stores body-frame u/v followed by yaw rate and the
    derivatives of the vertical and spin coordinates. Pitch is nose-up.
    """

    def __init__(self, config_path=DEFAULT_CONFIG):
        config_path = Path(config_path)
        cfg = json.loads(config_path.read_text(encoding="utf-8-sig"))

        def source(key):
            return source_path(config_path.parent, cfg[key])

        self.wheel_inertia = self._four(cfg, "wheel_inertia_kg_m2", minimum=0.01)
        self.drive_scale = self._four(cfg, "drive_scale", minimum=0., maximum=1.)
        self.camber = np.radians(self._four(cfg, "static_camber_deg", minimum=-20., maximum=20.))
        self.yaw_inertia = self._scalar(cfg, "yaw_inertia_kg_m2", 0.01)
        self.tires = Tires(source("tire_file"), self._four(cfg, "tire_pressure_kpa", minimum=1.))
        radius_settings = {"free_radius_m": cfg.get("rolling_radius_m", self.tires.unloaded_radius)} | cfg
        self.radius = self._scalar(radius_settings, "free_radius_m", 0.01)
        axle_radii = dict.fromkeys(("front_free_radius_m", "rear_free_radius_m"), self.radius) | cfg
        self.free_radii = np.repeat([self._scalar(axle_radii, key, 0.01)
                                    for key in ("front_free_radius_m", "rear_free_radius_m")], 2)
        gear_ratio = self._scalar(cfg, "gear_ratio", 0.01)
        efficiency = self._scalar(cfg, "gear_efficiency", 0.01, 1.)
        self.motor_rpm_per_wheel_rads = gear_ratio * 60. / (2. * np.pi)
        self.torque_multiplication = gear_ratio * efficiency
        self.motor = load_motor_curve(source("motor_envelope"), self._scalar(cfg, "dc_voltage", 1.))
        self.brake_capacity = self.maximum_brake_torque(cfg)
        self.suspension_project_path = source("suspension_project").resolve()
        project, _ = open_project(self.suspension_project_path)
        project["model"] = "rigid"
        project["case"] = {**project["case"], "scenario": "bump", "target": "all", "rear_delay": False}
        project["aero"]["enabled"] = False
        self.suspension = Vehicle(project, compile_project(project))
        p = self.suspension.p
        self.wheelbase = p["lf"] + p["lr"]
        self.height = p["h_cg"]
        self.wheel_x = np.array([p["lf"], p["lf"], -p["lr"], -p["lr"]])
        self.wheel_y = .5 * np.array([p["tf"], -p["tf"], p["tr"], -p["tr"]])
        geometry = cfg.get("suspension_geometry", {})
        if not isinstance(geometry, dict) or type(geometry.get("load_transfer_enabled", False)) is not bool:
            raise ValueError("suspension_geometry.load_transfer_enabled must be true or false")
        self.geometry_load_transfer = geometry.get("load_transfer_enabled", False)
        tables = geometry.get("tables", {})
        if not isinstance(tables, dict) or tables.keys() - TITLES.keys():
            raise ValueError("無法辨識懸吊幾何表格。")
        self.geometry_tables = {kind: GeometryTable(kind, spec) for kind, spec in tables.items()}
        gains = {"camber_gain_deg_per_25mm": [0.] * 4} | geometry
        self.camber_gain = np.radians(self._four(gains, "camber_gain_deg_per_25mm", minimum=-30., maximum=30.)) / 25.
        self.use_camber_gain = bool(np.any(self.camber_gain))
        self.roll_center_scale = np.array([p["h_rc_f"] / p["tf"], p["h_rc_r"] / p["tr"]])
        # The configured Anti ratios are full-car percentages, including the axle shares.
        anti_scale = self.height / (2. * self.wheelbase)
        self.anti_drive = anti_scale * np.array([p["lift_front"], p["lift_front"], -p["squat_rear"], -p["squat_rear"]])
        self.anti_brake = anti_scale * np.array([p["dive_front"], p["dive_front"], -p["lift_rear"], -p["lift_rear"]])
        self.track_widths = np.array([p["tf"], p["tr"]])
        self.aero = VehicleAero(aero_settings(cfg, config_path.parent), p["lf"], p["lr"],
                                (p["tf"] + p["tr"]) / 2., self.height)

    @staticmethod
    def _scalar(config, key, minimum, maximum=np.inf):
        value = float(config[key])
        if not np.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"{key} must be between {minimum:g} and {maximum:g}")
        return value

    @staticmethod
    def _four(config, key, *, minimum, maximum=np.inf):
        values = np.asarray(config[key], dtype=float)
        if (values.shape != (4,) or not np.isfinite(values).all()
                or np.any(values < minimum) or np.any(values > maximum)):
            raise ValueError(f"{key} requires four values between {minimum:g} and {maximum:g}")
        return values

    def maximum_brake_torque(self, config):
        front = self._scalar(config, "front_brake_bias", 0., 1.)
        bias = np.array([front, 1. - front])
        force = self._scalar(config, "max_pedal_force_n", 0.)
        raw = config.get("hydraulic_brake", {})
        if not isinstance(raw, dict):
            raise ValueError("hydraulic_brake must be a configuration object")
        data = hydraulic_defaults() | raw
        counts = [data["front_pistons"], data["rear_pistons"]]
        if any(type(count) is not int or count < 1 for count in counts):
            raise ValueError("Caliper piston counts must be positive integers")
        settings = SimpleNamespace(
            pedal_force=force, pedal_ratio=self._scalar(data, "pedal_ratio", .001),
            front_pistons=counts[0], rear_pistons=counts[1],
            master_diameter=np.array([self._scalar(data, f"{axle}_master_diameter_m", .001)
                                      for axle in ("front", "rear")]),
            front_caliper_diameter=self._scalar(data, "front_caliper_diameter_m", .001),
            rear_caliper_diameter=self._scalar(data, "rear_caliper_diameter_m", .001),
            disc_diameter=np.array([self._scalar(data, f"{axle}_disc_diameter_m", .001)
                                    for axle in ("front", "rear")]),
            pad_mu=self._scalar(data, "pad_mu", 0.),
        )
        return -hydraulic_brake(1., SimpleNamespace(settings=settings, brake_bias=bias))

    def initial_state(self, *, position=(0., 0.), yaw=0., speed=0., road=flat_road):
        if not np.isfinite(speed) or speed < 0:
            raise ValueError("Initial speed must be finite and nonnegative")
        state = np.zeros(STATE_SIZE)
        state[:3] = (*position, yaw)
        height = road(0.)[0]

        def static_aero(vertical):
            loads = self.aero_at(speed, 0., vertical)
            return np.r_[loads.sprung_forces, ZERO_WHEELS]

        self.trim = self.suspension.equilibrium(
            (height, ZERO_WHEELS, 0., 0.), include_aero=False,
            external_forces=static_aero if speed and self.aero.enabled else None,
        )
        state[VERTICAL] = self.trim
        state[DOF] = speed
        if speed:
            fz = self.suspension.tire_load(self.trim, np.zeros_like(self.trim), height, ZERO_WHEELS)
            loaded = tire_working_radius(self.free_radii, fz, self.suspension.tire_k)
            if np.any(loaded <= 0.):
                raise ValueError("靜態輪胎壓縮已達自由半徑，請確認輪胎自由半徑與垂向剛性。")
            state[DOF + SPIN.start:] = speed / loaded
        return state

    def aero_at(self, u, v, vertical):
        attitude = vertical[:3] - self.suspension.static_reference[:3]
        return self.aero.loads(u, v, *attitude)

    @staticmethod
    def axle_sample(table, column, axle_travel, roll_deg, steer_deg):
        coordinates = {"travel_mm": axle_travel[column % 2], "front_travel_mm": axle_travel[0],
                       "rear_travel_mm": axle_travel[1], "roll_deg": roll_deg, "steer_deg": steer_deg}
        return table.sample(column, *(coordinates[axis] for axis in table.axes))

    @staticmethod
    def wheel_samples(table, travel_mm, nominal_steer_deg, front_steer_deg):
        values = []
        for wheel, (travel, steer) in enumerate(zip(travel_mm, nominal_steer_deg)):
            coordinates = {"travel_mm": travel, "steer_deg": steer, "front_steer_deg": front_steer_deg}
            values.append(table.sample(wheel, *(coordinates[axis] for axis in table.axes)))
        return np.asarray(values)

    def geometry_reactions(self, body_fx, body_fy, travel_mm=None, roll_deg=0., steer_deg=0.):
        """Positive reactions act upward on the hubs and downward on the chassis."""
        roll_table, anti_table = (self.geometry_tables.get(kind) for kind in ("roll_center", "anti"))
        axle_travel = travel_mm.reshape(2, 2).mean(axis=1) if travel_mm is not None else (0., 0.)
        rc_scale = self.roll_center_scale
        if roll_table is not None:
            rc_scale = np.array([self.axle_sample(roll_table, i, axle_travel, roll_deg, steer_deg)
                                 for i in range(2)]) / (1000. * self.track_widths)
            if roll_table.relative:
                rc_scale += self.roll_center_scale
        front, rear = rc_scale * body_fy.reshape(2, 2).sum(axis=1)
        roll_center = np.array([front, -front, rear, -rear])
        longitudinal_force = body_fx.sum()
        anti_scale = self.anti_drive if longitudinal_force >= 0. else self.anti_brake
        if anti_table is not None:
            first = 2 if longitudinal_force >= 0. else 0
            f, r = (self.axle_sample(anti_table, i, axle_travel, roll_deg, steer_deg) for i in (first, first + 1))
            sampled = self.height / (200. * self.wheelbase) * np.array([f, f, -r, -r])
            anti_scale = anti_scale + sampled if anti_table.relative else sampled
        anti = longitudinal_force * anti_scale
        return roll_center, anti

    def evaluate(self, state, controls, road_input, *, telemetry=False):
        q, v = state[:DOF], state[DOF:]
        u, lateral, yaw_rate = v[:3]
        vertical, vertical_rate = q[VERTICAL], v[VERTICAL]
        omega = v[SPIN]
        road_height, road_speed = road_input
        vertical_accel, fz, _, suspension_force, _ = self.suspension.forces(
            vertical, vertical_rate, (road_height, road_speed, 0., 0.), details=True, include_aero=False,
        )
        loaded_radius = tire_working_radius(self.free_radii, fz, self.suspension.tire_k)

        # Ackermann road-wheel angles from the commanded equivalent central angle.
        curvature = np.tan(controls.steer_rad) / self.wheelbase
        nominal_steer = np.arctan2(np.array([curvature, curvature, 0., 0.]) * self.wheelbase,
                                   np.r_[1. - self.wheel_y[:2] * curvature, 1., 1.])
        camber_table, bump_table = (self.geometry_tables.get(kind) for kind in ("camber", "bump_steer"))
        travel_mm = None
        if camber_table is not None or bump_table is not None or self.use_camber_gain or (
                self.geometry_load_transfer and self.geometry_tables):
            travel_mm = 1000. * (self.suspension.travel @ vertical) - self.suspension.tender_reference_wheel_mm
        bump_steer = ZERO_WHEELS
        camber = self.camber
        front_steer_deg = np.degrees(controls.steer_rad)
        if camber_table is not None or bump_table is not None:
            nominal_steer_deg = np.degrees(nominal_steer)
        if bump_table is not None:
            bump_steer = self.wheel_samples(bump_table, travel_mm, nominal_steer_deg, front_steer_deg)
        if camber_table is not None:
            camber = camber + np.radians(self.wheel_samples(camber_table, travel_mm, nominal_steer_deg, front_steer_deg))
        elif self.use_camber_gain:
            camber = camber + self.camber_gain * travel_mm
        steer = nominal_steer + np.radians(bump_steer)
        cs, sn = np.cos(steer), np.sin(steer)
        arm = self.height + (vertical[0] - self.trim[0]) - road_height
        contact_u = u - yaw_rate * self.wheel_y + arm * vertical_rate[2]
        contact_v = lateral + yaw_rate * self.wheel_x + arm * vertical_rate[1]
        inclination = -WHEEL_SIDE * camber + vertical[1] * cs
        wheel_forward = cs * contact_u + sn * contact_v
        fx, fy, mz, slip, angle = self.tires.forces(
            fz, wheel_forward, -sn * contact_u + cs * contact_v,
            loaded_radius * omega, inclination,
        )
        body_fx, body_fy = cs * fx - sn * fy, sn * fx + cs * fy
        speed = np.hypot(u, lateral)
        aero = self.aero_at(u, lateral, vertical)
        mass = self.suspension.total_mass
        vertical_accel[:3] += (np.asarray(aero.sprung_forces) + [0., arm @ body_fy, arm @ body_fx]) / self.suspension.mass[:3]
        roll_center = anti = ZERO_WHEELS
        if self.geometry_load_transfer:
            roll_center, anti = self.geometry_reactions(body_fx, body_fy, travel_mm, np.degrees(vertical[1]),
                                                        front_steer_deg)
            # Route part of the contact moment through the links. The travel Jacobian
            # applies equal and opposite chassis/hub reactions without adding external load.
            vertical_accel += (self.suspension.travel.T @ (roll_center + anti)) / self.suspension.mass

        drive = (self.torque_multiplication * self.drive_scale * controls.throttle
                 * np.maximum(0., self.motor.torque_at(np.abs(omega) * self.motor_rpm_per_wheel_rads)))
        rolling = self.tires.rolling_resistance(fz, wheel_forward, omega, self.free_radii)
        free_torque = drive - loaded_radius * fx + rolling
        brake_limit = self.brake_capacity * controls.brake
        lock_blend = np.maximum(0., 1. - np.abs(omega) / BRAKE_LOCK_SPEED)
        brake_torque = np.clip(brake_limit * omega / BRAKE_LOCK_SPEED + free_torque * lock_blend,
                               -brake_limit, brake_limit)

        c_yaw, s_yaw = np.cos(q[2]), np.sin(q[2])
        rates = np.r_[c_yaw * u - s_yaw * lateral, s_yaw * u + c_yaw * lateral, v[2:],
                      (body_fx.sum() + aero.fx) / mass + yaw_rate * lateral,
                      (body_fy.sum() + aero.fy) / mass - yaw_rate * u,
                      (self.wheel_x @ body_fy - self.wheel_y @ body_fx + mz.sum() + aero.yaw) / self.yaw_inertia,
                      vertical_accel, (free_torque - brake_torque) / self.wheel_inertia]
        if not telemetry:
            return rates

        fitted_camber = np.degrees(-WHEEL_SIDE * self.tires.mirror * self.tires.fit_inclination(inclination))
        return {
            "x_m": q[0], "y_m": q[1], "yaw_deg": np.degrees(q[2]), "yaw_rate_deg_s": np.degrees(yaw_rate),
            "speed_mps": speed, "u_mps": u, "v_mps": lateral,
            "ax_mps2": rates[DOF] - yaw_rate * lateral, "ay_mps2": rates[DOF + 1] + yaw_rate * u,
            "heave_mm": 1000. * (vertical[0] - self.trim[0]), "roll_deg": np.degrees(vertical[1]),
            "pitch_deg": np.degrees(vertical[2]), "downforce_n": aero.downforce, "drag_n": aero.drag,
            "cop_x_m": aero.cop_x, "cop_y_m": aero.cop_y, "cop_z_m": aero.cop_z, "CL": aero.cl, "CD": aero.cd,
            "aero_roll_nm": aero.roll, "aero_pitch_nm": aero.pitch, "aero_yaw_nm": aero.yaw,
        } | {
            f"{corner}_{key}": values[i]
            for key, values in (
                ("wheel_travel_mm", 1000. * (self.suspension.travel @ (vertical - self.trim))),
                ("wheel_speed_rad_s", omega), ("wheel_spin_rad", q[SPIN]), ("fz_n", fz), ("fx_n", fx),
                ("rolling_radius_m", loaded_radius),
                ("fy_n", fy), ("mz_nm", mz), ("slip_ratio", slip), ("slip_angle_deg", np.degrees(angle)),
                ("steer_deg", np.degrees(steer)), ("nominal_steer_deg", np.degrees(nominal_steer)),
                ("bump_steer_deg", bump_steer), ("road_height_mm", 1000. * road_height),
                ("camber_deg", fitted_camber), ("roll_center_force_n", roll_center), ("anti_force_n", anti),
                ("suspension_force_n", suspension_force), ("drive_nm", drive), ("brake_nm", -brake_torque),
                ("rolling_resistance_nm", rolling),
            )
            for i, corner in enumerate(self.suspension.corners)
        }
