"""Quasi-static load transfer adapted from VehicleLab's rigid and distribution tools."""

import math

import numpy as np

from .engine import G, prepare_vehicle
from .geometry_lookup import GeometryLookup
from .project import AERO_DEFAULTS, CORNERS, downforce


def rigid_transfer(vehicle, ax_g, ay_g):
    mass, height = vehicle["m"], vehicle["h_cg"]
    wheelbase = vehicle["lf"] + vehicle["lr"]
    weight = mass * G
    front = weight * vehicle["lr"] / wheelbase
    longitudinal = -mass * G * ax_g * height / wheelbase
    axle_loads = np.array([front + longitudinal, weight - front - longitudinal])
    lateral = axle_loads * ay_g * height / [vehicle["tf"], vehicle["tr"]]
    static = np.repeat([front / 2, (weight - front) / 2], 2)
    return static, axle_loads, longitudinal, np.repeat(axle_loads / 2, 2) + wheel_deltas(lateral)


def wheel_deltas(lateral):
    front, rear = lateral
    return np.array([-front, front, -rear, rear])


def contact_patch(load, pressure_kpa, width_mm):
    if load <= 0:
        return 0., 0., 0.
    area_mm2 = 1000 * load / pressure_kpa
    width = min(.88 * width_mm, math.sqrt(4 * area_mm2 / math.pi))
    return area_mm2 / 100, 4 * area_mm2 / (math.pi * width), width


class TransferDistribution:
    def __init__(self, project, options):
        self.options = options
        p = project["vehicle"]
        self.mass, self.height = p["m"], p["h_cg"]
        self.wheelbase = p["lf"] + p["lr"]
        front = p["lr"] / self.wheelbase
        self.fractions = np.array([front, 1 - front])
        self.track = np.array([p["tf"], p["tr"]])
        self.static = np.repeat(self.mass * G / 2 * self.fractions, 2)
        self.aero = project.get("aero", AERO_DEFAULTS)
        unsprung = np.array([project["tires"][corner]["mu"] for corner in CORNERS]).reshape(2, 2).sum(axis=1)
        sprung = self.mass * self.fractions - unsprung
        if np.any(sprung <= 0):
            raise ValueError("各軸靜態質量須大於該軸簧下質量。")
        self.unsprung_moment_per_ay = unsprung * np.array([options["height_f"], options["height_r"]]) / 1000
        geometry = GeometryLookup(project)
        self.rc = np.array([geometry.value("h_rc_f"), geometry.value("h_rc_r")])
        self.geometry_moment_per_ay = sprung * self.rc
        self.roll_stiffness = suspension_roll_stiffness(project)
        self.total_roll_stiffness = float(sum(self.roll_stiffness))
        if self.total_roll_stiffness <= 0:
            raise ValueError("總側傾剛性須大於零。請檢查 Roll 與直推通道設定。")
        self.stiffness_share = self.roll_stiffness / self.total_roll_stiffness
        self.moment_per_ay = self.mass * self.height
        self.elastic_moment_per_ay = self.moment_per_ay - sum(self.unsprung_moment_per_ay) - sum(self.geometry_moment_per_ay)
        self.roll_gradient = math.degrees(G * self.elastic_moment_per_ay / self.total_roll_stiffness)
        unit_transfer = G * (self.unsprung_moment_per_ay + self.geometry_moment_per_ay
                             + self.elastic_moment_per_ay * self.stiffness_share) / self.track
        self.lift_limits_g = np.divide(self.static[::2], abs(unit_transfer), out=np.full(2, np.inf), where=unit_transfer != 0.)
        self.angle_samples = np.linspace(0., .5, 301)
        b, c, e = (options[key] for key in ("b", "c", "e"))
        argument = b * self.angle_samples
        curve = np.sin(c * np.arctan(argument - e * (argument - np.arctan(argument))))
        peak = np.argmax(curve)
        self.force_samples = curve[:peak + 1]
        self.angle_samples = self.angle_samples[:peak + 1]

    def at(self, ay_g):
        ay = ay_g * G
        speed_kmh = 3.6 * math.sqrt(abs(ay) * self.options["radius"])
        aero = downforce(self.aero, speed_kmh) if self.aero["enabled"] else 0.
        unsprung = ay * self.unsprung_moment_per_ay
        geometric = ay * self.geometry_moment_per_ay
        total_moment = ay * self.moment_per_ay
        elastic = ay * self.elastic_moment_per_ay
        paths = np.array([unsprung, geometric, elastic * self.stiffness_share])
        moments = paths.sum(axis=0)
        lateral = moments / self.track
        baseline = self.static + np.repeat(aero / 2 * self.fractions, 2)
        loads = baseline + wheel_deltas(lateral)
        demand = self.mass * ay * self.fractions
        capacity = self.tire_capacity(loads)
        axle_capacity = capacity.reshape(2, 2).sum(axis=1)
        axle_baseline = self.tire_capacity(baseline).reshape(2, 2).sum(axis=1)
        grip = axle_capacity / axle_baseline
        slips = [self.slip_angle(force, peak) for force, peak in zip(demand, axle_capacity)]
        lateral_forces = np.zeros(4)
        for axle, peak in enumerate(axle_capacity):
            if peak > 0:
                indices = slice(2 * axle, 2 * axle + 2)
                lateral_forces[indices] = demand[axle] * capacity[indices] / peak
        geometric_steer = math.copysign(math.degrees(math.atan(self.wheelbase / self.options["radius"])), ay) if ay else 0.
        steer = geometric_steer + slips[0] - slips[1] if all(value is not None for value in slips) else None
        return dict(loads=loads, baseline=baseline, paths=paths, lateral=lateral, speed_kmh=speed_kmh,
                    roll=math.degrees(elastic / self.total_roll_stiffness),
                    front_share=abs(lateral[0]) / sum(abs(lateral)) if np.any(lateral) else None,
                    front_moment_share=moments[0] / total_moment if total_moment else None,
                    moment_error=sum(moments) - total_moment, grip=grip, slips=slips, steer=steer,
                    forces=lateral_forces, understeer=(steer - geometric_steer) / ay_g if steer is not None and ay_g else None)

    def tire_capacity(self, loads):
        load = np.maximum(0., loads)
        o = self.options
        return o["mu"] * load * np.maximum(.05, 1 - o["sensitivity"] * (load / o["reference_load"] - 1))

    def slip_angle(self, demand, capacity):
        if demand == 0:
            return 0.
        if abs(demand) > capacity * self.force_samples[-1]:
            return None
        angle = np.interp(abs(demand) / capacity, self.force_samples, self.angle_samples)
        return math.copysign(math.degrees(angle), demand)


def suspension_roll_stiffness(project):
    # Use the actual channel maps and nonlinear forces at the no-aero static position.
    car, road = prepare_vehicle(project)
    inputs = road.at(0.)
    trim = car.equilibrium(inputs, include_aero=False)
    v = np.zeros(car.n)
    rotation = np.zeros(car.n)
    rotation[1] = 1e-6
    if car.flex:
        rotation[2] = rotation[1]
    before = car.forces(trim, v, inputs, True, include_aero=False)[3]
    after = car.forces(trim + rotation, v, inputs, True, include_aero=False)[3]
    slope = (after - before) / rotation[1]
    return (slope[1::2] - slope[::2]) * np.array([car.p["tf"], car.p["tr"]]) / 2
