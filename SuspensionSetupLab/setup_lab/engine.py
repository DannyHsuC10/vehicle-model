"""Full, half and quarter vehicles with work-conjugate nonlinear KC forces."""

from bisect import bisect_right
from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np

from .project import (AERO_DEFAULTS, AXES, CHANNELS, CORNERS, compile_project, downforce, model_channels,
                      model_corners)
from .road import Road
from .road_profiles import ISO_FREQUENCY_BAND

G = 9.81
# v13 Hydraulic_system and Modal_to_corner are the two directions of this map.
MODAL = np.array([[1, 1, 0, 0], [0, 0, 1, 1], [1, -1, 0, 0], [0, 0, 1, -1],
                  [1, -1, 1, -1], [1, -1, -1, 1], *np.eye(4)])


class Cancelled(Exception):
    pass


class Vehicle:
    def __init__(self, project, components):
        p = self.p = project["vehicle"]
        model = project["model"]
        self.corners = model_corners(model, project.get("model_sources", {}))
        self.nw = len(self.corners)
        self.axes = AXES[model]
        self.flex = model == "flex"
        lf, lr = p["lf"], p["lr"]
        self.wheelbase = lf + lr
        self.share = p["lr"] / self.wheelbase
        front, rear = self.share, 1 - self.share
        wheel_indices = [CORNERS.index(c) for c in self.corners]
        full_mu = np.array([project["tires"][c]["mu"] for c in CORNERS])
        mu = full_mu[wheel_indices]
        full_sprung = p["m"] - sum(full_mu)
        axle_shares = np.array([front, front, rear, rear])[wheel_indices]
        corner_sprung = full_sprung / 2 * axle_shares
        self.sprung = full_sprung if self.nw == 4 else sum(corner_sprung)
        self.total_mass = p["m"] if self.nw == 4 else self.sprung + sum(mu)
        half_f, half_r = p["tf"] / 2, p["tr"] / 2
        if model == "quarter":
            self.B = np.ones((1, 1))
            inertia = []
        elif model == "half_lr":
            is_front = self.corners[0].startswith("F")
            half_track = half_f if is_front else half_r
            self.B = np.array([[1, half_track], [1, -half_track]])
            inertia = [p["Ixx"] * (front if is_front else rear)]
            front, rear = (1., 0.) if is_front else (0., 1.)
        elif model == "half_fr":
            self.B = np.array([[1, lf], [1, -lr]])
            inertia = [p["Iyy"] / 2]
        elif self.flex:
            self.B = np.array([[1, half_f, 0, lf], [1, -half_f, 0, lf],
                               [1, 0, half_r, -lr], [1, 0, -half_r, -lr]])
            inertia = [p["Ixx"] * front, p["Ixx"] * rear, p["Iyy"]]
        else:
            self.B = np.array([[1, half_f, lf], [1, -half_f, lf], [1, half_r, -lr], [1, -half_r, -lr]])
            inertia = [p["Ixx"], p["Iyy"]]
        self.nb = 1 + len(inertia)
        self.mass = np.array([self.sprung, *inertia, *mu])
        self.n = self.mass.size
        self.travel = np.column_stack((-self.B, np.eye(self.nw)))
        self.modal = MODAL[:, wheel_indices]
        self.channel_travel = self.modal @ self.travel
        if model in ("quarter", "half_fr"):
            # A symmetric axle sees twice the one-wheel travel at the heave element.
            # Only the selected wheel receives its share of the axle force.
            self.channel_travel[:2] *= 2
        allowed = model_channels(model, self.corners)
        self.active = [(i, component) for i, (key, component) in enumerate(zip(CHANNELS, components))
                       if key in allowed and component.enabled]
        self.tire_k = np.array([project["tires"][c]["k"] * 1000 for c in self.corners])
        self.tire_c = np.array([project["tires"][c]["c"] for c in self.corners])
        self.weight = np.array([-G * self.sprung, *([0.] * (self.nb - 1)), *(-G * mu)])
        aero = project.get("aero", AERO_DEFAULTS)
        self.aero_downforce = downforce(aero, project["case"]["speed"] * 3.6) if aero["enabled"] else 0.
        corner_aero = self.aero_downforce / 2 * axle_shares
        self.aero_load = self.aero_downforce if self.nw == 4 else float(sum(corner_aero))
        self.nominal_load = G * (mu + corner_sprung) + corner_aero
        self.roll_split = np.array([front, rear])
        self.rc_transfer = (self.modal[2] * front * p["h_rc_f"] / p["tf"]
                            + self.modal[3] * rear * p["h_rc_r"] / p["tr"])
        # Models with pitch use one or two wheels per axle to carry the transfer.
        self.anti_scale = 1 / ((self.nw / 2) * self.wheelbase)

    def tire_load(self, q, v, height, road_velocity):
        penetration = np.maximum(0., height - q[self.nb:])
        # Fade viscous force into contact over the first millimetre of compression.
        contact = np.minimum(1., penetration / .001)
        return np.maximum(0., self.tire_k * penetration + contact * self.tire_c * (road_velocity - v[self.nb:]))

    def forces(self, q, v, inputs, details=False):
        height, road_velocity, ax, ay = inputs
        x, speed = self.channel_travel @ q, self.channel_travel @ v
        channel = np.zeros(len(CHANNELS))
        readings = np.zeros((len(CHANNELS), 6)) if details else None
        for i, component in self.active:
            reading = component.force(x[i], speed[i], details)
            if details:
                readings[i] = reading
                channel[i] = reading[0]
            else:
                channel[i] = reading
        suspension = self.modal.T @ channel
        tire = self.tire_load(q, v, height, road_velocity)
        p = self.p
        total = self.weight - self.travel.T @ suspension
        # Downforce acts vertically at the sprung centre of gravity.
        total[0] -= self.aero_load
        total[self.nb:] += tire
        twist_moment = 0.
        if self.flex:
            twist_moment = p["body_twist_k"] * (q[1] - q[2]) + p["body_twist_c"] * (v[1] - v[2])
            total[1] -= twist_moment
            total[2] += twist_moment

        # Constant RC/Anti transfer. The chassis receives the opposite reaction.
        if ay and self.axes[1]:
            fy = self.total_mass * ay
            moment = fy * p["h_cg"]
            if self.flex:
                total[1:3] += moment * self.roll_split
            else:
                total[1] += moment
            total += self.travel.T @ (fy * self.rc_transfer)
        if ax and self.axes[0]:
            moment = self.total_mass * ax * p["h_cg"]
            total[self.nb - 1] += moment
            anti_f = p["dive_front" if ax < 0 else "lift_front"]
            anti_r = p["lift_rear" if ax < 0 else "squat_rear"]
            geo = self.modal[0] * anti_f - self.modal[1] * anti_r
            total += self.travel.T @ (moment * self.anti_scale * geo)
        acceleration = total / self.mass
        return (acceleration, tire, readings, suspension, twist_moment) if details else acceleration

    def equilibrium(self, inputs):
        height, _, ax, ay = inputs
        zero_wheel, zero = np.zeros(self.nw), np.zeros(self.n)
        static_input = (height, zero_wheel, ax, ay)
        eps = 1e-7
        differences = np.eye(self.n) * eps

        def residual(q):
            return self.forces(q, zero, static_input) * self.mass

        def tangent(q):
            return np.column_stack([(residual(q + step) - residual(q - step)) / (2 * eps) for step in differences])

        for clearance in (.005, .025, .06, .12):
            q = np.zeros(self.n)
            q[self.nb:] = height - self.nominal_load / self.tire_k
            q[0] = np.mean(q[self.nb:]) - clearance
            for _ in range(40):
                force = residual(q)
                norm = np.linalg.norm(force)
                if norm < 1e-6:
                    jacobian = tangent(q)
                    stiffness = -(jacobian + jacobian.T) / 2
                    if np.linalg.eigvalsh(stiffness)[0] > 1e-7 and np.all(self.tire_load(q, zero, height, zero_wheel) > 0):
                        return q
                    break
                try:
                    direction = np.linalg.solve(tangent(q), -force)
                except np.linalg.LinAlgError:
                    break
                for scale in (1., .5, .25, .125, .0625, .03125):
                    candidate = q + scale * direction
                    if np.linalg.norm(residual(candidate)) < norm:
                        q = candidate
                        break
                else:
                    break
        raise ValueError("無法找到所有作用輪接地的穩定靜態平衡。請檢查承重彈簧、預壓、運動比及工況首筆資料。")


# Dormand–Prince 5(4) pair. The last stage also estimates the local error.
NODES = (0., 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1., 1.)
STAGES = np.zeros((7, 7))
for index, row in enumerate(((), (1 / 5,), (3 / 40, 9 / 40), (44 / 45, -56 / 15, 32 / 9),
                             (19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729),
                             (9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656),
                             (35 / 384, 0., 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84))):
    STAGES[index, :len(row)] = row
ERROR = STAGES[-1] - np.array([5179 / 57600, 0., 7571 / 16695, 393 / 640, -92097 / 339200, 187 / 2100, 1 / 40])


@dataclass
class Result:
    time: np.ndarray
    signals: dict
    groups: dict
    project: dict
    signature: tuple
    metrics: dict
    warnings: list
    steps: int
    corners: tuple
    motion: np.ndarray


def simulate(project, cancel=lambda: False, progress=lambda value: None, *, tolerance=1e-6):
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("積分容許誤差必須大於零。")
    project = deepcopy(project)
    vehicle = Vehicle(project, compile_project(project))
    road = Road(project["case"], vehicle.wheelbase, vehicle.corners, vehicle.axes)
    if cancel():
        raise Cancelled()
    equilibrium = vehicle.equilibrium(road.at(0.))
    n = vehicle.n
    dt = project["case"]["dt"]
    time = np.arange(math.ceil(road.duration / dt)) * dt
    time = np.append(time[time < road.duration - 1e-12], road.duration)
    history = np.zeros((len(time), 2 * n))
    state = np.zeros(2 * n)
    stages = np.empty((7, 2 * n))
    limit = .005
    if road.bump:
        limit = min(limit, project["case"]["period"] / 20)
    elif road.iso:
        limit = min(limit, 1 / (40 * ISO_FREQUENCY_BAND[1] * project["case"]["speed"]))
        if road.duration / limit > 600000:
            raise ValueError("ISO 路面需要超過 600,000 個積分步，請縮短模擬時間或降低車速。")
    step, attempts, now = min(dt, limit), 0, 0.

    for i, sample in enumerate(time[1:], 1):
        event_index = bisect_right(road.events, now + 1e-12)
        while now < sample - 1e-12:
            if cancel():
                raise Cancelled()
            end = min(sample, road.events[event_index]) if event_index < len(road.events) else sample
            h = min(step, end - now)
            attempts += 1
            if attempts > 600000 or h <= 1e-12:
                raise ValueError("積分無法達到誤差容許值。請檢查極端剛性、阻尼或路面資料。")
            last = now + h
            if road.iso and h == end - now and end in road.iso_onsets:
                # The road slope switches on at the event. Finish this step from the left.
                last = float(np.nextafter(end, now))
            for j, node in enumerate(NODES):
                trial = state + h * (STAGES[j, :j] @ stages[:j])
                stages[j, :n] = trial[n:]
                t = last if node == 1. else now + node * h
                stages[j, n:] = vehicle.forces(equilibrium + trial[:n], trial[n:], road.at(t))
            scale = tolerance * (0.01 + np.maximum(np.abs(state), np.abs(trial)))
            error = float(np.max(np.abs(h * (ERROR @ stages)) / scale))
            if not math.isfinite(error) or np.max(np.abs(trial)) > 1e5:
                raise ValueError("模擬數值發散，請檢查元件曲線與輸入。")
            if error <= 1.:
                now = end if end - now - h < 1e-12 else now + h
                state = trial
                if event_index < len(road.events) and now >= road.events[event_index] - 1e-12:
                    event_index += 1
            factor = 4. if error == 0. else min(4., max(.15, .9 * error**-.2))
            step = min(limit, h * factor)
        history[i] = state
        if i % 32 == 0:
            progress(.9 * i / (len(time) - 1))
    result = make_result(vehicle, road, project, equilibrium, time, history, attempts, cancel)
    progress(1.)
    return result


def make_result(vehicle, road, project, equilibrium, time, history, steps, cancel):
    nb, n = vehicle.nb, vehicle.n
    q, v = history[:, :n] + equilibrium, history[:, n:]
    acceleration = np.empty_like(q)
    loads = np.empty((len(time), vehicle.nw))
    readings = np.empty((len(time), len(CHANNELS), 6))
    suspension, roads = np.empty_like(loads), np.empty_like(loads)
    twist = np.empty(len(time))
    command = np.empty((len(time), 2))
    for i, t in enumerate(time):
        if i % 128 == 0 and cancel():
            raise Cancelled()
        inputs = road.at(t)
        roads[i], command[i] = inputs[0], inputs[2:]
        acceleration[i], loads[i], readings[i], suspension[i], twist[i] = vehicle.forces(q[i], v[i], inputs, True)

    signals, groups = {}, {}

    def add(group, label, unit, values):
        name = f"{label} [{unit}]"
        signals[name] = values
        groups.setdefault(group, []).append(name)

    add("車身", "升沉", "mm", history[:, 0] * 1000)
    add("車身", "升沉加速度", "m/s²", acceleration[:, 0])
    if vehicle.axes[0]:
        add("車身", "俯仰", "deg", np.rad2deg(history[:, nb - 1]))
    if vehicle.flex:
        add("車身", "前側傾", "deg", np.rad2deg(history[:, 1]))
        add("車身", "後側傾", "deg", np.rad2deg(history[:, 2]))
        add("車身", "車架扭角", "deg", np.rad2deg(q[:, 1] - q[:, 2]))
        add("車身", "車架扭矩", "N·m", twist)
    elif vehicle.axes[1]:
        add("車身", "側傾", "deg", np.rad2deg(history[:, 1]))
    for index, (active, label) in enumerate(zip(vehicle.axes, ("縱向加速度", "側向加速度"))):
        if active:
            add("輸入", label, "m/s²", command[:, index])
    add("空力", "全車下壓力", "N", np.full_like(time, vehicle.aero_downforce))
    if vehicle.nw < 4:
        add("空力", "模型承受下壓力", "N", np.full_like(time, vehicle.aero_load))
    travel = q @ vehicle.travel.T * 1000
    elapsed = np.diff(time)
    metrics = {}
    warnings = list(project["notes"])
    for i, corner in enumerate(vehicle.corners):
        add("輪荷", corner, "N", loads[:, i])
        add("輪端行程", f"{corner} 輪端行程", "mm", travel[:, i])
        add("輪端懸吊力", f"{corner} 輪端懸吊力", "N", suspension[:, i])
        add("輪端位移", corner, "mm", history[:, nb + i] * 1000)
        add("輸入", f"路面 {corner}", "mm", roads[:, i] * 1000)
        unloaded = loads[:, i] <= 1e-8
        air_time = np.sum((unloaded[:-1].astype(float) + unloaded[1:]) * elapsed / 2)
        metrics[corner] = (loads[0, i], np.min(loads[:, i]), np.max(loads[:, i]),
                           travel[0, i], np.min(travel[:, i]), np.max(travel[:, i]), air_time)
        if unloaded.any():
            warnings.append(f"{corner} 出現零輪荷，約 {air_time:.4f} s（依輸出取樣估算）。")
    if road.iso:
        groups["ISO 路面"] = [f"路面 {corner} [mm]" for corner in vehicle.corners]
    for i, component in vehicle.active:
        key = tuple(CHANNELS)[i]
        group = CHANNELS[key]
        labels = ("模態／輪端反力", "軸心行程", "軸心速度", "彈簧力", "阻尼力", "限位塊力")
        for j, (label, unit) in enumerate(zip(labels, ("N", "mm", "m/s", "N", "N", "N"))):
            add(group, f"{group} {label}", unit, readings[:, i, j])
        stroke = readings[:, i, 1]
        if (component.jounce_rate and np.any(stroke > component.jounce)
                or component.rebound_rate and np.any(stroke < -component.rebound)):
            warnings.append(f"{group} 觸及行程限位。")
    if nb > 1 and np.max(np.abs(q[:, 1:nb])) > .15:
        warnings.append("車身姿態超過約 8.6°，小角度幾何誤差可能增大。")
    if road.bump and any(start + project["case"]["period"] > road.duration for _, start in road.onsets):
        warnings.append("模擬時間不足以完整通過所有選定輪位的凸塊。")
    if road.iso and any(start >= road.duration for _, start in road.onsets):
        warnings.append("模擬結束前，部分選定輪位尚未進入 ISO 路面。")
    if road.maneuver and project["case"]["start"] + project["case"]["ramp"] > road.duration:
        warnings.append("模擬結束時，加速度仍未升至設定值。")
    heave_a2 = acceleration[:, 0]**2
    metrics["body_rms"] = math.sqrt(np.sum((heave_a2[1:] + heave_a2[:-1]) * elapsed / 2) / time[-1])
    signature = (project["model"], vehicle.corners, road.signature(), project["case"]["speed"])
    # Metres at the body attachment, wheel centre offset and road for each corner.
    motion = np.stack((q[:, :nb] @ vehicle.B.T, q[:, nb:], roads), axis=-1)
    return Result(time, signals, groups, project, signature, metrics, warnings, steps, vehicle.corners, motion)
