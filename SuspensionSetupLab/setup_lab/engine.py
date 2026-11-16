"""Full, half and quarter vehicles with nonlinear KC forces."""

from bisect import bisect_right
from copy import deepcopy
from dataclasses import dataclass, field
import math

import numpy as np

from .geometry_lookup import GeometryLookup, FIELDS as GEOMETRY_FIELDS
from .project import (AERO_DEFAULTS, AXES, CHANNELS, CORNERS, QUARTER_MODAL_CHANNELS, channel_enabled, compile_project, downforce,
                      model_channels, model_corners, project_roll_frame)
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
        self.geometry = GeometryLookup(project)
        self.geometry_reference_mm = np.zeros(2)
        self.axes = AXES[model]
        self.flex = model == "flex"
        kinds = self.geometry.active_kinds
        self.uses_geometry_lookup = (self.axes[0] and "anti" in kinds) or (self.axes[1] and "roll_center" in kinds)
        lf, lr = p["lf"], p["lr"]
        self.wheelbase = lf + lr
        self.share = p["lr"] / self.wheelbase
        front, rear = self.share, 1 - self.share
        wheel_indices = [CORNERS.index(c) for c in self.corners]
        self.axle_average = np.array([[float(index // 2 == axle) for index in wheel_indices] for axle in (0, 1)])
        self.axle_average /= np.maximum(1., self.axle_average.sum(axis=1, keepdims=True))
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
        self.wheels = slice(self.nb, self.nb + self.nw)
        self.mass = np.array([self.sprung, *inertia, *mu])
        self.travel = np.column_stack((-self.B, np.eye(self.nw)))
        self.roll_frame = None
        if model == "half_lr":
            frame = project_roll_frame(project)
            if frame["enabled"]:
                self.roll_frame = frame
                self.mass = np.append(self.mass, frame["mass"])
                self.travel = np.pad(self.travel, ((0, 0), (0, 1)))
        self.n = self.mass.size
        self.axle_travel = self.axle_average @ self.travel
        wheel_map = MODAL[:, wheel_indices]
        self.modal = wheel_map.copy()
        if model == "quarter":
            # A single wheel drives each axle mode at half travel. Apply that
            # same factor when projecting the mode's reaction back to the wheel.
            quarter_rows = [tuple(CHANNELS).index(key) for key in QUARTER_MODAL_CHANNELS]
            self.modal[quarter_rows] = .5 * np.abs(self.modal[quarter_rows])
        elif model in ("half_lr", "rigid", "flex"):
            # Heave uses half the sum and Roll half the difference, in both directions of the force map.
            self.modal[:4] *= .5
        self.channel_travel = self.modal @ self.travel
        self.wheel_force_map = self.modal.T.copy()
        if model == "half_fr":
            # Symmetric axle motion drives the shared Heave element at full travel;
            # the selected side receives half of its reaction.
            self.wheel_force_map[:, :2] *= .5
        allowed = model_channels(model, self.corners)
        self.active = [(i, component) for i, (key, component) in enumerate(zip(CHANNELS, components))
                       if key in allowed and component.enabled]
        self.tire_k = np.array([project["tires"][c]["k"] * 1000 for c in self.corners])
        self.tire_c = np.array([project["tires"][c]["c"] for c in self.corners])
        self.weight = np.zeros(self.n)
        self.weight[0], self.weight[self.wheels] = -G * self.sprung, -G * mu
        aero = project.get("aero", AERO_DEFAULTS)
        self.aero_downforce = downforce(aero, project["case"]["speed"] * 3.6) if aero["enabled"] else 0.
        self.corner_aero = self.aero_downforce / 2 * axle_shares
        self.aero_load = self.aero_downforce if self.nw == 4 else float(sum(self.corner_aero))
        self.nominal_load = G * (mu + corner_sprung)
        self.roll_split = np.array([front, rear])
        # Geometric load transfer uses the wheel directions without the spring linkage ratios.
        self.rc_front = wheel_map[2] * front / p["tf"]
        self.rc_rear = wheel_map[3] * rear / p["tr"]
        self.anti_front, self.anti_rear = wheel_map[:2]
        # Models with pitch use one or two wheels per axle to carry the transfer.
        self.anti_scale = 1 / ((self.nw / 2) * self.wheelbase)

    def tire_load(self, q, v, height, road_velocity):
        penetration = np.maximum(0., height - q[self.wheels])
        # Fade viscous force into contact over the first millimetre of compression.
        contact = np.minimum(1., penetration / .001)
        return np.maximum(0., self.tire_k * penetration + contact * self.tire_c * (road_velocity - v[self.wheels]))

    def roll_center_forces(self, ay, axle_travel):
        rc = (self.rc_front * self.geometry.value("h_rc_f", axle_travel)
              + self.rc_rear * self.geometry.value("h_rc_r", axle_travel))
        return (self.total_mass * ay) * rc

    def frame_reaction(self, displacement, velocity):
        return self.roll_frame["stiffness"] * displacement + self.roll_frame["damping"] * velocity

    def forces(self, q, v, inputs, details=False, *, include_aero=True, steady_frame=False):
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
        suspension = self.wheel_force_map @ channel
        tire = self.tire_load(q, v, height, road_velocity)
        p = self.p
        total = self.weight - self.travel.T @ suspension
        # Downforce acts vertically at the sprung centre of gravity.
        if include_aero:
            total[0] -= self.aero_load
        total[self.wheels] += tire
        twist_moment = 0.
        if self.flex:
            twist_moment = p["body_twist_k"] * (q[1] - q[2]) + p["body_twist_c"] * (v[1] - v[2])
            total[1] -= twist_moment
            total[2] += twist_moment

        # Geometry applies forces to the wheels and opposite reactions to the chassis.
        axle_travel = (0., 0.)
        if self.uses_geometry_lookup and (ax or ay):
            axle_travel = self.axle_travel @ q * 1000 - self.geometry_reference_mm
        if ay and self.axes[1]:
            fy = self.total_mass * ay
            moment = fy * p["h_cg"]
            if self.flex:
                total[1:3] += moment * self.roll_split
            else:
                total[1] += moment
        if self.axes[1] and (ay or self.roll_frame):
            rc = self.roll_center_forces(ay, axle_travel)
            if self.roll_frame:
                # As in lab_rollcenter, the body sees the commanded RC moment immediately.
                reaction = self.frame_reaction(q[-1], v[-1])
                total[-1] = rc[0] - reaction
                total[:self.nb] += self.travel[:, :self.nb].T @ rc
                total[self.wheels] += rc if steady_frame else (reaction, -reaction)
            else:
                total += self.travel.T @ rc
        if ax and self.axes[0]:
            moment = self.total_mass * ax * p["h_cg"]
            total[self.nb - 1] += moment
            anti_f = self.geometry.value("dive_front" if ax < 0 else "lift_front", axle_travel)
            anti_r = self.geometry.value("lift_rear" if ax < 0 else "squat_rear", axle_travel)
            geo = self.anti_front * anti_f - self.anti_rear * anti_r
            total += self.travel.T @ (moment * self.anti_scale * geo)
        acceleration = total / self.mass
        return (acceleration, tire, readings, suspension, twist_moment) if details else acceleration

    def equilibrium(self, inputs, *, include_aero=True):
        for _, component in self.active:
            component.set_static_reference(None)
        # Locate the stops and Tender transitions from the trim before applying downforce.
        height, speed, ax, ay = inputs
        reference_inputs = (height, speed, 0., 0.) if self.uses_geometry_lookup else inputs
        reference = self._solve_equilibrium(reference_inputs, include_aero=False)
        self.geometry_reference_mm = self.axle_travel @ reference * 1000
        self.tender_reference_wheel_mm = (self.travel @ reference) * 1000
        travel_mm = (self.channel_travel @ reference) * 1000
        for i, component in self.active:
            component.set_static_reference(component.motion_ratio.integrated(travel_mm[i]))
        if (include_aero and self.aero_load) or (self.uses_geometry_lookup and (ax or ay)):
            return self._solve_equilibrium(inputs, include_aero=include_aero, initial=reference)
        return reference

    def _solve_equilibrium(self, inputs, *, include_aero, initial=None):
        height, _, ax, ay = inputs
        zero_wheel, zero = np.zeros(self.nw), np.zeros(self.n)
        static_input = (height, zero_wheel, ax, ay)
        eps = 1e-7
        differences = np.eye(self.n) * eps

        def residual(q):
            return self.forces(q, zero, static_input, include_aero=include_aero, steady_frame=True) * self.mass

        def tangent(q):
            return np.column_stack([(residual(q + step) - residual(q - step)) / (2 * eps) for step in differences])

        nominal_load = self.nominal_load + self.corner_aero if include_aero else self.nominal_load
        guesses = [] if initial is None else [initial.copy()]
        for clearance in (.005, .025, .06, .12):
            q = np.zeros(self.n)
            q[self.wheels] = height - nominal_load / self.tire_k
            q[0] = np.mean(q[self.wheels]) - clearance
            guesses.append(q)
        for q in guesses:
            for _ in range(40):
                force = residual(q)
                norm = np.linalg.norm(force)
                if norm < 1e-6:
                    jacobian = tangent(q)
                    # The passive frame is in equilibrium at k*x = command; assess the vehicle at that limit.
                    physical = self.wheels.stop
                    static_jacobian = jacobian[:physical, :physical]
                    stiffness = -(static_jacobian + static_jacobian.T) / 2
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
class TenderSwitches:
    wheel_threshold_mm: float
    starts_bound: bool
    binding_times: np.ndarray
    release_times: np.ndarray
    wheel_reference_mm: float


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
    tender_switches: dict[str, TenderSwitches] = field(default_factory=dict)
    wheel_travel_limits: dict[str, dict[str, float]] = field(default_factory=dict)


def prepare_vehicle(project):
    vehicle = Vehicle(project, compile_project(project))
    road = Road(project["case"], vehicle.wheelbase, vehicle.corners, vehicle.axes)
    return vehicle, road


def simulate(project, cancel=lambda: False, progress=lambda value: None, *, tolerance=1e-6):
    project = deepcopy(project)
    vehicle, road = prepare_vehicle(project)
    if cancel():
        raise Cancelled()
    dt = project["case"]["dt"]
    if road.duration / dt > 30000:
        raise ValueError("動畫模式最多輸出 30,001 筆。請使用「純計算並匯出 CSV」，或增加輸出間隔。")
    equilibrium = vehicle.equilibrium(road.at(0.))
    time = np.fromiter(output_times(road.duration, dt), dtype=float)
    history = np.empty((len(time), 2 * vehicle.n))
    for i, (_, state, attempts) in enumerate(integrate_samples(vehicle, road, equilibrium, time, cancel, tolerance=tolerance)):
        history[i] = state
        if i % 32 == 0:
            progress(.9 * i / (len(time) - 1))
    result = make_result(vehicle, road, project, equilibrium, time, history, attempts, cancel)
    progress(1.)
    return result


def output_times(duration, dt):
    for i in range(math.ceil(duration / dt)):
        t = i * dt
        if t < duration - 1e-12:
            yield t
    yield duration


def integrate_samples(vehicle, road, equilibrium, time, cancel, *, tolerance=1e-6):
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("積分容許誤差必須大於零。")
    n = vehicle.n
    case = road.options
    dt = case["dt"]
    max_attempts = max(600000, 2 * math.ceil(road.duration / dt))
    state = np.zeros(2 * n)
    stages = np.empty((7, 2 * n))
    limit = .005
    if road.bump:
        limit = min(limit, case["period"] / 20)
    elif road.iso:
        limit = min(limit, 1 / (40 * ISO_FREQUENCY_BAND[1] * case["speed"]))
        if road.duration / limit > max_attempts:
            raise ValueError(f"ISO 路面需要超過 {max_attempts:,} 個積分步，請縮短模擬時間或降低車速。")
    step, attempts, now = min(dt, limit), 0, 0.

    for sample in time:
        event_index = bisect_right(road.events, now + 1e-12)
        while now < sample - 1e-12:
            if cancel():
                raise Cancelled()
            end = min(sample, road.events[event_index]) if event_index < len(road.events) else sample
            h = min(step, end - now)
            attempts += 1
            if attempts > max_attempts or h <= 1e-12:
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
        yield sample, state, attempts


def wheel_travel_threshold(mr, wheel_factor, shaft_threshold):
    # Positive motion ratio makes shaft travel monotonic in channel travel.
    low, high = sorted((shaft_threshold / max(mr.y), shaft_threshold / min(mr.y)))
    for _ in range(54):
        middle = low + (high - low) / 2
        if mr.integrated(middle) < shaft_threshold:
            low = middle
        else:
            high = middle
    return float((low + (high - low) / 2) / wheel_factor)


def tender_crossings(component, wheel_factor, time, stroke, wheel_reference_mm):
    shaft_threshold = component.tender_bind_stroke
    wheel_threshold = wheel_travel_threshold(component.motion_ratio, wheel_factor, shaft_threshold)
    distance = stroke - shaft_threshold
    if component.tender_after_static == 0.:
        distance[np.abs(distance) < 1e-7] = 0.
    bound = distance >= 0.
    before = np.flatnonzero(bound[:-1] != bound[1:])
    after = before + 1
    fraction = -distance[before] / (distance[after] - distance[before])
    crossings = time[before] + fraction * (time[after] - time[before])
    return TenderSwitches(wheel_threshold, bool(bound[0]), crossings[bound[after]], crossings[~bound[after]],
                          wheel_reference_mm)


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
    angles = [name for name in groups["車身"] if name.endswith("[deg]") and name != "車架扭角 [deg]"]
    if angles:
        groups["車身姿態"] = angles
    for index, (active, label) in enumerate(zip(vehicle.axes, ("縱向加速度", "側向加速度"))):
        if active:
            add("輸入", label, "m/s²", command[:, index])
    if vehicle.uses_geometry_lookup:
        geometry_travel = q @ vehicle.axle_travel.T * 1000 - vehicle.geometry_reference_mm
        for kind, supported in (("roll_center", vehicle.axes[1]), ("anti", vehicle.axes[0])):
            if supported and kind in vehicle.geometry.active_kinds:
                group, unit = ("Roll Center", "mm") if kind == "roll_center" else ("Anti", "%")
                for _, label, key, axle, units in GEOMETRY_FIELDS[kind]:
                    if vehicle.axle_average[axle].any():
                        values = [vehicle.geometry.value(key, travel) * units for travel in geometry_travel]
                        add(group, label, unit, np.array(values))
    add("空力", "全車下壓力", "N", np.full_like(time, vehicle.aero_downforce))
    if vehicle.nw < 4:
        add("空力", "模型承受下壓力", "N", np.full_like(time, vehicle.aero_load))
    travel = q @ vehicle.travel.T * 1000
    elapsed = np.diff(time)
    metrics = {}
    warnings = list(project["notes"])
    if project["model"] == "half_lr":
        # The RC force acts on the unsprung mass; the tire balances it in the opposite direction.
        axle_travel = geometry_travel if vehicle.uses_geometry_lookup else np.zeros((len(time), 2))
        geometry_force = np.array([vehicle.roll_center_forces(ay, stroke)
                                   for ay, stroke in zip(command[:, 1], axle_travel)])
        geometric = (geometry_force[:, 0] - geometry_force[:, 1]) / 2
        if vehicle.roll_frame:
            add("車架傳力", "Roll Center 指令", "N", geometric)
            geometric = vehicle.frame_reaction(q[:, -1], v[:, -1])
            add("車架傳力", "Roll Center 傳遞力", "N", geometric)
        elastic = (suspension[:, 1] - suspension[:, 0]) / 2
        tire = (loads[:, 1] - loads[:, 0]) / 2
        peaks = metrics["load_transfer_rate"] = {}
        for label, transfer in (("幾何", geometric), ("懸吊", elastic), ("路徑合計", geometric + elastic), ("輪胎端", tire)):
            change = transfer - transfer[0]
            rate = np.gradient(change, time)
            add("載重轉移量", f"{label}載重轉移量", "N", change)
            add("載重轉移速率", f"{label}載重轉移速率", "N/s", rate)
            peak = np.argmax(np.abs(rate))
            peaks[label] = (rate[peak], time[peak])
    if project["model"] == "quarter":
        modes = model_channels("quarter", vehicle.corners)[:-1]
        disabled = [CHANNELS[key] for key in modes if not channel_enabled(project["channels"][key], "quarter")]
        if disabled:
            warnings.append("單輪模型未計入已停用的通道：" + "、".join(disabled) + "。")
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
    tender_switches = {}
    wheel_travel_limits = {}
    travel_wheels = {}
    if project["model"] == "quarter":
        travel_wheels = dict.fromkeys(model_channels("quarter", vehicle.corners), 0)
    elif project["model"] == "half_fr":
        travel_wheels = {"heave_f": 0, "heave_r": 1}
    for i, component in vehicle.active:
        key = tuple(CHANNELS)[i]
        group = CHANNELS[key]
        labels = ("模態／輪端反力", "軸心行程", "軸心速度", "彈簧力", "阻尼力", "限位塊力")
        for j, (label, unit) in enumerate(zip(labels, ("N", "mm", "m/s", "N", "N", "N"))):
            add(group, f"{group} {label}", unit, readings[:, i, j])
        stroke = readings[:, i, 1]
        if key in travel_wheels:
            wheel = travel_wheels[key]
            # Use the displacement map. The half-car scales only the returned Heave force.
            wheel_factor = vehicle.modal[i, wheel]
            if component.spring_kind == "tender":
                tender_switches[key] = tender_crossings(component, wheel_factor, time, stroke,
                                                       float(vehicle.tender_reference_wheel_mm[wheel]))
            limits = {direction: wheel_travel_threshold(component.motion_ratio, wheel_factor, shaft)
                      for direction, rate, shaft in (("compression", component.jounce_rate, component.jounce),
                                                     ("rebound", component.rebound_rate, -component.rebound))
                      if rate}
            if limits:
                wheel_travel_limits[key] = limits
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
    motion = np.stack((q[:, :nb] @ vehicle.B.T, q[:, vehicle.wheels], roads), axis=-1)
    return Result(time, signals, groups, project, signature, metrics, warnings, steps, vehicle.corners, motion,
                  tender_switches, wheel_travel_limits)
