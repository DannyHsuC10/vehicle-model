"""Independent wheel travel sweeps with the remaining drive coordinates held fixed."""

from dataclasses import dataclass

import numpy as np

from .model import CORNERS, Motion
from .solver import Unreachable, solve
from .wheel_frame import WheelFrame


@dataclass(frozen=True)
class SuspensionSweep:
    travel_mm: np.ndarray
    rack_mm: tuple[float, ...]
    camber_deg: np.ndarray
    camber_gain: np.ndarray
    toe_deg: np.ndarray
    bump_steer_deg: np.ndarray
    bump_steer_rate: np.ndarray
    track_mm: np.ndarray
    wheelbase_mm: np.ndarray


@dataclass(frozen=True)
class FrontAxleSweep:
    rack_mm: np.ndarray
    travel_mm: tuple[float, float]
    yaw_deg: np.ndarray
    steer_deg: np.ndarray
    difference_deg: np.ndarray
    ackermann_pct: np.ndarray
    track_mm: float
    wheelbase_mm: float | None


def sweep_corners(model):
    if model is None or not model.standard_suspension:
        return ()
    return tuple(corner for corner in CORNERS if f"{corner}_WC" in model.point_names)


def has_steering(model, corner):
    return corner.startswith("F") and f"{corner}_RACK" in model.point_names and any(
        drive.name == "FRONT_RACK" and not drive.locked for drive in model.drives
    )


def has_front_axle(model):
    corners = sweep_corners(model)
    return all(corner in corners and has_steering(model, corner) for corner in ("FL", "FR"))


def _wheel_angles(frame, position, tangent, side):
    if frame.anchors is None:
        return np.full((4, len(position)), np.nan)
    x, y, z = frame.normal(position).T
    dx, dy, dz = frame.normal_rate(position, tangent).T
    horizontal = np.hypot(x, y)
    camber = np.arctan2(z, horizontal)
    toe = np.unwrap(np.arctan2(-x, y))
    with np.errstate(divide="ignore", invalid="ignore"):
        dh = (x * dx + y * dy) / horizontal
        camber_rate = (horizontal * dz - z * dh) / (horizontal**2 + z**2)
        toe_rate = (x * dy - y * dx) / horizontal**2
    undefined = horizontal <= 1e-12
    toe[undefined] = camber_rate[undefined] = toe_rate[undefined] = np.nan
    return -side * np.rad2deg((camber, camber_rate / 1000., toe, toe_rate / 1000.))


def _spacing(position, center, other, axis):
    if other is None:
        return np.full(len(position), np.nan)
    return 1000. * np.abs(position[:, center, axis] - position[:, other, axis])


def _sweep_samples(limits, samples, label):
    lower, upper = limits
    if not np.isfinite(limits).all() or lower >= upper:
        raise ValueError(f"{label}下限須小於上限，且兩者須為有限數值。")
    fraction = np.linspace(0., 1., samples)
    values = np.unique(lower * (1. - fraction) + upper * fraction)
    if lower < 0 < upper and 0. not in values:
        values = np.sort(np.r_[values, 0.])
    # Assemble at zero before continuing to the first displayed coordinate.
    path = np.r_[0., values[values != 0.]]
    order = np.where(values == 0., 0, np.searchsorted(path[1:], values) + 1)
    return values, path, order


def sweep_suspension(model, corner, travel_limits, rack_limits=(0., 0.), tires=None):
    """Sample wheel travel at up to three fixed rack displacements, all in mm."""
    if corner not in sweep_corners(model):
        raise ValueError("懸吊特性掃描需要標準懸吊模型及所選輪位的輪心 WC。")
    travel, path, order = _sweep_samples(travel_limits, 21, "輪跳")
    if not has_steering(model, corner):
        rack_limits = (0., 0.)
    left, right = rack_limits
    if not np.isfinite(rack_limits).all() or left > right:
        raise ValueError("齒條位移下限須小於或等於上限，且兩者須為有限數值。")
    rack_values = (left,) if left == right else (left, left / 2. + right / 2., right)
    points = {name: i for i, name in enumerate(model.point_names)}
    frame = WheelFrame.from_model(model, corner, points, (tires or {}).get(corner))
    opposite = "R" if corner.endswith("L") else "L"
    other_axle = "R" if corner.startswith("F") else "F"
    across = points.get(f"{corner[0]}{opposite}_WC")
    along = points.get(f"{other_axle}{corner[1]}_WC")
    side = 1 if corner.endswith("L") else -1
    drives = {drive.name: index for index, drive in enumerate(model.drives)}
    wheel_drive = drives[f"{corner}_WC_Z"]
    displacement = np.zeros((len(path), len(drives)))
    displacement[:, wheel_drive] = path / 1000.
    stationary = np.zeros_like(displacement)
    metrics = []
    for rack in rack_values:
        if "FRONT_RACK" in drives:
            displacement[:, drives["FRONT_RACK"]] = rack / 1000.
        try:
            result = solve(model, Motion(np.arange(len(path)), displacement, stationary, stationary))
        except Unreachable as error:
            coordinate = "" if error.sample is None else f"、輪跳 {path[error.sample]:g} mm"
            raise ValueError(f"齒條 {rack:g} mm{coordinate}：{error.reason}") from error
        camber, gain, toe, toe_rate = _wheel_angles(
            frame, result.position, result.sensitivity[..., wheel_drive], side
        )
        track = _spacing(result.position, frame.center, across, axis=1)
        wheelbase = _spacing(result.position, frame.center, along, axis=0)
        metrics.append(np.stack((camber, gain, toe, toe - toe[0], toe_rate, track, wheelbase))[:, order])
    return SuspensionSweep(travel, rack_values, *np.stack(metrics, axis=1))


def _ackermann_percent(steer_deg, track_mm, wheelbase_mm):
    left, right = steer_deg
    percent = np.full_like(left, np.nan)
    if wheelbase_mm is None:
        return percent
    # Both wheels must turn in the same direction. The inside wheel swaps with turn direction.
    valid = ((left * right > 0.) & (np.minimum(abs(left), abs(right)) > 1e-6)
             & (np.maximum(abs(left), abs(right)) < 90.))
    inner = np.where(left[valid] > 0., left[valid], -right[valid])
    outer = np.where(left[valid] > 0., right[valid], -left[valid])
    percent[valid] = (1. / np.tan(np.deg2rad(outer)) - 1. / np.tan(np.deg2rad(inner))) * (
        100. * wheelbase_mm / track_mm
    )
    return percent


def sweep_front_axle(model, heave_mm, roll_deg, rack_limits, wheelbase_mm=None, tires=None):
    """Sweep rack displacement at fixed front wheel travel set by heave and roll."""
    if not has_front_axle(model):
        raise ValueError("前軸掃描需要 FL、FR 兩輪的 WC、RACK 與 TOE。")
    if not np.isfinite(heave_mm) or not np.isfinite(roll_deg) or abs(roll_deg) >= 90.:
        raise ValueError("Heave 須為有限數值；Roll 須介於 −90° 與 90° 之間。")
    rack, path, order = _sweep_samples(rack_limits, 41, "齒條位移")
    points = {name: i for i, name in enumerate(model.point_names)}
    frames = tuple(WheelFrame.from_model(model, corner, points, (tires or {}).get(corner))
                   for corner in ("FL", "FR"))
    if any(frame.anchors is None for frame in frames):
        raise ValueError("前輪需要非共線的羊角硬點，以定義輪向。")
    front = model.reference[[frame.center for frame in frames]]
    track_mm = 1000. * abs(front[0, 1] - front[1, 1])
    if not np.isfinite(track_mm) or track_mm <= 1e-9:
        raise ValueError("前軸左右輪心需要有效的 Y 向間距。")
    rear_names = ("RL_WC", "RR_WC")
    if wheelbase_mm is None and all(name in points for name in rear_names):
        rear = model.reference[[points[name] for name in rear_names]]
        wheelbase_mm = 1000. * abs(front[:, 0].mean() - rear[:, 0].mean())
    if wheelbase_mm is not None and (not np.isfinite(wheelbase_mm) or wheelbase_mm <= 0.):
        raise ValueError("軸距須為大於零的有限數值。")
    roll_travel = .5 * track_mm * np.tan(np.deg2rad(roll_deg))
    travel = (heave_mm + roll_travel, heave_mm - roll_travel)
    drives = {drive.name: index for index, drive in enumerate(model.drives)}
    displacement = np.zeros((len(path), len(drives)))
    displacement[:, drives["FL_WC_Z"]] = travel[0] / 1000.
    displacement[:, drives["FR_WC_Z"]] = travel[1] / 1000.
    displacement[:, drives["FRONT_RACK"]] = path / 1000.
    stationary = np.zeros_like(displacement)
    try:
        result = solve(model, Motion(np.arange(len(path)), displacement, stationary, stationary))
    except Unreachable as error:
        coordinate = "" if error.sample is None else f"、齒條 {path[error.sample]:g} mm"
        raise ValueError(f"前左輪跳 {travel[0]:g} mm、前右輪跳 {travel[1]:g} mm{coordinate}：{error.reason}") from error

    yaw = []
    steer = []
    for frame in frames:
        x, y, _ = frame.normal(result.position).T
        angles = np.arctan2(-x, y)
        absolute = np.rad2deg(np.unwrap(np.r_[angles[0], angles[order]]))[1:]
        relative = absolute - np.rad2deg(angles[0])
        undefined = np.hypot(x, y) <= 1e-12
        absolute[undefined[order]] = np.nan
        relative[undefined[order] | undefined[0]] = np.nan
        yaw.append(absolute)
        steer.append(relative)
    steer = np.stack(steer)
    return FrontAxleSweep(rack, travel, np.stack(yaw), steer, steer[0] - steer[1],
                          _ackermann_percent(steer, track_mm, wheelbase_mm), track_mm, wheelbase_mm)
