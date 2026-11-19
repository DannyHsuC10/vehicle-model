"""Projected virtual motion and CSV exports for solved kinematic histories."""

import csv
from dataclasses import dataclass
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .solver import MotionResult


@dataclass(frozen=True)
class MotionRatio:
    time_s: np.ndarray
    input_point: str
    output_point: str
    input_direction: np.ndarray
    output_direction: np.ndarray
    drive: str | None
    input_displacement: np.ndarray
    output_displacement: np.ndarray
    input_rate: np.ndarray
    output_rate: np.ndarray
    motion_ratio: np.ndarray
    mechanical_advantage: np.ndarray
    status: np.ndarray


def _direction(value):
    if isinstance(value, str):
        axis = value.strip().lower()
        name = axis.lstrip("+-")
        if name not in ("x", "y", "z") or axis not in (name, "+" + name, "-" + name):
            raise ValueError(f"方向 {value!r} 須為 X、Y、Z 其中一軸（可含正負號），或三維向量。")
        direction = np.zeros(3)
        direction["xyz".index(name)] = -1 if axis.startswith("-") else 1
        return direction
    try:
        direction = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("方向須為非零的有限三維向量。") from exc
    if direction.shape != (3,) or not np.all(np.isfinite(direction)) or not np.any(direction):
        raise ValueError("方向須為非零的有限三維向量。")
    direction = direction / np.max(np.abs(direction))
    return direction / np.linalg.norm(direction)


def motion_ratio(result: "MotionResult", input_point, output_point,
                 input_direction="z", output_direction="z", *, drive=None) -> MotionRatio:
    """Return signed ds_out/ds_in and magnitude |ds_in/ds_out|.

    The default tangent follows the solved time path. A named drive instead
    supplies its Jacobian tangent with every other drive coordinate held fixed.
    Displacement columns use the first solved sample as their origin.
    """
    points = tuple(result.point_names)
    for point in (input_point, output_point):
        if point not in points:
            raise ValueError(f"找不到節點 {point!r}。")
    a, b = points.index(input_point), points.index(output_point)
    direction_a, direction_b = _direction(input_direction), _direction(output_direction)

    tangent = result.velocity
    if drive is not None:
        drive_names = tuple(result.drive_names)
        if drive not in drive_names:
            raise ValueError(f"找不到驅動 {drive!r}。")
        tangent = result.sensitivity[..., drive_names.index(drive)]

    displacement = result.position - result.position[0]
    in_rate, out_rate = tangent[:, a] @ direction_a, tangent[:, b] @ direction_b
    tangent_scale = np.max(np.abs(tangent), axis=(1, 2))
    available = np.isfinite(tangent_scale)
    tolerance = np.maximum(tangent_scale * 1e-12, 1e-12)
    in_nonzero = available & (np.abs(in_rate) > tolerance)
    out_nonzero = available & (np.abs(out_rate) > tolerance)
    ratio = np.full(len(result.time), np.nan)
    advantage = np.full(len(result.time), np.nan)
    np.divide(np.where(out_nonzero, out_rate, 0.0), in_rate, out=ratio, where=in_nonzero)
    np.divide(np.where(in_nonzero, np.abs(in_rate), 0.0), np.abs(out_rate),
              out=advantage, where=out_nonzero)

    status = np.full(len(result.time), "ok", dtype="U24")
    status[~in_nonzero] = "input_projection_zero"
    status[~out_nonzero] = "output_projection_zero"
    status[~in_nonzero & ~out_nonzero] = "both_projections_zero"
    status[available & (tangent_scale <= tolerance)] = "stationary"
    status[~available] = "drive_unavailable"
    return MotionRatio(result.time, input_point, output_point, direction_a, direction_b, drive,
                       displacement[:, a] @ direction_a, displacement[:, b] @ direction_b,
                       in_rate, out_rate, ratio, advantage, status)


def _write_csv(path, columns, rows):
    temporary_path = None
    try:
        with NamedTemporaryFile(mode="w", encoding="utf-8-sig", newline="", dir=path.parent,
                                suffix=".tmp", delete=False) as handle:
            temporary_path = Path(handle.name)
            writer = csv.writer(handle)
            writer.writerow(columns)
            for row in rows:
                writer.writerow(None if isinstance(value, (float, np.floating)) and not np.isfinite(value)
                                else value for value in row)
        temporary_path.replace(path)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def export_results(directory, result: "MotionResult", ratio: MotionRatio | None = None) -> tuple[Path, ...]:
    """Write long-form point and rod histories, with an optional selected ratio."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    points = directory / "point_kinematics.csv"
    rods = directory / "rod_kinematics.csv"
    displacement = getattr(result, "displacement", None)
    internal_points = set(getattr(result, "internal_points", ()))
    nodes = [(j, point) for j, point in enumerate(result.point_names) if point not in internal_points]
    _write_csv(points, ("time_s", "point", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s",
                        "ax_m_s2", "ay_m_s2", "az_m_s2", "dx_m", "dy_m", "dz_m"),
               ((time, point, *result.position[i, j], *result.velocity[i, j], *result.acceleration[i, j],
                 *(displacement[i, j] if displacement is not None else (None, None, None)))
                for i, time in enumerate(result.time) for j, point in nodes))
    _write_csv(rods, ("time_s", "rod", "azimuth_deg", "elevation_deg", "angle_from_reference_deg",
                      "omega_x_rad_s", "omega_y_rad_s", "omega_z_rad_s",
                      "alpha_x_rad_s2", "alpha_y_rad_s2", "alpha_z_rad_s2"),
               ((time, rod, result.azimuth_deg[i, j], result.elevation_deg[i, j], result.angle_deg[i, j],
                 *result.angular_velocity[i, j], *result.angular_acceleration[i, j])
                for i, time in enumerate(result.time) for j, rod in enumerate(result.link_names)))
    paths = (points, rods)
    if ratio is not None:
        ratio_path = directory / "motion_ratio.csv"
        rate_units = "m_s" if ratio.drive is None else "m_per_m"
        _write_csv(ratio_path, ("time_s", "input_point", "output_point", "input_direction_x", "input_direction_y",
                               "input_direction_z", "output_direction_x", "output_direction_y", "output_direction_z",
                               "tangent", "input_displacement_m", "output_displacement_m", f"input_rate_{rate_units}",
                               f"output_rate_{rate_units}", "motion_ratio", "ideal_mechanical_advantage", "status"),
                   ((time, ratio.input_point, ratio.output_point, *ratio.input_direction, *ratio.output_direction,
                     ratio.drive if ratio.drive is not None else "time_path", ratio.input_displacement[i],
                     ratio.output_displacement[i], ratio.input_rate[i], ratio.output_rate[i],
                     ratio.motion_ratio[i], ratio.mechanical_advantage[i], ratio.status[i])
                    for i, time in enumerate(ratio.time_s)))
        paths += (ratio_path,)
    return paths
