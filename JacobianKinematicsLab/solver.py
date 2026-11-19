"""Solve rigid distances, body attachments, and displacement drives in SI units."""

from dataclasses import dataclass
from math import ceil

import numpy as np


class Unreachable(ValueError):
    """The geometry or the requested motion cannot be solved on its current branch."""

    def __init__(self, reason, *, sample=None, time=None):
        self.reason = reason
        self.sample = sample
        super().__init__(reason if time is None else f"t = {time:.6g} s：{reason}")


@dataclass(frozen=True)
class MotionResult:
    """Kinematic history with node displacement relative to model.reference (m)."""

    point_names: tuple[str, ...]
    link_names: tuple[str, ...]
    drive_names: tuple[str, ...]
    time: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    azimuth_deg: np.ndarray
    elevation_deg: np.ndarray
    angle_deg: np.ndarray
    angular_velocity: np.ndarray
    angular_acceleration: np.ndarray
    condition: np.ndarray
    sensitivity: np.ndarray
    displacement: np.ndarray | None = None
    internal_points: tuple[str, ...] = ()


_RANK_TOLERANCE = 1e-7
_DERIVATIVE_TOLERANCE = 5e-8
_MAX_CONTINUATION_STEPS = 1000


def _max_abs(values):
    return np.max(np.abs(values), initial=0.0)


def _unwrap_azimuth(azimuth):
    for series in azimuth.T:
        edges = np.flatnonzero(np.diff(np.r_[False, np.isfinite(series), False]))
        for start, stop in edges.reshape(-1, 2):
            series[start:stop] = np.unwrap(series[start:stop])
    return np.degrees(azimuth)


class _Attachments:
    """A marked point carried by three noncollinear anchors of one rigid body."""

    def __init__(self, model, point_index, reference):
        attachments = tuple(getattr(model, "attachments", ()))
        names = [(entry.point, entry.a, entry.b, entry.c) for entry in attachments]
        if any(len(set(group)) != 4 for group in names):
            raise Unreachable("剛體附著點及其三個基準點須各自不同。")
        indices = np.array([[point_index[name] for name in group] for group in names], dtype=int)
        self.point, self.a, self.b, self.c = indices.reshape(-1, 4).T
        self.rows = np.arange(3 * len(attachments)).reshape(-1, 3)
        self.u, self.v = self.anchor_vectors(reference)
        span = np.max(np.abs(np.column_stack((self.u, self.v))), axis=1)
        self.require_valid((span > 0.) & np.isfinite(span), names)
        u, v = self.u / span[:, None], self.v / span[:, None]
        basis = np.stack((u, v, np.cross(u, v)), axis=2)
        singular = np.linalg.svd(basis, compute_uv=False)
        self.require_valid(singular[:, -1] > _RANK_TOLERANCE * singular[:, 0], names)
        offset = (reference[self.point] - reference[self.a]) / span[:, None]
        weights = np.linalg.solve(basis, offset[..., None])[..., 0]
        self.along_b = weights[:, 0:1]
        self.along_c = weights[:, 1:2]
        self.normal = weights[:, 2:3] / span[:, None]
        self.require_valid(np.isfinite(weights).all(axis=1) & np.isfinite(self.normal[:, 0]), names)

    @staticmethod
    def require_valid(valid, names):
        if not np.all(valid):
            point = names[np.flatnonzero(~valid)[0]][0]
            raise Unreachable(f"剛體附著點 {point} 的三個基準點共線或座標範圍無效。")

    @property
    def row_count(self):
        return self.rows.size

    def anchor_vectors(self, points):
        return points[self.b] - points[self.a], points[self.c] - points[self.a]

    def residual(self, displacement):
        du, dv = self.anchor_vectors(displacement)
        # Expand only the displacement from the reference cross product to avoid cancellation.
        cross_change = np.cross(self.u, dv) + np.cross(du, self.v + dv)
        residual = (displacement[self.point] - displacement[self.a]
                    - self.along_b * du - self.along_c * dv - self.normal * cross_change)
        return residual.ravel()

    def fill_jacobian(self, rows, displacement):
        du, dv = self.anchor_vectors(displacement)
        eye = np.eye(3)
        cross_b = np.cross(eye, (self.v + dv)[:, None, :]).swapaxes(1, 2)
        cross_c = np.cross((self.u + du)[:, None, :], eye).swapaxes(1, 2)
        grad_b = -self.along_b[:, None] * eye - self.normal[:, None] * cross_b
        grad_c = -self.along_c[:, None] * eye - self.normal[:, None] * cross_c
        rows[self.rows, self.point[:, None]] = eye
        rows[self.rows, self.a[:, None]] = -eye - grad_b - grad_c
        rows[self.rows, self.b[:, None]] = grad_b
        rows[self.rows, self.c[:, None]] = grad_c

    def acceleration_rhs(self, velocity):
        du, dv = self.anchor_vectors(velocity)
        return (2. * self.normal * np.cross(du, dv)).ravel()


class _Mechanism:
    def __init__(self, model):
        self.reference = np.asarray(model.reference, dtype=float)
        point_index = {name: index for index, name in enumerate(model.point_names)}
        fixed = set(model.fixed)
        self.moving = np.array([index for name, index in point_index.items() if name not in fixed], dtype=int)
        self.n_coordinates = 3 * len(self.moving)
        self.all_a = np.array([point_index[link.a] for link in model.links], dtype=int)
        self.all_b = np.array([point_index[link.b] for link in model.links], dtype=int)
        active = np.array([getattr(link, "constrained", True) and (link.a not in fixed or link.b not in fixed)
                           for link in model.links], dtype=bool)
        self.a, self.b = self.all_a[active], self.all_b[active]
        self.reference_axes = self.reference[self.all_b] - self.reference[self.all_a]
        self.axes = self.reference_axes[active]
        all_lengths = np.linalg.norm(self.reference_axes, axis=1)
        if np.any((all_lengths == 0) | ~np.isfinite(all_lengths)):
            raise Unreachable("固定長度桿件的端點重合或距離無效。")
        self.lengths = all_lengths[active]
        scale = float(np.median(self.lengths)) if self.lengths.size else 1.0
        self.position_tolerance = 2e-11 * scale
        self.continuation_step = 0.02 * scale

        self.n_links = len(self.a)
        self.attachments = _Attachments(model, point_index, self.reference)
        self.n_drives = len(model.drives)
        self.drive_start = self.n_links + self.attachments.row_count
        self.link_rows = np.arange(self.n_links)
        self.jacobian_rows = np.zeros((self.drive_start + self.n_drives, len(self.reference), 3))
        for row, drive in enumerate(model.drives, start=self.drive_start):
            self.jacobian_rows[row, point_index[drive.point]] += drive.direction
            if drive.b is not None:
                self.jacobian_rows[row, point_index[drive.b]] -= drive.direction
        self.drive_rows = self.jacobian_rows[self.drive_start:, self.moving].reshape(self.n_drives, self.n_coordinates)
        self.drive_rhs = np.zeros((len(self.jacobian_rows), self.n_drives))
        self.drive_rhs[self.drive_start:] = np.eye(self.n_drives)

    def _node_displacement(self, coordinates):
        displacement = np.zeros_like(self.reference)
        displacement[self.moving] = coordinates.reshape(-1, 3)
        return displacement

    def _equations(self, coordinates, drives):
        displacement = self._node_displacement(coordinates)
        change = displacement[self.b] - displacement[self.a]
        # Expand (r₀ + Δr)² - r₀² to retain precision near the reference pose.
        distances = np.einsum("ij,ij->i", change, 2 * self.axes + change) / (2 * self.lengths)
        residual = np.concatenate((distances, self.attachments.residual(displacement),
                                   self.drive_rows @ coordinates - drives))
        return residual, self.axes + change, displacement

    def _jacobian(self, axes, displacement):
        rows = self.jacobian_rows.copy()
        gradient = axes / self.lengths[:, None]
        rows[self.link_rows, self.a] = -gradient
        rows[self.link_rows, self.b] = gradient
        self.attachments.fill_jacobian(rows[self.n_links:self.drive_start], displacement)
        return rows[:, self.moving].reshape(len(rows), self.n_coordinates)

    def _factor(self, jacobian, *, reference=False):
        if not self.n_coordinates:
            return 1.0, lambda rhs: rhs[:0]
        left, singular, right = np.linalg.svd(jacobian, full_matrices=False)
        rank = int(np.count_nonzero(singular > singular[0] * _RANK_TOLERANCE)) if singular.size else 0
        if rank < self.n_coordinates:
            missing = self.n_coordinates - rank
            if reference:
                raise Unreachable(f"參考姿態的 Jacobian 秩不足，{missing} 個座標方向未受約束或在奇異位置。"
                                  "請檢查固定點、桿件及驅動方向。")
            raise Unreachable("Jacobian 接近奇異，無法唯一決定此姿態的運動。")

        def linear_solve(rhs):
            projected = left.T @ rhs
            scaled = projected / (singular if rhs.ndim == 1 else singular[:, None])
            return right.T @ scaled

        return float(singular[0] / singular[-1]), linear_solve

    def _position_step(self, coordinates, drives):
        for _ in range(30):
            residual, axes, displacement = self._equations(coordinates, drives)
            jacobian = self._jacobian(axes, displacement)
            correction = np.linalg.lstsq(jacobian, -residual, rcond=_RANK_TOLERANCE)[0]
            if max(_max_abs(residual), _max_abs(correction)) <= self.position_tolerance:
                self._factor(jacobian)
                return coordinates
            error = np.linalg.norm(residual)
            for exponent in range(10):
                trial = coordinates + correction * 0.5**exponent
                trial_residual, _, _ = self._equations(trial, drives)
                if np.linalg.norm(trial_residual) < error:
                    coordinates = trial
                    break
            else:
                break
        raise Unreachable("位置迭代未收斂，指定行程可能與桿長約束不相容。")

    def _advance_step(self, coordinates, start, end, remaining_splits=6):
        try:
            return self._position_step(coordinates, end)
        except Unreachable:
            if not remaining_splits:
                raise
            middle = start + (end - start) / 2
            coordinates = self._advance_step(coordinates, start, middle, remaining_splits - 1)
            return self._advance_step(coordinates, middle, end, remaining_splits - 1)

    def advance(self, coordinates, start, end):
        with np.errstate(over="ignore"):
            span = end - start
        distance = _max_abs(span)
        if distance == 0:
            return coordinates
        limit = self.continuation_step * _MAX_CONTINUATION_STEPS
        nonlinear = self.n_links or self.attachments.row_count
        if not np.isfinite(distance) or (nonlinear and distance > limit):
            raise Unreachable(f"單次驅動位移差超出續接求解範圍（{limit * 1000:.3g} mm）。"
                              "請縮短行程或增加時間採樣點。")
        steps = ceil(distance / self.continuation_step) if nonlinear else 1
        previous = start
        for step in range(1, steps + 1):
            target = end if step == steps else start + (step / steps) * span
            coordinates = self._advance_step(coordinates, previous, target)
            previous = target
        return coordinates

    @staticmethod
    def _require_consistent(jacobian, values, rhs, label):
        error = _max_abs(jacobian @ values - rhs)
        if not np.isfinite(error):
            raise Unreachable(f"{label}超出可計算範圍。")
        if error > _DERIVATIVE_TOLERANCE * _max_abs(rhs):
            raise Unreachable(f"指定{label}與剛體幾何約束不相容。")

    def state(self, coordinates, drive_position, drive_velocity, drive_acceleration):
        _, axes, displacement = self._equations(coordinates, drive_position)
        jacobian = self._jacobian(axes, displacement)
        condition, linear_solve = self._factor(jacobian)
        velocity_rhs = np.concatenate((np.zeros(self.drive_start), drive_velocity))
        tangent = linear_solve(np.column_stack((velocity_rhs, self.drive_rhs)))
        self._require_consistent(jacobian, tangent[:, 0], velocity_rhs, "速度")
        velocity = self._node_displacement(tangent[:, 0])

        relative_velocity = velocity[self.b] - velocity[self.a]
        curvature = np.einsum("ij,ij->i", relative_velocity, relative_velocity) / self.lengths
        acceleration_rhs = np.concatenate((-curvature, self.attachments.acceleration_rhs(velocity), drive_acceleration))
        if not np.isfinite(acceleration_rhs).all():
            raise Unreachable("加速度超出可計算範圍。")
        acceleration = linear_solve(acceleration_rhs)
        self._require_consistent(jacobian, acceleration, acceleration_rhs, "加速度")

        sensitivity = np.zeros((*self.reference.shape, self.n_drives))
        sensitivity[self.moving] = tangent[:, 1:].reshape(len(self.moving), 3, self.n_drives)
        # A redundant drive can be compatible on the actual path without allowing its own independent variation.
        sensitivity_error = np.max(np.abs(jacobian @ tangent[:, 1:] - self.drive_rhs), axis=0, initial=0.0)
        sensitivity[..., sensitivity_error > _DERIVATIVE_TOLERANCE] = np.nan
        return (displacement, velocity, self._node_displacement(acceleration),
                condition, sensitivity)

    def angular_motion(self, displacement, velocity, acceleration):
        axes = self.reference_axes + (displacement[:, self.all_b] - displacement[:, self.all_a])
        relative_velocity = velocity[:, self.all_b] - velocity[:, self.all_a]
        relative_acceleration = acceleration[:, self.all_b] - acceleration[:, self.all_a]
        length_squared = np.einsum("tli,tli->tl", axes, axes)
        horizontal = np.hypot(axes[..., 0], axes[..., 1])
        azimuth = np.arctan2(axes[..., 1], axes[..., 0])
        azimuth[horizontal <= 1e-12 * np.sqrt(length_squared)] = np.nan
        elevation = np.degrees(np.arctan2(axes[..., 2], horizontal))
        angle = np.degrees(np.arctan2(np.linalg.norm(np.cross(self.reference_axes, axes), axis=2),
                                       np.einsum("li,tli->tl", self.reference_axes, axes)))
        omega = np.cross(axes, relative_velocity) / length_squared[..., None]
        length_term = 2 * np.einsum("tli,tli->tl", axes, relative_velocity) / length_squared
        alpha = np.cross(axes, relative_acceleration) / length_squared[..., None] - length_term[..., None] * omega
        return _unwrap_azimuth(azimuth), elevation, angle, omega, alpha


def solve(model, motion):
    """Follow the reference assembly branch and analytically differentiate its constraint Jacobian."""
    mechanism = _Mechanism(model)
    time = np.asarray(motion.time, dtype=float)
    drive_position, drive_velocity, drive_acceleration = (
        np.asarray(series, dtype=float) for series in (motion.displacement, motion.velocity, motion.acceleration)
    )
    if time.ndim != 1 or not len(time) or not np.isfinite(time).all() or np.any(np.diff(time) <= 0):
        raise ValueError("時間須為有限、嚴格遞增的數列。")
    drive_shape = (len(time), mechanism.n_drives)
    if any(series.shape != drive_shape or not np.isfinite(series).all()
           for series in (drive_position, drive_velocity, drive_acceleration)):
        raise ValueError("各驅動須為每個時間點提供有限的位移、速度及加速度。")

    coordinates = np.zeros(mechanism.n_coordinates)
    previous_drive = np.zeros(mechanism.n_drives)
    mechanism._factor(mechanism._jacobian(mechanism.axes, np.zeros_like(mechanism.reference)), reference=True)
    node_shape = (len(time), *mechanism.reference.shape)
    displacement, velocity, acceleration = (np.empty(node_shape) for _ in range(3))
    condition = np.empty(len(time))
    sensitivity = np.empty((*node_shape, mechanism.n_drives))
    for index, sample_time in enumerate(time):
        target = drive_position[index]
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                coordinates = mechanism.advance(coordinates, previous_drive, target)
                (displacement[index], velocity[index], acceleration[index], condition[index], sensitivity[index]) = (
                    mechanism.state(coordinates, target, drive_velocity[index], drive_acceleration[index])
                )
        except FloatingPointError as error:
            raise Unreachable("輸入或運動超出可計算範圍。", sample=index, time=sample_time) from error
        except (Unreachable, np.linalg.LinAlgError) as error:
            raise Unreachable(str(error), sample=index, time=sample_time) from error
        previous_drive = target

    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            position = mechanism.reference + displacement
            azimuth, elevation, angle, omega, alpha = mechanism.angular_motion(displacement, velocity, acceleration)
    except FloatingPointError as error:
        raise Unreachable("節點位置或桿件角運動超出可計算範圍。") from error
    return MotionResult(tuple(model.point_names), tuple(link.name for link in model.links),
                        tuple(drive.name for drive in model.drives), time.copy(),
                        position, velocity, acceleration, azimuth, elevation, angle, omega, alpha, condition,
                        sensitivity, displacement=displacement, internal_points=getattr(model, "internal_points", ()))
