"""Wheel spindle orientation shared by the viewport and suspension analysis."""

from dataclasses import dataclass
from itertools import combinations

import numpy as np


@dataclass(frozen=True)
class TireSetup:
    width_mm: float = 190.5
    radius_mm: float = 205.74
    toe_deg: float | None = None
    camber_deg: float = 0.


def _unit(vector):
    length = np.linalg.norm(vector, axis=-1, keepdims=True)
    return np.divide(vector, length, out=np.full_like(vector, np.nan), where=length > 1e-12), length


def _basis(offsets):
    a, b = offsets[..., 0, :], offsets[..., 1, :]
    axis, a_length = _unit(a)
    normal, cross_length = _unit(np.cross(a, b))
    return np.stack((axis, np.cross(normal, axis), normal), axis=-2), a_length, cross_length


@dataclass(frozen=True)
class WheelFrame:
    center: int
    anchors: tuple[int, int] | None
    reference_normal: np.ndarray
    local_normal: np.ndarray

    @classmethod
    def from_model(cls, model, corner, points, setup=None):
        center = points.get(f"{corner}_WC")
        if center is None:
            return None
        setup = setup or TireSetup()
        offsets = model.reference - model.reference[center]
        side = 1 if corner.endswith("L") else -1
        forward = points.get(f"{corner}_FORWARD")
        yaw = 0.
        if setup.toe_deg is not None:
            yaw = np.deg2rad(-side * setup.toe_deg)
        elif forward is not None:
            dx, dy = offsets[forward, :2]
            yaw = np.arctan2(dy, dx)
        tilt = np.deg2rad(-side * setup.camber_deg)
        normal = np.array([-np.sin(yaw) * np.cos(tilt), np.cos(yaw) * np.cos(tilt), np.sin(tilt)])
        candidates = [points[f"{corner}_{key}"] for key in ("UBJ", "LBJ", "TOE", "FORWARD")
                      if f"{corner}_{key}" in points]
        pair = max(combinations(candidates, 2),
                   key=lambda ij: np.linalg.norm(np.cross(offsets[ij[0]], offsets[ij[1]])), default=None)
        if pair is not None:
            basis, _, _ = _basis(offsets[list(pair)])
            if np.isfinite(basis).all():
                return cls(center, pair, normal, basis @ normal)
        return cls(center, None, normal, normal)

    def _offsets(self, values):
        return values[..., list(self.anchors), :] - values[..., self.center, None, :]

    def normal(self, position):
        if self.anchors is None:
            return np.broadcast_to(self.reference_normal, (*position.shape[:-2], 3))
        basis, _, _ = _basis(self._offsets(position))
        normal = self.local_normal @ basis
        return np.where(np.isfinite(normal), normal, self.reference_normal)

    def normal_rate(self, position, tangent):
        """Differentiate the orthonormal wheel basis along a displacement tangent."""
        offsets, rates = self._offsets(position), self._offsets(tangent)
        basis, a_length, cross_length = _basis(offsets)
        axis, normal = basis[..., 0, :], basis[..., 2, :]
        a, b = offsets[..., 0, :], offsets[..., 1, :]
        da, db = rates[..., 0, :], rates[..., 1, :]

        def unit_rate(unit, length, rate):
            perpendicular = rate - np.sum(unit * rate, axis=-1, keepdims=True) * unit
            return np.divide(perpendicular, length, out=np.full_like(rate, np.nan), where=length > 1e-12)

        d_axis = unit_rate(axis, a_length, da)
        d_normal = unit_rate(normal, cross_length, np.cross(da, b) + np.cross(a, db))
        d_second = np.cross(d_normal, axis) + np.cross(normal, d_axis)
        return self.local_normal @ np.stack((d_axis, d_second, d_normal), axis=-2)
