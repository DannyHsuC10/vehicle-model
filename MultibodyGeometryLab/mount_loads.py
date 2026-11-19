"""Recover the forces that the suspension applies at the chassis mounts."""

from dataclasses import dataclass

import numpy as np

from .deformable import DamperResult, DynamicsModel, EXTERNAL_INDEX, elastic_node_forces
from .elements import DAMPER_POINT
from .geometry import FIXED, LINKS
from .solver import LINK_END, LINK_START, N_FIXED, WHEEL, Unreachable, VehicleResult


@dataclass(frozen=True)
class ChassisMountLoads:
    """XYZ forces on one chassis corner, in N with ground +Z upward."""

    by_mount: dict[str, np.ndarray]
    total: np.ndarray


def compute_mount_loads(model: DynamicsModel, result: VehicleResult, dampers: dict[str, DamperResult],
                        body_acceleration: float, *, rotation: np.ndarray | None = None,
                        wheel_accelerations: dict[str, np.ndarray] | None = None) -> dict[str, ChassisMountLoads]:
    """Use constrained point dynamics to recover A-arm, rack and damper forces.

    When supplied, rotation maps chassis axes to ground axes and each wheel
    acceleration is the absolute hub acceleration in ground axes. Otherwise
    body_acceleration describes a vertical chassis translation added to the
    relative hub acceleration. The returned reaction vectors use ground axes.
    Unsprung mass is concentrated at each wheel center.
    """
    rotation = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    ground_z_in_chassis = rotation[2]
    loads = {}
    for corner in model.geometry.corners:
        name = corner.name
        pose, spec, damper = result.corners[name], model.corners[name], dampers[name]
        if damper.tire_force is None:
            raise ValueError(f"{name} · 計算車身接點力需要輪胎正向力。")

        required = np.zeros_like(pose.position)
        if wheel_accelerations is None:
            hub_acceleration = pose.acceleration[WHEEL]
            vertical_acceleration = body_acceleration + model.gravity
        else:
            hub_acceleration = rotation.T @ wheel_accelerations[name]
            vertical_acceleration = model.gravity
        required[WHEEL] = (spec.mass * hub_acceleration
                           + (spec.mass * vertical_acceleration - damper.tire_force) * ground_z_in_chassis)
        elastic = elastic_node_forces(model, name, pose, damper)
        required -= elastic[:EXTERNAL_INDEX]
        if model.constraint_layouts is None:
            start, end, lengths, expected_rank = LINK_START, LINK_END, np.asarray(corner.lengths), len(LINKS)
        else:
            layout = model.constraint_layouts[name]
            start, end, lengths, expected_rank = layout.start, layout.end, layout.lengths, layout.rank

        # J^T multiplies the physical link forces at the moving points. The
        # last Jacobian row is the selected travel coordinate, not a link.
        jacobian = pose.jacobian[:len(start)]
        targets = required[N_FIXED:].ravel()
        try:
            multipliers, _, rank, _ = np.linalg.lstsq(jacobian.T, targets, rcond=None)
        except np.linalg.LinAlgError as error:
            raise Unreachable(f"{name} · 無法求出車身接點力。") from error
        if rank != expected_rank or not np.isfinite(multipliers).all():
            raise Unreachable(f"{name} · 連桿約束退化，無法求出車身接點力。")
        if np.linalg.norm(jacobian.T @ multipliers - targets) > 1e-6 * max(1., np.linalg.norm(targets)):
            raise Unreachable(f"{name} · 車身接點反力無法滿足運動方程，請檢查元件與行程。")

        # The constraint gradient at each endpoint is (start - end) / L0,
        # with opposite signs at start and end. Its fixed endpoint force acts
        # on the chassis, so the two ends obey action and reaction.
        points = np.vstack((pose.position, damper.fixed_point))
        link_gradients = (points[start] - points[end]) / lengths[:, None]
        start_forces = multipliers[:, None] * link_gradients
        forces = elastic.copy()
        np.add.at(forces, start, start_forces)
        np.add.at(forces, end, -start_forces)
        fixed_forces = np.vstack((forces[:N_FIXED], forces[EXTERNAL_INDEX])) @ rotation.T
        fixed_forces.setflags(write=False)
        by_mount = dict(zip((*FIXED, DAMPER_POINT), fixed_forces))
        total = fixed_forces.sum(axis=0)
        total.setflags(write=False)
        loads[name] = ChassisMountLoads(by_mount, total)
    return loads
