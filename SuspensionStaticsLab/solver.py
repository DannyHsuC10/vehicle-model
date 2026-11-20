"""Force and moment equilibrium on the original, undeformed geometry."""

from dataclasses import dataclass
from math import hypot, isfinite, pi, sqrt

import numpy as np

from .model import CHASSIS


@dataclass(frozen=True)
class Load:
    target: str
    xyz: tuple[float, float, float]
    end: str = ""  # Blank for a point force, A/B for a rod-end applied couple.


@dataclass(frozen=True)
class SectionProperties:
    area_mm2: float
    second_moment_mm4: float
    modulus_mm3: float
    gyration_mm: float
    polar_moment_mm4: float
    polar_modulus_mm3: float
    polar_gyration_mm: float


@dataclass(frozen=True)
class Section:
    outer_mm: float = 19.
    inner_mm: float = 15.
    young_gpa: float = 205.

    def area_mm2(self):
        if not (isfinite(self.outer_mm) and isfinite(self.inner_mm) and self.outer_mm > self.inner_mm >= 0.):
            raise ValueError("截面需要外徑 > 內徑 ≥ 0。")
        area = pi / 4. * (self.outer_mm - self.inner_mm) * (self.outer_mm + self.inner_mm)
        if not isfinite(area) or area <= 0.:
            raise ValueError("截面積超出數值範圍。")
        return area

    def young_mpa(self):
        modulus = self.young_gpa * 1000.
        if not isfinite(modulus) or modulus <= 0.:
            raise ValueError("楊氏模數必須為有限正數。")
        return modulus

    def properties(self):
        area = self.area_mm2()
        gyration = hypot(self.outer_mm, self.inner_mm) / 4.
        second_moment = area * gyration * gyration
        modulus = second_moment / (self.outer_mm / 2.)
        values = (second_moment, modulus, gyration, 2. * second_moment, 2. * modulus, sqrt(2.) * gyration)
        if not all(isfinite(value) and value > 0. for value in values):
            raise ValueError("截面性質超出數值範圍。")
        return SectionProperties(area, *values)


@dataclass(frozen=True)
class Equilibrium:
    forces: dict[str, float]
    lengths_mm: dict[str, float]
    support_force: tuple[float, float, float]
    support_moment: tuple[float, float, float]
    origin: str | None
    rank: int
    equations: int


@dataclass(frozen=True)
class MemberResult:
    force_n: float
    length_mm: float
    area_mm2: float
    stress_mpa: float
    strain: float
    change_mm: float


def unit_vector(a, b, label):
    delta = b - a
    length = hypot(*delta)
    if not isfinite(length) or length <= 1e-9:
        raise ValueError(f"{label} 的兩端太近，或座標超出數值範圍。")
    return delta / length, length


def solve(model, loads):
    if not model.rods:
        raise ValueError("目前沒有可求解的軸力桿件。")
    points = {name: np.asarray(xyz, dtype=float) for name, xyz in model.points.items()}
    rods = {rod.name: rod for rod in model.rods}
    owners = dict(model.owners)
    pins = set()
    for rod in model.rods:
        for point in (rod.a, rod.b):
            if point not in owners:
                node = (point,)  # Namespaced separately from rigid bodies.
                pins.add(node)
                owners[point] = node

    incident = {body: [] for body in (*model.bodies, *sorted(pins))}
    for point, body in owners.items():
        if body in incident:
            incident[body].append(points[point])
    for joint in model.joints:
        for body in (joint.body, joint.base):
            if body in incident:
                incident[body].append(points[joint.point])
                if joint.axis_end:
                    incident[body].append(points[joint.axis_end])

    rows, centers, scales = {}, {}, {}
    nrows = 0
    for body, positions in incident.items():
        width = 3 if body in pins else 6
        rows[body] = slice(nrows, nrows + width)
        nrows += width
        centers[body] = np.mean(positions, axis=0)
        scales[body] = max(.001, float(np.max(np.linalg.norm(positions - centers[body], axis=1))))

    def apply(vector, body, position, force, moment=None):
        if body == CHASSIS:
            return
        block = vector[rows[body]]
        block[:3] += force
        if body not in pins:
            torque = np.cross(position - centers[body], force)
            if moment is not None:
                torque += moment
            block[3:] += torque / scales[body]

    # A positive scalar pulls each endpoint toward the other endpoint.
    columns, lengths = [], {}
    for rod in model.rods:
        direction, length = unit_vector(points[rod.a], points[rod.b], rod.name)
        column = np.zeros(nrows)
        apply(column, owners[rod.a], points[rod.a], direction)
        apply(column, owners[rod.b], points[rod.b], -direction)
        columns.append(column)
        lengths[rod.name] = length * 1000.

    # A ball transmits three forces. A hinge adds two moment reactions,
    # leaving rotation about its own axis free in the equilibrium equations.
    for joint in model.joints:
        at = points[joint.point]
        for direction in np.eye(3):
            column = np.zeros(nrows)
            apply(column, joint.body, at, direction)
            apply(column, joint.base, at, -direction)
            columns.append(column)
        if joint.axis_end:
            axis, _ = unit_vector(at, points[joint.axis_end], f"{joint.body} 鉸鏈")
            normal = np.cross(axis, np.eye(3)[np.argmin(np.abs(axis))])
            normal /= np.linalg.norm(normal)
            for direction in (normal, np.cross(axis, normal)):
                column = np.zeros(nrows)
                moment = scales[joint.body] * direction
                apply(column, joint.body, at, np.zeros(3), moment)
                apply(column, joint.base, at, np.zeros(3), -moment)
                columns.append(column)

    origin = points[model.cg] if model.cg else np.zeros(3)
    net_force, net_moment = np.zeros(3), np.zeros(3)
    applied = np.zeros(nrows)
    for load in loads:
        xyz = np.asarray(load.xyz, dtype=float)
        if xyz.shape != (3,) or not np.isfinite(xyz).all():
            raise ValueError("外力或力矩需要三個有限的 XYZ 分量。")
        if load.end:
            rod = rods.get(load.target)
            if rod is None or load.end not in ("A", "B"):
                raise ValueError(f"找不到力矩目標 {load.target} 的桿端。")
            point = rod.a if load.end == "A" else rod.b
            if owners[point] in pins:
                raise ValueError(f"{point} 是只傳力的節點，請選擇連到剛體的桿端施加力矩。")
            apply(applied, owners[point], points[point], np.zeros(3), xyz)
            net_moment += xyz
        else:
            if load.target not in owners:
                raise ValueError(f"{load.target} 尚未連上受力模型，請選擇桿端或剛體上的硬點。")
            at = points[load.target]
            apply(applied, owners[load.target], at, xyz)
            net_force += xyz
            net_moment += np.cross(at - origin, xyz)

    matrix = np.column_stack(columns)
    if not all(np.isfinite(values).all() for values in (matrix, applied, net_force, net_moment)):
        raise ValueError("座標或外載超出可求解的數值範圍。")
    try:
        forces, _, rank, _ = np.linalg.lstsq(matrix, -applied, rcond=1e-10)
    except np.linalg.LinAlgError as exc:
        raise ValueError("平衡矩陣無法分解，請檢查硬點座標及桿件配置。") from exc
    if rank < len(columns):
        raise ValueError(f"內力無唯一解（矩陣秩 {rank}/{len(columns)}）。請檢查重複桿件、共線硬點或多餘約束。")
    residual = matrix @ forces + applied
    if not np.isfinite(forces).all() or np.max(np.abs(residual), initial=0.) > 1e-7 * max(1., np.max(np.abs(applied), initial=0.)):
        worst = max(rows, key=lambda body: np.linalg.norm(residual[rows[body]]))
        label = worst[0] if worst in pins else worst
        raise ValueError(f"{label} 無法平衡目前外載（矩陣秩 {rank}/{nrows}）。請檢查推桿、轉向拉桿與支承，或調整載重方向。")
    return Equilibrium(dict(zip(rods, map(float, forces))), lengths, tuple(-net_force), tuple(-net_moment), model.cg, rank, nrows)


def member_results(equilibrium, sections):
    results = {}
    for name, force in equilibrium.forces.items():
        section = sections[name]
        area = section.area_mm2()
        length = equilibrium.lengths_mm[name]
        stress = force / area
        strain = stress / section.young_mpa()
        change = strain * length
        if not all(isfinite(value) for value in (stress, strain, change)):
            raise ValueError(f"{name} 的應力、應變或變形量超出數值範圍。")
        results[name] = MemberResult(force, length, area, stress, strain, change)
    return results
