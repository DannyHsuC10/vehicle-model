"""Embedded suspension curves and rectangular interpolation grids."""

import csv
from pathlib import Path

import numpy as np

from ._vendor.suspension.geometry_lookup import FIELDS
from ._vendor.suspension.project import CORNERS


TITLES = {"roll_center": "Roll Center", "anti": "Anti", "camber": "Camber", "bump_steer": "Bump Steer"}
AXLE_AXES = (("travel_mm",), ("front_travel_mm", "rear_travel_mm"), ("travel_mm", "roll_deg"),
             ("travel_mm", "steer_deg"), ("steer_deg",))
WHEEL_AXES = (("travel_mm",), ("travel_mm", "steer_deg"), ("steer_deg",),
              ("travel_mm", "front_steer_deg"))
AXES = {
    "roll_center": AXLE_AXES, "anti": AXLE_AXES, "camber": WHEEL_AXES, "bump_steer": WHEEL_AXES,
}
OUTPUTS = {kind: tuple(field[0] for field in fields) for kind, fields in FIELDS.items()}
OUTPUTS["camber"] = tuple(f"{corner.lower()}_camber_delta_deg" for corner in CORNERS)
OUTPUTS["bump_steer"] = tuple(f"{corner.lower()}_bump_steer_deg" for corner in CORNERS)
DELTA_OUTPUTS = {
    "roll_center": ("front_rc_delta_mm", "rear_rc_delta_mm"),
    "anti": tuple(name.replace("_pct", "_delta_pct") for name in OUTPUTS["anti"]),
}


class GeometryTable:
    def __init__(self, kind, spec):
        self.axes = tuple(spec["axes"])
        if self.axes not in AXES[kind]:
            raise ValueError(f"{TITLES[kind]} 的查表軸不支援。")
        self.relative = spec.get("relative", False)
        if type(self.relative) is not bool or (self.relative and kind not in DELTA_OUTPUTS):
            raise ValueError(f"{TITLES[kind]} 的變化量表格格式有誤。")
        try:
            rows = np.asarray(spec["rows"], dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{TITLES[kind]} 的表格必須是數值。") from exc
        n = len(self.axes)
        if (rows.ndim != 2 or not 1 <= len(rows) <= 20000
                or rows.shape[1] != n + len(OUTPUTS[kind]) or not np.isfinite(rows).all()):
            raise ValueError(f"{TITLES[kind]} 需要 1–20,000 列且每個欄位皆須為有限數值。")
        bounds = {"roll_center": 10000., "anti": 200., "camber": 90., "bump_steer": 45.}
        if np.any(np.abs(rows[:, n:]) > bounds[kind]):
            raise ValueError(f"{TITLES[kind]} 數值超過 ±{bounds[kind]:g}。請確認 CSV 單位。")
        self.nodes = tuple(np.unique(rows[:, axis]) for axis in range(n))
        indices = tuple(np.searchsorted(nodes, rows[:, axis]) for axis, nodes in enumerate(self.nodes))
        shape = tuple(len(nodes) for nodes in self.nodes)
        unique_count = len(np.unique(rows[:, :n], axis=0))
        if unique_count != len(rows):
            raise ValueError(f"{TITLES[kind]} 有重複查表座標。")
        if n == 2 and (min(shape) < 2 or unique_count != shape[0] * shape[1]):
            raise ValueError(f"{TITLES[kind]} 曲面需要完整矩形網格，兩個軸都至少需要兩個座標。")
        self.values = np.empty((*shape, len(OUTPUTS[kind])))
        self.values[indices] = rows[:, n:]

    def sample(self, column, *coordinates):
        if len(self.axes) == 1:
            return np.interp(coordinates[0], self.nodes[0], self.values[:, column])
        a, b = self.nodes
        x, y = coordinates
        i = min(max(int(np.searchsorted(a, x)) - 1, 0), len(a) - 2)
        j = min(max(int(np.searchsorted(b, y)) - 1, 0), len(b) - 2)
        u = max(0., min(1., (x - a[i]) / (a[i + 1] - a[i])))
        v = max(0., min(1., (y - b[j]) / (b[j + 1] - b[j])))
        f = self.values
        return ((1. - u) * ((1. - v) * f[i, j, column] + v * f[i, j + 1, column])
                + u * ((1. - v) * f[i + 1, j, column] + v * f[i + 1, j + 1, column]))


def read_table(path, kind):
    path = Path(path)
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream, strict=True)
        try:
            header = tuple(text.strip().lower() for text in next(reader, ()))
            forms = [(axes, False, OUTPUTS[kind]) for axes in AXES[kind]]
            if kind in DELTA_OUTPUTS:
                forms.extend((axes, True, DELTA_OUTPUTS[kind]) for axes in AXES[kind])
            layout = next(((axes, relative) for axes, relative, outputs in forms if header == axes + outputs), None)
            if layout is None:
                raise ValueError("CSV 欄名格式不符，可由匯入視窗儲存範本。")
            axes, relative = layout
            rows = []
            for row in reader:
                if not any(field.strip() for field in row):
                    continue
                if len(row) != len(header) or len(rows) >= 20000:
                    raise ValueError(f"第 {reader.line_num} 列欄位數有誤，或超過 20,000 列。")
                try:
                    rows.append([float(field) for field in row])
                except ValueError as exc:
                    raise ValueError(f"第 {reader.line_num} 列須填數值。") from exc
        except csv.Error as exc:
            raise ValueError(f"第 {reader.line_num} 列 CSV 格式有誤。") from exc
    spec = {"axes": list(axes), "rows": rows, "source_name": path.name}
    if relative:
        spec["relative"] = True
    GeometryTable(kind, spec)
    return spec


def write_template(path, kind, axes, *, relative=False):
    from itertools import product

    coordinates = {"travel_mm": (-25., 0., 25.), "front_travel_mm": (-25., 0., 25.),
                   "rear_travel_mm": (-25., 0., 25.), "roll_deg": (-2., 0., 2.),
                   "steer_deg": (-15., 0., 15.), "front_steer_deg": (-15., 0., 15.)}
    samples = {"roll_center": (30., 40.), "anti": (15., 15., 15., 15.),
               "camber": (0., 0., 0., 0.), "bump_steer": (0., 0., 0., 0.)}
    with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        outputs = DELTA_OUTPUTS[kind] if relative else OUTPUTS[kind]
        writer.writerow((*axes, *outputs))
        values = (0.,) * len(outputs) if relative else samples[kind]
        for row in product(*(coordinates[axis] for axis in axes)):
            writer.writerow((*row, *values))
