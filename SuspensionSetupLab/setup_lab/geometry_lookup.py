"""One-dimensional geometry curves referenced to the no-aero static wheel travel."""

import csv
from pathlib import Path

from .components import Curve, MAX_CURVE_POINTS, number


# CSV column, display name, existing vehicle field, axle index, units per SI unit
FIELDS = {
    "roll_center": (
        ("front_rc_mm", "前 Roll Center", "h_rc_f", 0, 1000.),
        ("rear_rc_mm", "後 Roll Center", "h_rc_r", 1, 1000.),
    ),
    "anti": (
        ("front_anti_dive_pct", "煞車前 Anti-dive", "dive_front", 0, 100.),
        ("rear_anti_lift_pct", "煞車後 Anti-lift", "lift_rear", 1, 100.),
        ("front_anti_lift_pct", "加速前 Anti-lift", "lift_front", 0, 100.),
        ("rear_anti_squat_pct", "加速後 Anti-squat", "squat_rear", 1, 100.),
    ),
}
TITLES = {"roll_center": "Roll Center", "anti": "Anti"}


def csv_header(kind):
    return ("travel_mm", *(column for column, *_ in FIELDS[kind]))


def validate_tables(tables):
    if not isinstance(tables, dict) or tables.keys() - FIELDS.keys():
        raise ValueError("幾何查表設定格式有誤。")
    return {kind: validate_rows(kind, rows) for kind, rows in tables.items()}


def validate_rows(kind, rows):
    if not isinstance(rows, list) or not 1 <= len(rows) <= MAX_CURVE_POINTS:
        raise ValueError(f"{TITLES[kind]} 需要 1–{MAX_CURVE_POINTS} 筆資料。")
    previous = float("-inf")
    validated = []
    for i, row in enumerate(rows, 1):
        if not isinstance(row, list) or len(row) != len(csv_header(kind)):
            raise ValueError(f"{TITLES[kind]} 第 {i} 筆欄位數有誤。")
        travel = number(row[0], "輪端行程 (mm)", -1000., 1000.)
        if travel <= previous:
            raise ValueError(f"{TITLES[kind]} 的 travel_mm 必須嚴格遞增。")
        values = [number(value, label, -10 * units if kind == "roll_center" else -200.,
                         10 * units if kind == "roll_center" else 200.)
                  for value, (_, label, _, _, units) in zip(row[1:], FIELDS[kind])]
        validated.append([travel, *values])
        previous = travel
    return validated


def read_csv(path, kind):
    header = csv_header(kind)
    rows = []
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream, strict=True)
        try:
            if tuple(value.strip().lower() for value in next(reader, ())) != header:
                raise ValueError("CSV 欄名須依序為 " + ",".join(header))
            for row in reader:
                if not row or not any(value.strip() for value in row):
                    continue
                if len(rows) >= MAX_CURVE_POINTS:
                    raise ValueError(f"幾何查表最多 {MAX_CURVE_POINTS} 筆資料。")
                try:
                    rows.append([float(value) for value in row])
                except ValueError as exc:
                    raise ValueError(f"CSV 第 {reader.line_num} 列須填入數值。") from exc
        except csv.Error as exc:
            raise ValueError(f"CSV 第 {reader.line_num} 列格式有誤。") from exc
    return validate_rows(kind, rows)


class GeometryLookup:
    def __init__(self, project):
        self.fixed = project["vehicle"]
        self.curves = {}
        for kind, rows in project.get("geometry_tables", {}).items():
            for index, (_, _, key, axle, units) in enumerate(FIELDS[kind], 1):
                self.curves[key] = (axle, Curve([(row[0], row[index] / units) for row in rows]))

    def value(self, key, axle_travel=(0., 0.)):
        if key not in self.curves:
            return self.fixed[key]
        axle, curve = self.curves[key]
        return curve.lookup(axle_travel[axle])
