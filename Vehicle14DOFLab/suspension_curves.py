"""CSV schemas for spring, damper, and motion ratio curves."""

import csv
from pathlib import Path

from ._vendor.suspension.components import MAX_CURVE_POINTS, damper_points, points, spring_points
from ._vendor.suspension.project import DAMPER_FORCE_HEADER, MR_HEADER, SPRING_FORCE_HEADER


FORMATS = {
    ("spring", "stages"): (("travel_mm", "stiffness_n_per_mm"), ((0., 140.), (20., 250.))),
    ("spring", "force"): (SPRING_FORCE_HEADER, ((0., 0.), (10., 1400.), (20., 3200.))),
    ("damper", "stages"): (("speed_mps", "compression_ns_per_m", "rebound_ns_per_m"),
                           ((0., 1500., 2200.), (.20, 900., 1400.), (.40, 750., 1100.))),
    ("damper", "force"): (DAMPER_FORCE_HEADER, ((0., 0., 0.), (.2, 300., 440.), (.4, 480., 720.))),
    ("motion", "ratio"): (MR_HEADER, ((-20., .95), (0., 1.), (20., 1.05))),
}


def read_curve_csv(path, family):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream, strict=True)
        try:
            header = tuple(field.strip().lower() for field in next(reader, ()))
            kind = next((mode for (group, mode), (columns, _) in FORMATS.items()
                         if group == family and columns == header), None)
            if kind is None:
                choices = " 或 ".join(", ".join(columns) for (group, _), (columns, _) in FORMATS.items()
                                     if group == family)
                raise ValueError(f"CSV 第一列欄名須為 {choices}。")
            rows = []
            for row in reader:
                if not any(field.strip() for field in row):
                    continue
                if len(rows) >= MAX_CURVE_POINTS:
                    raise ValueError(f"曲線 CSV 最多 {MAX_CURVE_POINTS} 筆。")
                try:
                    if len(row) != len(header):
                        raise ValueError()
                    rows.append([float(field) for field in row])
                except ValueError as exc:
                    raise ValueError(f"第 {reader.line_num} 列須有 {len(header)} 欄數值。") from exc
        except csv.Error as exc:
            raise ValueError(f"第 {reader.line_num} 列的 CSV 格式有誤。") from exc

    stages = kind == "stages"
    if family == "spring":
        return kind, [list(row) for row in spring_points(rows, stages=stages)]
    if family == "motion":
        return kind, [list(row) for row in points(rows, "運動比", minimum_y=1e-6)]
    curves = tuple([list(row) for row in damper_points([[r[0], r[i]] for r in rows], label, stages=stages)]
                   for i, label in ((1, "壓縮阻尼"), (2, "回彈阻尼")))
    return kind, curves


def save_curve_template(path, family, kind):
    header, examples = FORMATS[family, kind]
    with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(examples)
