"""Read LapSimLab suspension travel and solve the matching geometry poses."""
import csv
import math
from pathlib import Path

from .engine import Unreachable


LAP_PAGES = ("front", "rear", "sv")
TRAVEL_COLUMNS = tuple(f"zs_minus_zu_{corner}_mm" for corner in ("FL", "FR", "RL", "RR"))


def read_lap_travels(path):
    """Return four signed wheel-center travels per sample, positive in bump."""
    travels = []
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream, strict=True)
        header = reader.fieldnames or []
        invalid = [name for name in TRAVEL_COLUMNS if header.count(name) != 1]
        if invalid:
            raise ValueError("缺少或重複的相對位移欄位：" + "、".join(invalid))
        for row in reader:
            try:
                # LapSimLab stores sprung minus unsprung. A shorter distance
                # means that the wheel center has moved up relative to the body.
                travel = tuple(-float(row[name]) for name in TRAVEL_COLUMNS)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"CSV 第 {reader.line_num} 行的相對位移不是數字。") from exc
            if not all(math.isfinite(value) for value in travel):
                raise ValueError(f"CSV 第 {reader.line_num} 行的相對位移須為有限值。")
            travels.append(travel)
    if not travels:
        raise ValueError("CSV 沒有圈速模擬資料。")
    return tuple(travels)


def calculate_lap_metrics(engine, page, travels, cancelled=lambda: False):
    if page == "sv":
        solve = engine.solve_sv
    else:
        solve = engine.fv[page].solve
    rows = []
    for fl, fr, rl, rr in travels:
        if cancelled():
            return None
        if page == "front":
            pair = fl, fr
        elif page == "rear":
            pair = rl, rr
        else:
            # SV has one wheel per axle. Left/right differences belong to FV.
            pair = (fl + fr)/2, (rl + rr)/2
        try:
            rows.append(solve(*pair, wheel_center=True)["metrics"])
        except Unreachable:
            continue
    return {"rows": rows, "total": len(travels)}
