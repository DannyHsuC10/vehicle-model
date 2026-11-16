"""Evaluate Roll Center heights with the selected half-car simulation."""

from dataclasses import dataclass

from .engine import Cancelled, Result, simulate
from .geometry_lookup import FIELDS, GeometryLookup
from .project import DEFAULT_SOURCES


OFFSETS_MM = range(-10, 11)


@dataclass
class RollCenterScan:
    axle: str
    center_mm: float
    lookup: bool
    heights: list[float]
    peaks: list[float | None]
    best_index: int
    best_result: Result
    failures: list[str]


def scan_roll_center(project, cancel, progress):
    axle = project.get("model_sources", {}).get("half_lr", DEFAULT_SOURCES["half_lr"])
    column = 1 if axle == "front" else 2
    _, _, key, _, units = FIELDS["roll_center"][column - 1]
    geometry = GeometryLookup(project)
    lookup = "roll_center" in geometry.active_kinds
    center = geometry.value(key) * units
    heights = [center + offset for offset in OFFSETS_MM]
    peaks, failures = [], []
    best = None
    for i, (offset, height) in enumerate(zip(OFFSETS_MM, heights)):
        if cancel():
            raise Cancelled()
        candidate = {**project, "vehicle": project["vehicle"].copy()}
        if lookup:
            tables = project["geometry_tables"].copy()
            rows = [row.copy() for row in tables["roll_center"]]
            for row in rows:
                row[column] += offset
            tables["roll_center"] = rows
            candidate["geometry_tables"] = tables
        else:
            candidate["vehicle"][key] = height / units
        try:
            result = simulate(candidate, cancel, lambda fraction: progress((i + fraction) / len(heights)))
        except ValueError as exc:
            failures.append(f"{height:g} mm：{exc}")
            peaks.append(None)
        else:
            peak = abs(float(result.metrics["load_transfer_rate"]["輪胎端"][0]))
            peaks.append(peak)
            score = (peak, abs(offset))
            if best is None or score < best[0]:
                best = score, i, result
        progress((i + 1) / len(heights))
    if cancel():
        raise Cancelled()
    if best is None:
        raise ValueError("21 個高度皆無法求解。\n" + failures[0])
    return RollCenterScan(axle, center, lookup, heights, peaks, best[1], best[2], failures)
