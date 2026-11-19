"""Editable two-point suspension elements and their project representation."""

from dataclasses import asdict, dataclass
from math import isfinite

from SuspensionGeometryLab.suspension_geometry_lab.hardpoints import CORNERS


DAMPER_POINT = "DAMPER"
PUSH_ROD = "Push Rod"
KINDS = ("rigid", "spring_damper")


@dataclass(frozen=True)
class Member:
    name: str
    a: str
    b: str
    kind: str
    stiffness_n_m: float | None = None
    damping_ns_m: float | None = None


def default_elements():
    from .geometry import LINKS

    rows = [Member(f"{a} → {b}", a, b, "rigid") for a, b in LINKS]
    rows.append(Member(PUSH_ROD, DAMPER_POINT, "UBJ", "spring_damper"))
    return {name: [asdict(row) for row in rows] for name in CORNERS}


def validate_elements(data):
    from .geometry import POINTS

    if not isinstance(data, dict) or set(data) != set(CORNERS):
        raise ValueError("機構元件必須包含 FL、FR、RL、RR 四個輪位。")
    points = (*POINTS, DAMPER_POINT)
    permitted = {"name", "a", "b", "kind", "stiffness_n_m", "damping_ns_m"}
    result = {}
    for corner, records in data.items():
        if not isinstance(records, list) or not 1 <= len(records) <= 64:
            raise ValueError(f"{corner} · 需要 1 至 64 個機構元件。")
        names = set()
        rows = []
        for record in records:
            if (not isinstance(record, dict) or not {"name", "a", "b", "kind"} <= record.keys()
                    or record.keys() - permitted):
                raise ValueError(f"{corner} · 元件欄位不完整或含未知設定。")
            member = Member(**record)
            if (not isinstance(member.name, str) or not member.name.strip()
                    or len(member.name) > 64 or member.name in names):
                raise ValueError(f"{corner} · 元件名稱須為 1 至 64 字，且不可重複。")
            names.add(member.name)
            if member.a not in points or member.b not in points or member.a == member.b:
                raise ValueError(f"{corner} · {member.name} 需要兩個不同的既有接點。")
            if member.kind not in KINDS:
                raise ValueError(f"{corner} · {member.name} 的元件類型無效。")
            if member.name == PUSH_ROD and {member.a, member.b} != {DAMPER_POINT, "UBJ"}:
                raise ValueError(f"{corner} · Push Rod 必須連接 DAMPER 與 UBJ，可變更其元件類型。")
            for value, low, high, label in ((member.stiffness_n_m, 100., 500_000., "剛性"),
                                            (member.damping_ns_m, 0., 5000., "阻尼")):
                if value is not None and (type(value) not in (int, float) or not isfinite(value)
                                          or not low <= value <= high):
                    raise ValueError(f"{corner} · {member.name} 的{label}須在 {low:g} 至 {high:g} 之間，或留空沿用輪位值。")
            rows.append(asdict(member))
        if PUSH_ROD not in names:
            raise ValueError(f"{corner} · 請保留 Push Rod 作為既有的 DAMPER–UBJ 線條。")
        result[corner] = rows
    return result
