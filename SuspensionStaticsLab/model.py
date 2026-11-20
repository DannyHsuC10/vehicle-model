"""Read hardpoints and describe axial load paths through rigid assemblies."""

import csv
import json
import math
from dataclasses import dataclass
from itertools import combinations
from pathlib import Path

import numpy as np


EXAMPLE = Path(__file__).with_name("examples") / "hardpoints.csv"
CORNERS = ("FL", "FR", "RL", "RR")
CHASSIS = "CHASSIS"
ALIASES = {
    "UFI": "UCA_F", "URI": "UCA_R", "LFI": "LCA_F", "LRI": "LCA_R",
    "WHEEL_CENTER": "WC", "ROD_IN": "RACK", "ROD_OUT": "TOE",
    "TIE_IN": "RACK", "TIE_OUT": "TOE", "TIE_ROD_IN": "RACK", "TIE_ROD_OUT": "TOE",
    "PUSH_OUT_LCA": "PUSH_OUT", "PUSHROD_IN": "PUSH_IN", "PUSHROD_OUT": "PUSH_OUT",
}
ARM_POINTS = {"UCA_F", "UCA_R", "LCA_F", "LCA_R", "UBJ", "LBJ"}
CORNER_POINTS = ARM_POINTS | {
    "RACK", "TOE", "WC", "CONTACT", "CONSTRUCTION_CONTACT", "FORWARD",
    "PUSH_IN", "PUSH_OUT", "PUSH_OUT_UPRIGHT", "PUSH_OUT_UCA",
    "ROCKER_A", "ROCKER_B", "DAMPER_IN", "DAMPER_BODY",
}


@dataclass(frozen=True)
class Rod:
    name: str
    a: str
    b: str


@dataclass(frozen=True)
class Joint:
    body: str
    base: str
    point: str
    axis_end: str | None = None


@dataclass(frozen=True)
class Wheel:
    corner: str
    center: str
    contact: str
    heading: str | None


@dataclass(frozen=True)
class Model:
    points: dict
    owners: dict
    bodies: tuple[str, ...]
    rods: tuple[Rod, ...]
    joints: tuple[Joint, ...] = ()
    rigid_edges: tuple[tuple[str, str], ...] = ()
    wheels: tuple[Wheel, ...] = ()
    cg: str | None = None
    cop: str | None = None
    notes: tuple[str, ...] = ()


def number(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label}需要有限數值。") from exc
    if isinstance(value, bool) or not math.isfinite(result):
        raise ValueError(f"{label}需要有限數值。")
    return result


def read_hardpoints(path):
    """Read the same m/mm named-point formats used by the kinematics lab."""
    with Path(path).open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle, strict=True)
        reader.fieldnames = [field.strip().lower() for field in (reader.fieldnames or ())]
        headers = set(reader.fieldnames)
        units = [unit for unit in ("m", "mm") if {f"{axis}_{unit}" for axis in "xyz"} <= headers]
        if len(headers) != len(reader.fieldnames) or len(units) != 1 or not headers & {"name", "point"}:
            raise ValueError("硬點 CSV 需要 name 或 point，以及完整的 x_m/y_m/z_m 或 x_mm/y_mm/z_mm。")
        unit = units[0]
        scale = .001 if unit == "mm" else 1.
        field = "name" if "name" in headers else "point"
        points = {}
        for row in reader:
            if None in row or None in row.values():
                raise ValueError(f"CSV 第 {reader.line_num} 列的欄數不一致。")
            name = row[field].strip()
            if not name:
                raise ValueError(f"CSV 第 {reader.line_num} 列缺少硬點名稱。")
            if field == "point" and row.get("corner", "").strip():
                name = f"{row['corner'].strip()}_{name}"
            if name in points:
                raise ValueError(f"CSV 有重複硬點 {name}。")
            points[name] = tuple(number(row[f"{axis}_{unit}"], f"{name} 的 {axis.upper()}") * scale for axis in "xyz")
    if not points:
        raise ValueError("CSV 沒有硬點資料。")
    return points


def wheel_points(points):
    local = {corner: {} for corner in CORNERS}
    prefixes = [name.upper().partition("_") for name in points]
    prefixed = any(corner in CORNERS and ALIASES.get(key, key) in ARM_POINTS | {"WC"}
                   for corner, _, key in prefixes)
    for name, (corner, _, key) in zip(points, prefixes):
        if corner not in local and not prefixed:
            corner, key = "FL", name.upper()
        key = ALIASES.get(key, key)
        if corner not in local or key not in CORNER_POINTS:
            continue
        if key in local[corner]:
            raise ValueError(f"{corner} 的 {key} 有重複定義。")
        local[corner][key] = name
    wheels = []
    for corner, parts in local.items():
        if "WC" not in parts:
            continue
        center = parts["WC"]
        contact = parts.get("CONTACT") or parts.get("CONSTRUCTION_CONTACT")
        if contact is None:
            x, y, z = points[center]
            contact = f"{corner}_CONTACT"
            points[contact] = (x, y, 0. if .1 <= z <= .45 else z - .196)
        wheels.append(Wheel(corner, center, contact, parts.get("FORWARD")))
    return local, tuple(wheels)


def build_model(hardpoints, topology=None):
    points = dict(hardpoints)
    corners, wheels = wheel_points(points)
    cg = next((name for name in points if name.upper() == "CG"), None)
    if cg is None and wheels:
        centers = np.array([points[wheel.center] for wheel in wheels])
        cg = "CG"
        points[cg] = (*centers.mean(axis=0)[:2], min(points[wheel.contact][2] for wheel in wheels) + .31)
    if topology is not None:
        return custom_model(points, topology, wheels, cg)

    owners = {cg: CHASSIS} if cg else {}
    cop = next((name for name in points if name.upper() == "COP"), None)
    if cop is None and len(wheels) == 4:
        contacts_xyz = np.asarray([points[wheel.contact] for wheel in wheels])
        x, y, _ = .45 * contacts_xyz[:2].mean(axis=0) + .55 * contacts_xyz[2:].mean(axis=0)
        cop = "COP"
        points[cop] = (x, y, points[cg][2])
    if cop:
        owners[cop] = CHASSIS
    rods, bodies, joints, edges, notes = [], [], [], [], []
    contacts = {wheel.corner: wheel.contact for wheel in wheels}
    for corner, local in corners.items():
        if not (ARM_POINTS & local.keys() or corner in contacts):
            continue
        upright = f"{corner}_UPRIGHT"
        assign(owners, upright, local, ("UBJ", "LBJ", "TOE", "WC", "FORWARD"))
        if corner in contacts:
            owners[contacts[corner]] = upright
        if upright in owners.values():
            bodies.append(upright)
        assign(owners, CHASSIS, local, ("UCA_F", "UCA_R", "LCA_F", "LCA_R", "RACK", "ROCKER_A", "ROCKER_B", "DAMPER_BODY"))
        draw_edges(edges, local, ("UBJ", "LBJ", "TOE", "WC"))
        if "WC" in local:
            edges.append((local["WC"], contacts[corner]))

        outboard = [key for key in ("PUSH_OUT", "PUSH_OUT_UCA", "PUSH_OUT_UPRIGHT") if key in local]
        if len(outboard) > 1:
            raise ValueError(f"{corner} 有多個推桿外端，請保留一個。")
        push = outboard[0] if outboard else None
        push_connected = "PUSH_IN" in local and push is not None
        for arm, ball, push_key in (("UCA", "UBJ", "PUSH_OUT_UCA"), ("LCA", "LBJ", "PUSH_OUT")):
            a, b = f"{arm}_F", f"{arm}_R"
            if push_connected and push == push_key and {a, b, ball} <= local.keys():
                body = f"{corner}_{arm}"
                bodies.append(body)
                owners[local[push]] = body
                joints.extend((Joint(body, CHASSIS, local[a], local[b]), Joint(body, upright, local[ball])))
                draw_edges(edges, local, (a, b, ball, push))
                notes.append(f"{body} 的推桿接在 A 臂中段，已用剛性 A 臂與鉸鏈表示。")
            else:
                for mount in (a, b):
                    add_rod(rods, corner, local, f"{mount}-{ball}", mount, ball)

        add_rod(rods, corner, local, "TIE_ROD", "RACK", "TOE")
        if push_connected and push == "PUSH_OUT_UPRIGHT":
            owners[local[push]] = upright
            draw_edges(edges, local, ("LBJ", "WC", push))
        if push_connected and local[push] in owners:
            add_rod(rods, corner, local, "PUSH_ROD", push, "PUSH_IN")
            rocker = ("ROCKER_A", "ROCKER_B", "DAMPER_IN", "DAMPER_BODY")
            if set(rocker) <= local.keys():
                body = f"{corner}_ROCKER"
                bodies.append(body)
                assign(owners, body, local, ("PUSH_IN", "DAMPER_IN"))
                joints.append(Joint(body, CHASSIS, local["ROCKER_A"], local["ROCKER_B"]))
                add_rod(rods, corner, local, "DAMPER", "DAMPER_IN", "DAMPER_BODY")
                draw_edges(edges, local, (*rocker[:3], "PUSH_IN"))
            else:
                owners[local["PUSH_IN"]] = CHASSIS
                draw_edges(edges, local, ("ROCKER_A", "ROCKER_B", "PUSH_IN"))
                notes.append(f"{corner} 的推桿內端固定於車身。提供搖臂軸與阻尼器兩端可改用鉸接搖臂。")
        else:
            notes.append(f"{corner} 無完整推桿受力路徑。求解時會檢查平衡與約束。")

    if not rods:
        notes.append("目前只有硬點座標。請使用標準懸吊點名，或載入自訂拓撲 JSON。")
    return Model(points, owners, tuple(bodies), tuple(rods), tuple(joints), tuple(dict.fromkeys(edges)), wheels, cg, cop, tuple(notes))


def assign(owners, body, local, keys):
    for key in keys:
        if key in local:
            owners[local[key]] = body


def add_rod(rods, corner, local, label, a, b):
    if a in local and b in local:
        rods.append(Rod(f"{corner}_{label}", local[a], local[b]))


def draw_edges(edges, local, keys):
    edges.extend(combinations((local[key] for key in keys if key in local), 2))


def read_topology(path):
    with Path(path).open(encoding="utf-8-sig") as handle:
        return json.load(handle)


def custom_model(points, data, wheels, cg):
    if not isinstance(data, dict):
        raise ValueError("拓撲 JSON 需要一個物件。")
    fixed, groups, members = data.get("fixed", []), data.get("bodies", {}), data.get("rods", [])
    if not isinstance(fixed, list) or not isinstance(groups, dict) or not isinstance(members, list):
        raise ValueError("拓撲請使用 fixed 陣列、bodies 物件及 rods 陣列。")
    if CHASSIS in groups or any(not name.strip() for name in groups):
        raise ValueError("自訂剛體須有名稱，CHASSIS 為保留名稱。")
    owners, edges = {}, []
    for body, names in ((CHASSIS, fixed), *groups.items()):
        if not isinstance(names, list) or (body != CHASSIS and not names):
            raise ValueError(f"{body} 需要硬點名稱陣列。")
        for name in names:
            if not isinstance(name, str) or name not in points or name in owners:
                raise ValueError(f"{body} 使用未知或重複指派的硬點 {name}。")
            owners[name] = body
        if body != CHASSIS:
            edges.extend(combinations(names, 2))
    rods = []
    labels = set()
    for member in members:
        if not isinstance(member, dict):
            raise ValueError("每支桿件需要 name、a、b 欄位。")
        label, a, b = (member.get(key) for key in ("name", "a", "b"))
        if not all(isinstance(value, str) and value.strip() for value in (label, a, b)):
            raise ValueError("每支桿件需要 name、a、b 欄位。")
        if label in labels or a == b or a not in points or b not in points:
            raise ValueError(f"桿件 {label} 使用重複名稱或無效端點。")
        rods.append(Rod(label, a, b))
        labels.add(label)
    cop = next((name for name in points if name.upper() == "COP"), None)
    return Model(points, owners, tuple(groups), tuple(rods), rigid_edges=tuple(edges), wheels=wheels, cg=cg, cop=cop)
