"""Read hardpoints, define mechanism topology and prescribe timed motion."""

import csv
import json
import math
from dataclasses import dataclass
from itertools import combinations
from pathlib import Path

import numpy as np


EXAMPLES = Path(__file__).resolve().parent / "examples"
CORNERS = ("FL", "FR", "RL", "RR")
MOUNTS = ("UCA_F", "UCA_R", "LCA_F", "LCA_R")
JOINTS = ("UBJ", "LBJ")
ALIASES = {"UFI": "UCA_F", "URI": "UCA_R", "LFI": "LCA_F", "LRI": "LCA_R",
           "WHEEL_CENTER": "WC", "ROD_IN": "RACK", "ROD_OUT": "TOE", "TIE_IN": "RACK", "TIE_OUT": "TOE",
           "TIE_ROD_IN": "RACK", "TIE_ROD_OUT": "TOE", "PUSH_OUT_LCA": "PUSH_OUT",
           "PUSHROD_IN": "PUSH_IN", "PUSHROD_OUT": "PUSH_OUT"}
PUSH_OUTBOARD = ("PUSH_OUT", "PUSH_OUT_UCA", "PUSH_OUT_UPRIGHT")
PUSH_INBOARD = ("PUSH_IN", "ROCKER_A", "ROCKER_B")
CONSTRUCTION_POINTS = {"GROUND", "AXIS_TOP", "CONTACT", "CONSTRUCTION_CONTACT"}
PROFILES = ("pulse", "sine", "ramp")


@dataclass(frozen=True)
class Link:
    name: str
    a: str
    b: str
    constrained: bool = True


@dataclass(frozen=True)
class Attachment:
    point: str
    a: str
    b: str
    c: str


@dataclass(frozen=True)
class Drive:
    name: str
    point: str
    direction: tuple[float, float, float]
    b: str | None = None
    amplitude_mm: float = 0.
    profile: str = "pulse"
    locked: bool = False


@dataclass(frozen=True)
class ReferenceSettings:
    name: str
    source: str
    bump_mm: float
    droop_mm: float
    samples: int


@dataclass(frozen=True)
class Model:
    name: str
    point_names: tuple[str, ...]
    reference: np.ndarray
    fixed: tuple[str, ...]
    links: tuple[Link, ...]
    drives: tuple[Drive, ...]
    duration: float = 2.
    samples: int = 121
    attachments: tuple[Attachment, ...] = ()
    reference_settings: ReferenceSettings | None = None
    internal_points: tuple[str, ...] = ()
    standard_suspension: bool = False


@dataclass(frozen=True)
class Motion:
    time: np.ndarray
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


def _number(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label}需要有限數值。") from exc
    if isinstance(value, bool) or not math.isfinite(result):
        raise ValueError(f"{label}需要有限數值。")
    return result


def read_hardpoints(path):
    """Read one row per static XYZ point, returning an ordered mapping in metres."""
    with Path(path).open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle, strict=True)
        if not reader.fieldnames:
            raise ValueError("硬點 CSV 是空的。")
        reader.fieldnames = [name.strip().lower() for name in reader.fieldnames]
        headers = set(reader.fieldnames)
        if len(headers) != len(reader.fieldnames):
            raise ValueError("硬點 CSV 有重複欄名。")
        units = [unit for unit in ("m", "mm") if {f"{a}_{unit}" for a in "xyz"} <= headers]
        if len(units) != 1 or not ({"name", "point"} & headers):
            raise ValueError("硬點 CSV 需要 name（或 point）及整組 x_m/y_m/z_m 或 x_mm/y_mm/z_mm。")
        unit = units[0]
        scale = .001 if unit == "mm" else 1.
        point_field = "name" if "name" in headers else "point"
        result = {}
        for row in reader:
            if None in row or None in row.values():
                raise ValueError(f"硬點 CSV 第 {reader.line_num} 列的欄數不一致。")
            name = row[point_field].strip()
            if not name:
                raise ValueError(f"硬點 CSV 第 {reader.line_num} 列缺少點名。")
            if point_field == "point" and row.get("corner", "").strip():
                name = f"{row['corner'].strip()}_{name}"
            if name in result:
                raise ValueError(f"硬點 CSV 有重複點名 {name}。")
            result[name] = tuple(_number(row[f"{a}_{unit}"], f"硬點 {name} 的 {a.upper()} ") * scale
                                 for a in "xyz")
    if not result:
        raise ValueError("硬點 CSV 沒有座標列。")
    return result


def _model(name, points, fixed, links, drives, duration=2., samples=121, *, attachments=(),
           reference_settings=None, internal_points=(), standard_suspension=False):
    reference = np.array(list(points.values()), dtype=float)
    reference.setflags(write=False)
    return Model(name, tuple(points), reference, tuple(fixed), tuple(links), tuple(drives), duration, samples,
                 tuple(attachments), reference_settings, tuple(internal_points), standard_suspension)


def _upright(corner, local, points, links, drives, attachments):
    """Build one rigid upright and attach any additional markers to its triangle."""
    wheel = local.get("WC")
    toe = local.get("TOE")
    if "FORWARD" in local and wheel is None:
        raise ValueError(f"{corner} 的 FORWARD 需要搭配輪心 WC。")
    if wheel is not None and "FORWARD" not in local:
        forward = local["FORWARD"] = f"{corner}_FORWARD"
        x, y, z = points[wheel]
        points[forward] = (x + .10, y, z)

    front_marker = local.get("FORWARD")
    anchors = JOINTS
    if toe is not None:
        anchors += ("TOE",)
    elif wheel is not None:
        lower = np.asarray(points[local["LBJ"]])
        kingpin = np.asarray(points[local["UBJ"]]) - lower
        marker_keys = ("WC", "FORWARD")
        offsets = np.asarray([points[local[key]] for key in marker_keys]) - lower
        third = marker_keys[np.argmax(np.linalg.norm(np.cross(kingpin, offsets), axis=1))]
        anchors += (third,)
        dx, dy = np.asarray(points[front_marker][:2]) - points[wheel][:2]
        lock = _direction([-dy, dx, 0.], f"{corner} 的輪心至 FORWARD 水平方向")
        drives.append(Drive(f"{corner}_toe_lock", front_marker, lock, b=wheel, locked=True))

    upright = list(JOINTS)
    if wheel:
        upright.append("WC")
    if toe:
        upright.append("TOE")
    elif front_marker:
        upright.append("FORWARD")
    anchor_set = set(anchors)
    attached = upright + ["FORWARD"] if toe and front_marker else upright
    for key in attached:
        if key not in anchor_set:
            attachments.append(Attachment(local[key], *(local[a] for a in anchors)))
    for a, b in combinations(upright, 2):
        links.append(Link(f"{corner}_{a}-{b}", local[a], local[b], a in anchor_set and b in anchor_set))
    if toe and front_marker:
        links.append(Link(f"{corner}_WHEEL_HEADING", wheel, front_marker, constrained=False))
    return anchors


def _pushrod(corner, local, upright, fixed, links, attachments):
    outboard = [key for key in PUSH_OUTBOARD if key in local]
    if not outboard and not set(PUSH_INBOARD).intersection(local):
        return
    if len(outboard) != 1 or not set(PUSH_INBOARD) <= local.keys():
        raise ValueError(f"{corner} 的推桿需提供一個 PUSH_OUT、PUSH_OUT_UCA 或 PUSH_OUT_UPRIGHT，"
                         "以及 PUSH_IN、ROCKER_A、ROCKER_B。")
    out = outboard[0]
    anchors = {"PUSH_OUT": ("LCA_F", "LCA_R", "LBJ"),
               "PUSH_OUT_UCA": ("UCA_F", "UCA_R", "UBJ"), "PUSH_OUT_UPRIGHT": upright}[out]
    if len(anchors) != 3:
        raise ValueError(f"{corner} 的 PUSH_OUT_UPRIGHT 需要輪心 WC 或轉向拉桿外端 TOE 定義羊角姿態。")
    attachments.append(Attachment(local[out], *(local[a] for a in anchors)))
    a, b, inner = (local[key] for key in ("ROCKER_A", "ROCKER_B", "PUSH_IN"))
    fixed.extend((a, b))
    links.extend((Link(f"{corner}_PUSH_ROD", local[out], inner),
                  Link(f"{corner}_ROCKER_A-IN", a, inner),
                  Link(f"{corner}_ROCKER_B-IN", b, inner),
                  Link(f"{corner}_ROCKER_AXIS", a, b)))


def _front_rack(points, links, drives):
    left, right = "FL_RACK", "FR_RACK"
    ends = [key for key in (left, right) if key in points]
    if not ends:
        return
    axis = (0., 1., 0.)
    if len(ends) == 2:
        delta = np.asarray(points[left]) - points[right]
        if delta[1] <= 0.:
            raise ValueError("前齒條需要 FL_RACK 的 Y 大於 FR_RACK，以定義向左為正的輸入。")
        axis = _direction(delta.tolist(), "前齒條軸向")
        links.append(Link("FRONT_RACK_BAR", right, left))

    # Each end follows the same axis. The rack bar fixes their separation.
    transverse = np.cross(axis, np.eye(3)[np.argmin(np.abs(axis))])
    transverse /= np.linalg.norm(transverse)
    normal = np.cross(axis, transverse)
    for end in ends:
        drives.extend(Drive(f"{end}_guide_{i}", end, tuple(vector), locked=True)
                      for i, vector in enumerate((transverse, normal), 1))
    drives.append(Drive("FRONT_RACK", ends[0], axis))


def _suspension_model(points, name, reference_settings=None):
    """Infer double wishbones from the lab's standard named static hardpoints."""
    canonical = {}
    allowed = set(MOUNTS + JOINTS + ("WC", "FORWARD", "RACK", "TOE") + PUSH_OUTBOARD + PUSH_INBOARD)
    for raw, xyz in points.items():
        corner, _, key = raw.upper().partition("_")
        if corner in CORNERS and key in CONSTRUCTION_POINTS:
            continue
        key = ALIASES.get(key, key)
        if corner not in CORNERS or key not in allowed:
            raise ValueError(f"硬點 {raw} 不符合懸吊點名規則。請選擇機構 JSON 來定義各點。")
        qualified = f"{corner}_{key}"
        if qualified in canonical:
            raise ValueError(f"硬點 {qualified} 有重複定義。")
        canonical[qualified] = xyz

    imported = set(canonical)
    fixed, links, drives, attachments = [], [], [], []
    bump = reference_settings.bump_mm if reference_settings else 25.
    for corner in CORNERS:
        local = {name[len(corner) + 1:]: name for name in canonical if name.startswith(corner + "_")}
        if not local:
            continue
        missing = set(MOUNTS + JOINTS) - local.keys()
        if missing:
            raise ValueError(f"{corner} 缺少懸吊硬點 {', '.join(sorted(missing))}。")
        if ("RACK" in local) != ("TOE" in local):
            raise ValueError(f"{corner} 的轉向拉桿需要同時提供 RACK 與 TOE。")
        fixed.extend(local[k] for k in MOUNTS)
        members = [("UCA_F", "UBJ"), ("UCA_R", "UBJ"), ("LCA_F", "LBJ"), ("LCA_R", "LBJ")]
        if "RACK" in local:
            if corner.startswith("R"):
                fixed.append(local["RACK"])
            members.append(("RACK", "TOE"))
        links.extend(Link(f"{corner}_{a}-{b}", local[a], local[b]) for a, b in members)
        anchors = _upright(corner, local, canonical, links, drives, attachments)
        _pushrod(corner, local, anchors, fixed, links, attachments)
        target = local.get("WC", local["UBJ"])
        drives.append(Drive(f"{target}_Z", target, (0., 0., 1.), amplitude_mm=bump))
    if not links:
        raise ValueError("沒有可建立機構的輪位。")
    _front_rack(canonical, links, drives)
    samples = reference_settings.samples if reference_settings else 121
    return _model(name, canonical, fixed, links, drives, samples=samples, attachments=attachments,
                  reference_settings=reference_settings,
                  internal_points=(name for name in canonical if name not in imported), standard_suspension=True)


def _mapping(value, allowed, required, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label} 必須是 JSON 物件。")
    unknown = value.keys() - set(allowed)
    missing = set(required) - value.keys()
    if unknown:
        raise ValueError(f"{label} 有未知欄位 {', '.join(sorted(unknown))}。")
    if missing:
        raise ValueError(f"{label} 缺少 {', '.join(sorted(missing))}。")
    return value


def _array(value, label):
    if not isinstance(value, list):
        raise ValueError(f"{label} 必須是 JSON 陣列。")
    return value


def _name(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} 需要名稱。")
    return value.strip()


def _known_point(value, points, label):
    name = _name(value, label)
    if name not in points:
        raise ValueError(f"{label} 使用未匯入的硬點 {name}。")
    return name


def _direction(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{label} 需要三個 XYZ 數值。")
    vector = tuple(_number(v, label) for v in value)
    scale = max(map(abs, vector))
    if scale == 0:
        raise ValueError(f"{label} 需要非零方向。")
    scaled = tuple(v / scale for v in vector)
    length = math.hypot(*scaled)
    return tuple(v / length for v in scaled)


def _timing(duration, samples):
    duration = _number(duration, "工況時間 ")
    count = _number(samples, "取樣數 ")
    if duration <= 0 or not count.is_integer() or not 2 <= count <= 5001:
        raise ValueError("工況時間需大於零，取樣數需為 2 到 5001 的整數。")
    return duration, int(count)


def _read_reference_settings():
    keys = ("name", "source", "analysis")
    with (EXAMPLES / "reference_settings.json").open(encoding="utf-8") as handle:
        data = _mapping(json.load(handle), keys, keys, "懸吊基準")
    analysis_keys = ("bump_mm", "droop_mm", "samples")
    analysis = _mapping(data["analysis"], analysis_keys, analysis_keys, "懸吊基準工況")
    bump = _number(analysis["bump_mm"], "基準上壓 ")
    droop = _number(analysis["droop_mm"], "基準下垂 ")
    if bump < 0. or droop < 0.:
        raise ValueError("基準上壓與下垂行程須以非負數表示。")
    _, samples = _timing(2., analysis["samples"])
    return ReferenceSettings(_name(data["name"], "懸吊基準"), _name(data["source"], "懸吊基準來源"),
                             bump, droop, samples)


def _read_mechanism(points, path):
    with Path(path).open(encoding="utf-8-sig") as handle:
        data = json.load(handle)
    if isinstance(data, dict) and {"vehicle", "front", "rear"} <= data.keys():
        raise ValueError("此檔案是基準車輛參數 JSON。請用「懸吊範例」載入基準硬點 CSV，"
                         "或用「自訂機構範例」載入配對的機構 JSON 與硬點 CSV。")
    data = _mapping(data, ("schema_version", "name", "fixed", "links", "drives", "attachments",
                           "duration_s", "samples"), ("fixed", "links", "drives"), "機構")
    if type(data.get("schema_version", 1)) is not int or data.get("schema_version", 1) != 1:
        raise ValueError("機構 JSON 需要 schema_version 1。")
    name = _name(data.get("name", Path(path).stem), "機構")
    fixed = tuple(_known_point(p, points, "固定點") for p in _array(data["fixed"], "fixed"))
    if len(set(fixed)) != len(fixed):
        raise ValueError("機構 JSON 有重複固定點。")
    links = []
    for row in _array(data["links"], "links"):
        member = _mapping(row, ("name", "a", "b", "constrained"), ("name", "a", "b"), "桿件")
        label = _name(member["name"], "桿件")
        a = _known_point(member["a"], points, label)
        b = _known_point(member["b"], points, label)
        if a == b:
            raise ValueError(f"桿件 {label} 的兩端必須不同。")
        constrained = member.get("constrained", True)
        if type(constrained) is not bool:
            raise ValueError(f"桿件 {label} 的 constrained 需為布林值。")
        links.append(Link(label, a, b, constrained))
    attachments = []
    keys = ("point", "a", "b", "c")
    for row in _array(data.get("attachments", []), "attachments"):
        data_row = _mapping(row, keys, keys, "剛體附著")
        point, *anchors = (_known_point(data_row[key], points, "剛體附著") for key in keys)
        if len({point, *anchors}) != 4:
            raise ValueError(f"剛體附著 {point} 需要三個不同的基準點，且不能包含自身。")
        attachments.append(Attachment(point, *anchors))
    if len({attachment.point for attachment in attachments}) != len(attachments):
        raise ValueError("機構 JSON 有重複的剛體附著點。")
    drives = []
    for row in _array(data["drives"], "drives"):
        drive = _mapping(row, ("name", "point", "b", "direction", "amplitude_mm", "profile", "locked"),
                         ("name", "point", "direction"), "驅動")
        label = _name(drive["name"], "驅動")
        target = _known_point(drive["point"], points, label)
        other = _known_point(drive["b"], points, label) if "b" in drive else None
        if target == other:
            raise ValueError(f"驅動 {label} 的兩個點必須不同。")
        vector = _direction(drive["direction"], f"{label} 的方向")
        amplitude = _number(drive.get("amplitude_mm", 0.), f"{label} 的振幅 ")
        profile = drive.get("profile", "pulse")
        locked = drive.get("locked", False)
        if profile not in PROFILES or type(locked) is not bool or (locked and amplitude != 0):
            raise ValueError(f"{label} 的 profile 需為 pulse、sine 或 ramp。locked 需為布林值，鎖定時振幅為零。")
        drives.append(Drive(label, target, vector, other, amplitude, profile, locked))
    for entries, label in ((links, "桿件"), (drives, "驅動")):
        if len({entry.name for entry in entries}) != len(entries):
            raise ValueError(f"機構 JSON 有重複{label}名稱。")
    duration, samples = _timing(data.get("duration_s", 2.), data.get("samples", 121))
    return _model(name, points, fixed, links, drives, duration, samples, attachments=attachments)


def load_model(hardpoints_path, mechanism_path=None, *, points=None):
    """Load spatial hardpoints and an optional explicit mechanism definition."""
    try:
        path = Path(hardpoints_path)
        if points is None:
            points = read_hardpoints(path)
        if mechanism_path is not None:
            return _read_mechanism(points, mechanism_path)
        settings = _read_reference_settings() if path.resolve() == EXAMPLES / "hardpoints.csv" else None
        return _suspension_model(points, settings.name if settings else path.stem, settings)
    except (csv.Error, UnicodeError) as exc:
        raise ValueError(f"無法讀取 UTF-8 硬點 CSV 或機構 JSON。{exc}") from exc


def make_motion(model, *, duration=None, samples=None, amplitudes_mm=None, profiles=None):
    """Generate reference-relative displacement and exact time derivatives."""
    duration, samples = _timing(model.duration if duration is None else duration,
                               model.samples if samples is None else samples)
    amplitudes = amplitudes_mm or {}
    profile_overrides = profiles or {}
    unknown = (amplitudes.keys() | profile_overrides.keys()) - {drive.name for drive in model.drives if not drive.locked}
    if unknown:
        raise ValueError(f"無法修改驅動 {', '.join(sorted(unknown))} 的振幅或行程形式。")
    time = np.linspace(0., duration, samples)
    displacement = np.zeros((samples, len(model.drives)))
    velocity = np.zeros_like(displacement)
    acceleration = np.zeros_like(displacement)
    with np.errstate(over="ignore", invalid="ignore"):
        for i, drive in enumerate(model.drives):
            amplitude = _number(amplitudes.get(drive.name, drive.amplitude_mm), f"{drive.name} 的振幅 ") / 1000.
            profile = profile_overrides.get(drive.name, drive.profile)
            if profile not in PROFILES:
                raise ValueError(f"{drive.name} 的行程形式需為 pulse、sine 或 ramp。")
            if amplitude == 0.:
                continue
            if profile == "ramp":
                displacement[:, i] = amplitude * (time / duration)
                velocity[:, i] = amplitude / duration
                continue
            phase = (2. * np.pi) * (time / duration)
            frequency = np.float64(2. * np.pi) / duration
            if profile == "pulse":
                displacement[:, i] = .5 * amplitude * (1. - np.cos(phase))
                velocity[:, i] = .5 * amplitude * frequency * np.sin(phase)
                acceleration[:, i] = .5 * amplitude * frequency**2 * np.cos(phase)
            else:
                displacement[:, i] = amplitude * np.sin(phase)
                velocity[:, i] = amplitude * frequency * np.cos(phase)
                acceleration[:, i] = -amplitude * frequency**2 * np.sin(phase)
    if (np.any(np.diff(time) <= 0.)
            or not all(np.isfinite(a).all() for a in (displacement, velocity, acceleration))):
        raise ValueError("工況設定超出可計算的時間或振幅範圍。")
    return Motion(time, displacement, velocity, acceleration)


def default_motion(model):
    return make_motion(model)
