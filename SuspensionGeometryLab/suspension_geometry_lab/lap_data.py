"""Read LapSimLab suspension travel and solve the matching geometry poses."""
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from .engine import Unreachable
from .wheel_end_geometry import rotation_matrix


LAP_PAGES = ("front", "rear", "sv")
TRAVEL_COLUMNS = tuple(f"zs_minus_zu_{corner}_mm" for corner in ("FL", "FR", "RL", "RR"))
POSE_COLUMNS = ("heave_mm", "roll_deg", "pitch_deg")
TURN_COLUMNS = ("curvature_1_m", "ay_g")
FRONT_STEER_COLUMNS = ("steer_FL_deg", "steer_FR_deg")


@dataclass(frozen=True)
class LapPose:
    heave_mm: float
    roll_deg: float
    pitch_deg: float
    turn_signal: float


@dataclass(frozen=True)
class LapData:
    travels: tuple[tuple[float, ...], ...]
    poses: tuple[LapPose, ...]
    turn_column: str | None
    front_steer_deg: tuple[tuple[float, float], ...] = ()


def read_lap_data(path):
    """Read bump-positive travel and any available body poses and steering."""
    travels = []
    poses = []
    front_steer = []
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream, strict=True)
        header = reader.fieldnames or []
        invalid = [name for name in TRAVEL_COLUMNS if header.count(name) != 1]
        if invalid:
            raise ValueError("缺少或重複的相對位移欄位：" + "、".join(invalid))
        turn_column = next((name for name in TURN_COLUMNS if name in header), None)
        pose_columns = (*POSE_COLUMNS, turn_column) if turn_column and all(name in header for name in POSE_COLUMNS) else ()
        if any(header.count(name) != 1 for name in pose_columns):
            raise ValueError("CSV 有重複的車身姿態或轉彎方向欄位。")
        has_steer = all(name in header for name in FRONT_STEER_COLUMNS)
        if has_steer and any(header.count(name) != 1 for name in FRONT_STEER_COLUMNS):
            raise ValueError("CSV 有重複的前輪轉向角欄位。")
        for row in reader:
            # LapSimLab stores sprung minus unsprung. Negative means bump.
            travels.append(tuple(-lap_number(row, name, reader.line_num) for name in TRAVEL_COLUMNS))
            if pose_columns:
                poses.append(LapPose(*(lap_number(row, name, reader.line_num) for name in pose_columns)))
            if has_steer:
                fl, fr = FRONT_STEER_COLUMNS
                front_steer.append((lap_number(row, fl, reader.line_num), lap_number(row, fr, reader.line_num)))
    if not travels:
        raise ValueError("CSV 沒有圈速模擬資料。")
    return LapData(tuple(travels), tuple(poses), turn_column if poses else None, tuple(front_steer))


def lap_number(row, name, line):
    try:
        value = float(row[name])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"CSV 第 {line} 行的 {name} 不是數字。") from exc
    if not math.isfinite(value):
        raise ValueError(f"CSV 第 {line} 行的 {name} 須為有限值。")
    return value


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


def road_camber(outward_axis, road_up):
    # The outward wheel axis slopes down for positive (top-out) camber.
    vertical = sum(component*up for component, up in zip(outward_axis, road_up))
    return -math.degrees(math.asin(max(-1., min(1., vertical))))


def calculate_lap_camber(engine, axle, lap, cancelled=lambda: False, *, include_steering=False):
    """Compare body pose, FV suspension motion and optional front steering."""
    if include_steering and (axle != "front" or not lap.front_steer_deg):
        raise ValueError("輪端圈速 Camber 需要前軸及 CSV 的左右前輪轉角。")
    solver = engine.fv[axle]
    wheel_x = engine.car.lf if axle == "front" else -engine.car.lr
    cg_height = engine.car.h_cog
    prefix = "F" if axle == "front" else "R"
    offset = 0 if axle == "front" else 2
    if include_steering:
        kingpins = engine.scene.geometry["kingpins"]
        pin_x = {corner: kingpins[corner]["UBJ"][0]-kingpins[corner]["LBJ"][0] for corner in ("FL", "FR")}
    rows = []
    corners = 0
    for sample, (travels, pose) in enumerate(zip(lap.travels, lap.poses), 1):
        if cancelled():
            return None
        if pose.turn_signal == 0:
            continue
        corners += 1
        left_turn = pose.turn_signal > 0
        pair = travels[offset:offset+2]
        try:
            left, right, _ = solver.wheel_center_sides(*pair)
        except Unreachable:
            continue
        roll, pitch = math.radians(pose.roll_deg), math.radians(pose.pitch_deg)
        # Road Z row of Ry(-pitch) @ Rx(roll) in X forward, Y left, Z up.
        road_up = (math.sin(pitch), math.cos(pitch)*math.sin(roll), math.cos(pitch)*math.cos(roll))
        row = {"sample": sample, "turn": "left" if left_turn else "right",
               lap.turn_column: pose.turn_signal, "heave_mm": pose.heave_mm,
               "roll_deg": pose.roll_deg, "pitch_deg": pose.pitch_deg}
        wheels = ((1, left, pair[0]), (-1, right, pair[1]))
        if not left_turn:
            wheels = wheels[::-1]
        for role, (side, state, travel) in zip(("inner", "outer"), wheels):
            corner = prefix + ("L" if side == 1 else "R")
            wheel_y, wheel_z = state["wheel_center"]
            # FV stores horizontal positive to the right. Rotate about the
            # static CG, then translate the body by the measured heave.
            center_z = (road_up[0]*wheel_x - road_up[1]*wheel_y
                        + road_up[2]*(wheel_z-cg_height) + cg_height)*1000 - pose.heave_mm
            body_camber = side*state["angle"]
            outward = (0., side*math.cos(body_camber), -math.sin(body_camber))
            row.update({
                f"{role}_wheel": corner,
                f"{role}_bump_mm": travel,
                f"{role}_center_road_z_mm": float(center_z),
                f"{role}_camber_body_deg": math.degrees(body_camber),
                f"{role}_camber_before_deg": road_camber((0., side, 0.), road_up),
                f"{role}_camber_after_deg": road_camber(outward, road_up),
            })
            if include_steering:
                steer_deg = lap.front_steer_deg[sample-1][0 if side == 1 else 1]
                # FV moves the joints in YZ, retaining the WE3D longitudinal
                # separation. Steering rotates the moved wheel about that pin.
                rightward, upward = state["kua"]-state["lak"]
                pin = (pin_x[corner], -rightward, upward)
                steered = rotation_matrix(pin, math.radians(steer_deg)) @ outward
                row[f"{role}_steer_deg"] = steer_deg
                row[f"{role}_camber_steered_deg"] = road_camber(steered, road_up)
        rows.append(row)
    return {"rows": rows, "valid": len(rows), "total": corners, "straight": len(lap.poses)-corners}
