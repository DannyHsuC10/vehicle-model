"""Editable example load cases resolved into forces at the CG and tire contacts."""

import numpy as np

from .model import number
from .solver import Load


STATIC = "靜態負載"
ACCEL = "重心加速"
BUMP = "單輪受衝擊"
AERO = "空力下壓"
CASES = (STATIC, ACCEL, BUMP, AERO)
MASS_KG = 321.
GRAVITY = 9.81
DEFAULT_ACCELERATION = (.65 * GRAVITY, 0., 0.)
DESCRIPTIONS = {
    STATIC: "321 kg／重力與四輪接地反力",
    ACCEL: "+X 前、+Y 左、+Z 上。變更後按「套用範例」更新載荷。",
    BUMP: "第一個可用輪位承受 +Z 峰值 1800 N",
    AERO: "321 kg／800 N 下壓，載重分配依 COP 座標",
}


def example_loads(model, kind, *, mass_kg=MASS_KG, acceleration_mps2=DEFAULT_ACCELERATION, include_gravity=True):
    if kind not in CASES:
        raise ValueError("請選擇有效的範例工況。")
    if kind == BUMP:
        if not model.wheels:
            raise ValueError("單輪衝擊範例需要至少一個輪心 WC。")
        return [Load(model.wheels[0].contact, (0., 0., 1800.))]
    if len(model.wheels) != 4 or model.cg is None:
        raise ValueError("這個工況需要 FL、FR、RL、RR 四個輪心及重心。")
    required = [model.cg, *(wheel.contact for wheel in model.wheels)]
    if kind == AERO:
        required.append(model.cop)
    if any(point not in model.owners for point in required):
        raise ValueError("範例需要把 CG、接地點及適用的 COP 指派到剛體或車身。")

    contacts = np.asarray([model.points[wheel.contact] for wheel in model.wheels])
    cg = np.asarray(model.points[model.cg])
    relative = contacts - cg
    balance = np.vstack((np.ones(4), relative[:, 1], -relative[:, 0]))
    mass_kg = number(mass_kg, "重心質量")
    if mass_kg <= 0.:
        raise ValueError("重心質量必須大於零。")
    gravity = GRAVITY if kind != ACCEL or include_gravity else 0.
    weight = mass_kg * gravity
    inertial = np.zeros(3)
    if kind == ACCEL:
        acceleration = np.asarray(acceleration_mps2, dtype=float)
        if acceleration.shape != (3,) or not np.isfinite(acceleration).all():
            raise ValueError("重心加速度需要三個有限的 XYZ 分量。")
        inertial = -mass_kg * acceleration
    apparent_weight = weight - inertial[2]
    if not np.isfinite(apparent_weight) or not np.isfinite(inertial).all():
        raise ValueError("重心質量或加速度超出可計算的範圍。")
    downforce = 800. if kind == AERO else 0.
    cop = np.asarray(model.points[model.cop]) if downforce else cg
    aero_moment = np.cross(cop - cg, (0., 0., -downforce))
    rear_x, front_x = relative[2:, 0].mean(), relative[:2, 0].mean()
    if abs(front_x - rear_x) < 1e-6:
        raise ValueError("範例需要不同 X 座標的前後軸。")
    front_share = -rear_x / (front_x - rear_x)
    nominal = np.repeat([front_share, 1. - front_share], 2) / 2.
    correction, _, rank, _ = np.linalg.lstsq(balance, (1., 0., 0.) - balance @ nominal, rcond=1e-10)
    if rank < 3:
        raise ValueError("四輪接地點在 XY 平面上沒有足夠的幾何範圍。")
    shares = nominal + correction
    reactions = np.zeros_like(relative, dtype=float)
    reactions[:, :2] = -shares[:, None] * inertial[:2]
    remaining = aero_moment + np.cross(relative, reactions).sum(axis=0)
    initial = shares * apparent_weight + downforce * np.repeat([.45, .55], 2) / 2.
    target = (apparent_weight + downforce, -remaining[0], -remaining[1])
    increment = np.linalg.lstsq(balance, target - balance @ initial, rcond=1e-10)[0]
    reactions[:, 2] = initial + increment
    if min(reactions[:, 2]) < -1e-6:
        raise ValueError("目前工況會使至少一個車輪離地，請調整重心或加速度，或自行設定輪荷。")

    loads = [Load(model.cg, (inertial[0], inertial[1], -apparent_weight))]
    loads.extend(Load(wheel.contact, tuple(force)) for wheel, force in zip(model.wheels, reactions))
    if downforce:
        loads.append(Load(model.cop, (0., 0., -downforce)))
    return loads
