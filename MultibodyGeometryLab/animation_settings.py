"""Preset metadata and validation shared by the editor and motion solver."""

import math

from ._geometry_support.hardpoints import CORNERS


PRESETS = {
    "single_wheel": "單輪抬升",
    "heave": "全車升降（Heave）",
    "roll": "全車側傾（Roll）",
    "front_steer": "前輪轉向",
}
DEFAULT_AMPLITUDES = {"single_wheel": 25., "heave": 20., "roll": 2., "front_steer": 8.}
AMPLITUDE_LIMITS = {"single_wheel": 100., "heave": 100., "roll": 15., "front_steer": 50.}
DURATION_LIMITS = (0.5, 12.)


def default_animation():
    return {"preset": "single_wheel", "corner": "FL", "amplitude": DEFAULT_AMPLITUDES["single_wheel"],
            "duration": 3., "loop": True}


def validate_animation(data):
    fields = {"preset", "corner", "amplitude", "duration"}
    if not isinstance(data, dict) or not fields <= data.keys() or data.keys() - fields - {"loop"}:
        raise ValueError("動畫設定需要工況、輪位、幅度與週期。")
    preset, corner = data["preset"], data["corner"]
    if not isinstance(preset, str) or preset not in PRESETS or corner not in CORNERS:
        raise ValueError("未知的動畫工況或輪位。")
    amplitude, duration, loop = data["amplitude"], data["duration"], data.get("loop", True)
    for value, label in ((amplitude, "動畫幅度"), (duration, "動畫週期")):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"{label}需要有限數值。")
    limit = AMPLITUDE_LIMITS[preset]
    unit = "°" if preset == "roll" else "mm"
    if not 0 < amplitude <= limit:
        raise ValueError(f"{PRESETS[preset]}的幅度須大於 0 且不超過 {limit:g} {unit}。")
    if not DURATION_LIMITS[0] <= duration <= DURATION_LIMITS[1]:
        raise ValueError(f"動畫週期須在 {DURATION_LIMITS[0]:g} 到 {DURATION_LIMITS[1]:g} 秒之間。")
    if type(loop) is not bool:
        raise ValueError("循環播放設定需要布林值。")
    return dict(preset=preset, corner=corner, amplitude=float(amplitude), duration=float(duration), loop=loop)
