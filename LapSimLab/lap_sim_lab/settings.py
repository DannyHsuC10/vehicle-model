from dataclasses import dataclass
import math
from pathlib import Path


INPUT_DIR = Path(__file__).resolve().parents[1] / "input_data"


@dataclass(frozen=True)
class Settings:
    track_path: Path = INPUT_DIR / "fsae_A_2025.csv"
    tire_path: Path = INPUT_DIR / "hoosier_43075_r20_w8.json"
    motor_path: Path = INPUT_DIR / "motor_speed_vs_torque.csv"
    aero_path: Path = INPUT_DIR / "modal_aero.json"
    track_width: float = 3.0
    step_m: float = 0.5
    max_speed: float = 30.0
    safety_factor: float = 0.98
    passes: int = 3
    aero_enabled: bool = True
    mass: float = 320.0
    wheelbase: float = 1.53
    cg_from_front: float = 0.7498400000000001
    cg_height: float = 0.3016451511
    front_track: float = 1.25
    rear_track: float = 1.24
    tire_radius: float = 0.203
    tire_stiffness: float = 80185.0
    front_heave_stiffness: float = 140000.0
    front_roll_stiffness: float = 100000.0
    rear_stiffness_ratio: float = 1.0
    gear_ratio: float = 11.2
    gear_efficiency: float = 0.95
    front_torque_scale: float = 0.67
    rear_torque_scale: float = 1.0
    front_brake_bias: float = 0.7
    brake_leverage: float = 12.6
    pedal_force: float = 500.0

    @property
    def rear_heave_stiffness(self) -> float:
        return self.front_heave_stiffness * self.rear_stiffness_ratio

    @property
    def rear_roll_stiffness(self) -> float:
        return self.front_roll_stiffness * self.rear_stiffness_ratio

    def validate(self):
        for name, label in POSITIVE_FIELDS:
            if not math.isfinite(value := getattr(self, name)) or value <= 0:
                raise ValueError(f"{label}須為有限的正數。")
        for name, label in (
            ("cg_height", "重心高度"),
            ("pedal_force", "踏板力"),
        ):
            if not math.isfinite(value := getattr(self, name)) or value < 0:
                raise ValueError(f"{label}須為有限的非負數。")
        if not 0 < self.cg_from_front < self.wheelbase:
            raise ValueError("重心距前軸須大於零且小於軸距。")
        for name, label in (("safety_factor", "側向安全係數"), ("gear_efficiency", "傳動效率")):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{label}須大於零且不超過 1。")
        for name, label in (
            ("front_torque_scale", "前輪限扭比例"),
            ("rear_torque_scale", "後輪限扭比例"),
            ("front_brake_bias", "前軸煞車比例"),
        ):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f"{label}須介於 0 和 1。")
        if type(self.passes) is not int or not 1 <= self.passes <= 30:
            raise ValueError("前後向掃描次數須為 1 到 30 的整數。")


POSITIVE_FIELDS = (
    ("track_width", "賽道寬度"),
    ("step_m", "取樣間距"),
    ("max_speed", "速度上限"),
    ("mass", "總質量"),
    ("wheelbase", "軸距"),
    ("front_track", "前輪距"),
    ("rear_track", "後輪距"),
    ("tire_radius", "輪胎自由半徑"),
    ("tire_stiffness", "輪胎垂直剛性"),
    ("front_heave_stiffness", "前軸 Heave 剛性"),
    ("front_roll_stiffness", "前軸 Roll 剛性"),
    ("rear_stiffness_ratio", "後／前剛性比"),
    ("rear_heave_stiffness", "後軸 Heave 剛性"),
    ("rear_roll_stiffness", "後輪 Roll 剛性"),
    ("gear_ratio", "固定減速比"),
    ("brake_leverage", "煞車機械增益"),
)
