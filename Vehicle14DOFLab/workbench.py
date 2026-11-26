"""Project and result handling shared by the desktop workbench."""

import csv
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from pathlib import Path
from shutil import copy2
from tempfile import TemporaryDirectory

import numpy as np

from ._vendor.suspension.engine import G
from ._vendor.suspension.project import open_project

from .aero_setup import aero_settings
from .model import DEFAULT_CONFIG, Vehicle14DOF, hydraulic_defaults
from .resources import LAB, RESULTS, SKIDPAD, source_path


SOURCE_KEYS = ("tire_file", "motor_envelope")


@dataclass
class Profile:
    path: Path
    vehicle: dict
    suspension: dict

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        vehicle = json.loads(path.read_text(encoding="utf-8-sig"))
        vehicle.pop("rolling_resistance_coefficient", None)
        old_radius = vehicle.pop("rolling_radius_m", None)
        if "free_radius_m" not in vehicle:
            vehicle["free_radius_m"] = old_radius if old_radius is not None else Vehicle14DOF(path).radius
        vehicle.pop("brake_model", None)
        vehicle.pop("brake_leverage", None)
        vehicle["hydraulic_brake"] = hydraulic_defaults() | vehicle.get("hydraulic_brake", {})
        suspension, _ = open_project(source_path(path.parent, vehicle["suspension_project"]))
        for key in SOURCE_KEYS:
            if key in vehicle:
                vehicle[key] = str(source_path(path.parent, vehicle[key]))
        vehicle["aero"] = aero_settings(vehicle, path.parent)
        vehicle.pop("aero_file", None)
        return cls(path, vehicle, suspension)

    def save(self, folder):
        folder = Path(folder).resolve()
        if folder.exists():
            raise FileExistsError(f"資料夾已存在：{folder}")
        folder.parent.mkdir(parents=True, exist_ok=True)
        config = deepcopy(self.vehicle)
        config["suspension_project"] = "suspension.json"
        with TemporaryDirectory(prefix=".profile-", dir=folder.parent) as staging:
            staged = Path(staging) / "profile"
            staged.mkdir()
            for key in SOURCE_KEYS:
                source = source_path(self.path.parent, config[key])
                relative = Path("data") / key / source.name
                target = staged / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                copy2(source, target)
                config[key] = relative.as_posix()
            for name, data in (("vehicle.json", config), ("suspension.json", self.suspension)):
                (staged / name).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                                          encoding="utf-8")
            Vehicle14DOF(staged / "vehicle.json")
            staged.rename(folder)
        return folder / "vehicle.json"


def number(value, label, *, minimum=None, maximum=None, integer=False):
    try:
        result = int(value) if integer else float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{label}須輸入{'整數' if integer else '數字'}。") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label}須為有限值。")
    if minimum is not None and result < minimum:
        raise ValueError(f"{label}須大於或等於 {minimum:g}。")
    if maximum is not None and result > maximum:
        raise ValueError(f"{label}須小於或等於 {maximum:g}。")
    return result


def run_arguments(mode, values):
    def read(key, label, **limits):
        return str(number(values[key], label, **limits))

    args = [
        "--initial-speed", str(number(values["initial"], "起步車速", minimum=0) / 3.6),
        "--step", read("step", "積分步長", minimum=.0001, maximum=.02),
        "--time-limit", read("time_limit", "模擬時間上限", minimum=.001),
    ]
    if mode == "直線加速":
        return args + ["--straight", read("length", "直線距離", minimum=.001)]
    path = SKIDPAD if mode == "9.125 m 定圓" else Path(values["track"]).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"找不到賽道 CSV：{path}")
    return args + [
        "--track", str(path),
        "--width", read("width", "賽道寬度", minimum=.001),
        "--laps", read("laps", "計時圈數", minimum=1, integer=True),
        "--warmup-laps", read("warmup", "暖胎圈數", minimum=0, integer=True),
        "--max-speed", str(number(values["speed"], "目標車速", minimum=.001) / 3.6),
        "--lateral-limit", str(number(values["lateral"], "側向加速度目標", minimum=.001) * G),
    ]


def existing_results():
    summaries = [*RESULTS.rglob("*_summary.csv"), *LAB.glob("*_summary.csv")]
    candidates = (summary.with_name(summary.name.removesuffix("_summary.csv") + ".csv") for summary in summaries)
    return sorted((path for path in candidates if path.is_file()), key=lambda path: path.stat().st_mtime, reverse=True)


@dataclass
class Result:
    path: Path
    data: np.ndarray
    timing: list[dict]

    def source(self, key):
        name = self.timing[0].get(key) if self.timing else None
        if not name:
            return None
        snapshot = {"vehicle_config": "vehicle.json", "suspension_config": "suspension.json"}
        if key in snapshot:
            local = self.path.parent / "inputs" / snapshot[key]
            if local.is_file():
                return local
        return source_path(self.path.parent, name)

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        if path.name.endswith("_summary.csv"):
            path = path.with_name(path.name.removesuffix("_summary.csv") + ".csv")
        data = np.genfromtxt(path, names=True, delimiter=",", encoding="utf-8-sig", ndmin=1)
        if not {"time_s", "x_m", "y_m", "speed_mps", "ay_mps2"}.issubset(data.dtype.names or ()) or data.size < 2:
            raise ValueError("CSV 缺少 14 DOF 逐點遙測欄位或有效資料。")
        fundamental = ("time_s", "x_m", "y_m", "speed_mps", "ay_mps2")
        if (not all(np.isfinite(data[key]).all() for key in fundamental)
                or np.any(np.diff(data["time_s"]) <= 0)):
            raise ValueError("遙測數值必須有限且時間必須依序遞增。")
        summary = path.with_name(path.stem + "_summary.csv")
        timing = []
        if summary.is_file():
            with summary.open(encoding="utf-8-sig", newline="") as file:
                timing = list(csv.DictReader(file))
        return cls(path, data, timing)
