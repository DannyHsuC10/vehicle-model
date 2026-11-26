"""Paths for bundled inputs, saved runs, and older repository references."""

import os
from pathlib import Path


LAB = Path(__file__).resolve().parent
INPUT_DATA = LAB / "input_data"
DEFAULT_TRACK = INPUT_DATA / "fsae_A_2025.csv"
SKIDPAD = LAB / "examples" / "skidpad.csv"
RESULTS = LAB / "results"
RESULT_GROUPS = {"圈速": "lap", "9.125 m 定圓": "skidpad", "直線加速": "straight"}


def result_group(track_path):
    if track_path is None:
        return RESULT_GROUPS["直線加速"]
    return RESULT_GROUPS["9.125 m 定圓" if Path(track_path).stem.casefold() == "skidpad" else "圈速"]


def source_path(folder, name):
    path = (folder / name).resolve()
    if path.is_relative_to(LAB) and path.is_file():
        return path
    parts = Path(str(name).replace("\\", "/")).parts
    legacy = {
        ("SuspensionDynamicsLab", "projects", "LP03.json"): INPUT_DATA / "LP03.json",
        ("LapSimLab", "input_data", DEFAULT_TRACK.name): DEFAULT_TRACK,
        ("LapSimLab", "input_data", "lp03_voltage_envelope.csv"): INPUT_DATA / "lp03_voltage_envelope.csv",
        ("LapSimLab", "input_data", "modal_aero.json"): INPUT_DATA / "modal_aero.json",
        ("Vehicle14DOFLab", "examples", SKIDPAD.name): SKIDPAD,
        ("Vehicle14DOFLab", "lp03.json"): LAB / "lp03.json",
    }
    for suffix, candidate in legacy.items():
        if parts[-len(suffix):] == suffix:
            return candidate
    if parts[-4:-1] == ("LapSimLab", "Pacejka_tire_model", "tir"):
        bundled = INPUT_DATA / "tir" / parts[-1]
        if bundled.is_file():
            return bundled
    return path


def path_reference(path, folder):
    try:
        return Path(os.path.relpath(path, folder)).as_posix()
    except ValueError:
        return str(path)
