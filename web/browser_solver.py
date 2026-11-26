"""Bridge between the browser worker and the shared 14DOF simulation."""

from copy import deepcopy
import csv
import json
import math
from pathlib import Path
from shutil import copyfile
from tempfile import TemporaryDirectory
from time import monotonic
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np

from Vehicle14DOFLab._vendor.suspension.engine import G
from Vehicle14DOFLab._vendor.suspension.project import open_project
from Vehicle14DOFLab.aero_setup import aero_settings
from Vehicle14DOFLab.lap import Driver, Track, simulate_laps
from Vehicle14DOFLab.model import DEFAULT_CONFIG, Vehicle14DOF
from Vehicle14DOFLab.plots import write_plots
from Vehicle14DOFLab.resources import SKIDPAD
from Vehicle14DOFLab.straight import simulate_straight


UPLOADS = {
    "vehicle": "vehicle.json", "suspension": "suspension.json",
    "tire": "tire.tir", "motor": "motor.csv", "track": "track.csv",
    "aero": "aero.json",
}
PROFILE_INPUTS = {"suspension_project": "suspension", "tire_file": "tire", "motor_envelope": "motor"}
PLOT_COLUMNS = (
    "time_s", "x_m", "y_m", "yaw_deg", "speed_mps", "ax_mps2", "ay_mps2",
    "heave_mm", "roll_deg", "pitch_deg", "downforce_n", "drag_n",
    "cop_x_m", "cop_y_m", "cop_z_m", "throttle", "brake",
) + tuple(
    f"{corner}_{name}"
    for name in ("fz_n", "wheel_travel_mm", "slip_ratio", "slip_angle_deg", "drive_nm", "brake_nm",
                 "steer_deg", "wheel_spin_rad", "rolling_radius_m", "camber_deg")
    for corner in ("FL", "FR", "RL", "RR")
)


def read_json(path):
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError(f"{path.name} 須使用 JSON 物件。")
    return data


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def write_csv(path, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def apply_settings(vehicle, suspension, overrides):
    roots = {"vehicle": vehicle, "suspension": suspension}
    curve_scales = {"spring_scale": ("spring",), "damper_scale": ("compression", "rebound"), "mr_scale": ("mr",)}
    for key, value in overrides.items():
        group, *parts = key.split(".")
        if group == "vehicle" and parts[:2] == ["suspension_geometry", "camber_gain_deg_per_25mm"]:
            vehicle.setdefault("suspension_geometry", {}).setdefault("camber_gain_deg_per_25mm", [0.] * 4)
        target = roots[group]
        for part in parts[:-1]:
            target = target[int(part)] if isinstance(target, list) else target[part]
        name = int(parts[-1]) if isinstance(target, list) else parts[-1]
        if group == "suspension" and parts[:1] == ["channels"] and name == "mr_fixed":
            if type(value) not in (int, float) or not math.isfinite(value) or value < 1e-6:
                raise ValueError("固定運動比須為有限正數，且至少為 0.000001。")
            target["mr"] = [[0., value]]
        elif group == "suspension" and parts[:1] == ["channels"] and name in curve_scales:
            if name == "mr_scale" and f"suspension.channels.{parts[1]}.mr_fixed" in overrides:
                continue
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("懸吊倍率須為有限正數。")
            for curve in curve_scales[name]:
                if curve in target:
                    target[curve] = [[x, y * value] for x, y in target[curve]]
            if name == "spring_scale":
                for rate in ("main_rate", "tender_rate"):
                    if rate in target:
                        target[rate] *= value
        else:
            target[name] = value


def symmetrize_settings(vehicle, suspension):
    for key in ("wheel_inertia_kg_m2", "tire_pressure_kpa"):
        values = vehicle[key]
        if isinstance(values, list) and len(values) == 4:
            vehicle[key] = [values[0]] * 4
    for settings, key in ((vehicle, "static_camber_deg"), (vehicle, "drive_scale"),
                          (vehicle.get("suspension_geometry"), "camber_gain_deg_per_25mm")):
        values = settings.get(key) if isinstance(settings, dict) else None
        if isinstance(values, list) and len(values) == 4:
            settings[key] = [values[0], values[0], values[2], values[2]]

    tires, channels = suspension["tires"], suspension["channels"]
    for corner in ("FR", "RL", "RR"):
        for key in ("k", "c"):
            tires[corner][key] = tires["FL"][key]
    for left, right in (("FL", "FR"), ("RL", "RR")):
        tires[right]["mu"] = tires[left]["mu"]
        channels[right] = deepcopy(channels[left])


def snapshot(inputs, uploaded, overrides):
    defaults = read_json(DEFAULT_CONFIG)
    config = defaults | (read_json(uploaded["vehicle"]) if "vehicle" in uploaded else {})
    for key, kind in PROFILE_INPUTS.items():
        source = uploaded.get(kind, DEFAULT_CONFIG.parent / defaults[key])
        copyfile(source, inputs / UPLOADS[kind])
        config[key] = UPLOADS[kind]
    aero_file = uploaded.get("aero")
    config.pop("aero_file", None)
    if aero_file is not None:
        config["aero_file"] = aero_file.name
        copyfile(aero_file, inputs / aero_file.name)
        aero = config.get("aero", {}).copy()
        aero.pop("coefficients", None)
        config["aero"] = aero
    config["aero"] = aero_settings(config, inputs)
    config.pop("aero_file", None)
    suspension_file = inputs / UPLOADS["suspension"]
    suspension, _ = open_project(suspension_file)
    apply_settings(config, suspension, overrides)
    symmetrize_settings(config, suspension)
    for key, kind in PROFILE_INPUTS.items():
        config[key] = UPLOADS[kind]
    config.pop("aero_file", None)
    write_json(suspension_file, suspension)
    path = inputs / "vehicle.json"
    write_json(path, config)
    return path


def run(payload, progress, working_directory):
    options = json.loads(payload)
    working_directory = Path(working_directory)
    uploaded = {
        key: working_directory / "uploads" / name
        for key, name in UPLOADS.items() if (working_directory / "uploads" / name).is_file()
    }
    mode = options["mode"]
    if mode not in ("custom", "skidpad", "straight"):
        raise ValueError("請選擇有效的工況。")
    if mode == "custom" and "track" not in uploaded:
        raise ValueError("請匯入自訂賽道 CSV。")

    with TemporaryDirectory() as staging:
        run_dir = Path(staging)
        inputs = run_dir / "inputs"
        inputs.mkdir()
        vehicle = Vehicle14DOF(snapshot(inputs, uploaded, options.get("overrides", {})))
        track = None
        if mode == "straight":
            length = 75.
            centerline = np.array([[0., 0.], [length, 0.]])
            case_name = f"{length:g} m 直線加速"
        else:
            track_input = SKIDPAD if mode == "skidpad" else uploaded["track"]
            copyfile(track_input, inputs / "track.csv")
            track = Track.from_csv(inputs / "track.csv", width=options["width"])
            length = float(track.length)
            centerline = np.vstack((track.xy, track.xy[0]))
            case_name = options["courseName"]

        total_laps = 1 if track is None else options["laps"] + options["warmup"]
        last_report = float("-inf")

        def position(lap, time, distance):
            nonlocal last_report
            now = monotonic()
            if now - last_report >= .25:
                last_report = now
                progress(lap, total_laps, float(time), float(distance), float(length))

        kwargs = dict(initial_speed=options["initial"] / 3.6, step=options["step"],
                      time_limit=options["limit"], on_position=position)
        sources = {"vehicle_config": "inputs/vehicle.json", "suspension_config": "inputs/suspension.json"}
        if track is None:
            rows = simulate_straight(vehicle, length, **kwargs)
            timing = [{
                "distance_m": length, "elapsed_time_s": rows[-1]["time_s"],
                "exit_speed_kmh": 3.6 * rows[-1]["speed_mps"], **sources,
            }]
            times = [float(rows[-1]["time_s"])]
            map_start = 0.
            deviation = max(abs(row["centerline_error_m"]) for row in rows)
        else:
            driver = Driver(track, vehicle.wheelbase, max_speed=options["speed"] / 3.6,
                            lateral_limit=options["lateral"] * G)
            result = simulate_laps(vehicle, track, driver, laps=options["laps"], warmup_laps=options["warmup"], **kwargs)
            rows, times = result.telemetry, result.lap_times
            boundaries = result.timed_boundaries_s
            map_start, deviation = boundaries[0], result.max_centerline_error_m
            timing = [{
                "lap": index + 1, "lap_time_s": lap_time, "start_time_s": boundaries[index],
                "finish_time_s": boundaries[index + 1], "track_length_m": length, "track_csv": "inputs/track.csv",
                "track_width_m": options["width"], "target_max_speed_kmh": options["speed"],
                "target_lateral_accel_mps2": options["lateral"] * G, **sources,
            } for index, lap_time in enumerate(times)]

        telemetry = run_dir / "telemetry.csv"
        summary = run_dir / "telemetry_summary.csv"
        write_csv(telemetry, rows)
        write_csv(summary, timing)
        write_plots(telemetry, rows, centerline, case_name, map_start=map_start)
        write_json(run_dir / "run.json", options)
        copyfile(telemetry, working_directory / telemetry.name)
        copyfile(summary, working_directory / summary.name)
        with ZipFile(working_directory / "run.zip", "w", ZIP_DEFLATED) as archive:
            for path in sorted(run_dir.rglob("*")):
                if path.is_file():
                    archive.write(path, path.relative_to(run_dir).as_posix())

        return json.dumps({
            "mode": mode, "courseName": case_name, "length": length, "centerline": centerline.tolist(),
            "times": times, "timing": timing, "deviation": float(deviation), "warmupEnd": map_start,
            "geometry": {
                "axles": np.column_stack((vehicle.wheel_x, vehicle.wheel_y)).tolist(),
                "wheelRadii": vehicle.free_radii.tolist(), "cgHeight": vehicle.height,
                "roadWidth": options.get("width", 3.),
            },
            "series": {key: [float(row[key]) for row in rows] for key in PLOT_COLUMNS},
        }, allow_nan=False, separators=(",", ":"))
