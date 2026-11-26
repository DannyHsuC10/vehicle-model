"""Command-line entry point for the 14 DOF lap simulator."""

import argparse
import csv
from datetime import datetime
from pathlib import Path
from time import monotonic

import numpy as np

from .lap import (DEFAULT_LATERAL_LIMIT_MPS2, DEFAULT_MAX_SPEED_MPS, DEFAULT_TRACK,
                  Driver, LapIncomplete, Track, simulate_laps)
from .model import DEFAULT_CONFIG, Vehicle14DOF
from .plots import write_plots
from .resources import RESULTS, path_reference, result_group
from .straight import simulate_straight


PROGRESS_INTERVAL_S = 2.


def write_csv(path, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description="LP03 14 DOF lap and straight-line simulation")
    parser.add_argument("--gui", action="store_true", help="Open the desktop workbench")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    course = parser.add_mutually_exclusive_group()
    course.add_argument("--track", type=Path, default=DEFAULT_TRACK,
                        help=f"Closed LapSimLab LINE/ARC CSV (default: {DEFAULT_TRACK.name})")
    course.add_argument("--straight", type=float, metavar="METRES", help="Full-throttle straight-line test")
    parser.add_argument("--width", type=float, default=3., help="Track width in m")
    parser.add_argument("--laps", type=int, default=1, help="Number of timed laps")
    parser.add_argument("--warmup-laps", type=int, default=0, help="Laps before timing starts")
    parser.add_argument("--initial-speed", type=float, default=0., help="Starting speed in m/s")
    parser.add_argument("--max-speed", type=float, default=DEFAULT_MAX_SPEED_MPS,
                        help=f"Driver speed limit in m/s (default: {DEFAULT_MAX_SPEED_MPS:.3f})")
    parser.add_argument("--lateral-limit", type=float, default=DEFAULT_LATERAL_LIMIT_MPS2,
                        help=f"Driver lateral target in m/s² (default: {DEFAULT_LATERAL_LIMIT_MPS2:.3f})")
    parser.add_argument("--step", type=float, default=.005, help="Maximum RK4 step in s")
    parser.add_argument("--time-limit", type=float, default=240., help="Simulation time limit in s")
    parser.add_argument("--output", type=Path, help="Telemetry CSV (default: a new run folder under results)")
    parser.add_argument("--summary", type=Path, help="Timing CSV (default: add _summary to --output)")
    args = parser.parse_args(argv)
    if args.gui:
        from .gui import main as open_workbench

        open_workbench(args.config)
        return 0

    try:
        print("Loading track and vehicle parameters...", flush=True)
        straight = args.straight is not None
        if straight and (args.warmup_laps or args.laps != 1):
            raise ValueError("Warm-up and multiple laps apply to closed tracks")
        track_path = None if straight else args.track.resolve()
        config_path = args.config.resolve()
        default_output = (RESULTS / result_group(track_path)
                          / datetime.now().strftime("run_%Y%m%d_%H%M%S_%f") / "telemetry.csv")
        output = (args.output or default_output).resolve()
        summary = args.summary.resolve() if args.summary else output.with_name(f"{output.stem}_summary.csv")
        if summary == output:
            raise ValueError("Lap summary and telemetry must use different file paths")
        track = None if straight else Track.from_csv(track_path, width=args.width)
        length = args.straight if straight else track.length
        vehicle = Vehicle14DOF(config_path)
        if {summary, output} & {track_path, config_path, vehicle.suspension_project_path}:
            raise ValueError("Output paths must differ from the track and configuration files")
        case_name = f"{length:g} m straight acceleration" if straight else track_path.stem
        print(f"Course: {case_name} | {length:.2f} m", flush=True)
        if track_path is not None:
            print(f"Track CSV: {track_path} | {args.laps} timed lap(s)", flush=True)
            print(f"Driver targets: {3.6 * args.max_speed:.1f} km/h, "
                  f"lateral {args.lateral_limit:.3f} m/s²", flush=True)
        print(f"Vehicle configuration: {config_path}", flush=True)
        print(f"Suspension configuration: {vehicle.suspension_project_path}", flush=True)
        print(f"Roll Center / Anti load transfer: {'on' if vehicle.geometry_load_transfer else 'off'}", flush=True)
        print("Tire rolling resistance: SAE empirical approximation", flush=True)
        print("Brake model: hydraulic", flush=True)
        print(f"Aero model: {vehicle.aero.mode if vehicle.aero.enabled else 'off'}", flush=True)
        print("Solving initial equilibrium...", flush=True)
        last_report, last_lap = 0., 0

        def lap_label(lap):
            if straight:
                return "Straight"
            if lap <= args.warmup_laps:
                return f"Warm-up {lap}/{args.warmup_laps}"
            return f"Lap {lap - args.warmup_laps}/{args.laps}"

        def show_position(lap, time, distance):
            nonlocal last_report, last_lap
            now = monotonic()
            if lap == last_lap and now - last_report < PROGRESS_INTERVAL_S:
                return
            within_lap = max(0., min(length, distance - (lap - 1) * length))
            percent = min(99.9, 100. * within_lap / length)
            print(f"{lap_label(lap)} | {percent:5.1f}% | {within_lap:.1f}/{length:.1f} m"
                  f" | simulation t = {time:.2f} s", flush=True)
            last_report, last_lap = now, lap

        def show_crossing(lap, time):
            print(f"{lap_label(lap)} | 100.0% | finish crossed at simulation t = {time:.3f} s", flush=True)

        simulation = dict(initial_speed=args.initial_speed, step=args.step, time_limit=args.time_limit,
                          on_position=show_position)
        sources = {"vehicle_config": path_reference(config_path, summary.parent),
                   "suspension_config": path_reference(vehicle.suspension_project_path, summary.parent)}
        if straight:
            rows = simulate_straight(vehicle, length, **simulation)
            finish = rows[-1]
            show_crossing(1, finish["time_s"])
            timing = [{"distance_m": length, "elapsed_time_s": finish["time_s"],
                       "exit_speed_kmh": 3.6 * finish["speed_mps"], **sources}]
            centerline, map_start = np.array([[0., 0.], [length, 0.]]), 0.
            report = [f"Elapsed: {finish['time_s']:.3f} s | Exit speed: {3.6 * finish['speed_mps']:.2f} km/h"]
        else:
            driver = Driver(track, vehicle.wheelbase, max_speed=args.max_speed, lateral_limit=args.lateral_limit)
            result = simulate_laps(vehicle, track, driver, laps=args.laps, warmup_laps=args.warmup_laps,
                                   progress=show_crossing, **simulation)
            rows, boundaries = result.telemetry, result.timed_boundaries_s
            timing = [
                {"lap": i + 1, "lap_time_s": lap_time, "start_time_s": boundaries[i],
                 "finish_time_s": boundaries[i + 1], "track_length_m": length,
                 "track_csv": path_reference(track_path, summary.parent),
                 "track_width_m": args.width,
                 "target_max_speed_kmh": 3.6 * args.max_speed,
                 "target_lateral_accel_mps2": args.lateral_limit, **sources}
                for i, lap_time in enumerate(result.lap_times)
            ]
            centerline, map_start = np.vstack((track.xy, track.xy[0])), boundaries[0]
            report = [f"Lap {i}: {time:.3f} s" for i, time in enumerate(result.lap_times, 1)]
            report.append(f"Peak centerline deviation: {result.max_centerline_error_m:.3f} m")
        print("Writing timing, telemetry and plots...", flush=True)
        output.parent.mkdir(parents=True, exist_ok=True)
        summary.parent.mkdir(parents=True, exist_ok=True)
        write_csv(summary, timing)
        write_csv(output, rows)
        plots = write_plots(output, rows, centerline, case_name, map_start=map_start)
    except (ValueError, TypeError, KeyError, OSError, LapIncomplete) as error:
        parser.exit(1, f"Simulation failed: {error}\n")

    for message in report:
        print(message)
    print(f"Timing summary: {summary}")
    print(f"Telemetry: {output}")
    for plot in plots:
        print(f"Figure: {plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
