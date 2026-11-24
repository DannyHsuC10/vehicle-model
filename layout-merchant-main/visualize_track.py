"""Visualization tool for Formula Student Driverless track layouts.

This module provides functionality to visualize track layouts stored as JSON files.
The tracks contain cone positions, colors, start positions, and timing line information.
"""

# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "matplotlib",
#     "numpy",
#     "typer",
# ]
# ///
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import typer

INDEX_TO_COLOR = ["black", "gold", "blue", "orange", "red"]
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_LAYOUT = SCRIPT_DIR / "layouts" / "track_1.json"
DEFAULT_LAPOUT = SCRIPT_DIR / "layouts"
DEFAULT_OUTPUT = SCRIPT_DIR / "track_png"
DEFAULT_RELATIVE_RADIUS_DROP = 0.25


def analyze_layout(data: dict, relative_radius_drop: float) -> dict:
    """Estimate braking events when centerline radius decreases into a tighter turn."""
    coordinates = np.column_stack((data["x"], data["y"]))
    colors = np.asarray(data["color"])
    yellow = np.flatnonzero(colors == 1)
    blue = np.flatnonzero(colors == 2)
    if not len(yellow) or not len(blue):
        return {
            "centerline": np.empty((0, 2)),
            "braking_indices": np.array([], dtype=int),
            "braking_centers": np.empty((0, 2)),
            "braking_events": 0,
            "minimum_radius_m": None,
        }

    distances = np.linalg.norm(
        coordinates[yellow, None, :] - coordinates[None, blue, :], axis=2
    )
    smaller_is_yellow = len(yellow) <= len(blue)
    ordered_distances = distances if smaller_is_yellow else distances.T
    n, m = ordered_distances.shape
    costs = np.full((n + 1, m + 1), np.inf)
    costs[0, :] = 0
    for i in range(1, n + 1):
        for j in range(i, m + 1):
            pair_cost = costs[i - 1, j - 1] + ordered_distances[i - 1, j - 1] ** 2
            skip_cost = costs[i, j - 1]
            costs[i, j] = min(pair_cost, skip_cost)

    pairs = []
    i, j = n, m
    while i:
        pair_cost = costs[i - 1, j - 1] + ordered_distances[i - 1, j - 1] ** 2
        if j > i and costs[i, j - 1] <= pair_cost:
            j -= 1
            continue
        if smaller_is_yellow:
            pairs.append((yellow[i - 1], blue[j - 1]))
        else:
            pairs.append((yellow[j - 1], blue[i - 1]))
        i -= 1
        j -= 1
    pairs.reverse()

    centerline = np.array(
        [(coordinates[yellow_index] + coordinates[blue_index]) / 2 for yellow_index, blue_index in pairs]
    )
    if len(centerline) < 3:
        return {
            "centerline": centerline,
            "braking_indices": np.array([], dtype=int),
            "braking_centers": np.empty((0, 2)),
            "braking_events": 0,
            "minimum_radius_m": None,
        }

    segment_lengths = np.linalg.norm(np.diff(centerline, axis=0), axis=1)
    typical_spacing = float(np.median(segment_lengths[segment_lengths > 0]))
    closed = (
        typical_spacing > 0
        and np.linalg.norm(centerline[0] - centerline[-1]) <= 1.5 * typical_spacing
    )
    if closed:
        previous = np.roll(centerline, 1, axis=0)
        following = np.roll(centerline, -1, axis=0)
        curvature_indices = np.arange(len(centerline))
    else:
        previous = centerline[:-2]
        following = centerline[2:]
        curvature_indices = np.arange(1, len(centerline) - 1)

    current = centerline[curvature_indices]
    first_side = current - previous
    second_side = following - current
    chord = following - previous
    a = np.linalg.norm(first_side, axis=1)
    b = np.linalg.norm(second_side, axis=1)
    c = np.linalg.norm(chord, axis=1)
    cross = first_side[:, 0] * second_side[:, 1] - first_side[:, 1] * second_side[:, 0]
    denominator = a * b * c
    curvature = np.divide(
        2 * cross,
        denominator,
        out=np.zeros_like(cross, dtype=float),
        where=denominator > 1e-12,
    )
    curvature_magnitude = np.abs(curvature)
    if closed:
        smoothed_curvature = (
            np.roll(curvature_magnitude, 1)
            + curvature_magnitude
            + np.roll(curvature_magnitude, -1)
        ) / 3
        previous_curvature = np.roll(smoothed_curvature, 1)
        next_curvature = np.roll(smoothed_curvature, -1)
        peaks = (smoothed_curvature >= previous_curvature) & (
            smoothed_curvature > next_curvature
        )
        troughs = (smoothed_curvature <= previous_curvature) & (
            smoothed_curvature < next_curvature
        )
    else:
        padded_curvature = np.pad(curvature_magnitude, (1, 1), mode="edge")
        smoothed_curvature = (
            padded_curvature[:-2] + padded_curvature[1:-1] + padded_curvature[2:]
        ) / 3
        peaks = np.zeros(len(smoothed_curvature), dtype=bool)
        troughs = np.zeros(len(smoothed_curvature), dtype=bool)
        if len(smoothed_curvature) > 2:
            peaks[1:-1] = (smoothed_curvature[1:-1] >= smoothed_curvature[:-2]) & (
                smoothed_curvature[1:-1] > smoothed_curvature[2:]
            )
            troughs[1:-1] = (
                smoothed_curvature[1:-1] <= smoothed_curvature[:-2]
            ) & (smoothed_curvature[1:-1] < smoothed_curvature[2:])
        if len(smoothed_curvature) > 1:
            troughs[0] = smoothed_curvature[0] < smoothed_curvature[1]
            peaks[-1] = smoothed_curvature[-1] > smoothed_curvature[-2]

    trough_indices = np.flatnonzero(troughs)
    braking_indices = []
    for peak_index in np.flatnonzero(peaks):
        preceding_troughs = trough_indices[trough_indices < peak_index]
        if len(preceding_troughs):
            trough_index = preceding_troughs[-1]
        elif closed and len(trough_indices):
            trough_index = trough_indices[-1]
        else:
            continue

        peak_value = smoothed_curvature[peak_index]
        if peak_value <= 1e-12:
            continue
        radius_drop = 1 - smoothed_curvature[trough_index] / peak_value
        if radius_drop >= relative_radius_drop:
            braking_indices.append(peak_index)

    braking_indices = curvature_indices[np.asarray(braking_indices, dtype=int)]

    if len(braking_indices):
        braking_centers = centerline[braking_indices]
    else:
        braking_centers = np.empty((0, 2))
    maximum_curvature = float(np.max(smoothed_curvature))

    return {
        "centerline": centerline,
        "braking_indices": braking_indices,
        "braking_centers": braking_centers,
        "braking_events": len(braking_indices),
        "minimum_radius_m": 1 / maximum_curvature if maximum_curvature > 1e-12 else None,
    }


def visualize_cones(
    path: Path,
    output_path: Path | None = None,
    analysis: dict | None = None,
):
    """Visualize cone positions and track features from a JSON layout file.

    This function reads a track layout JSON file and creates a matplotlib visualization
    showing cone positions (colored by type), the start position with orientation arrow,
    and the timing line.

    Args:
        path: Path to the JSON file containing the track layout data. The file must
            contain keys: 'x', 'y', 'color', 'start_position', 'start_orientation',
            'timing_line_position', 'timing_line_orientation', and 'timing_line_width'.

    Returns:
        None. Displays the plot using matplotlib.pyplot.show().

    Note:
        Cone colors are mapped as follows:
        0: unknown (black), 1: yellow (gold), 2: blue, 3: orange_small (orange),
        4: orange_big (red).
    """
    path = Path(path).expanduser()
    if not path.is_absolute():
        path_from_cwd = Path.cwd() / path
        path = path_from_cwd if path_from_cwd.exists() else SCRIPT_DIR / path

    if not path.exists():
        print(f"Path {path} does not exist.")
        return

    data = json.loads(path.read_text(encoding="utf-8"))

    _, ax = plt.subplots()

    if "start_orientation" in data:
        heading_rad = np.deg2rad(data["start_orientation"])

        n = 3

        ax.arrow(
            data["start_position"][0],
            data["start_position"][1],
            np.cos(heading_rad) * n,
            np.sin(heading_rad) * n,
            head_width=1.5,
            head_length=1,
            fc="k",
            ec="k",
        )

    # visualize the timing line
    heading_angle = np.deg2rad(data["timing_line_orientation"]) + np.pi / 2
    heading = np.array([np.cos(heading_angle), np.sin(heading_angle)])

    width = data["timing_line_width"]
    point_1_timing_line = np.array(data["timing_line_position"]) + heading * width / 2
    point_2_timing_line = np.array(data["timing_line_position"]) - heading * width / 2

    ax.plot(
        [point_1_timing_line[0], point_2_timing_line[0]],
        [point_1_timing_line[1], point_2_timing_line[1]],
        "k",
    )

    cones = np.column_stack((data["x"], data["y"]))

    ax.scatter(cones[:, 0], cones[:, 1], c=[INDEX_TO_COLOR[x] for x in data["color"]])
    if analysis is not None and len(analysis["centerline"]):
        centerline = analysis["centerline"]
        ax.plot(centerline[:, 0], centerline[:, 1], color="green", linewidth=1.2)
        if len(analysis["braking_indices"]):
            braking_points = centerline[analysis["braking_indices"]]
            ax.scatter(
                braking_points[:, 0],
                braking_points[:, 1],
                c="red",
                marker="x",
                s=30,
                label="Braking curvature",
            )
            ax.legend()

    ax.set_title(f"{path.stem}: {len(cones)} cones")

    plt.axis("equal")
    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close()
    else:
        plt.show()


def batch_visualize(
    lapout: Path = DEFAULT_LAPOUT,
    output_dir: Path = DEFAULT_OUTPUT,
    relative_radius_drop: float = DEFAULT_RELATIVE_RADIUS_DROP,
):
    """Generate track PNGs and relative-radius braking statistics."""
    lapout = Path(lapout).expanduser()
    output_dir = Path(output_dir).expanduser()

    if not lapout.is_absolute():
        lapout = SCRIPT_DIR / lapout
    if not output_dir.is_absolute():
        output_dir = SCRIPT_DIR / output_dir

    if not lapout.exists():
        print(f"Lapout directory does not exist: {lapout}")
        return

    if not 0 <= relative_radius_drop <= 1:
        raise typer.BadParameter("relative-radius-drop must be between 0 and 1")

    json_files = sorted(lapout.rglob("*.json"))
    if not json_files:
        print(f"No JSON files found in: {lapout}")
        return

    output_dir.mkdir(parents=True, exist_ok=True)
    analyses = []
    for json_path in json_files:
        data = json.loads(json_path.read_text(encoding="utf-8"))
        analysis = analyze_layout(data, relative_radius_drop)
        analyses.append((json_path, data, analysis))
        visualize_cones(json_path, output_dir / f"{json_path.stem}.png")
        print(f"Generated: {output_dir / f'{json_path.stem}.png'}")

    csv_path = output_dir / "braking_statistics.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "track",
                "centerline_points",
                "braking_events",
                "minimum_radius_m",
                "relative_radius_drop_threshold",
            ],
        )
        writer.writeheader()
        for json_path, _, analysis in analyses:
            writer.writerow(
                {
                    "track": json_path.stem,
                    "centerline_points": len(analysis["centerline"]),
                    "braking_events": analysis["braking_events"],
                    "minimum_radius_m": analysis["minimum_radius_m"],
                    "relative_radius_drop_threshold": relative_radius_drop,
                }
            )

    most_braking = max(analyses, key=lambda item: item[2]["braking_events"])
    least_braking = min(analyses, key=lambda item: item[2]["braking_events"])
    for label, (json_path, _, analysis) in (
        ("most", most_braking),
        ("least", least_braking),
    ):
        annotated_path = output_dir / f"braking_{label}.png"
        visualize_cones(json_path, annotated_path, analysis)
        print(
            f"{label.title()} braking: {json_path.stem} "
            f"({analysis['braking_events']} events) -> {annotated_path}"
        )

    print(f"Braking statistics saved to {csv_path}")
    print(f"Generated {len(json_files)} PNG files in {output_dir}")


def main(
    path: Path = DEFAULT_LAYOUT,
    lapout: Path | None = None,
    output_dir: Path = DEFAULT_OUTPUT,
    relative_radius_drop: float = DEFAULT_RELATIVE_RADIUS_DROP,
):
    """Visualize a track from the repository."""
    if lapout is not None:
        batch_visualize(lapout, output_dir, relative_radius_drop)
    else:
        visualize_cones(path)


if __name__ == "__main__":
    typer.run(main)
