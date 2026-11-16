"""Convert a Pacejka tire-force map (.npz or .pkl) to an envelope .npz."""

from __future__ import annotations

import argparse
from pathlib import Path
from tempfile import NamedTemporaryFile

import numpy as np

if __package__:
    from .tire_mapping_tool import DEFAULT_OUTPUTS, TireForceMap
else:
    from tire_mapping_tool import DEFAULT_OUTPUTS, TireForceMap


def _sample_fx_limits(
    fx: np.ndarray, fy: np.ndarray, fy_samples: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Intersect each sampled Fy with the closed convex hull in the Fx–Fy plane."""
    next_fx = np.roll(fx, -1)
    next_fy = np.roll(fy, -1)
    fy_step = next_fy - fy
    target = fy_samples[:, None]
    intersects = (target >= np.minimum(fy, next_fy)) & (
        target <= np.maximum(fy, next_fy)
    )
    # At constant-Fy edges the endpoints are also vertices of adjacent edges.
    weight = np.divide(
        target - fy, fy_step, out=np.zeros(intersects.shape), where=fy_step != 0
    )
    crossings = fx + weight * (next_fx - fx)
    # A branch with no Fx of its sign has zero available force.
    brake = np.min(crossings, axis=1, where=intersects, initial=0)
    drive = np.max(crossings, axis=1, where=intersects, initial=0)
    return brake, drive


def export_envelope(
    map_path: str | Path, output_path: str | Path = "pacejka_envelope.npz"
) -> Path:
    """Save the signed Fx limits on 301 evenly spaced Fy fractions per load.

    The fraction runs from fy_min to fy_max, and fy_pure is the larger
    absolute Fy limit. The 1D rrt is sampled at zero slip angle and ratio,
    or the closest in-grid values when zero is outside.
    """
    map_path, output_path = Path(map_path), Path(output_path)
    if map_path.suffix.lower() not in {".npz", ".pkl"}:
        raise ValueError("Input must be a Pacejka tire-force map (.npz or .pkl).")
    if output_path.suffix.lower() != ".npz" or map_path.resolve() == output_path.resolve():
        raise ValueError("Choose a separate .npz file for the envelope output.")

    tire_map = TireForceMap.load(map_path)
    fz = tire_map.fz_axis
    slip_angle = tire_map.alpha_axis_deg
    slip_ratio = tire_map.kappa_axis
    tables = tire_map.payload["tables"]
    hulls = tire_map.payload["limit_envelopes"]

    fy_fraction = np.linspace(0.0, 1.0, 301)
    fy_min = np.empty_like(fz)
    fy_max = np.empty_like(fz)
    brake = np.empty((fz.size, fy_fraction.size))
    drive = np.empty_like(brake)
    for index, load in enumerate(fz):
        if load in hulls:
            fx, fy = hulls[load]["Fx"], hulls[load]["Fy"]
        else:
            # The mapping tool omits hulls with fewer than three unique points.
            points = np.unique(
                np.column_stack((tables["FX"][index].ravel(), tables["FY"][index].ravel())),
                axis=0,
            )
            if not 0 < len(points) < 3:
                raise ValueError(f"Missing force envelope at Fz={load:g} N.")
            fx, fy = points.T
        fy_min[index], fy_max[index] = fy.min(), fy.max()
        fy_samples = fy_min[index] + fy_fraction * (fy_max[index] - fy_min[index])
        fy_samples[-1] = fy_max[index]
        brake[index], drive[index] = _sample_fx_limits(fx, fy, fy_samples)

    rrt = tire_map.evaluate(
        fz,
        np.clip(0.0, slip_angle.min(), slip_angle.max()),
        np.clip(0.0, slip_ratio.min(), slip_ratio.max()),
        outputs=("RRT",),
    )["RRT"]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    file = NamedTemporaryFile(dir=output_path.parent, delete=False)
    temporary_path = Path(file.name)
    try:
        with file:
            np.savez_compressed(
                file,
                fz=fz,
                fy_fraction=fy_fraction,
                fy_min=fy_min,
                fy_max=fy_max,
                fy_pure=np.maximum(-fy_min, fy_max),
                brake=brake,
                drive=drive,
                rrt=rrt,
                slip_angle=slip_angle,
                slip_ratio=slip_ratio,
                **{f"grid_{name}": tables[name].astype(np.float32) for name in DEFAULT_OUTPUTS},
            )
        temporary_path.replace(output_path)
    finally:
        temporary_path.unlink(missing_ok=True)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_path", type=Path, help="Input .npz or trusted .pkl tire-force map")
    parser.add_argument("--output", type=Path, default=Path("pacejka_envelope.npz"))
    args = parser.parse_args()
    try:
        output_path = export_envelope(args.map_path, args.output)
    except (OSError, KeyError, ValueError) as error:
        parser.error(str(error))
    print(f"Saved Pacejka envelope to {output_path.resolve()}")


if __name__ == "__main__":
    main()
