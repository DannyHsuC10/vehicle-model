"""Package the static 14DOF workbench and its shared Python solver."""

from pathlib import Path
import json
import sys
from shutil import copy2
from zipfile import ZIP_DEFLATED, ZipFile


ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "web"
LAB = ROOT / "Vehicle14DOFLab"
DIST = ROOT / "dist" / "vehicle14dof"
SOLVER_FILES = (
    "__init__.py", "aero_setup.py", "geometry_tables.py", "lap.py", "model.py",
    "plots.py", "resources.py", "straight.py", "lp03.json",
    "input_data/LP03.json", "input_data/fsae_A_2025.csv",
    "input_data/lp03_voltage_envelope.csv", "input_data/modal_aero.json",
    "input_data/tir/Hoosier_43075_R20_W8_candidate.tir", "examples/skidpad.csv",
)


def main():
    sys.path.insert(0, str(ROOT))
    from Vehicle14DOFLab.lap import Track
    from Vehicle14DOFLab.resources import DEFAULT_TRACK, SKIDPAD

    DIST.mkdir(parents=True, exist_ok=True)
    for name in ("index.html", "style.css", "app.js", "charts.js", "worker.js"):
        copy2(WEB / name, DIST / name)
    with ZipFile(DIST / "solver.zip", "w", ZIP_DEFLATED) as archive:
        for name in SOLVER_FILES:
            archive.write(LAB / name, f"Vehicle14DOFLab/{name}")
        for path in sorted((LAB / "_vendor").rglob("*.py")):
            archive.write(path, path.relative_to(ROOT).as_posix())
        archive.write(LAB / "_vendor/pacejka/LICENSE", "Vehicle14DOFLab/_vendor/pacejka/LICENSE")
        archive.write(WEB / "browser_solver.py", "browser_solver.py")
    guides = {}
    for mode, path in (("lap", DEFAULT_TRACK), ("skidpad", SKIDPAD)):
        track = Track.from_csv(path)
        guides[mode] = {
            "xy": [*track.xy.tolist(), track.xy[0].tolist()],
            "length": float(track.length),
        }
    (DIST / "courses.json").write_text(json.dumps(guides, separators=(",", ":")), encoding="utf-8")
    copy2(LAB / "_vendor/pacejka/LICENSE", DIST / "pacejka-license.txt")
    print(DIST)


if __name__ == "__main__":
    main()
