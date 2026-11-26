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
PUBLIC_TIRE = "input_data/tir/lp03-demo.mf61"
SOLVER_FILES = (
    "__init__.py", "aero_setup.py", "geometry_tables.py", "lap.py", "model.py",
    "plots.py", "resources.py", "straight.py", "lp03.json",
    "input_data/LP03.json",
    "input_data/lp03_voltage_envelope.csv", "input_data/modal_aero.json",
    PUBLIC_TIRE, "examples/skidpad.csv",
)


def zero_parameters(value):
    if isinstance(value, dict):
        return {key: zero_parameters(item) for key, item in value.items()}
    if isinstance(value, list):
        return [zero_parameters(item) for item in value]
    if type(value) in (bool, int, float):
        return type(value)(0)
    return value


def main():
    sys.path.insert(0, str(ROOT))
    from Vehicle14DOFLab.lap import Track
    from Vehicle14DOFLab.aero_setup import aero_settings
    from Vehicle14DOFLab.resources import SKIDPAD
    from Vehicle14DOFLab._vendor.pacejka.tir_io import read_tir, write_tir

    vehicle = json.loads((LAB / "lp03.json").read_text(encoding="utf-8"))
    if vehicle["tire_file"] != PUBLIC_TIRE:
        raise ValueError("The web build only packages the public LP03 demo tire.")

    DIST.mkdir(parents=True, exist_ok=True)
    for name in ("index.html", "favicon.svg", "style.css", "app.js", "charts.js", "replay.js",
                 "vehicle-settings.js", "worker.js"):
        copy2(WEB / name, DIST / name)
    with ZipFile(DIST / "solver.zip", "w", ZIP_DEFLATED) as archive:
        for name in SOLVER_FILES:
            archive.write(LAB / name, f"Vehicle14DOFLab/{name}")
        for path in sorted((LAB / "_vendor").rglob("*.py")):
            archive.write(path, path.relative_to(ROOT).as_posix())
        archive.write(LAB / "_vendor/pacejka/LICENSE", "Vehicle14DOFLab/_vendor/pacejka/LICENSE")
        archive.write(WEB / "browser_solver.py", "browser_solver.py")
    track = Track.from_csv(SKIDPAD)
    courses = {
        "skidpad": {"length": float(track.length)},
        "straight": {"length": 75.},
    }
    (DIST / "courses.json").write_text(json.dumps(courses, separators=(",", ":")), encoding="utf-8")
    copy2(SKIDPAD, DIST / "skidpad-example.csv")
    vehicle["aero"] = aero_settings(vehicle, LAB)
    suspension = json.loads((LAB / "input_data/LP03.json").read_text(encoding="utf-8-sig"))
    defaults = {"vehicle": vehicle, "suspension": suspension}
    (DIST / "vehicle-defaults.json").write_text(json.dumps(defaults, ensure_ascii=False), encoding="utf-8")

    examples = DIST / "examples"
    examples.mkdir(exist_ok=True)
    inputs = {"suspension_project": "suspension.json", "tire_file": "tire.tir", "motor_envelope": "motor.csv"}
    copy2(LAB / vehicle["motor_envelope"], examples / inputs["motor_envelope"])
    blank_suspension = zero_parameters(suspension) | {
        "schema_version": suspension["schema_version"], "name": "Suspension template", "notes": [],
    }
    for name, data in (
        ("vehicle.json", zero_parameters(vehicle) | inputs),
        ("suspension.json", blank_suspension),
        ("aero.json", zero_parameters(vehicle["aero"]["coefficients"])),
    ):
        (examples / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    tire_source = LAB / PUBLIC_TIRE
    tire_metadata = {("MDI_HEADER", "FILE_VERSION"), ("MODEL", "FITTYP")}
    blank_tire = {(section, key): 0 for section, parameters in read_tir(tire_source).sections.items()
                  for key, value in parameters.items()
                  if isinstance(value, (int, float)) and (section, key) not in tire_metadata}
    write_tir(tire_source, examples / inputs["tire_file"], blank_tire, validate=False)

    copy2(LAB / "_vendor/pacejka/LICENSE", DIST / "pacejka-license.txt")
    print(DIST)


if __name__ == "__main__":
    main()
