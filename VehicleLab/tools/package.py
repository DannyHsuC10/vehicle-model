"""Create a portable source archive without logs, caches or external packages."""
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    selected=[ROOT/name for name in ("README.md","VALIDATION.md","run.py","Start-VehicleLab.bat","Start-VehicleLab.ps1","Start-Tests.bat","requirements-optional.txt",".gitignore")]
    for folder in ("vehicle_lab","tests","tools","examples","input_data","projects","計算說明"):
        selected.extend(p for p in (ROOT/folder).rglob("*") if p.is_file() and "__pycache__" not in p.parts)
    with zipfile.ZipFile(ROOT/"VehicleLab.zip","w",zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(selected):
            if path.exists():archive.write(path,Path("VehicleLab")/path.relative_to(ROOT))
    print(ROOT/"VehicleLab.zip")


if __name__=="__main__":main()
