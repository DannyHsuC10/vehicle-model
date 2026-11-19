# 保留每種輸出類型的最新檔案，刪除舊檔案
from pathlib import Path
import re

PATTERNS = [
    r"^(output)_\d{8}_\d{6}\.[^./]+$",
    r"^(rod_forces)_\d{8}_\d{6}\.[^./]+$",
    r"^(static_report)_\d{8}_\d{6}\.[^./]+$",
]



def latest_files_by_prefix(directory: Path) -> dict[str, Path]:
    latest = {}
    regexes = [re.compile(pattern) for pattern in PATTERNS]

    for file_path in directory.iterdir():
        if not file_path.is_file():
            continue

        name = file_path.name
        for regex in regexes:
            match = regex.match(name)
            if not match:
                continue

            prefix = match.group(1)  # 直接拿類型名稱

            current_latest = latest.get(prefix)
            if current_latest is None or name > current_latest.name:
                latest[prefix] = file_path
            break

    return latest


def cleanup_old_outputs(directory: Path) -> int:
    latest = latest_files_by_prefix(directory)
    deleted_count = 0

    for file_path in directory.iterdir():
        if not file_path.is_file():
            continue

        name = file_path.name
        for prefix, latest_file in latest.items():
            if name == latest_file.name:
                break
            if name.startswith(prefix + "_") and re.match(rf"^{re.escape(prefix)}_\d{{8}}_\d{{6}}\.[^./]+$", name):
                try:
                    file_path.unlink()
                    deleted_count += 1
                except OSError as error:
                    print(f"無法刪除 {file_path}: {error}")
                break

    return deleted_count


def main() -> None:
    output_dir = Path(__file__).parent
    print(f"正在保留 {output_dir} 中每種輸出類型的最新檔案...\n")

    latest = latest_files_by_prefix(output_dir)
    if latest:
        print("最新檔案：")
        for prefix, file_path in latest.items():
            print(f"  {prefix}: {file_path.name}")
        print()

    deleted = cleanup_old_outputs(output_dir)

    if deleted:
        print(f"已刪除 {deleted} 個舊檔案。")
    else:
        print("沒有需要刪除的舊檔案。")


if __name__ == "__main__":
    main()
