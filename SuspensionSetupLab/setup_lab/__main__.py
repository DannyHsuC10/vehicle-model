import argparse
import tkinter as tk

from .engine import simulate
from .analysis_page import write_csv
from .gui import SetupApp
from .project import default_project, open_project


def main():
    parser = argparse.ArgumentParser(description="SuspensionSetupLab 懸吊調校")
    parser.add_argument("--project", help="開啟／匯入 JSON 專案")
    parser.add_argument("--csv", help="直接計算並輸出 CSV")
    args = parser.parse_args()
    try:
        project, destination = open_project(args.project) if args.project else (default_project(), None)
        if args.csv:
            write_csv(args.csv, simulate(project))
            return
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    root = tk.Tk()
    SetupApp(root, project, destination)
    root.mainloop()


if __name__ == "__main__":
    main()
