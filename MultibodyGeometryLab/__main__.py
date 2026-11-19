import argparse
from pathlib import Path
import tkinter as tk

from .gui import Workbench
from .project import default_project, read_project


def main():
    parser = argparse.ArgumentParser(description="Four-corner multibody suspension geometry workbench.")
    parser.add_argument("--project", type=Path, help="Open a MultibodyGeometryLab or SuspensionGeometryLab JSON file.")
    args = parser.parse_args()
    root = None
    try:
        data, native = read_project(args.project) if args.project else (default_project(), False)
        root = tk.Tk()
        Workbench(root, data, args.project if native else None)
    except (ValueError, ArithmeticError, OSError) as error:
        if root is not None:
            root.destroy()
        parser.error(str(error))
    root.mainloop()


if __name__ == "__main__":
    main()
