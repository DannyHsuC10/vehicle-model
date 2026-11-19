from pathlib import Path

from loader import load_mechanism_data
from Core_Engine.Multibody_engine_v3 import MechanismSolver
from vpython_renderer import VPythonRenderer


def main():
    base_dir = Path(__file__).resolve().parent
    nodes_path = base_dir / "data" / "nodes.csv"
    rods_path = base_dir / "data" / "rods.csv"
    wheels_path = base_dir / "data" / "wheel.csv"

    print("Loading mechanism data...")
    mech = load_mechanism_data(nodes_path, rods_path, wheels_path)
    print(
        "Loaded mechanism:",
        "nodes=", len(mech.nodes),
        "rods=", len(mech.rods),
        "wheels=", len(mech.wheels),
    )

    print("Creating VPythonRenderer...")
    renderer = VPythonRenderer(mech)
    print("VPythonRenderer created successfully:", renderer)

    print("Calling renderer.update() once...")
    renderer.update()
    print("renderer.update() returned")

    print("Initialization test completed. If a VPython window opened, close it manually.")


if __name__ == "__main__":
    main()
