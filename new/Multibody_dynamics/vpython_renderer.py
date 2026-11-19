# vpython_renderer.py
from vpython import *

# ============================================================
# Visual Node
# ============================================================

class VisualNode:

    def __init__(self, node_data):

        self.node = node_data

        self.body = sphere(
            pos=vector(*self.node.pos),
            radius=15,
            color=color.cyan,
            opacity=0.5
        )

        self.label = label(
            pos=vector(*self.node.pos),
            text=self.node.name,
            xoffset=20,
            yoffset=20,
            height=10,
            box=False
        )

    # ========================================================

    def update(self):

        self.body.pos = vector(*self.node.pos)

        self.label.pos = vector(*self.node.pos)


# ============================================================
# Visual Rod
# ============================================================

class VisualRod:

    def __init__(self, rod_data):

        self.rod = rod_data

        a = self.rod.a.pos
        b = self.rod.b.pos

        self.body = cylinder(
            pos=vector(*a),
            axis=vector(*(b - a)),
            radius=self.rod.radius,
            opacity=self.rod.opacity
        )

    # ========================================================

    def update(self):

        a = self.rod.a.pos
        b = self.rod.b.pos

        self.body.pos = vector(*a)

        self.body.axis = vector(*(b - a))


# ============================================================
# Visual Wheel
# ============================================================

class VisualWheel:

    def __init__(self, wheel_data):

        self.wheel = wheel_data

        a = self.wheel.node1.pos
        b = self.wheel.node2.pos

        self.body = cylinder(
            pos=vector(*a),
            axis=vector(*(b - a)),
            radius=self.wheel.radius,
            color=color.gray(self.wheel.color),
            opacity=self.wheel.opacity
        )

    # ========================================================

    def update(self):

        a = self.wheel.node1.pos
        b = self.wheel.node2.pos

        self.body.pos = vector(*a)

        self.body.axis = vector(*(b - a))


# ============================================================
# Renderer
# ============================================================

class VPythonRenderer:

    def __init__(self, mechanism_data):

        self.data = mechanism_data

        self.visual_nodes = {}
        self.visual_rods = {}
        self.visual_wheels = {}

        self.build()
        

    # ========================================================

    def build(self):

        # ====================================================
        # nodes
        # ====================================================

        for name, node in self.data.nodes.items():

            self.visual_nodes[name] = VisualNode(node)

        # ====================================================
        # rods
        # ====================================================

        for name, rod in self.data.rods.items():

            self.visual_rods[name] = VisualRod(rod)

        # ====================================================
        # wheels
        # ====================================================

        for name, wheel in self.data.wheels.items():

            self.visual_wheels[name] = VisualWheel(wheel)

    # ========================================================

    def update(self):

        for node in self.visual_nodes.values():

            node.update()

        for rod in self.visual_rods.values():

            rod.update()

        for wheel in self.visual_wheels.values():

            wheel.update()


# ============================================================
# Simple Static Test
# ============================================================

if __name__ == "__main__":# show static structure

    from pathlib import Path
    from loader import load_mechanism_data
    from Core_Engine.Multibody_engine_v3 import MechanismSolver


    BASE_DIR = Path(__file__).resolve().parent

    nodes_path = BASE_DIR / "data" / "nodes.csv"
    rods_path = BASE_DIR / "data" / "rods.csv"
    wheels_path = BASE_DIR / "data" / "wheel.csv"

    # ========================================================
    # load mechanism
    # ========================================================

    mech = load_mechanism_data(
        nodes_path,
        rods_path,
        wheels_path
    )

    # ========================================================
    # create solver
    # ========================================================

    solver = MechanismSolver(mech)

    # ========================================================
    # create renderer
    # ========================================================
    print("正在建立 VPython 渲染器...")
    renderer = VPythonRenderer(mech)

    # ========================================================
    # static loop
    # ========================================================
    print("正在啟動 VPython 渲染迴圈...")
    while True:
        # =========================
        # VPython 視窗設定
        # =========================
        scene.width = 1600
        scene.height = 800
        scene.autoscale = False

        rate(60)

        renderer.update()
    