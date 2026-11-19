# static_engine.py
# 產生靜態畫面確認座標正確用
from vpython import *
import csv
import numpy as np

def load_nodes(path, NodeClass, visual=True):# 載入點
    nodes = {}
    
    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        #print(reader.fieldnames)# 印出

        for row in reader:
            pos = [float(row['x']), float(row['y']), float(row['z'])]
            fixed = bool(int(row['fixed']))
            
            node = NodeClass(pos, fixed=fixed, visual=visual)
            nodes[row['name']] = node
    
    return nodes

def load_rods(path, nodes, RodClass, visual=True):# load rod csv

    rods = {}

    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)

        for row in reader:
            a = nodes[row["a"]]
            b = nodes[row["b"]]

            name = row["name"]
            if name in rods:
                raise ValueError(f"Rod name 重複: {name}")

            # =========================
            # 解析數值（安全轉型）
            # =========================
            def fnum(key, default=None):
                val = row.get(key, None)
                return default if val in [None, ""] else float(val)

            def fint(key, default=None):
                val = row.get(key, None)
                return default if val in [None, ""] else int(val)

            # =========================
            # visual parameters
            # =========================
            radius = fnum("r", 10)
            opacity = fnum("opa", 1.0)

            # =========================
            # solver control
            # =========================
            Ft = fint("Ft", 1)  # 1 = active, 0 = ignore

            # =========================
            # create rod
            # =========================
            rod = RodClass(
                a, b,
                name=name,
                radius=radius,
                opa=opacity,
                visual=visual
            )

            # attach extra properties
            rod.use_in_solver = bool(Ft)

            rods[name] = rod

    return rods

def load_wheels(path, nodes, WheelClass, visual=True):# load_wheels csv

    wheels = {}

    with open(path, newline='', encoding='utf-8-sig') as f:

        reader = csv.DictReader(f)

        for row in reader:

            name = row["name"]

            # ===== node mapping =====
            node1 = nodes[row["P1"]]
            node2 = nodes[row["P2"]]

            # ===== parameters =====
            radius = float(row["R"])

            opacity = (
                float(row["opa"])
                if row["opa"] != ""
                else 0.8
            )

            # ===== optional color =====
            if "color" in row and row["color"] != "":
                gray_level = float(row["color"])
                wheel_color = color.gray(gray_level)
            else:
                wheel_color = color.gray(0.5)

            # ===== create =====
            wheel = WheelClass(
                node1,
                node2,
                radius=radius,
                color=wheel_color,
                opacity=opacity,
                visual=visual
            )

            wheels[name] = wheel

    return wheels

def show_structure(nodes, rods):# 靜態顯示

    # 畫 Node
    for name, node in nodes.items():
        pos = node.pos

        node.ball.pos = vector(*pos)

        label(
            pos=vector(*pos),
            text=f"{name}",
            xoffset=20,
            yoffset=20,
            height=10,
            box=False
        )

    # 畫 Rod（只畫一次）
    for rod in rods.values():
        a = rod.a.pos
        b = rod.b.pos

        rod.cyl = cylinder(
            pos=vector(*a),
            axis=vector(*(b - a)),
            radius=1)

# =================================(力學工具)

class StaticAnalyzer:

    def __init__(self, state):

        self.state = state
        self.nodes = state.nodes

        self.rods = state.active_rods  # dict

    # =========================
    # Solve static equilibrium
    # =========================
    def solve(self):

        nodes = self.nodes
        rods = list(self.rods.values())

        A = []
        b = []

        for node in nodes.values():

            if node.fixed:
                continue

            for axis in range(3):

                row = np.zeros(len(rods))

                for i, r in enumerate(rods):

                    d = r.b.pos - r.a.pos
                    L = np.linalg.norm(d)

                    if L < 1e-12:
                        continue

                    u = d / L

                    if r.a == node:
                        sign = 1
                    elif r.b == node:
                        sign = -1
                    else:
                        continue

                    row[i] += sign * u[axis]

                A.append(row)
                b.append(-node.force[axis])

        A = np.array(A)
        b = np.array(b)

        x, *_ = np.linalg.lstsq(A, b, rcond=None)

        # write back
        for i, r in enumerate(rods):
            r.force = x[i]

        return x
        
        # Compute node force balance
    def compute_node_forces(self):

        results = {}

        for node_name, node in self.nodes.items():

            total = np.zeros(3)

            for r in self.rods.values():

                d = r.b.pos - r.a.pos
                L = np.linalg.norm(d)

                if L < 1e-12:
                    continue

                u = d / L

                if r.a == node:
                    total += r.force * u

                elif r.b == node:
                    total -= r.force * u

            total += node.force

            results[node_name] = total

        return results
    
