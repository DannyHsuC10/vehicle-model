# loader.py
import csv
import numpy as np


# ============================================
# Raw Data Model
# ============================================

class NodeData:

    def __init__(self, name, pos, fixed=False):

        self.name = name
        self.pos = np.array(pos, dtype=float)
        self.fixed = fixed
        self.m = 0.0


class RodData:
    def __init__(self, name, node_a, node_b):
        self.name = name

        # topology
        self.a = node_a
        self.b = node_b

        # geometry
        self.radius = 10.0
        self.opacity = 1.0

        # solver (Ft=1: 剛性拘束, Ft=0: 彈簧阻尼或純外力桿)
        self.Ft = 1

        # ====================================================
        # 新增彈簧阻尼特性 (Spring-Damper)
        # ====================================================
        self.k = 0.0  # 彈簧剛度
        self.c = 0.0  # 阻尼係數

        # material
        self.area = None
        self.E = None
        self.max_stress = None
        
class WheelData:

    def __init__(
        self,
        name,
        node1,
        node2,
        radius
    ):

        self.name = name

        self.node1 = node1
        self.node2 = node2

        self.radius = radius
        self.opacity = 0.8
        self.color = 0.5


# ============================================
# Loader
# ============================================

class MechanismData:

    def __init__(self):

        self.nodes = {}
        self.rods = {}
        self.wheels = {}


# ============================================
# CSV Loader
# ============================================
def load_mechanism_data(nodes_path, rods_path, wheels_path=None):
    mech = MechanismData()

    # ========================================
    # Nodes (保持不變)
    # ========================================
    with open(nodes_path, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row["name"]
            pos = [float(row["x"]), float(row["y"]), float(row["z"])]
            fixed = bool(int(row["fixed"]))
            m = float(row.get("m", 0.0))
            mech.nodes[name] = NodeData(name=name, pos=pos, fixed=fixed)
            mech.nodes[name].m = m

    # ========================================
    # loader.py 內的 Rods 讀取區塊
    # ========================================
    with open(rods_path, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row["name"]
            a = mech.nodes[row["a"]]
            b = mech.nodes[row["b"]]

            rod = RodData(name, a, b)

            # 讀取基礎幾何與邏輯
            rod.radius = float(row.get("r", 10))
            rod.opacity = float(row.get("opa", 1.0))
            rod.Ft = int(row.get("Ft", 1))

            # ====================================================
            # 🚀 根據 Ft = 0 進行 if 判斷與預設值設定
            # ====================================================
            if rod.Ft == 0:
                # 如果是彈簧阻尼桿 (Ft=0)，且 CSV 沒寫 k, c，就給一組標準避震器預設值
                rod.k = float(row.get("k", 150.0))  # 預設剛度 150
                rod.c = float(row.get("c", 2.5))    # 預設阻尼 2.5
            else:
                # 如果是剛性桿 (Ft=1)，k 和 c 自然就是 0
                rod.k = 0.0
                rod.c = 0.0

            # 材料性質 (保持不變)
            rod.area = float(row.get("A", 0))
            rod.E = float(row.get("E", 0))
            rod.max_stress = float(row.get("max", 0))

            mech.rods[name] = rod\
            
    # ========================================
    # Wheels (保持不變)
    # ========================================
    if wheels_path is not None:
        with open(wheels_path, newline='', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = row["name"]
                node1 = mech.nodes[row["P1"]]
                node2 = mech.nodes[row["P2"]]
                wheel = WheelData(name=name, node1=node1, node2=node2, radius=float(row["R"]))
                wheel.opacity = float(row.get("opa", 0.8))
                wheel.color = float(row.get("color", 0.5))
                mech.wheels[name] = wheel

    return mech

# =========================
if __name__ == "__main__":
    from pathlib import Path

    BASE_DIR = Path(__file__).resolve().parent


    # ============================================
    # paths
    # ============================================

    nodes_path = BASE_DIR / "data" / "nodes.csv"
    rods_path = BASE_DIR / "data" / "rods.csv"
    wheels_path = BASE_DIR / "data" / "wheel.csv"


    # ============================================
    # load
    # ============================================

    mech = load_mechanism_data(
        nodes_path,
        rods_path,
        wheels_path
    )


    # ============================================
    # summary
    # ============================================

    print("\n==============================")
    print("MECHANISM LOAD SUCCESS")
    print("==============================")

    print(f"Nodes  : {len(mech.nodes)}")
    print(f"Rods   : {len(mech.rods)}")
    print(f"Wheels : {len(mech.wheels)}")


    # ============================================
    # nodes
    # ============================================

    print("\n--- Nodes ---")

    for name, node in mech.nodes.items():

        print(
            f"{name:15s} | "
            f"pos={node.pos} | "
            f"fixed={node.fixed}"
        )


    # ============================================
    # rods
    # ============================================

    print("\n--- Rods ---")

    for name, rod in mech.rods.items():

        print(
            f"{name:15s} | "
            f"{rod.a.name} -> {rod.b.name} | "
            f"Ft={rod.Ft}"
        )


    # ============================================
    # wheels
    # ============================================

    print("\n--- Wheels ---")

    for name, wheel in mech.wheels.items():

        print(
            f"{name:15s} | "
            f"{wheel.node1.name} -> {wheel.node2.name} | "
            f"R={wheel.radius}"
        )