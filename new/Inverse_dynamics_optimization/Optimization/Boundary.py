# 邊界條件設定
import pandas as pd
from pathlib import Path
import csv

#from Model_Analysis.Inverse_dynamics import Multibody_engine as mb# 不直接導入避免衝突
#from Model_Analysis.Inverse_dynamics.Static_engine import load_nodes

FILTER_DIR = Path(__file__).resolve().parent.parent
nodes_csv_path = Path(FILTER_DIR / "data" / "nodes.csv")

design_vars_csv_path = Path(FILTER_DIR / "data" / "design_vars.csv")

def load_nodes(path, NodeClass, visual=True):# 和靜態引擎一樣的函式，為了避免呼叫關係偶和太嚴重後面改到出事
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

class GeometryProvider:
    def __init__(self):
        self.nodes_csv_path = nodes_csv_path
        self._base_nodes = None

        df = pd.read_csv(design_vars_csv_path)
        self.target_names = df["name"].unique()

    def get_base_nodes(self):
        if self._base_nodes is None:
            from Model_Analysis.Inverse_dynamics import Multibody_engine as mb# 嘗試延後導入必免衝突(但似乎沒有用)
            nodes = load_nodes(self.nodes_csv_path, mb.Node, visual=False)

            if self.target_names is None:
                self._base_nodes = {
                    name: node.pos.copy()
                    for name, node in nodes.items()
                }
            else:
                self._base_nodes = {}

                for name in self.target_names:
                    if name not in nodes:
                        raise ValueError(f"{name} not found in nodes.csv")

                    self._base_nodes[name] = nodes[name].pos.copy()

        return self._base_nodes

class DesignVariableManager:
    def __init__(self, base_nodes):
        """
        base_nodes:
            dict
            ex:
            {
                "tie_rod": [30, -20, 10],
                "rack": [0, 0, 0]
            }
        """
        self.df = pd.read_csv(design_vars_csv_path)

        self.names = list(zip(self.df["name"], self.df["axis"]))
        self.bounds = list(zip(self.df["lower"], self.df["upper"]))
        self.x0 = self.df["initial"].values

        self.base_nodes = base_nodes

    def vector_to_dict(self, x):
        result = {}

        # 先複製原始 geometry
        for name, coord in self.base_nodes.items():
            result[name] = coord.copy()

        # 只覆蓋被最佳化軸向
        for (name, axis), value in zip(self.names, x):
            idx = {"x": 0, "y": 1, "z": 2}[axis]
            result[name][idx] = value

        return result
