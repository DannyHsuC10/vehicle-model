import csv
import numpy as np

class MaterialEngine:

    def compute(self, state):

        for r in state.active_rods.values():

            F = r.force
            A = r.area
            E = r.E
            L0 = r.L0

            stress = F / A
            strain = F / (A * E)
            delta_L = strain * L0
            stiffness = (A * E) / L0
            energy = (F**2 * L0) / (2 * A * E)

            # store state
            state.stress[r.name] = stress
            state.strain[r.name] = strain
            state.delta_L[r.name] = delta_L
            state.stiffness[r.name] = stiffness
            state.energy[r.name] = energy

            # sync object
            r.stress = stress
            r.strain = strain
            r.delta_L = delta_L
            r.k = stiffness
            r.energy = energy

            # safety check
            if r.max_stress > 0:
                r.safety_ratio = abs(stress) / r.max_stress
                r.fail = abs(stress) > r.max_stress
            else:
                r.safety_ratio = None
                r.fail = False

            state.fail[r.name] = r.fail
            state.safety_ratio[r.name] = r.safety_ratio
            state.max_stress[r.name] = r.max_stress
            
class AnalysisState:

    def __init__(self, nodes, rods):

        self.nodes = nodes
        self.rods = rods

        #  active rods only ONCE
        self.active_rods = {
            name: r
            for name, r in rods.items()
            if r.Ft == 1
        }

        self.node_forces = {}

        self.stress = {}
        self.strain = {}
        self.delta_L = {}
        self.stiffness = {}
        self.energy = {}
        self.max_stress = {}
        self.fail = {}
        self.safety_ratio = {}

class EngineeringRod:

    def __init__(self, name):

        self.name = name

        # topology
        self.a = None
        self.b = None

        # geometry
        self.pa = None
        self.pb = None

        self.L0 = None
        self.direction0 = None

        # material
        self.radius = None
        self.area = None
        self.E = None
        self.max_stress = None

        # solver result
        self.force = 0.0

        # analysis flag (raw from CSV)
        self.Ft = 1

        # results
        self.stress = None
        self.strain = None
        self.delta_L = None
        self.k = None
        self.energy = None

class EngineeringNode:

    def __init__(self, name):

        self.name = name

        self.pos = np.zeros(3)

        self.fixed = False

        self.force = np.zeros(3)

def load_nodes(path):

    nodes = {}

    with open(path, newline='', encoding='utf-8-sig') as f:

        reader = csv.DictReader(f)

        for row in reader:

            node = EngineeringNode(row["name"])

            node.pos = np.array([
                float(row["x"]),
                float(row["y"]),
                float(row["z"])
            ])

            node.fixed = bool(int(row["fixed"]))

            nodes[node.name] = node

    return nodes


def load_rod_table(path):

    with open(path, newline='', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))
    

def initialize_rod_properties(nodes, rod_table):

    rods = {}

    for row in rod_table:

        rod = EngineeringRod(row["name"])

        # topology
        rod.a = nodes[row["a"]]
        rod.b = nodes[row["b"]]

        rod.Ft = int(row.get("Ft", 1))

        # geometry
        rod.pa = rod.a.pos
        rod.pb = rod.b.pos

        d = rod.pb - rod.pa

        rod.L0 = np.linalg.norm(d)
        rod.direction0 = d / rod.L0

        # material
        rod.radius = float(row["r"])
        rod.area = float(row["A"])
        rod.E = float(row["E"])
        rod.max_stress = float(row.get("max", 0))
        rods[rod.name] = rod

    return rods

