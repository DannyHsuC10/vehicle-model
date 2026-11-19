from Model_Analysis import output
import numpy as np
from Model_Analysis.Inverse_dynamics import Multibody_engine as mb
from Model_Analysis.Inverse_dynamics import Static_engine as se
from  Model_Analysis.Inverse_dynamics import Material_engine as mater

def solve_static(nodes, rods):

    state = mater.AnalysisState(nodes, rods)

    sa = se.StaticAnalyzer(state)

    sa.solve()

    state.node_forces = sa.compute_node_forces()

    material_engine = mater.MaterialEngine()
    material_engine.compute(state)

    return state

class StaticSimulationRunner:

    def __init__(self, cfg):

        self.cfg = cfg

        # =========================
        # engineering objects
        # =========================
        self.eng_nodes = None
        self.eng_rods = None

        # =========================
        # visual objects
        # =========================
        self.visual_nodes = None
        self.visual_rods = None

        # =========================
        # results
        # =========================
        self.state = None
        self.report = None

    def build(self, static_cfg):

        files = static_cfg["files"]

        # ==================================================
        # ENGINEERING OBJECTS
        # ==================================================

        eng_nodes = mater.load_nodes(
            files["nodes_path"]
        )

        rod_table = mater.load_rod_table(
            files["rods_path"]
        )

        eng_rods = mater.initialize_rod_properties(
            eng_nodes,
            rod_table
        )

        # ==================================================
        # VISUAL OBJECTS (VPYTHON)
        # ==================================================

        visual_nodes = se.load_nodes(
            files["nodes_path"],
            mb.Node
        )

        visual_rods = se.load_rods(
            files["rods_path"],
            visual_nodes,
            mb.Rod
        )

        return (
            eng_nodes,
            eng_rods,
            visual_nodes,
            visual_rods
        )
    
    def apply_forces(self):

        static_cfg = self.cfg.static()

        for f in static_cfg.get("forces", []):

            node_name = f["node"]

            value = np.array(f["value"])

            if node_name in self.eng_nodes:

                self.eng_nodes[node_name].force = value

        return self

    def solve(self):
        
        self.state = solve_static(
            self.eng_nodes,
            self.eng_rods
        )

        self.report = output.build_report(
            self.state
        )

        return self
        
    def export(self):

        out_cfg = self.cfg.static().get("output", {})

        if out_cfg.get("report", False):

            output.save_report(self.report)

        if out_cfg.get("csv", False):

            output.save_rod_forces_csv(
                self.eng_rods
            )

        return self

    def run(self):

        static_cfg = self.cfg.static()

        (
            self.eng_nodes,
            self.eng_rods,
            self.visual_nodes,
            self.visual_rods
        ) = self.build(static_cfg)

        return (
            self.apply_forces()
                .solve()
                .export()
        )

