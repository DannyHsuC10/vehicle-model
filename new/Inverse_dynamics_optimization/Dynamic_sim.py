from Model_Analysis import simulator
from Model_Analysis import output as out
from Model_Analysis.UI import build
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent


class JsonSimulation:
    """Wrap a JSON-driven simulation so it can be run directly or used by optimization."""

    def __init__(self, config_filename, dv_manager=None, x=None, nodes_path=None, rods_path=None):
        self.cfg = build.Config(BASE_DIR / config_filename)
        self.dv_manager = dv_manager
        self.x = x
        self.nodes_path = nodes_path
        self.rods_path = rods_path
        self.SIM_TIME = self.cfg.require("simulation", "SIM_TIME")
        self.DT = self.cfg.require("simulation", "DT")

    def build(self, enable_visual=False):
        sim = simulator.SimulationSystem(
            enable_visual=enable_visual,
            nodes_path=self.nodes_path,
            rods_path=self.rods_path
        )

        if self.cfg.get("geometry_override", "enable", default=False):
            nodes = self.cfg.require("geometry_override", "nodes")
            sim.override_node_positions(nodes)

        if self.dv_manager is not None and self.x is not None:
            node_dict = self.dv_manager.vector_to_dict(self.x)
            sim.override_node_positions(node_dict)

        sim.sim.prev_x = sim.solver.pack()

        build.build_inputs(sim, self.cfg, self.SIM_TIME)
        build.build_targets(sim, self.cfg)

        return sim

    def run(self, enable_visual=False):
        sim = self.build(enable_visual=enable_visual)
        sim.run(SIM_TIME=self.SIM_TIME, DT=self.DT)
        return sim, sim.success

# =========================================================
# MAIN
# =========================================================
if __name__ == "__main__":
    sim_runner = JsonSimulation("data/test_set.json")
    enable_visual = sim_runner.cfg.get("simulation", "enable_visual", default=False)
    sim, success = sim_runner.run(enable_visual=enable_visual)

    if not success:
        raise RuntimeError("Simulation failed.")

    if sim_runner.cfg.get("output", "save_csv", default=False):
        out.save_to_csv(sim)

    plot_cfg = sim_runner.cfg.get("output", "plot", default=[])
    save = sim_runner.cfg.get("output", "save_plot", default=False)
    sim.plot(plot_cfg, save)
