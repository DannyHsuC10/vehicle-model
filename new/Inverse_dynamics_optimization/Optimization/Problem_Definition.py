
# =========================(最佳化問題定義)
class OptimizationProblem:
    def __init__(self, simulation_builder, objective, bounds, x0):
        self.simulation_builder = simulation_builder
        self.objective  = objective
        self.bounds = bounds
        self.x0 = x0

        self._history = []

    def evaluate(self, x):
        sim_runner = self.simulation_builder(x)
        sim, success = sim_runner.run(enable_visual=False)

        if not success:
            cost = 1e10
            self._history.append(cost)
            return cost, False

        metrics = self.objective.extract(sim)
        cost = self.objective.compute(metrics)

        self._history.append(cost)
        return cost, True

    def simulate(self, x, visualize=False):
        sim_runner = self.simulation_builder(x)
        return sim_runner.run(enable_visual=visualize)

    def reset_history(self):
        self._history = []

    @property
    def history(self):
        return self._history
