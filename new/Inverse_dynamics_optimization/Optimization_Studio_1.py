# Optimized api.py (最佳化測試)
import numpy as np
import Steering as st
from Optimization import Problem_Definition as Pd
from Optimization import Boundary as bd
from Optimization import Optimizer as opt

# ========================(問題定義與求解)
class SteeringObjective:# 目標成本函數
    def extract(self, sim):
        fx = np.array(sim.logger.data["tie_rod_fx"])
        vx = np.array(sim.logger.data["tie_rod_vx"])

        return {
            "force_std": np.std(fx),
            "force_max": np.max(np.abs(fx)),
            "velocity_mean": np.mean(np.abs(vx)),
        }

    def compute(self, metrics):
        return (100
            #1 * metrics["force_std"] -
            -0.5 * metrics["force_max"])

geo_provider = bd.GeometryProvider()
dv_manager = bd.DesignVariableManager(geo_provider.get_base_nodes())

# builder
def steering_sim_builder(x):
    return st.SteeringSimulation(dv_manager, x)

# components
objective = SteeringObjective()

# problem（通用)
problem = Pd.OptimizationProblem(
    simulation_builder=steering_sim_builder,
    objective=objective,
    bounds=dv_manager.bounds,
    x0=dv_manager.x0
)

# optimize
optimizer = opt.ScipyOptimizer(method="L-BFGS-B", maxiter=5)
best_x, best_cost, history, sim, success = optimizer.optimize(problem)
