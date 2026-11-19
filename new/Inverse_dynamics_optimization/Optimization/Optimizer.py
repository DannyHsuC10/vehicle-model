# 最佳化方法

from scipy.optimize import minimize
from scipy.optimize import differential_evolution
import cma
import numpy as np
from pathlib import Path
import os
from datetime import datetime

FILTER_DIR = Path(__file__).resolve().parent

class BaseOptimizer:
    def run_post_process(self, problem, best_x, best_cost):
        print("\n" + "="*30)
        print(f"Optimization Finished!")
        print(f"Best Cost: {best_cost:.6f}")
        print(f"Best x: {best_x}")
        print("="*30)

        if not problem.history:
            print("Warning: No history data to plot.")
            return best_x, best_cost, []

        try:
            import matplotlib.pyplot as plt# 嘗試延後導入必免衝突(但似乎沒有用)
            plt.figure(figsize=(10, 5))

            valid_history = [h for h in problem.history if h < 1e9]

            if valid_history:
                #plt.plot(problem.history, label="Cost History", color='blue', alpha=0.3)# 看有沒有要加過去紀錄

                running_best = np.minimum.accumulate(problem.history)
                plt.plot(running_best, label="Best So Far", color='red', linewidth=2)

                plt.yscale("log")
                plt.xlabel("Evaluation Steps")
                plt.ylabel("Cost (Log Scale)")
                plt.title("Convergence Curve")
                plt.legend()
                plt.grid(True, which="both", ls="-", alpha=0.2)

                # =========================
                # 存檔區塊
                # =========================
                save_dir = FILTER_DIR/"figures"
                os.makedirs(save_dir, exist_ok=True)

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"convergence_{timestamp}_cost{best_cost:.3f}.png"
                save_path = os.path.join(save_dir, filename)

                plt.savefig(save_path, dpi=300, bbox_inches="tight")
                print(f"Figure saved to: {save_path}")

                print("Displaying plots... (Close window to continue)")
                plt.show()

            else:
                print("All simulations failed (Cost was always inf).")

        except Exception as e:
            print(f"Plotting failed due to environment issues: {e}")

        try:
            sim, success = problem.simulate(best_x, visualize=True)
        except Exception as e:
            print(f"Final simulation visualization failed: {e}")
            sim, success = None, False
        print(f"Final simulation success: {success}")
        return best_x, best_cost, problem.history, sim, success

class ScipyOptimizer(BaseOptimizer):# 給scipy最小化策略接口
    def __init__(self, method="L-BFGS-B", maxiter=100):
        self.method = method
        self.maxiter = maxiter # 停止條件：最大迭代次數

    def optimize(self, problem):
        problem.reset_history()
        def obj(x):
            cost, _ = problem.evaluate(x)
            print(f"cost={cost:.4f}")
            return cost

        res = minimize(obj, problem.x0, bounds=problem.bounds, 
                       method=self.method, options={'maxiter': self.maxiter})
        return self.run_post_process(problem, res.x, res.fun)

class DEOptimizer(BaseOptimizer):# 差分演算法 DE
    def __init__(self, maxiter=50, popsize=15, tol=1e-3):
        self.maxiter = maxiter # 停止條件 1
        self.popsize = popsize
        self.tol = tol         # 停止條件 2：收斂精度

    def optimize(self, problem):
        problem.reset_history()
        def obj(x):
            cost, _ = problem.evaluate(x)
            print(f"cost={cost:.4f}")
            return cost

        res = differential_evolution(
            obj, bounds=problem.bounds, maxiter=self.maxiter, 
            popsize=self.popsize, tol=self.tol, seed=42
        )
        return self.run_post_process(problem, res.x, res.fun)

class CMAESOptimizer(BaseOptimizer):# 演化策略 CMA-ES 
    def __init__(self, sigma=2.0, maxiter=100):
        self.sigma = sigma
        self.maxiter = maxiter

    def optimize(self, problem):
        problem.reset_history()
        def obj(x):
            cost, _ = problem.evaluate(x)
            return cost

        lower = [b[0] for b in problem.bounds]
        upper = [b[1] for b in problem.bounds]
        
        es = cma.CMAEvolutionStrategy(
            problem.x0, self.sigma, {'bounds': [lower, upper], 'maxiter': self.maxiter}
        )
        res = es.optimize(obj)
        return self.run_post_process(problem, res.result.xbest, res.result.fbest)

class RandomSearchOptimizer(BaseOptimizer):# 隨機選取(驗證工具)
    def __init__(self, n_iter=50):
        self.n_iter = n_iter # 停止條件：跑完次數就停

    def optimize(self, problem):
        problem.reset_history()
        best_x = None
        best_cost = np.inf

        for i in range(self.n_iter):
            x = np.array([np.random.uniform(l, h) for (l, h) in problem.bounds])
            cost, success = problem.evaluate(x)
            
            if success and cost < best_cost:
                best_cost = cost
                best_x = x
            
            if i % 10 == 0: print(f"Iter {i}: current best {best_cost:.4f}")

        return self.run_post_process(problem, best_x, best_cost)
    

