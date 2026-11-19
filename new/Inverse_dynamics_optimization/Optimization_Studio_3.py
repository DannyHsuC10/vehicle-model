# Optimized api.py (阿克曼與出力最佳化)
print("start")
"""# 順序要注意(可能是import 初始化底層 run time and UI 衝突)
from Optimization import Boundary as bd
from Optimization import Optimizer as opt
"""

from Optimization import Boundary as bd
from Optimization import Optimizer as opt


from Optimization import Problem_Definition as Pd
from scipy.interpolate import interp1d
# import matplotlib.pyplot as plt  # 移除頂級導入
import numpy as np
import Steering as st

# ========================(問題定義與求解)
class SteeringTarget:
    def __init__(self, file):

        data = np.genfromtxt(file, delimiter=",", names=True)

        # rack（mm）
        self.rack = data["rack_mm"]

        # delta（deg → rad）
        self.delta = np.deg2rad(data["delta_l"])
        self.delta_r = np.deg2rad(data["delta_r"])
        

        # 清理 NaN
        mask = ~np.isnan(self.rack) & ~np.isnan(self.delta)
        self.rack = self.rack[mask]
        self.delta = self.delta[mask]

        # 排序
        idx = np.argsort(self.rack)
        self.rack = self.rack[idx]
        self.delta = self.delta[idx]

def plane_from_3points(p1, p2, p3):# 平面計算
    v1 = p2 - p1
    v2 = p3 - p1

    n = np.cross(v1, v2)
    A, B, C = n

    D = -np.dot(n, p1)

    return A, B, C, D

plane = plane_from_3points(p1 = np.array([78.645,572.247,182.293]), p2 = np.array([78.645,552.441,179.515]), p3 = np.array([18.645,552.441,179.515]))

def point_to_plane_distance(p, plane):# 計算點到直線距離給懲罰
    A, B, C, D = plane
    x, y, z = p

    return abs(A*x + B*y + C*z + D) / np.sqrt(A**2 + B**2 + C**2)
#print(point_to_plane_distance(np.array([74.66667477,554.20925261,179.48272778]), plane))

class SteeringObjective:

    def __init__(self, target):
        self.target = target

    def extract(self, sim):

        # ===== 原始資料 =====
        rack_abs = np.array(sim.logger.data["rack_y"])
        delta_abs = np.array(sim.logger.data["uAtie_thetaz"])
        tau = np.array(sim.logger.data["tau"])

        x = np.array(sim.logger.data["tie_rod_x"])
        y = np.array(sim.logger.data["tie_rod_y"])
        z = np.array(sim.logger.data["tie_rod_z"])
        P_tie_rod = x,y,z

        # ===== 轉成相對量（核心）=====
        rack = rack_abs - rack_abs[0]
        delta = delta_abs - delta_abs[0]
        max_delta = abs(max(delta))
        min_delta = abs(min(delta))
        # ===== 排序 =====
        idx = np.argsort(rack)
        rack = rack[idx]
        delta = delta[idx]

        # ===== 建立 mapping =====
        f = interp1d(rack, delta, fill_value="extrapolate")

        # ===== 對齊 target =====
        delta_sim = f(self.target.rack)

        return {
            "error": (delta_sim - self.target.delta),
            "torque":tau,
            "max_delta":max_delta,
            "min_delta":min_delta,
            "P_tie_rod":P_tie_rod
        }


    def compute(self, metrics):
        max = np.rad2deg(metrics["max_delta"])
        min = np.rad2deg(metrics["min_delta"])

        #print(max,min)
        if max < 26:
            p = 100000*(26-max)
        elif min < 26:
            p = 100000*(26-min)
        else:
            p = 0
            
        d_to_P = point_to_plane_distance(metrics["P_tie_rod"], plane)# 與bump steer 平面距離
        all_d_to_P = sum(d_to_P)# 總距離和
        #print(np.mean(d_to_P))

        err = metrics["error"]
        tau = metrics["torque"]
        # cost
        mse = 0.1-np.mean(abs(tau))/200000+p+np.mean(err**2)+np.mean(d_to_P)/1000# 力矩+轉角+阿克曼角+bump steer
        
        #mse = np.mean(err**2)+np.mean(d_to_P)/1000+p# 阿克曼+轉角+bump steer
        # ===== 最基本 cost =====
        #mse = np.mean(err**2)+p# 阿克曼角+轉角
        return mse*10000


def get_steering_curve(sim, target):# 曲線提取

    rack = np.array(sim.logger.data["rack_y"])
    delta = np.array(sim.logger.data["uAtie_thetaz"])

    # ===== 相對化（用中點最穩）=====
    mid = len(rack)//2
    rack = rack - rack[mid]
    delta = delta - delta[mid]

    # ===== 排序 =====
    idx = np.argsort(rack)
    rack = rack[idx]
    delta = delta[idx]

    # ===== 插值對齊 target =====
    f = interp1d(rack, delta, fill_value="extrapolate")

    delta_interp = f(target.rack)

    return target.rack, delta_interp


geo_provider = bd.GeometryProvider()#<< pd進來
dv_manager = bd.DesignVariableManager(geo_provider.get_base_nodes())#<< pd進來

# builder
def steering_sim_builder(x):
    return st.SteeringSimulation(dv_manager, x)

# components
target = SteeringTarget("rack_steering_angle_rela.csv")
objective = SteeringObjective(target)

# problem（通用)
problem = Pd.OptimizationProblem(
    simulation_builder=steering_sim_builder,
    objective=objective,
    bounds=dv_manager.bounds,
    x0=dv_manager.x0
)

# optimize
#optimizer = opt.ScipyOptimizer(method="L-BFGS-B", maxiter=8)
optimizer = opt.CMAESOptimizer(sigma = 2.0, maxiter=5)
best_x, best_cost, history, sim, success = optimizer.optimize(problem)


# --------------------------------------比較
# ===== target =====
target = SteeringTarget("rack_steering_angle_rela.csv")

# ===== initial sim =====
sim_init, _ = steering_sim_builder(dv_manager.x0).run(enable_visual=False)

# ===== optimized sim（你已經有）=====
sim_opt = sim

# ===== 取得曲線 =====
rack_t = target.rack
delta_target = target.delta

_, delta_init = get_steering_curve(sim_init, target)
_, delta_opt = get_steering_curve(sim_opt, target)



import matplotlib.pyplot as plt  # 在需要繪圖時才導入
plt.figure()
plt.plot(rack_t, delta_target, label="Target", linewidth=2)
#plt.plot(rack_t, target.delta_r, label="Target", linewidth=2)
plt.plot(rack_t, delta_init, "--", label="Initial")
plt.plot(rack_t, delta_opt, label="Optimized")

plt.xlabel("Rack (mm)")
plt.ylabel("Steering Angle (rad)")
plt.title("Steering Curve Fitting")

plt.legend()
plt.grid()

plt.show()

plt.figure()

plt.plot(rack_t, delta_init - delta_target, "--", label="Initial Error")
plt.plot(rack_t, delta_opt - delta_target, label="Optimized Error")

plt.axhline(0)

plt.xlabel("Rack (mm)")
plt.ylabel("Error (rad)")
plt.title("Fitting Error")

plt.legend()
plt.grid()

plt.show()
