# simulator
from pathlib import Path
from turtle import pos
from Model_Analysis.Inverse_dynamics import Multibody_engine as mb
from Model_Analysis.Inverse_dynamics.Multibody_engine import PositionConstraint
import numpy as np
from Model_Analysis.Inverse_dynamics import Static_engine as sta
from Model_Analysis.Inverse_dynamics import Kinematics_engine as kin
from vpython import *
from Model_Analysis.Inverse_dynamics import Virtual_work_engine as viw
from Model_Analysis import output


# Inverse_dynamics
FILTER_DIR = Path(__file__).resolve().parent.parent

class SimulationCore:
    def __init__(self, solver, kin, vw_engine,nodes):
        self.solver = solver
        self.kin = kin
        self.vw_engine = vw_engine
        # 初始打包狀態
        self.prev_x = solver.pack()
        self.nodes = nodes   # 加這行

    def step(self, input_func, dt, t, force_configs=None, torque_targets=None):
        """
        核心執行步：
        1. 執行輸入運動 (更新 PositionConstraint 的 target)
        2. 求解幾何位置 x
        3. 利用位置差值 (dx/dt) 作為速度項 b,透過 Jacobian 求出全系統速度 xdot
        4. 計算虛功
        """
        # [1] 更新輸入軌跡 (例如執行 rack_ctrl.set_target(...))
        input_func()

        # [2] 幾何求解：得到滿足所有約束（含 PositionConstraint）的新位置 x
        # 注意：此時輸入點必須是 fixed=False 才能進入 x 向量
        success, x = self.solver.solve(self.prev_x)

        if not success:
            return None

        # [3] 速度求解 (Velocity Level Kinematics)
        """
        速度求解先計算幾何之後再進去運動學求解器，
        必須要分該來做之後比較方便除錯。
        塞進去之後出事沒有辦法找錯誤
        """
        if self.prev_x is None:
            xdot = np.zeros_like(x)
            J = self.solver.jacobian(x)
        else:
            # 數值速度估計項 b：這是為了滿足幾何位移，機構各點「應該」具備的速度趨勢
            # 對於驅動點，這會精確對應到 (target_new - target_old) / dt
            b_numeric = (x - self.prev_x) / dt 
            
            # 取得當前幾何下的 Jacobian 矩陣
            J = self.solver.jacobian(x)
            
            # 核心邏輯：b = J * xdot。
            # 我們利用 lstsq 解出最符合當前位移趨勢的速度向量 xdot
            # 這會自動處理好輸入端與輸出端的速度分配
            b = J @ b_numeric
            xdot, _ = self.kin.solve_velocity(x, b)

        # [4] 加速度求解
        xddot = self.kin.solve_acceleration(x, xdot, J, dt)

        # [5] 通用虛功計算
        if force_configs:# 多力量輸入配置
            for config in force_configs:

                f_input_spec = config["in_force"]

                if callable(f_input_spec):
                    current_f_in = f_input_spec(x, xdot, xddot, t)
                else:
                    current_f_in = f_input_spec

                self.vw_engine.update_force_context(
                    xdot,
                    config["in_node"],
                    config["in_axis"],
                    current_f_in
                )
            
        """
        if force_configs:# 單力量輸入配置
            # 目前假設單一主要動力源 (例如轉向力)，若有多個可擴充為清單
            config = force_configs[0] 
            f_input_spec = config["in_force"]
            
            if callable(f_input_spec):
                current_f_in = f_input_spec(x, xdot, xddot, t)
            else:
                current_f_in = f_input_spec
            
            # 準備好這一動的力量環境
            self.vw_engine.update_force_context(xdot, config["in_node"], config["in_axis"], current_f_in)
        """
        # [6] 力矩計算

        torque_results = {}

        if torque_targets:
            for config in torque_targets:

                name = config["name"]

                # node 可以是 name 或物件
                node = config["node"]

                if isinstance(node, str):
                    node = self.nodes[node]

                axis = config["axis"]

                tau = self.vw_engine.compute_torque_at_point(
                    node,
                    xdot,
                    axis
                )

                torque_results[name] = tau# 保留之後計算多力矩的擴充用
        else:
            tau = None
        # [6] 更新狀態
        self.prev_x = x.copy()
        # 注意：我們現在回傳 vw_engine 本身，讓 Logger 去 get
        return x, xdot, xddot, self.vw_engine,tau

class DataLogger:
    def __init__(self):
        self.data = {}
        self.node_targets = []
        self.rod_targets = []
        self.force_targets = []
        self.prev_theta = {}

    def reset(self):
        self.data.clear()
        self.prev_theta.clear()

    def set_node_targets(self, targets):
        self.node_targets = targets

    def set_rod_targets(self, targets):
        self.rod_targets = targets

    def set_force_targets(self, targets):
        self.force_targets = targets

    def log(self, t, nodes, rods, kin, vw_env, torque_results, xdot, xddot):
        self._add("t", t)

        # ===== Node =====
        for name in self.node_targets:
            node = nodes[name]
            pos = node.pos
            # log 位置
            for i, axis in enumerate(["x", "y", "z"]):
                self._add(f"{name}_{axis}", pos[i])
            # log 速度與加速度
            v = kin.get_velocity(node, xdot)
            a = kin.get_acceleration(node, xddot)

            speed = np.linalg.norm(v)
            acc_mag = np.linalg.norm(a)

            self._add(f"{name}_v", speed)
            self._add(f"{name}_a", acc_mag)

            for i, axis in enumerate(["x", "y", "z"]):
                self._add(f"{name}_v{axis}", v[i])
                self._add(f"{name}_a{axis}", a[i])

            # --- 虛功受力 (新邏輯：視為節點屬性) ---
            f_vec = vw_env.get_node_force(node, xdot)
            self._add(f"{name}_f", np.linalg.norm(f_vec))
            for i, ax in enumerate(["x", "y", "z"]):
                self._add(f"{name}_f{ax}", f_vec[i])

        # ===== Rod =====
        for name in self.rod_targets:
            rod = rods[name]
            # 角度求解（連續化處理）
            theta_geom = kin.rod_orientation_angles(rod)

            # 初始化
            if name not in self.prev_theta:
                self.prev_theta[name] = theta_geom.copy()
                theta_cont = theta_geom.copy()
            else:
                theta_prev = self.prev_theta[name]
                theta_cont = np.zeros_like(theta_geom)

                for i in range(3):
                    dtheta = theta_geom[i] - theta_prev[i]

                    if dtheta > np.pi:
                        dtheta -= 2*np.pi
                    elif dtheta < -np.pi:
                        dtheta += 2*np.pi

                    theta_cont[i] = theta_prev[i] + dtheta

                self.prev_theta[name] = theta_cont.copy()

            # log（用連續角）
            for i, axis in enumerate(["x", "y", "z"]):
                self._add(f"{name}_theta{axis}", theta_cont[i])
            # 角速度與加速度
            omega = kin.rod_angular_velocity(rod, xdot)
            alpha = kin.rod_angular_acceleration(rod, xddot)

            omega_mag = np.linalg.norm(omega)
            alpha_mag = np.linalg.norm(alpha)

            self._add(f"{name}_omega", omega_mag)
            self._add(f"{name}_alpha", alpha_mag)

            for i, axis in enumerate(["x", "y", "z"]):
                self._add(f"{name}_omega{axis}", omega[i])
                self._add(f"{name}_alpha{axis}", alpha[i])

        if tau is not None:
            self._add(f"tau",torque_results)# 紀錄力矩

    def _add(self, key, value):
        if key not in self.data:
            self.data[key] = []
        self.data[key].append(value)

class Visualizer:
    def __init__(self, solver, visual_objects = None):
        self.solver = solver
        self.visual_objects = visual_objects or []

    def update(self):
        for n in self.solver.nodes:
            n.update_visual()

        for r in self.solver.constraints:
            if hasattr(r, "update"):
                r.update()

        # ===== extra visuals =====
        for obj in self.visual_objects:
            obj.update()

class SimulationSystem:
    def __init__(self, enable_visual=False, nodes_path=None, rods_path=None, wheel_path = None):

        # ===== 建構 =====
        self.solver = mb.MechanismSolver()

        # 使用提供的路徑或預設路徑
        if nodes_path is None:
            nodes_path = FILTER_DIR/"data"/"nodes.csv"
        if rods_path is None:
            rods_path = FILTER_DIR/"data"/"rods.csv"
        if wheel_path is None:
            wheel_path = FILTER_DIR/"data"/"wheel.csv"

        self.nodes = sta.load_nodes(
            nodes_path, mb.Node, visual=enable_visual)
        
        self.rods = sta.load_rods(
            rods_path, self.nodes, mb.Rod, visual=enable_visual)

        for n in self.nodes.values():
            self.solver.add_node(n)

        for r in self.rods.values():
            self.solver.add_constraint(r)

        self.solver.build_index()

        # ===== 模組 =====
        self.kin = kin.KinematicsAnalyzer(self.solver)
        self.vw_engine = viw.VirtualWorkEngine(self.solver, self.kin)
        self.sim = SimulationCore(self.solver, self.kin,self.vw_engine,self.nodes)
        self.logger = DataLogger()
        self.plotter = output.Plotter(self.logger.data)
        self.viz = Visualizer(self.solver) if enable_visual else None

        # ===== runtime =====
        self.input_funcs = [] # 輸入函式
        self.force_configs = [] # 初始化力學配置清單
        self.torque_targets = [] # 力矩計算
        self.visual_objects = []

         # 視覺化必須要放在後面 
        if enable_visual:
            self.wheels = sta.load_wheels(wheel_path, self.nodes, mb.Wheel, visual=enable_visual)
            for w in self.wheels.values():
                self.viz.visual_objects.append(w)

        self.success = None  # 用於記錄模擬是否成功完成

    def add_input(self, func):# 增加多運動軌跡輸入（例如多個 PositionConstraint）
        self.input_funcs.append(func)

    def set_targets(self, node_targets=None, rod_targets=None, force_targets=None,torque_targets=None):
        if node_targets:
            self.logger.set_node_targets(node_targets)
        if rod_targets:
            self.logger.set_rod_targets(rod_targets)
        if force_targets:
            # 指定義輸入原
            self.force_configs = force_targets
        if torque_targets:
            self.torque_targets = torque_targets

    def run(self, SIM_TIME, DT = 0.05):
        self.success = True
        steps = int(SIM_TIME / DT)
        # =========================
        # VPython 視窗設定
        # =========================
        if self.viz is not None:
            scene.width = 1600
            scene.height = 800
            scene.autoscale = True
        for i in range(steps):

            if self.viz is not None:
                rate(60)  # 限制更新速度（60 FPS）
            t = i * DT

            result = self.sim.step(
                lambda: [f(t) for f in self.input_funcs], 
                DT, 
                t, 
                force_configs=self.force_configs,
                torque_targets=self.torque_targets
            )

            if result is None:
                print(f"Solver Break at t={t:.3f}")
                self.success = False
                break

            x, xdot, xddot, vw_env, torque_results = result

            # 紀錄數據：vw_results 的 Key 已經在 step 中根據 config["name"] 決定好了
            self.logger.log(t, self.nodes, self.rods, self.kin, vw_env, torque_results, xdot, xddot)

            if self.viz is not None:
                self.viz.update()
        

    def plot(self, keys, save=False):
        """
        keys: list of plot key
        save: 是否儲存圖片
        """

        if not isinstance(keys, (list, tuple)):
            keys = [keys]  # 允許單一輸入

        if save:
            save_dir = Path(FILTER_DIR/"Model_Analysis"/ "figures")
            save_dir.mkdir(exist_ok=True)

        for key in keys:
            fig = self.plotter.plot(key, show=not save)  # 需要你的 Plotter 支援 show 控制

            if save:
                save_path = save_dir / f"{key}.png"
                fig.savefig(save_path)
                print(f"Saved: {save_path}")

    def override_node_positions(self, overrides, update_rods=True):
        """
        覆寫節點初始位置 + 可選同步桿長
        """

        # ===== 更新 node =====
        for name, val in overrides.items():

            if name not in self.nodes:
                raise ValueError(f"[override] Node not found: {name}")

            node = self.nodes[name]
            pos = node.pos.copy()

            # 支援兩種寫法
            if isinstance(val, dict):
                for i, axis in enumerate(["x", "y", "z"]):
                    if axis in val:
                        pos[i] = val[axis]
            else:
                pos = np.array(val, dtype=float)

            node.set_pos(pos)

        # ===== 更新 rod 長度 =====
        if update_rods:

            for r in self.rods.values():
    
                # 重新計算目前長度
                current_len = np.linalg.norm(r.b.pos - r.a.pos)

                # 更新 constraint 長度
                r.L = current_len

        # ===== 同步視覺 =====
        if self.viz is not None:
            self.viz.update()
