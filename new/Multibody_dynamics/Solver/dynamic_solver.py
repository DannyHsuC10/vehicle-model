# dymamic_solver.py
# ============================================================
import numpy as np

class DynamicSolver:
    def __init__(self, mechanism_solver, state, force_manager):
        self.solver = mechanism_solver
        self.state = state
        self.force_manager = force_manager
        self.n_dof = len(state.q)

        # 初始化廣義質量矩陣
        self.M = np.eye(self.n_dof)

    def set_node_mass(self, node, mass):
        if node.fixed or node.idx is None:
            return
        idx = node.idx
        self.M[idx : idx + 3, idx : idx + 3] = np.eye(3) * mass

    def solve_acceleration(self):
            q = self.state.q
            dq = self.state.dq

            # 1. 計算廣義外力向量 Q
            Q = self.force_manager.compute_total_Q(self.state, self.n_dof)

            # 2. 獲取當前位形下的約束雅可比矩陣 J 與殘差 Phi
            J = self.solver.jacobian(q)
            phi = self.solver.residual(q)
            m = J.shape[0]

            if m == 0:
                self.state.ddq = np.linalg.solve(self.M, Q)
                return self.state.ddq, np.array([])

            # 3. 計算 J_dot * q_dot 的向心/科氏力修正項
            v_driven_gamma = -self.solver.compute_qd_jd_terms(q, dq)

            # 4. Baumgarte 穩定化
            alpha = 10.0
            beta = 10.0
            vel_drift = J @ dq  
            gamma = v_driven_gamma - 2.0 * alpha * vel_drift - (beta**2) * phi

            # 5. 建構擴增系統矩陣 (Augmented System KKT Matrix)
            A = np.block([
                [self.M, J.T],
                [J, np.zeros((m, m))]
            ])

            b = np.concatenate([Q, gamma])

            # 6. 線性聯立求解 加速度 (ddq) 與 拉格朗日乘子 (lamb)
            try:
                # 正常狀況下高效求解
                x = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                # 修正此處：當機構遭遇死點/奇異點（Singular Matrix）時，改用 pseudo-inverse 穩健求解
                x = np.linalg.lstsq(A, b, rcond=1e-9)[0]

            ddq = x[: self.n_dof]
            lamb = x[self.n_dof:]

            self.state.ddq = ddq
            return ddq, lamb

    def step(self, dt):
        # 1. 求解當前加速度
        self.solve_acceleration()

        # 2. 半隱式歐拉積分 (Semi-Implicit Euler)
        self.state.dq += self.state.ddq * dt
        self.state.q += self.state.dq * dt

        # 3. 將新的幾何座標同步回所有節點物件中
        self.state.update_positions()