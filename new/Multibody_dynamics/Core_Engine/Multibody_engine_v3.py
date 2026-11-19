# Multibody_engine.py
# ============================================================
# Pure Multibody Constraint Solver
# ============================================================
# 功能：
# - 只負責幾何拘束求解
# - 不包含 VPython
# - 不建立視覺物件
# - 直接使用 MechanismData 內的資料物件
#
# 結構：
# MechanismData
#   ├── NodeData
#   ├── RodData
#   └── WheelData
#
# Solver
#   ├── constraints
#   ├── pack/unpack
#   ├── residual
#   ├── jacobian
#   └── solve
#
# ============================================================
import numpy as np

def get_pos(node, x):
    if node.fixed:
        return node.pos
    return x[node.idx : node.idx + 3]

class Constraint:
    def residual(self, x):
        raise NotImplementedError

    def jacobian(self, x, n_dof):
        raise NotImplementedError

    def compute_gamma(self, x, dx, n_dof):
        """計算 J_dot * q_dot 的貢獻項"""
        raise NotImplementedError

class RodConstraint(Constraint):
    def __init__(self, rod):
        self.rod = rod
        self.a = rod.a
        self.b = rod.b
        self.L0 = None

    def initialize(self):
        self.L0 = np.linalg.norm(self.a.pos - self.b.pos)

    def residual(self, x):
        pa = get_pos(self.a, x)
        pb = get_pos(self.b, x)
        # 採用平方項約束，極大地提高多體動力學求解與時間微分的數值穩定性
        return 0.5 * (np.dot(pa - pb, pa - pb) - self.L0**2)

    def jacobian(self, x, n_dof):
        pa = get_pos(self.a, x)
        pb = get_pos(self.b, x)
        r = pa - pb  # 方向向量

        j = np.zeros(n_dof)
        if not self.a.fixed:
            j[self.a.idx : self.a.idx + 3] = r
        if not self.b.fixed:
            j[self.b.idx : self.b.idx + 3] = -r
        return j

    def compute_gamma(self, x, dx, n_dof):
        """
        對平方項約束進行二次時間微分後：
        J_dot * q_dot = (v_a - v_b)^T * (v_a - v_b)
        注意：這裡要變號移到方程右側，動力學常規定義其為正值或負值，
        這裡直接返回 (v_a - v_b)^2，供 solver 使用。
        """
        va = np.zeros(3) if self.a.fixed else dx[self.a.idx : self.a.idx + 3]
        vb = np.zeros(3) if self.b.fixed else dx[self.b.idx : self.b.idx + 3]
        rel_v = va - vb
        return np.dot(rel_v, rel_v)

class MechanismSolver:
    def __init__(self, mechanism_data):
        self.data = mechanism_data
        self.nodes = list(mechanism_data.nodes.values())
        self.constraints = []

        self.build_index()
        self.build_constraints()
        self.initialize_constraints()

    def build_index(self):
        idx = 0
        for node in self.nodes:
            if not node.fixed:
                node.idx = idx
                idx += 3
            else:
                node.idx = None

    def build_constraints(self):
        self.constraints = []
        for rod in self.data.rods.values():
            if rod.Ft == 1:
                self.constraints.append(RodConstraint(rod))

    def initialize_constraints(self):
        for c in self.constraints:
            c.initialize()

    def residual(self, x):
        res = []
        for c in self.constraints:
            r = c.residual(x)
            if np.isscalar(r):
                res.append(r)
            else:
                res.extend(r)
        return np.array(res, dtype=float)

    def jacobian(self, x):
        rows = []
        n_dof = len(x)
        for c in self.constraints:
            j = c.jacobian(x, n_dof)
            if j.ndim == 1:
                rows.append(j)
            else:
                for r in j:
                    rows.append(r)
        return np.array(rows, dtype=float)

    def compute_qd_jd_terms(self, x, dx):
        """累積所有約束的 J_dot * q_dot"""
        n_dof = len(x)
        gamma_terms = []
        for c in self.constraints:
            g = c.compute_gamma(x, dx, n_dof)
            if np.isscalar(g):
                gamma_terms.append(g)
            else:
                gamma_terms.extend(g)
        return np.array(gamma_terms, dtype=float)