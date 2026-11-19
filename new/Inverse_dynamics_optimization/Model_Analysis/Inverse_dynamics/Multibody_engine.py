# 多體關係求解器
from vpython import *
import numpy as np

from scipy.optimize import least_squares

# =========================
# Node（節點 : 系統中的「點」）
# =========================
class Node:
    """
    一個 Node 代表機構中的「幾何點」
    
    功能：
    - 作為未知變數（若 fixed=False)
    - 或固定參考點（若 fixed=True)
    
    屬性：
    - pos   : 3D 座標(numpy array)
    - fixed : 是否為固定點（不參與求解）
    - idx   : 在 solver 向量 x 中的索引（由 solver 分配）
    """
    def __init__(self, pos, fixed=False, radius=15, color=color.cyan, visual=True):

        self.pos = np.array(pos, dtype=float)
        self.fixed = fixed
        self.idx = None
        self.pos = np.array(pos)
        self.visual = visual

        # ===== 靜力 =====
        # 外力（靜力核心）
        self.force = np.zeros(3)
        self.rods = []# connected rods


        if visual:
            self.ball = sphere(
                pos=vector(*pos),
                radius=radius,
                color=color,
                opacity=0.5
            )
        else:# 不建立物件
            self.ball = None
                
    def update_visual(self):
        """將數值解同步到畫面"""
        if self.ball is not None:
            self.ball.pos = vector(*self.pos)
    
    def set_pos(self, pos):
        self.pos = np.array(pos, dtype=float)

        # 如果有 visual，順便更新
        if hasattr(self, "visual") and self.visual:
            self.update_visual()


# =========================
# rod（桿件 : 幾何距離約束）
# =========================
class Rod:
    """
    桿件（最重要的結構元件）

    功能：
    - 幾何：約束兩點距離固定
    - 視覺：畫出連桿

    👉 一個物件解決兩件事（推薦架構）
    """
    def __init__(self, a, b, name=None, radius=10, opa = 1.0, visual=True):

        self.a = a
        self.b = b
        self.name = name

        self.L = np.linalg.norm(a.pos - b.pos)# 幾何長度

        # ===== solver control =====
        self.use_in_solver = True

        # ===== visual =====
        self.visual = visual
        self.radius = radius
        self.opacity = opa

        # ===== static =====
        self.lambda_force = 0.0

        if visual:
            self.cyl = cylinder(
                pos=vector(*a.pos),
                axis=vector(*(b.pos - a.pos)),
                radius=radius,
                opacity=opa
            )
        else:
            self.cyl = None

    # ===== 幾何 constraint =====
    def residual(self, x):
        pa = get_pos(self.a, x)
        pb = get_pos(self.b, x)
        return np.linalg.norm(pa - pb) - self.L

    # ===== 視覺更新 =====
    def update(self):
        if self.cyl is not None:
            self.cyl.pos = vector(*self.a.pos)
            self.cyl.axis = vector(*(self.b.pos - self.a.pos))

    def jacobian(self, x, n_dof):
        pa = get_pos(self.a, x)
        pb = get_pos(self.b, x)

        r = pa - pb
        L = np.linalg.norm(r)

        if L < 1e-8:
            return np.zeros(n_dof)

        j = np.zeros(n_dof)

        if not self.a.fixed:
            j[self.a.idx:self.a.idx+3] = r / L

        if not self.b.fixed:
            j[self.b.idx:self.b.idx+3] = -r / L

        return j
    
    def direction(self):
        d = self.b.pos - self.a.pos
        return d / np.linalg.norm(d)
# =========================
# Constraint 共用工具
# =========================
def get_pos(node, x):# 確認約束情況
    """
    從 solver 向量 x 取得 node 的位置
    
    邏輯：
    - 如果是 fixed node → 直接用 node.pos
    - 如果是變數 node → 從 x 中取
    
    x 的結構：
    [x1,y1,z1, x2,y2,z2, ...]
    """
    if node.fixed:
        return node.pos
    return x[node.idx:node.idx+3]

# =========================
# Constraint：位置約束（輸入 / 驅動）
# =========================
class PositionConstraint:
    """
    強制某個 node = 指定位置
    
    用途：
    - 當作 input(例如 steering / suspension motion)
    - 相當於 Dirichlet boundary condition
    
    residual:
    [x - xt, y - yt, z - zt]
    """

    def __init__(self, node):
        self.node = node
        self.target = node.pos.copy()
        self.lambda_force = np.zeros(3)

    def set_target(self, target):
        """更新目標位置（每一幀呼叫）"""
        self.target = target

    def residual(self, x):
        p = get_pos(self.node, x)
        return p - self.target  # 3 DOF constraint

    def jacobian(self, x, n_dof):# 點約束支援運動
        j = np.zeros((3, n_dof))

        if not self.node.fixed:
            idx = self.node.idx
            j[:, idx:idx+3] = np.eye(3)

        return j

# =========================
# Solver（核心求解器）
# =========================
class MechanismSolver:
    """
    多體幾何約束求解器
    
    核心概念：
    - nodes = 未知變數
    - constraints = 方程式
    - solve = 找到滿足所有 constraint 的解
    
    數學形式：
        minimize || f(x) ||^2
    """

    def __init__(self):
        self.nodes = []
        self.constraints = []
        self.eps = 1e-10

    # ===== 建模 =====
    def add_node(self, n):
        self.nodes.append(n)

    def add_constraint(self, c):
        self.constraints.append(c)

    # ===== 建立未知數 mapping =====
    def build_index(self):
        """
        將所有「非固定 node」映射到 solver 向量 x
        
        例：
        node1 → x[0:3]
        node2 → x[3:6]
        """
        idx = 0
        for n in self.nodes:
            if not n.fixed:
                n.idx = idx
                idx += 3

    # ===== pack / unpack =====
    def pack(self):
        """
        將所有未知 node 打包成 x
        
        用於：
        - solver 初始值
        """
        x = []
        for n in self.nodes:
            if not n.fixed:
                x.extend(n.pos)
        return np.array(x)

    def unpack(self, x):
        """
        將 solver 解回寫到 node
        
        這一步才真正「更新機構幾何」
        """
        for n in self.nodes:
            if not n.fixed:
                n.pos = x[n.idx:n.idx+3]

    # ===== 殘差組裝 =====
    def residual(self, x):
        """
        將所有 constraint 組合成一個大 residual 向量
        
        least_squares 會讓：
            sum(residual^2) 最小
        """
        res = []

        for c in self.constraints:
            r = c.residual(x)

            # scalar → 單一方程
            if np.isscalar(r):
                res.append(r)
            else:
                # 向量 → 多個方程（例如 PositionConstraint）
                res.extend(r)

        return res

    # ===== 求解 =====
    def solve(self, x0, tol=1e-4, max_step=100, verbose=False):

        sol = least_squares(
            self.residual,
            x0,
            method='lm'   # <<關鍵(快速求解)
        )
        """
        sol = least_squares(# 另一種解法（較慢但更穩定）
                self.residual,
                x0,
                method='trf',
                x_scale='jac'
            )
        """
        res_norm = np.linalg.norm(sol.fun)

        if (not sol.success) or (res_norm > tol):
            return False, x0

        # ===== step check =====
        for n in self.nodes:
            if not n.fixed:
                new = sol.x[n.idx:n.idx+3]
                old = x0[n.idx:n.idx+3]

                if np.linalg.norm(new - old) > max_step:
                    if verbose:
                        print("X step too large")
                    return False, x0
        # ===== clean tiny numerical noise =====

        sol.x[np.abs(sol.x) < self.eps] = 0.0
        self.unpack(sol.x)
        return True, sol.x

    def jacobian(self, x):# Solver.jacobian
        J = []
        n_dof = len(x)

        for c in self.constraints:
            if hasattr(c, "jacobian"):
                j = c.jacobian(x, n_dof)

                if j.ndim == 1:
                    J.append(j)
                else:
                    for row in j:
                        J.append(row)

        return np.array(J)
        
    def generalized_force(self):

        Q = np.zeros(len(self.pack()))

        for n in self.nodes:

            if not n.fixed:

                idx = n.idx

                Q[idx:idx+3] += n.force

        return Q


# =========================
# Wheel（純視覺輪胎）
# =========================
class Wheel:

    def __init__(
        self,
        node1,
        node2,
        radius,
        color,
        opacity=0.8,
        visual=True
    ):

        self.node1 = node1
        self.node2 = node2

        self.radius = radius

        self.visual = visual

        if visual:

            axis = self.node2.pos - self.node1.pos

            self.body = cylinder(
                pos=vector(*self.node1.pos),
                axis=vector(*axis),
                radius=radius,
                color=color,
                opacity=opacity
            )

        else:
            self.body = None

    def update(self):# 更新輪胎讓輪胎轉
        if self.body is None:
            return

        axis = self.node2.pos - self.node1.pos

        self.body.pos = vector(*self.node1.pos)
        self.body.axis = vector(*axis)
