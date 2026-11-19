import numpy as np


class MechanismState:
    def __init__(self, solver):
        self.solver = solver
        # 統一使用 solver 建立好的節點順序，避免兩邊 index 不一致
        self.nodes = solver.nodes
        self.rods = solver.data.rods  # 確保抓到完整的 rods 字典
        self._build_index()

        self.q = self.pack()
        self.dq = np.zeros_like(self.q)
        self.ddq = np.zeros_like(self.q)

    def _build_index(self):
        # 沿用 solver 的分配，確保兩者完全同步
        self.dof_nodes = [node for node in self.nodes if not node.fixed]
        # 總自由度
        self.n_dof = len(self.dof_nodes) * 3

    def pack(self):
        q = []
        for node in self.nodes:
            if not node.fixed:
                q.extend(node.pos)
        return np.array(q, dtype=float)

    def unpack(self):
        for node in self.nodes:
            if not node.fixed:
                node.pos = self.q[node.idx : node.idx + 3]

    def update_positions(self):
        self.unpack()

    def get_velocity(self, node):
        """為力學引擎（如阻尼器、輪胎）提供即時的節點 3D 速度查詢"""
        if node.fixed or node.idx is None:
            return np.zeros(3)
        return self.dq[node.idx : node.idx + 3]