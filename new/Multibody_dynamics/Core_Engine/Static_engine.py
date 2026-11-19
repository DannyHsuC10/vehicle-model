# static_engine.py

import numpy as np

class StaticAnalyzer:
    def __init__(self, mech, force_manager, dynamic_solver=None):
        """
        將原本傳入的 state 改為直接傳入 mech (MechanismData)
        這樣才能精準、安全地抓到最完整的節點與連桿字典！
        """
        self.mech = mech
        self.nodes = mech.nodes  # 直接從 mech 拿所有節點
        self.rods = mech.rods    # 直接從 mech 拿所有連桿 (完美對接 CSV 數據)
        self.force_manager = force_manager
        self.dyn = dynamic_solver

    def solve(self, state):
        """
        將當前的動態 state 作為參數傳進來。
        因為狀態（位置、加速度）每一步都在變，但機構拓樸（mech）是不變的。
        """
        # 1. 篩選出 Ft=1 的剛性桿件
        rigid_rods = [r for r in self.rods.values() if r.Ft == 1]
        
        if len(rigid_rods) == 0:
            return np.array([])

        A = []
        b = []

        # 2. 計算當前時間步「全系統」的綜合廣義外力 Q_total
        # 這裡需要傳入當前的 state
        Q_ext = self.force_manager.compute_total_Q(state, state.n_dof)

        for node in self.nodes.values():
            # 🚀 核心修正：安全反查這個節點在當前 state 中的 index
            # 因為你的 NodeData 目前沒有紀錄 idx，我們用剛剛的速度反查邏輯
            if node.fixed:
                continue
                
            try:
                node_dof_index = state.dof_nodes.index(node)
                idx = node_dof_index * 3
            except ValueError:
                # 如果可動節點不小心沒被納入 state.dof_nodes，跳過避免報錯
                continue

            # 擷取該節點受到的綜合外力 [F_x, F_y, F_z]
            node_ext_force = Q_ext[idx : idx + 3]

            # 達朗貝爾動態慣性力修正 (-m * a)
            if self.dyn is not None and hasattr(state, 'ddq'):
                node_acc = state.ddq[idx : idx + 3]
                inertia_force = -node.m * node_acc
                node_total_ext = node_ext_force + inertia_force
            else:
                node_total_ext = node_ext_force

            # 3. 建立 3 個軸向的靜力平衡方程
            for axis in range(3):
                row = np.zeros(len(rigid_rods))
                for i, r in enumerate(rigid_rods):
                    # 力量方向由連桿當前的即時位置 pb - pa 決定
                    d = r.b.pos - r.a.pos
                    L = np.linalg.norm(d)
                    if L < 1e-12:
                        continue
                    u = d / L

                    if r.a == node:
                        sign = 1
                    elif r.b == node:
                        sign = -1
                    else:
                        continue

                    row[i] += sign * u[axis]

                A.append(row)
                b.append(-node_total_ext[axis])

        A = np.array(A)
        b = np.array(b)

        # 4. 最小平方法求解剛性連桿內力
        try:
            x, *_ = np.linalg.lstsq(A, b, rcond=1e-9)
        except Exception:
            x = np.zeros(len(rigid_rods))

        # 5. 將求解結果寫回 mech.rods 數據庫中
        for i, r in enumerate(rigid_rods):
            r.force = x[i]

        return x