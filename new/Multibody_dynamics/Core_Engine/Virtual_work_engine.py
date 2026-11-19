# virtual_work_engine.py
# 虛功引擎
import numpy as np

# virtual_work_engine.py
# 修正版：基於幾何雅可比投影的機械利益引擎
import numpy as np

class VirtualWorkEngine:
    def __init__(self, solver, kin_analyzer):
        self.solver = solver
        self.kin = kin_analyzer

    def compute_mechanical_advantage(self, state, in_node, in_axis, out_node, out_axis):
        """
        利用運動學對偶性與雅可比矩陣，精準求解機構的純機械利益 (力比)。
        MA = F_out / F_in = V_in / V_out
        """
        if in_node.fixed or (out_node.fixed and out_node.idx is None):
            return 0.0

        # 1. 確保方向向量為單位向量
        u_in = np.array(in_axis, dtype=float) / np.linalg.norm(in_axis)
        u_out = np.array(out_axis, dtype=float) / np.linalg.norm(out_axis)

        # 2. 獲取當前機構的幾何約束雅可比矩陣 J (m, n_dof)
        J = self.solver.jacobian(state.q)
        m, n_dof = J.shape

        # 3. 建構速度層面的擴增系統
        # 我們要強迫輸入節點沿著 u_in 方向以 1.0 的虛速度運動
        # 這等同於增加一個額外的運動學約束: e_in^T * v = 1.0
        e_in = np.zeros(n_dof)
        e_in[in_node.idx : in_node.idx + 3] = u_in

        # 將此輸入速度約束串接在原本的雅可比矩陣下方
        A_kin = np.vstack([J, e_in])
        
        # 右側項：原本的約束不允許運動 (全為 0)，而我們強加的約束速度為 1.0
        b_kin = np.zeros(m + 1)
        b_kin[-1] = 1.0

        # 4. 求解整個機構在該輸入驅動下的「相容虛速度分佈向量 v_virtual」
        try:
            v_virtual = np.linalg.solve(A_kin, b_kin)
        except np.linalg.LinAlgError:
            # 防止過度約束或死點，使用最小平方法穩健求解
            v_virtual = np.linalg.lstsq(A_kin, b_kin, rcond=1e-9)[0]

        # 5. 淬取輸出節點在觀測方向上的虛速度
        v_out_vec = v_virtual[out_node.idx : out_node.idx + 3]
        v_out_val = np.dot(v_out_vec, u_out)

        # 6. 根據虛功原理 (忽略摩擦與動能): F_in * V_in = F_out * V_out
        # 既然 V_in = 1.0，則 MA = F_out / F_in = 1.0 / V_out
        if abs(v_out_val) < 1e-8:
            return 0.0  # 輸出端在該方向不運動，無法傳遞力量 (力比為 0 或趨近無窮，視鎖死狀態而定)

        # 注意物理方向：如果 V_out 運動方向與你定義的 out_axis 同向，代表輸出力與輸出運動同向
        ma = 1.0 / v_out_val
        return ma

    def get_estimated_force(self, state, in_node, in_axis, f_in_val, out_node, out_axis):
        """
        給定實際輸入力 (N)，計算理論輸出有效力
        """
        ma = self.compute_mechanical_advantage(state, in_node, in_axis, out_node, out_axis)
        return f_in_val * ma