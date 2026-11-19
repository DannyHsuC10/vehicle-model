# virtual_work_engine.py
# 虛功引擎
import numpy as np
from Model_Analysis import Geometry as geo

class VirtualWorkEngine:
    def __init__(self, solver, kin_analyzer):
        self.solver = solver
        self.kin = kin_analyzer
        self._current_power_ratio = 0.0 # 中間變數：P_in / 1.0 (假設輸入力為1)
        self._current_power_in = 0.0
    def update_force_context(self, xdot, in_node, in_axis, f_in_val):
        """
        更新當前時間步的力學上下文。
        計算：輸入功率 = F_in * (V_in · axis_in)
        """
        v_in_vec = self.kin.get_velocity(in_node, xdot)
        in_axis_unit = in_axis / np.linalg.norm(in_axis)
        # 這是整個系統的「能量源」
        self._current_power_in = f_in_val * np.dot(v_in_vec, in_axis_unit)

    def get_node_force(self, node, xdot):
        """
        根據功率守恆:F_out · V_out = P_in
        假設 F_out 與 V_out 同向 (能量最有效傳遞路徑)
        """
        if node.fixed:
            return np.zeros(3)
            
        v_node = self.kin.get_velocity(node, xdot)
        v_mag2 = np.dot(v_node, v_node)
        
        if v_mag2 > 1e-8:
            # F = (P / V^2) * V-
            return (self._current_power_in / v_mag2) * v_node
        return np.zeros(3)
            
    def compute_torque_at_point(self, node, xdot, axis):
        """
        計算某個 node 對指定軸的力矩
        """

        axis_unit, P0 = axis

        # 力
        F = self.get_node_force(node, xdot)

        # 力臂
        r = geo.line_point_min_distance(node.pos, axis)

        # torque
        tau_vec = np.cross(r, F)

        return np.dot(tau_vec, axis_unit)
    
