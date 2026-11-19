# Inverse kinematics
import numpy as np

class KinematicsAnalyzer:
    """
    機構動力學後處理與虛擬感測器引擎
    負責即時分析、淬取所有節點與連桿的三維運動狀態（速度、加速度、角速度、角加速度）
    """
    def __init__(self, solver):
        self.solver = solver

    # ===== 基礎節點運動學查詢 =====
    def get_velocity(self, node, dq):
        """獲取特定節點的 3D 速度向量"""
        if node.fixed or node.idx is None:
            return np.zeros(3)
        return dq[node.idx : node.idx + 3]

    def get_acceleration(self, node, ddq):
        """獲取特定節點的 3D 加速度向量"""
        if node.fixed or node.idx is None:
            return np.zeros(3)
        return ddq[node.idx : node.idx + 3]

    def get_speed(self, node, dq):
        """獲取特定節點的合速度大小 (Speed)"""
        v = self.get_velocity(node, dq)
        return np.linalg.norm(v)

    def get_acc_magnitude(self, node, ddq):
        """獲取特定節點的合加速度大小"""
        a = self.get_acceleration(node, ddq)
        return np.linalg.norm(a)

    # ===== 連桿（Rod）旋轉運動學查詢 =====
    def rod_orientation_angles(self, rod):
        """計算連桿當前在空間中的三個平面投影角度 (尤拉角/方向角趨勢)"""
        r = rod.b.pos - rod.a.pos
        L = np.linalg.norm(r)

        if L < 1e-8:
            return np.zeros(3)

        r_hat = r / L
        # 三個平面的投影角 (弧度)
        theta_z = np.arctan2(r_hat[1], r_hat[0])  # XY 平面 (Yaw 傾向)
        theta_y = np.arctan2(r_hat[0], r_hat[2])  # ZX 平面 (Pitch 傾向)
        theta_x = np.arctan2(r_hat[2], r_hat[1])  # YZ 平面 (Roll 傾向)

        theta = np.rad2deg(np.array([theta_x, theta_y, theta_z]))
        return theta

    def rod_angular_velocity(self, rod, dq):
        """計算連桿當前的 3D 角速度向量 omega (rad/s)"""
        r = rod.b.pos - rod.a.pos
        L2 = np.dot(r, r)
        
        if L2 < 1e-8:
            return np.zeros(3)

        va = self.get_velocity(rod.a, dq)
        vb = self.get_velocity(rod.b, dq)
        v_rel = vb - va

        # 剛體公式: omega = (r x v_rel) / |r|^2
        return np.cross(r, v_rel) / L2

    def rod_angular_acceleration(self, rod, dq, ddq):
        """
        計算連桿當前的 3D 角加速度向量 alpha (rad/s^2)
        考慮了向心項在與 r 外積時消去的物理特性
        """
        r = rod.b.pos - rod.a.pos
        L2 = np.dot(r, r)
        
        if L2 < 1e-8:
            return np.zeros(3)

        aa = self.get_acceleration(rod.a, ddq)
        ab = self.get_acceleration(rod.b, ddq)
        a_rel = ab - aa

        # alpha = (r x a_rel) / |r|^2
        return np.cross(r, a_rel) / L2
    
    def get_omega_magnitude(self, rod, dq):
        """獲取連桿的合角速度大小"""
        omega = self.rod_angular_velocity(rod, dq)
        return np.linalg.norm(omega)

    def get_alpha_magnitude(self, rod, dq, ddq):
        """獲取連桿的合角加速度大小"""
        alpha = self.rod_angular_acceleration(rod, dq, ddq)
        return np.linalg.norm(alpha)
    
