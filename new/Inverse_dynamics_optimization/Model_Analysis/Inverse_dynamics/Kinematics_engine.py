# Inverse kinematics
import numpy as np


class KinematicsAnalyzer:# 運動學引擎
    def __init__(self, solver):
        self.solver = solver
        self.prev_J = None
        self.prev_x = None

    # ===== 工具 =====
    def get_velocity(self, node, xdot):# 速度求解
        if node.fixed:
            return np.zeros(3)
        return xdot[node.idx:node.idx+3]

    def get_acceleration(self, node, xddot):# 加速度求解
        if node.fixed:
            return np.zeros(3)
        return xddot[node.idx:node.idx+3]

    def solve_velocity(self, x, b=None):# 速度求解
        J = self.solver.jacobian(x)

        if b is None:
            b = np.zeros(J.shape[0])

        xdot = np.linalg.lstsq(J, b, rcond=None)[0]

        return xdot, J

    def solve_acceleration(self, x, xdot, J, dt):# 加速度求解
        if self.prev_J is None:
            self.prev_J = J.copy()
            return np.zeros_like(xdot)

        dJdt = (J - self.prev_J) / dt

        rhs = - dJdt @ xdot

        xddot = np.linalg.lstsq(J, rhs, rcond=None)[0]

        self.prev_J = J.copy()

        return xddot

    def rod_orientation_angles(self, rod):# 角度求解
        pa = rod.a.pos
        pb = rod.b.pos

        r = pb - pa
        L = np.linalg.norm(r)

        if L < 1e-8:
            return np.zeros(3)

        r_hat = r / L

        # 三個平面投影角
        theta_z = np.arctan2(r_hat[1], r_hat[0])  # xy plane
        theta_y = np.arctan2(r_hat[0], r_hat[2])  # zx plane
        theta_x = np.arctan2(r_hat[2], r_hat[1])  # yz plane

        return np.array([theta_x, theta_y, theta_z])

    def rod_angular_velocity(self, rod, xdot):# 角度速度求解
        pa = rod.a.pos# a點位置
        pb = rod.b.pos# b點位置

        va = self.get_velocity(rod.a, xdot)
        vb = self.get_velocity(rod.b, xdot)

        r = pb - pa
        v_rel = vb - va

        L2 = np.dot(r, r)
        if L2 < 1e-8:
            return np.zeros(3)

        omega = np.cross(r, v_rel) / L2 # 外積/點積
        return omega
    
    def rod_angular_acceleration(self, rod, xddot):# 角度加速度求解
        pa = rod.a.pos
        pb = rod.b.pos

        aa = self.get_acceleration(rod.a, xddot)
        ab = self.get_acceleration(rod.b, xddot)

        r = pb - pa
        a_rel = ab - aa

        L2 = np.dot(r, r)
        if L2 < 1e-8:
            return np.zeros(3)

        alpha = np.cross(r, a_rel) / L2 # 外積/點積
        return alpha
    
    def get_speed(self, node, xdot):# 合速度
        v = self.get_velocity(node, xdot)
        return np.linalg.norm(v)

    def get_acc_magnitude(self, node, xddot):# 合加速度
        a = self.get_acceleration(node, xddot)
        return np.linalg.norm(a)
    
    def get_omega_magnitud(self, node, xdot):# 合角速度
        v = self.rod_angular_velocity(node, xdot)
        return np.linalg.norm(v)

    def get_alpha_magnitude(self, node, xddot):# 合角加速度
        a = self.rod_angular_acceleration(node, xddot)
        return np.linalg.norm(a)


