# motion.py
# 常用輸入與運動軌跡函數
import Geometry as geo
import numpy as np
from itertools import product
from Core_Engine.Multibody_engine_v3 import PositionConstraint

# 運動控制函數=====================================
def rotate_around_axis(point, axis, theta): # 繞軸旋轉點
    """
    將 point 繞通過 P0、方向為 axis_unit 的軸旋轉 theta (rad)

    參數:
        point    : (3,) 要旋轉的點
        axis     : (axis_unit, P0) 軸的單位向量和通過的點
        theta    : 旋轉角 (rad)

    回傳:
        point_rot : (3,) 旋轉後的位置
    """
    point = geo.to_numpy(point)
    axis_unit,P0 = axis
    # 平移到以 P0 為原點
    p_rel = point - P0

    # 旋轉矩陣
    R = geo.rotation_matrix(axis_unit, theta)

    # 旋轉
    p_rot_rel = R @ p_rel

    # 平移回原座標
    point_rot = P0 + p_rot_rel

    return point_rot

def Slider_motion(point, axis, dx):# 滑塊移動
    point = geo.to_numpy(point)
    axis_unit, P0 = axis
    
    # 投影到軸
    v = point - P0
    proj = np.dot(v, axis_unit)

    # 沿軸移動
    new_proj = proj + dx

    return P0 + new_proj * axis_unit

def face_shift(point, axis, dx, f_axis):# 面推移
     
    is_valid, magnitude = geo.check_projection_validity(axis, f_axis)
    if not is_valid:
        # 拋出警告或給予預設值，避免 Solver 崩潰
        print(f"Warning: Projection vector is nearly parallel to rotation axis! (dot: {magnitude:.4f})")
        theta = 0
    else:
        theta = geo.get_theta_from_projection(dx, point, axis, f_axis)
        
    new_pos = rotate_around_axis(point, axis, theta)
    return new_pos

# 時間輸入控制函數=====================================
def harmonic_input(t, SIM_TIME, x_max, x_min, times=1, phase0=0.0):

    amplitude = (x_max - x_min) / 2
    offset = (x_max + x_min) / 2

    omega = 2*np.pi*times / SIM_TIME

    s = omega*t + phase0

    x = offset + amplitude*np.sin(s)

    return x

def linear_input(t, SIM_TIME, x_max, x_min, times=1):

    s = (t / SIM_TIME) * times
    s = s % 1.0   # normalized

    x = x_min + (x_max - x_min) * s

    return x

# 運動輸入與軌跡設定=====================================

class MotionConfig:
    def __init__(self,
                 motion_type,   # s: slider, r: rotate, f: face
                 input_type,    # l: linear, h: harmonic
                 x_max,
                 x_min,
                 times=1,
                 normal_vector=None):
        
        self.motion_type = motion_type
        self.input_type = input_type
        self.x_max = x_max
        self.x_min = x_min
        self.times = times
        self.normal_vector = normal_vector

    def get_input(self, t, SIM_TIME):
        if self.input_type == "h":
            return harmonic_input(t, SIM_TIME, self.x_max, self.x_min, self.times)
        elif self.input_type == "l":
            return linear_input(t, SIM_TIME, self.x_max, self.x_min, self.times)
        else:
            raise ValueError("Invalid input type")

    def apply_motion(self, P_init, axis, x):
        if self.motion_type == "s":
            return Slider_motion(P_init, axis, x)
        elif self.motion_type == "r":
            return rotate_around_axis(P_init, axis, x)
        elif self.motion_type == "f":
            return face_shift(P_init, axis, x, self.normal_vector)
        else:
            raise ValueError("Invalid motion type")


class SweepSolver:

    def __init__(self, solver, mech):

        self.solver = solver
        self.mech = mech
        self.drivers = []

    def add_driver(self, node, axis, motion_config):
        constraint = PositionConstraint(node)
        self.solver.add_constraint(constraint)

        self.drivers.append({
            "node": node,
            "axis": np.array(axis),
            "motion": motion_config,
            "constraint": constraint
        })

    # ========================================================
    # ✔ SWEEP CORE (correct version)
    # ========================================================

    def solve(self, n_steps=20, n_cycles=50, SIM_TIME=10):

        results = []

        # ============================================
        # OUTER LOOP = sweep DOF A
        # ============================================

        for i in range(n_steps):

            t_outer = i / (n_steps - 1)

            x = self.solver.pack()

            # fix first driver as sweep variable
            d0 = self.drivers[0]

            val_A = d0["motion"].x_min + \
                    t_outer * (d0["motion"].x_max - d0["motion"].x_min)

            target_A = d0["motion"].apply_motion(
                d0["node"].pos,
                d0["axis"],
                val_A
            )

            d0["constraint"].set_target(target_A)

            # ============================================
            # INNER LOOP = full motion cycle (DOF B etc)
            # ============================================

            inner_history = []

            for j in range(n_cycles):

                t = j * SIM_TIME / n_cycles

                # update remaining drivers (B, C, ...)
                for d in self.drivers[1:]:

                    val = d["motion"].get_input(t, SIM_TIME)

                    target = d["motion"].apply_motion(
                        d["node"].pos,
                        d["axis"],
                        val
                    )

                    d["constraint"].set_target(target)

                success, x = self.solver.solve(x)

                if not success:
                    continue

                snapshot = {
                    name: node.pos.copy()
                    for name, node in self.mech.nodes.items()
                }

                inner_history.append(snapshot)

            results.append(inner_history)
        print(len(results))
        return results
    