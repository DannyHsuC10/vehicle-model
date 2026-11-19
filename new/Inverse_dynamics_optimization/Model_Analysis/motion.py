# motion.py
# 常用輸入與運動軌跡函數
import Model_Analysis.Geometry as geo
import numpy as np

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
def harmonic_input(t, SIM_TIME, x_max=1, x_min=-1, times=1):# 簡諧運動
    # 1. 基本參數計算
    amplitude = (x_max - x_min) / 2
    offset = (x_max + x_min) / 2
    
    # 2. 計算相位 phi，確保 t=0 時 x=0
    # 這裡加入 clip 確保數值安全，防止 arcsin 定義域失效
    ratio = -offset / amplitude
    ratio = np.clip(ratio, -1.0, 1.0)
    phi = np.arcsin(ratio)
    
    # 3. 計算頻率
    period = SIM_TIME / times
    omega = 2 * np.pi / period
    
    # 4. 計算當前位置
    x = offset + amplitude * np.sin(omega * t + phi)
    return -x

def linear_input(t, SIM_TIME, x_max=1, x_min=-1, times=1):# 線性往返運動
    # 1. 總行程計算 (一個完整的 0 -> max -> min -> 0 循環)
    L = (x_max - x_min) * 2
    
    # 2. 計算目前累積的行程 s
    v = L / SIM_TIME * times
    s = (t * v) % L  # 確保 s 永遠在 [0, L) 區間
    
    # 3. 根據行程進度決定位置 x
    # 區段 A: 0 -> x_max (行程長度 = x_max)
    if s <= x_max:
        x = s
    # 區段 B: x_max -> x_min (行程長度 = x_max - x_min)
    elif s <= 2*x_max - x_min:
        # 這裡從 x_max 開始往回扣除超過部分的位移
        x = 2*x_max - s
    # 區段 C: x_min -> 0 (剩餘行程)
    else:
        # 這裡從 x_min 開始往正方向補回
        x = s - L
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

