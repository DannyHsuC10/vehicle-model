# Geometry
# 幾何工具庫
import numpy as np
import math

def to_numpy(vec): # 幾何>>坐標系
    """
    將輸入轉為 numpy array
    支援：
    - numpy array
    - vpython vector
    """
    if hasattr(vec, "x"):
        return np.array([vec.x, vec.y, vec.z])
    return np.array(vec)

def rotation_matrix(axis, theta):# 旋轉矩陣
    """回傳繞 axis 旋轉 theta 弧度的旋轉矩陣"""
    axis = axis / np.linalg.norm(axis)
    kx, ky, kz = axis
    K = np.array([
        [0, -kz, ky],
        [kz, 0, -kx],
        [-ky, kx, 0]
    ])
    I = np.eye(3)
    return I + np.sin(theta)*K + (1 - np.cos(theta))*(K @ K)

def mirror_point(point,plane = "Y"): # 鏡射矩陣
    """
    對平面鏡射,plane 可選 "X", "Y", "Z" 分別對 YZ, XZ, XY 平面鏡射
    """
    point = to_numpy(point)
    if plane == "Y":
        # 對 XZ 平面鏡射，即 Y 座標變號
        return np.array([point[0], -point[1], point[2]])
    elif plane == "X":
        # 對 YZ 平面鏡射，即 X 座標變號
        return np.array([-point[0], point[1], point[2]])
    elif plane == "Z":
        # 對 XY 平面鏡射，即 Z 座標變號
        return np.array([point[0], point[1], -point[2]])
    else:
        raise ValueError("Invalid plane specified. Use 'X', 'Y', or 'Z'.")

def axis_unitization(P1,P0): # 軸單位化向量
    """
    將 P1 和 P0 定義的向量單位化，定義軸的方向和位置
    參數:
        P1, P0 : (3,) 定義軸的兩點
    """
    # 轉換到幾何層
    P1 = to_numpy(P1)
    P0 = to_numpy(P0)

    axis_dir = P1 - P0
    axis_unit = axis_dir / np.linalg.norm(axis_dir)
    return [axis_unit,P0]

def two_point_vector(P1, P0, L = None):# 兩點向量
    """
    兩點向量

    參數:
        P1 : (3,) 軸上的點
        P0 : (3,) 軸上的另一點
        L  : float (可選) 向量的長度

    回傳:
        vec : (3,) 兩點向量
    """
    P1 = to_numpy(P1)
    P0 = to_numpy(P0)
    if L == None:
        vec = P1-P0
    else:
        vec = axis_unitization(P1,P0)[0]*L
    
    return vec

def line_point_min_distance(point, axis):# 點到直線最短距離
    """
    計算點到直線距離
    參數:
        point    : (3,) 要計算距離的點
        axis     : (axis_unit, P0) 軸的單位向量和通過的點
    回傳:
        r_vec : (3,) 從點到直線的向量
    """
    axis_unit,P0 = axis
    # 點到直線最段距離
    t = np.dot(point - P0, axis_unit)
    P_closest = P0 + t * axis_unit

    # 力臂（global）
    r_vec = point - P_closest

    return r_vec

def line_point_constant_distance(axis, point, distance):# 點到直線固定距離(兩個解)
    """
    在給定軸上找出與 point 距離為 distance 的點（最多兩個）

    參數:
        axis     : [axis_unit, P0]
        point    : (3,)
        distance : float

    回傳:
        points : list of (3,) → 可能有 0, 1, 2 個解
    """
    point = to_numpy(point)
    axis_unit, P0 = axis

    w = P0 - point

    b = 2 * np.dot(w, axis_unit)
    c = np.dot(w, w) - distance**2

    discriminant = b**2 - 4 * c

    if discriminant < 0:
        return None  # 無解

    elif np.isclose(discriminant, 0):
        t = -b / 2
        return [P0 + t * axis_unit]*2  # 一個解，重複返回兩次

    else:
        sqrt_d = np.sqrt(discriminant)
        t1 = (-b + sqrt_d) / 2
        t2 = (-b - sqrt_d) / 2

        X1 = P0 + t1 * axis_unit
        X2 = P0 + t2 * axis_unit

        return [X1, X2]

def line_length(P1,P0):# 桿件長度計算
    P1 = to_numpy(P1)
    P0 = to_numpy(P0)
    return np.linalg.norm(P1 - P0)

def close_to_point(points, target = np.array([0, 0, 0])):# 接近點
    """
    判斷 point 是否接近 target（距離小於 tol）

    參數:
        point  : (3,)
        target : (3,)

    回傳:
        最接近目標點的點
    """
    points = to_numpy(points)
    target = to_numpy(target)
    P_close = min(points, key=lambda p: np.linalg.norm(p - target))

    return P_close

def project_point_to_line(point, axis):# 點投影到直線
    """
    計算點到直線的投影（最近點）

    參數:
        point : (3,) 要投影的點
        axis  : (axis_unit, P0)
                axis_unit → 單位方向向量
                P0        → 線上的一點

    回傳:
        P_proj : (3,) 投影點（直線上最近點）
        t      : float(沿著 axis_unit 的參數距離）
    """
    point = to_numpy(point)
    axis_unit, P0 = axis

    # 向量：P0 → point
    w = point - P0

    # 投影長度（沿著軸的距離）
    t = np.dot(w, axis_unit)

    # 投影點
    P_proj = P0 + t * axis_unit

    return P_proj

def get_theta_from_projection(x, point_init, axis, proj_vec):# 投影距離反求旋轉角
    """
    x        : 目標投影位移量 (float)
    point_init : 初始點位置
    axis     : (axis_unit, P0) 旋轉軸
    proj_vec : 投影的方向向量 (例如 [0, 0, 1])
    """
    axis_unit, P0 = axis
    proj_vec = proj_vec / np.linalg.norm(proj_vec)
    
    # 1. 取得旋轉半徑向量 r (從軸指向起始點)
    r_vec = line_point_min_distance(point_init, axis)
    R = np.linalg.norm(r_vec)
    if np.isclose(R, 0): return 0
    
    # 2. 定義旋轉平面內的兩個正交基準向量 (u, v)
    # u 方向就是初始半徑方向
    u_dir = r_vec / R
    # v 方向是旋轉切線方向 (利用外積)
    v_dir = np.cross(axis_unit, u_dir)
    
    # 3. 計算這兩個基準向量在目標投影方向上的權重
    alpha = np.dot(u_dir, proj_vec)
    beta = np.dot(v_dir, proj_vec)
    
    # 旋轉 theta 後的點相對於 P_closest 的向量為：
    # r(theta) = R * cos(theta) * u_dir + R * sin(theta) * v_dir
    # 相對於初始位置的投影位移 x 為：
    # x = [r(theta) - r(0)] · proj_vec
    # x = R * [ (cos(theta)-1)*alpha + sin(theta)*beta ]
    
    # 整理成 a*sin(theta) + b*cos(theta) = c 的形式
    # (R*beta)*sin(theta) + (R*alpha)*cos(theta) = x + R*alpha
    
    a = R * beta
    b = R * alpha
    c = x + R * alpha
    
    # 解 a*sin + b*cos = c -> 使用輔助角公式
    # sqrt(a^2 + b^2) * sin(theta + phi) = c, 其中 phi = atan2(b, a)
    try:
        phi = math.atan2(b, a)
        denom = math.sqrt(a**2 + b**2)
        
        # 檢查是否有解 (投影位移不能超過物理極限)
        ratio = c / denom
        if ratio > 1: ratio = 1
        if ratio < -1: ratio = -1
        
        theta = math.asin(ratio) - phi
        return theta
    except:
        return 0
    
def check_projection_validity(axis_unit, proj_vec, tolerance=0.1):# 檢查投影有效性
    """
    檢查旋轉軸與投影向量是否過於接近平行
    """
    # 確保向量都是單位向量
    a = axis_unit[0]
    p = proj_vec / np.linalg.norm(proj_vec)
    
    # 計算內積 (cos theta)
    dot_product = abs(np.dot(a, p))
    
    # 如果 dot_product 接近 1，代表平行
    if dot_product > (1.0 - tolerance):
        return False, dot_product
    return True, dot_product
