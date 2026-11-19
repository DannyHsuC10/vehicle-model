# motion 運動模組

## 1. 繞軸旋轉運動

### 1.1 旋轉運動的數學模型

```python
def rotate_around_axis(point, axis, theta):
    """將 point 繞通過 P0、方向為 axis_unit 的軸旋轉 theta (rad)"""
```

**運動軌跡方程**：
給定軸 $[\hat{\mathbf{a}}, P_0]$ 和旋轉角 $\theta$，點 $P$ 的旋轉軌跡為：

#### 步驟1：平移到軸為原點
$$\vec{p}_{\text{rel}} = P - P_0$$

#### 步驟2：應用旋轉矩陣
使用羅德裡格旋轉公式（見Geometry.md）：
$$\mathbf{R}(\hat{\mathbf{a}}, \theta) = \mathbf{I} + \sin(\theta)[\hat{\mathbf{a}}]_{\times} + (1-\cos(\theta))[\hat{\mathbf{a}}]_{\times}^2$$

旋轉後的相對位置：
$$\vec{p}_{\text{rot,rel}} = \mathbf{R}(\hat{\mathbf{a}}, \theta) \cdot \vec{p}_{\text{rel}}$$

#### 步驟3：平移回原坐標係
$$P_{\text{rot}} = P_0 + \vec{p}_{\text{rot,rel}}$$

**幾何意義**：
- 點沿圓弧軌跡運動
- 圓弧半徑：$r = \|\vec{p}_{\text{rel}}\|_{\perp} = \|P - P_0 - (P-P_0) \cdot \hat{\mathbf{a}}\|$
- 圓弧中心：$C = P_0 + [(P-P_0) \cdot \hat{\mathbf{a}}]\hat{\mathbf{a}}$

**機制應用**：
- 連接桿繞絞鏈旋轉
- 轉向係統中的搖臂運動

---

## 2. 滑塊直線運動

### 2.1 約束滑塊運動

```python
def Slider_motion(point, axis, dx):
    """沿軸方向移動點，位移量為 dx"""
```

**運動學模型**：
給定軸 $[\hat{\mathbf{a}}, P_0]$、初始點 $P$、位移量 $\Delta x$

#### 步驟1：計算點在軸上的投影參數
$$t_0 = (P - P_0) \cdot \hat{\mathbf{a}}$$

#### 步驟2：沿軸更新投影參數
$$t_{\text{new}} = t_0 + \Delta x$$

#### 步驟3：計算新位置
$$P_{\text{new}} = P_0 + t_{\text{new}} \cdot \hat{\mathbf{a}}$$

**數學性質**：
- 只改變沿軸方向的分量
- 垂直於軸的分量保持不變
- 滿足 $(P_{\text{new}} - P_{\text{old}}) \cdot \hat{\mathbf{a}} = \Delta x$

**約束條件**：
將點約束在軸上移動，是1自由度約束：
$$\text{Constraint}: \quad (P - P_0) \times \hat{\mathbf{a}} = \vec{0}$$

**機制應用**：
- 齒條-齒轮機構中的齒條直線運動
- 滑塊-曲柄機構中的滑塊沿導軌運動

---

## 3. 面推移運動

### 3.1 面推移的數學模型

```python
def face_shift(point, axis, dx, f_axis):
    """沿法向方向推移平面上的點"""
```

**運動描述**：
面推移是一種將點沿垂直於某平面方向推移的運動。本質上是將滑塊運動轉換為旋轉運動的機制。

**數學推導**：

1. **有效性檢查**：
首先檢查旋轉軸與投影方向是否過於接近平行：
$$\cos\theta_{\text{axis}} = |\hat{a} \cdot \hat{p}|$$

若 $|\hat{a} \cdot \hat{p}| > 1 - \epsilon$，則投影計算不穩定。

2. **投影位移到旋轉角的轉換**（詳見 Geometry.md）：
$$x = R \left[ (\cos\theta - 1)\alpha + \sin\theta \cdot \beta \right]$$

其中：
- $R$：旋轉半徑（點到軸的距離）
- $\alpha = \hat{u} \cdot \hat{p}$：徑向權重
- $\beta = \hat{v} \cdot \hat{p}$：切向權重

3. **求解旋轉角**：
使用輔助角公式：
$$\theta = \arcsin\left(\frac{x + R\alpha}{\sqrt{(R\beta)^2 + (R\alpha)^2}}\right) - \phi$$

其中 $\phi = \arctan2(R\alpha, R\beta)$

4. **應用旋轉**：
$$P_{\text{new}} = \text{rotate\_around\_axis}(P, \text{axis}, \theta)$$

**幾何意義**：
- 輸入：沿法向的線性位移 $dx$
- 輸出：繞軸的旋轉運動 $\theta$
- 將線性運動轉換為旋轉運動

**機制應用**：
- 凸輪機構中的從動件運動
- 搖臂式轉向機構
- 液壓缸驅動的臂桿運動

---

## 4. 時間輸入控制

### 4.1 簡諧運動輸入

```python
def harmonic_input(t, SIM_TIME, x_max=1, x_min=-1, times=1):
    """簡諧運動：正弦波形輸入"""
```

**數學模型**：

1. **基本參數**：
$$A = \frac{x_{\text{max}} - x_{\text{min}}}{2}, \quad x_0 = \frac{x_{\text{max}} + x_{\text{min}}}{2}$$

2. **相位計算**（確保 $t=0$ 時 $x=0$）：
$$\phi = \arcsin\left(\frac{-x_0}{A}\right)$$

3. **角頻率**：
$$\omega = \frac{2\pi \times \text{times}}{T_{\text{sim}}}$$

4. **運動方程**：
$$x(t) = x_0 + A \sin(\omega t + \phi)$$

**運動特性**：
- 速度：$\dot{x}(t) = A\omega \cos(\omega t + \phi)$
- 加速度：$\ddot{x}(t) = -A\omega^2 \sin(\omega t + \phi)$
- 簡諧運動的加速度與位移成正比（彈簧特性）

**應用場景**：
- 週期性振動測試
- 平滑啟動/停止運動
- 模擬彈性元件行為

### 4.2 線性往返運動

```python
def linear_input(t, SIM_TIME, x_max=1, x_min=-1, times=1):
    """線性往返運動：鋸齒波輸入"""
```

**數學模型**：

1. **總行程**（一個完整循環）：
$$L = 2(x_{\text{max}} - x_{\text{min}})$$

2. **速度**：
$$v = \frac{L}{T_{\text{sim}}} \times \text{times}$$

3. **累積行程**：
$$s = (vt) \mod L$$

4. **位置映射**（三段式）：
$$
x(s) = 
\begin{cases} 
s & \text{區段 A: } 0 \to x_{\text{max}} \\
2x_{\text{max}} - s & \text{區段 B: } x_{\text{max}} \to x_{\text{min}} \\
s - L & \text{區段 C: } x_{\text{min}} \to 0
\end{cases}
$$

**運動特性**：
- 速度恆定（鋸齒波）
- 在端點處速度突變（加速度無限大）
- 適合模擬線性執行器

**應用場景**：
- 液壓缸直線運動
- 馬達驅動的往返運動
- 疲勞測試載入

---

## 5. 運動配置類別

### 5.1 MotionConfig 類別

```python
class MotionConfig:
    def __init__(self, motion_type, input_type, x_max, x_min, times=1, normal_vector=None):
```

**運動類型 (motion_type)**：
| 代碼 | 運動類型 | 自由度 |
|------|----------|--------|
| `s` | 滑塊運動 (Slider) | 1 DOF |
| `r` | 旋轉運動 (Rotate) | 1 DOF |
| `f` | 面推移 (Face shift) | 1 DOF |

**輸入類型 (input_type)**：
| 代碼 | 輸入類型 | 特性 |
|------|----------|------|
| `h` | 簡諧輸入 (Harmonic) | 平滑連續 |
| `l` | 線性輸入 (Linear) | 鋸齒波 |

**配置參數**：
- `x_max`, `x_min`：運動範圍
- `times`：運動循環次數
- `normal_vector`：面推移的法向向量（僅用於 `f` 類型）

### 5.2 運動apply流程

```python
def get_input(self, t, SIM_TIME):
    # 根據時間計算輸入值 x
    return harmonic_input(t, ...) 或 linear_input(t, ...)

def apply_motion(self, P_init, axis, x):
    # 根據運動類型應用到初始位置
    if motion_type == "s": return Slider_motion(...)
    if motion_type == "r": return rotate_around_axis(...)
    if motion_type == "f": return face_shift(...)
```

**完整流程**：
$$t \xrightarrow{\text{get\_input}} x(t) \xrightarrow{\text{apply\_motion}} P(t)$$

---

## 6. 運動學關鍵公式總結

| 運動類型 | 自由度 | 約束形式 | 輸入-輸出關係 |
|---------|-------|--------|--------------|
| 繞軸旋轉 | 1 DOF | 距軸固定 $r=\text{const}$ | $\theta \to P_{\text{rot}}$ |
| 滑塊運動 | 1 DOF | 垂直分量為零 | $\Delta x \to P_{\text{new}}$ |
| 面推移 | 1 DOF | 投影到法向 | $\Delta x \to \theta \to P_{\text{rot}}$ |
| 簡諧輸入 | — | — | $t \to A\sin(\omega t + \phi)$ |
| 線性輸入 | — | — | $t \to vt$ (鋸齒波) |
