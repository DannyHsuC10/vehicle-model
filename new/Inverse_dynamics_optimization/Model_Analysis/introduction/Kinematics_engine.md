# Kinematics_engine.py - 運動學引擎

## 1. 速度反運動學

### 1.1 約束冗餘速度方程

對於約束 $\mathbf{c}(\mathbf{x}) = \mathbf{0}$，對時間求導：

$$\frac{d}{dt}\mathbf{c}(\mathbf{x}) = \frac{\partial \mathbf{c}}{\partial \mathbf{x}} \cdot \frac{d\mathbf{x}}{dt} = \mathbf{0}$$

即：
$$\mathbf{J}(\mathbf{x}) \cdot \dot{\mathbf{x}} = \mathbf{0}$$

其中：
- $\mathbf{J} = \frac{\partial \mathbf{c}}{\partial \mathbf{x}}$ → 約束雅可比矩陣
- $\dot{\mathbf{x}} = [v_1, v_2, \ldots, v_n]^T$ → 節點速度向量

### 1.2帶有驅動項的速度求解

當系統有驅動約束（如PositionConstraint）時：

$$\mathbf{J}(\mathbf{x}) \cdot \dot{\mathbf{x}} = \mathbf{b}$$

其中 $\mathbf{b}$ 是驅動約束的速度項：

$$b_i = \frac{dP_{\text{target},i}}{dt}$$

### 1.3 最小二乘求解

問題變為過定系統（約束多於自由度）：

$$\min{\dot{\mathbf{x}}} \|\mathbf{J}\dot{\mathbf{x}} - \mathbf{b}\|_2^2$$

使用最小二乘解：
$$\dot{\mathbf{x}} = (\mathbf{J}^T\mathbf{J})^{-1}\mathbf{J}^T\mathbf{b}$$

或數值上使用偽逆：
$$\dot{\mathbf{x}} = \mathbf{J}^{\dagger}\mathbf{b}$$

```python
def solve_velocity(self, x, b=None):
    J = self.solver.jacobian(x)
    if b is None:
        b = np.zeros(J.shape[0])
    xdot = np.linalg.lstsq(J, b, rcond=None)[0]
    return xdot, J
```

---

## 2. 加速度

### 2.1 加速度方程推導

對 $\mathbf{J}\dot{\mathbf{x}} = \mathbf{b}$ 再求時間導數：

$$\frac{d}{dt}(\mathbf{J}\dot{\mathbf{x}}) = \frac{d\mathbf{b}}{dt}$$

展開：
$$\frac{d\mathbf{J}}{dt}\dot{\mathbf{x}} + \mathbf{J}\frac{d\dot{\mathbf{x}}}{dt} = \frac{d\mathbf{b}}{dt}$$

即：
$$\frac{d\mathbf{J}}{dt}\dot{\mathbf{x}} + \mathbf{J}\ddot{\mathbf{x}} = \frac{d\mathbf{b}}{dt}$$

整理：
$$\mathbf{J}\ddot{\mathbf{x}} = \frac{d\mathbf{b}}{dt} - \frac{d\mathbf{J}}{dt}\dot{\mathbf{x}}$$

### 2.2 數值雅可比時間導數

$$\frac{d\mathbf{J}}{dt} \approx \frac{\mathbf{J}^{(t)} - \mathbf{J}^{(t-\Delta t)}}{\Delta t}$$

```python
def solve_acceleration(self, x, xdot, J, dt):
    if self.prev_J is None:
        self.prev_J = J.copy()
        return np.zeros_like(xdot)
    
    dJdt = (J - self.prev_J) / dt
    rhs = - dJdt @ xdot  # 右邊項
    
    xddot = np.linalg.lstsq(J, rhs, rcond=None)[0]
    self.prev_J = J.copy()
    
    return xddot
```

---

## 3. 剛體轉動運動學

### 3.1 連桿角速度

對於連接兩個節點 $A$ 和 $B$ 的連桿，角速度向量為：

```python
def rod_angular_velocity(self, rod, xdot):
    """通過端點速度計算約束剛體的角速度"""
```

**推導**：
連桿指向向量：
$$\vec{r} = P_B - P_A$$

端點速度差：
$$\vec{v}_{\text{rel}} = v_B - v_A$$

約束條件（距離恆定）：
$$\frac{d}{dt}\|\vec{r}\|^2 = \frac{d}{dt}(\vec{r} \cdot \vec{r}) = 0$$

$$2\vec{r} \cdot \frac{d\vec{r}}{dt} = 0$$

$$\vec{r} \cdot \vec{v}_{\text{rel}} = 0$$

對於剛體旋轉：
$$v_B - v_A = \vec{\omega} \times \vec{r}$$

因此：
$$\vec{\omega} \times \vec{r} = \vec{v}_{\text{rel}}$$

用外積的性質：
$$\vec{\omega} = \frac{\vec{r} \times \vec{v}_{\text{rel}}}{\vec{r} \cdot \vec{r}} = \frac{\vec{r} \times \vec{v}_{\text{rel}}}{L^2}$$

**向量形式**：
$$\boldsymbol{\omega} = \frac{\vec{r} \times (v_B - v_A)}{|\vec{r}|^2}$$

### 3.2 連桿角加速度

類似推導加速度級別：

```python
def rod_angular_acceleration(self, rod, xddot):
    """通過端點加速度計算角加速度"""
```

$$\boldsymbol{\alpha} = \frac{\vec{r} \times (a_B - a_A)}{L^2}$$

**物理意義**：只有垂直於桿指向的加速度分量産生旋轉

### 3.3 連桿角度紀錄

```python
def rod_orientation_angles(self, rod):
    """旋轉角度計算"""
```

`rod_orientation_angles` 用於計算連桿在三個主平面上的方向角，並作為紀錄連桿姿態的依據。這對於機構仿真中需要追蹤桿件轉動、角度變化或角度累積的應用非常重要。
**定義**
桿的兩端點位置向量：
$$
\mathbf{p}_a = (x_a, y_a, z_a), \quad \mathbf{p}_b = (x_b, y_b, z_b)
$$

方向向量：
$$
\mathbf{r} = \mathbf{p}_b - \mathbf{p}_a = (x_b - x_a, \; y_b - y_a, \; z_b - z_a)
$$

長度：
$$
L = \|\mathbf{r}\| = \sqrt{(x_b - x_a)^2 + (y_b - y_a)^2 + (z_b - z_a)^2}
$$

若 \(L < 10^{-8}\)，則方向角定義為零。

單位方向向量：
$$
\hat{\mathbf{r}} = \frac{\mathbf{r}}{L} = \left( \frac{r_x}{L}, \frac{r_y}{L}, \frac{r_z}{L} \right)
$$

---
1. **相對於 XY 平面的角度（方位角）**  
$$
\theta_z = \arctan2(\hat{r}_y, \hat{r}_x)
$$

2. **相對於 XZ 平面的角度**  
$$
\theta_y = \arctan2(\hat{r}_x, \hat{r}_z)
$$

3. **相對於 YZ 平面的角度**  
$$
\theta_x = \arctan2(\hat{r}_z, \hat{r}_y)
$$

**方法輸出**

該方法會返回一個長度為 3 的向量：

- `$\theta_x$` : YZ 平面角度
- `$\theta_y$` : ZX 平面角度
- `$\theta_z$` : XY 平面角度

## 4. 運動速度與加速度的標量

### 4.1 節點速率（速度大小）

```python
def get_speed(self, node, xdot):
    """節點速度的歐幾裡得范數"""
```

$$v = \|\vec{v}\| = \sqrt{v_x^2 + v_y^2 + v_z^2}$$

### 4.2 節點加速度大小

```python
def get_acc_magnitude(self, node, xddot):
    """節點加速度的大小"""
```

$$a = \|\vec{a}\| = \sqrt{a_x^2 + a_y^2 + a_z^2}$$

### 4.3 連桿角速率與角加速率

```python
def get_omega_magnitud(self, node, xdot):
    """連桿角速度的大小"""

def get_alpha_magnitude(self, node, xddot):
    """連桿角加速度的大小"""
```

$$\omega = \|\vec{\omega}\|, \quad \alpha = \|\vec{\alpha}\|$$

---

## 5. 核心速度求解方程

### 5.1 一般形式

給定數值位移：
$$\Delta \mathbf{x} = \mathbf{x}^{(t)} - \mathbf{x}^{(t-\Delta t)}$$

相應的"平均速度項"：
$$\mathbf{b}_{\text{numeric}} = \frac{\Delta \mathbf{x}}{\Delta t}$$

這應该滿足約束條件 $\mathbf{J}\dot{\mathbf{x}} = \mathbf{b}$

通過最小二乘找到一致的全局速度：
$$\dot{\mathbf{x}} = \mathbf{J}^{\dagger}(\mathbf{J}\mathbf{b}_{\text{numeric}})$$

---

## 6. 物理解釋

### 6.1 速度分配

當輸入點以速度 $v_{\text{in}}$ 運動時，各輸出點会根據機構幾何自動分配速度，體現了機械優勢（Mechanical Advantage）

### 6.2 奇異配置

當 $\text{det}(\mathbf{J}) \approx 0$ 時，機構處於奇異位置，速度可能發散或不確定

### 6.3 加速度的二階效應

雅可比儀率 $\frac{d\mathbf{J}}{dt}$ 項産生的加速度是配置相關的非線性效應

---

## 8. 應用在工作流

```
1. 位置求解（位置級）
   → 通過非線性優化得到 x

2. 速度求解（速度級）
   方程：J(x) * ẋ = Δx/Δt
   → 求解ẋ

3. 加速度求解（加速度級）
   方程：J(x) * ẍ = (dJ/dt * ẋ)方向
   → 求解ẍ

4. 輸出運動參數
   → 用於虛功、力量計算
```

---

## 總結公式表

| 量 | 數學定義 | 代碼函數 |
|----|--------|--------|
| 節點速度 | $\dot{\mathbf{x}} = \mathbf{J}^{\dagger}\mathbf{b}$ | `solve_velocity` |
| 節點加速度 | $\ddot{\mathbf{x}} = \mathbf{J}^{\dagger}(\frac{d\mathbf{b}}{dt} - \frac{d\mathbf{J}}{dt}\dot{\mathbf{x}})$ | `solve_acceleration` |
| 連桿角速度 | $\vec{\omega} = \frac{\vec{r} \times \vec{v}_{\text{rel}}}{L^2}$ | `rod_angular_velocity` |
| 連桿角加速度 | $\vec{\alpha} = \frac{\vec{r} \times \vec{a}_{\text{rel}}}{L^2}$ | `rod_angular_acceleration` |
