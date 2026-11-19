# Geometry 幾何學模組

## 1. 座標係轉換

### 1.1 向量規範化函數

```python
def to_numpy(vec):
    """轉為 numpy array"""
```

**功能**：統一座標表示格式，支援vpython向量與numpy數組

**數學基礎**：
向量 $\vec{v}$ 可表示：
$$\vec{v} = \begin{pmatrix} v_x \\ v_y \\ v_z \end{pmatrix}$$

---

## 2. 旋轉與變換

### 2.1 旋轉矩陣（Rodrigues）

```python
def rotation_matrix(axis, theta):
    """回傳繞 axis 旋軸轉 theta 弧度的旋轉矩陣"""
```

**(Rodrigues' Rotation Formula)**

對於繞單位向量 $\hat{\mathbf{k}} = (k_x, k_y, k_z)$ 旋轉角度 $\theta$ 的旋轉矩陣：

$$\mathbf{R}(\hat{\mathbf{k}}, \theta) = \mathbf{I} + \sin(\theta) [\hat{\mathbf{k}}]_{\times} + (1-\cos(\theta))[\hat{\mathbf{k}}]_{\times}^2$$

其中斜對稱矩陣（skew-symmetric matrix）為：
$$[\hat{\mathbf{k}}]_{\times} = \begin{pmatrix} 0 & -k_z & k_y \\ k_z & 0 & -k_x \\ -k_y & k_x & 0 \end{pmatrix}$$

**幾何意義**：
- 繞任意軸旋轉點的標准方法
- 用於連桿機制中的相對位置計算

**代碼實現**：
```
R = I + sin(θ)*K + (1-cos(θ))*K²
```

### 2.2 鏡射變換

```python
def mirror_point(point, plane = "Y"):
    """對平面鏡射"""
```

**鏡射矩陣**：
- 對YZ平面（Y方向）鏡射（Y坐標變號）：
$$\mathbf{M}_{Y} = \begin{pmatrix} 1 & 0 & 0 \\ 0 & -1 & 0 \\ 0 & 0 & 1 \end{pmatrix}$$

- 對XZ平面（X方向）鏡射（X坐標變號）：
$$\mathbf{M}_{X} = \begin{pmatrix} -1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{pmatrix}$$

- 對XY平面（Z方向）鏡射（Z坐標變號）：
$$\mathbf{M}_{Z} = \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & -1 \end{pmatrix}$$

**應用**：對稱機構的鏡像點計算

---

## 3. 軸與直線運算

### 3.1 軸的單位化

```python
def axis_unitization(P1, P0):
    """將 P1 和 P0 定義的向量單位化"""
```

**數學定義**：
給定兩點 $P_0$ 和 $P_1$，軸的方向和位置定義為：

$$\hat{\mathbf{a}} = \frac{\vec{P_1P_0}}{|\vec{P_1P_0}|} = \frac{P_1 - P_0}{\|P_1 - P_0\|}$$

軸表示為：$[\hat{\mathbf{a}}, P_0]$（方向向量 + 通過點）

**應用**：齒條運動軸、轉向軸等的標准化

### 3.2 點到直線的最短距離

```python
def line_point_min_distance(point, axis):
    """計算點到直線距離和最近點"""
```

**數學推導**：
給定點 $P$ 和直線 $\mathbf{L}(t) = P_0 + t\hat{\mathbf{a}}$

點在直線上的投影參數：
$$t = (P - P_0) \cdot \hat{\mathbf{a}}$$

最近點：
$$P_{\text{closest}} = P_0 + t\hat{\mathbf{a}}$$

點到直線的距離向量（力臂）：
$$\vec{r} = P - P_{\text{closest}}$$

**力學應用**：計算力臂，用於虛功計算

### 3.3 點到直線的固定距離求解

```python
def line_point_constant_distance(axis, point, distance):
    """在直線上找出與point距離為distance的點（最多兩個解）"""
```

**問題描述**：
找所有點 $X = P_0 + t\hat{\mathbf{a}}$ 使得 $\|X - P\| = d$

**數學求解**：
將條件代入：
$$\|P_0 + t\hat{\mathbf{a}} - P\|^2 = d^2$$

展開得二次方程：
$$t^2 + bt + c = 0$$

其中：
$$b = 2(\vec{w} \cdot \hat{\mathbf{a}}), \quad c = \|\vec{w}\|^2 - d^2, \quad \vec{w} = P_0 - P$$

判別式：$\Delta = b^2 - 4c$

- $\Delta < 0$：無解
- $\Delta = 0$：一個解
- $\Delta > 0$：兩個解

$$t = \frac{-b \pm \sqrt{\Delta}}{2}$$

**機制應用**：連桿機制的反演，求解連桿兩端滿足約束的位置

---

## 4. 長度與距離計算

### 4.1 桿件長度

```python
def line_length(P1, P0):
    """計算兩點間距離"""
```

$$L = \|P_1 - P_0\| = \sqrt{(x_1-x_0)^2 + (y_1-y_0)^2 + (z_1-z_0)^2}$$

### 4.2 近似最近點

```python
def close_to_point(points, target = np.array([0, 0, 0])):
    """判斷哪個點最接近目標"""
```

尋找 argmin 使得 $\|P_i - P_{\text{target}}\|$ 最小

**應用**：在多解中選擇合理的物理解

### 4.3 點在直線上的投影

```python
def project_point_to_line(point, axis):
    """將點投影到直線"""
```

**投影參數**：
$$t = (P - P_0) \cdot \hat{\mathbf{a}}$$

**投影點**：
$$P_{\text{proj}} = P_0 + t\hat{\mathbf{a}}$$

返回投影點和參數 $t$（沿軸的有向距離）

---

## 5. 運動輸入形式

### 5.1 投影距離反求旋轉角

```python
def get_theta_from_projection(x, point_init, axis, proj_vec):
    """根據目標投影位移量反求旋轉角度"""
```

**問題描述**：
已知旋轉軸上某點繞軸旋轉 $\theta$ 後，在特定投影方向上的位移量為 $x$，求對應的旋轉角 $\theta$。

**數學推導**：

1. **建立旋轉半徑向量**：
$$\vec{r} = P - P_{\text{closest}}$$
其中 $P_{\text{closest}}$ 為點到旋轉軸的最近點

半徑長度：$R = \|\vec{r}\|$

2. **建立局部座標系**：
- $\hat{u}$ 方向：半徑方向（初始位置）
$$\hat{u} = \frac{\vec{r}}{R}$$

- $\hat{v}$ 方向：旋轉切線方向（利用外積）
$$\hat{v} = \hat{a} \times \hat{u}$$

3. **旋轉後的位置表示**：
$$\vec{r}(\theta) = R \cos(\theta) \hat{u} + R \sin(\theta) \hat{v}$$

4. **投影位移與旋轉角的關係**：
$$x = [\vec{r}(\theta) - \vec{r}(0)] \cdot \hat{p}$$

其中 $\hat{p}$ 為投影方向單位向量

展開得：
$$x = R \left[ (\cos\theta - 1)\alpha + \sin\theta \cdot \beta \right]$$

其中：
$$\alpha = \hat{u} \cdot \hat{p}, \quad \beta = \hat{v} \cdot \hat{p}$$

5. **轉換為標準形式**：
整理成 $a\sin\theta + b\cos\theta = c$ 的形式：
$$(R\beta)\sin\theta + (R\alpha)\cos\theta = x + R\alpha$$

使用輔助角公式求解：
$$\sqrt{a^2 + b^2} \sin(\theta + \phi) = c$$

其中 $\phi = \arctan2(b, a)$

最終解：
$$\theta = \arcsin\left(\frac{c}{\sqrt{a^2 + b^2}}\right) - \phi$$

**應用場景**：
- 輸入：目標投影位移量 $x$
- 輸出：對應的旋轉角 $\theta$
- 用於：根據給定的運動量（位移）反求所需的旋轉角度

### 5.2 投影有效性檢查

```python
def check_projection_validity(axis_unit, proj_vec, tolerance=0.1):
    """檢查旋轉軸與投影向量是否過於接近平行"""
```

**數學判斷**：
計算旋轉軸方向 $\hat{a}$ 與投影方向 $\hat{p}$ 的夾角餘弦值：
$$\cos\theta_{\text{axis}} = |\hat{a} \cdot \hat{p}|$$

若 $\cos\theta_{\text{axis}} > 1 - \text{tolerance}$，表示兩向量接近平行，投影計算會不穩定。

**幾何意義**：
- 當旋轉軸與投影方向平行時，旋轉產生的投影位移為零
- 此時無法從投影位移反求旋轉角（奇異點）
