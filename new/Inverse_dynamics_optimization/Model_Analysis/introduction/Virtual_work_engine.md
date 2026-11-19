# Virtual_work_engine 虛功引擎

## 1. 虛功原理基礎

### 1.1 虛功定義

對於受力點 $P$ 受到力 $\vec{F}$ 作用，经歷虛位移 $\delta \vec{r}$，虛功為：

$$\delta W = \vec{F} \cdot \delta \vec{r}$$

### 1.2 係統虛功平衡

對於約束係統，在滿足約束的任何虛位移下，虛功應平衡：

$$\sum_i \vec{F}_i \cdot \delta \vec{r}_i = 0$$

### 1.3 能量守恒（功率守恒）

在約束條件下，輸入功率應等於係統各點的功率之和：

$$P_{\text{in}} = \vec{F}_{\text{in}} \cdot \vec{v}_{\text{in}}$$

係統處於理想約束（無摩擦、無能量損失）時：

$$P_{\text{in}} = \sum_i \vec{F}_i \cdot \vec{v}_i$$

---

## 2. 力量傳递的數學模型

### 2.1 輸入功率計算

```python
def update_force_context(self, xdot, in_node, in_axis, f_in_val):
    """計算輸入功率"""
```

給定：
- 輸入節點 $N_{\text{in}}$
- 輸入軸單位向量 $\hat{\mathbf{a}}_{\text{in}}$
- 輸入力大小 $F_{\text{in}}$

輸入節點的速度：
$$\vec{v}_{\text{in}} = \dot{\mathbf{x}}[\text{idx}(N_{\text{in}})]$$

輸入力矢量（沿軸方向）：
$$\vec{F}_{\text{in}} = F_{\text{in}} \cdot \hat{\mathbf{a}}_{\text{in}}$$

輸入功率：
$$P_{\text{in}} = F_{\text{in}} \cdot (\vec{v}_{\text{in}} \cdot \hat{\mathbf{a}}_{\text{in}})$$

### 2.2 節點處的約束力

在功率守恒原則下，假設各節點處的力與速度方向相同（最大傳递效率）：

$$\vec{F}_i \parallel \vec{v}_i$$

設 $\vec{F}_i = \lambda_i \vec{v}_i$，則功率平衡：

$$P_{\text{in}} = \sum_i \vec{F}_i \cdot \vec{v}_i = \sum_i \lambda_i v_i^2$$

對於單個節點 $i$，假設從係統中獲得所有輸入功率：

$$P_{\text{in}} = \vec{F}_i \cdot \vec{v}_i = \lambda_i v_i^2$$

解得反比例係數：
$$\lambda_i = \frac{P_{\text{in}}}{v_i^2}$$

因此節點處的力：
$$\vec{F}_i = \frac{P_{\text{in}}}{v_i^2} \cdot \vec{v}_i$$

```python
def get_node_force(self, node, xdot):
    """根據功率守恆獲得節點處的力"""
    if node.fixed:
        return np.zeros(3)
    
    v_node = self.kin.get_velocity(node, xdot)
    v_mag2 = np.dot(v_node, v_node)
    
    if v_mag2 > 1e-8:
        # F = (P / V²) * V
        return (self._current_power_in / v_mag2) * v_node
    return np.zeros(3)
```

### 2.3 力矩計算方法

虛功引擎中，力矩計算先從節點速度推導節點力，然後將該力對指定軸做力矩投影。

給定軸 $\mathbf{a}$ 與節點位置 $\mathbf{p}$，計算步驟如下：

1. 由 `get_node_force()` 取得節點受力 $\vec{F}$。
2. 取節點到軸的最短距離向量 $\vec{r}$。
3. 計算力矩向量 $\vec{\tau} = \vec{r} \times \vec{F}$。
4. 取軸方向分量 $\tau = \vec{\tau} \cdot \hat{\mathbf{a}}$。

```python
axis_unit, P0 = axis
F = self.get_node_force(node, xdot)
r = geo.line_point_min_distance(node.pos, axis)
tau_vec = np.cross(r, F)
return np.dot(tau_vec, axis_unit)
```

此方法可用於 `torque_targets` 設定中，將節點虛功力轉換為指定軸的力矩輸出。

---

## 3. 力學優勢 (Mechanical Advantage)

### 3.1 定義

輸出力與輸入力的比值：

$$MA = \frac{F_{\text{out}}}{F_{\text{in}}}$$

或用功率守恆推導：

$$MA = -\frac{v_{\text{in}}}{v_{\text{out}}}$$

（負號表示相反方向）

### 3.2 從速度比推導力比

功率守恆：
$$F_{\text{in}} \cdot v_{\text{in}} = F_{\text{out}} \cdot v_{\text{out}}$$

$$\frac{F_{\text{out}}}{F_{\text{in}}} = \frac{v_{\text{in}}}{v_{\text{out}}}$$

### 3.3 時變力學優勢

由於機構幾何不斷變化，力學優勢随時間變化

---

## 4. 理想約束下的力

### 4.1 約束反力

對於約束條件 $\mathbf{c}(\mathbf{x}) = \mathbf{0}$，約束力必鬚滿足：

$$\mathbf{F}_{\text{constraint}} = \mathbf{J}^T \boldsymbol{\lambda}$$

其中 $\boldsymbol{\lambda}$ 是拉格朗日乘數向量

### 4.2 功率相容性

約束反力做功必鬚為零（理想約束）：

$$\mathbf{F}_{\text{constraint}} \cdot \dot{\mathbf{x}} = \mathbf{J}^T\boldsymbol{\lambda} \cdot \dot{\mathbf{x}} = \boldsymbol{\lambda} \cdot (\mathbf{J}\dot{\mathbf{x}}) = 0$$

這是因為 $\mathbf{J}\dot{\mathbf{x}} = \mathbf{b}$ 為約束驅動速度，約束力垂直於可行速度方向

## 5. 算法流程圖


```
輸入力 F_in
  ↓
[計算inputAxis上的速度分量] v_in·axis
  ↓
P_in = F_in * (v_in·axis)
  ↓
對每個節點i：
  ├─ v_i = 運動學引擎獲取
  ├─ |v_i|² 的計算
  ├─ F_i = (P_in / |v_i|²) * v_i
  └─ 輸出 F_i
```

---

## 6. 能量分析方法

### 6.1 功率平衡檢验

在仿真結束時驗證：

$$\int_0^T P_{\text{in}} \, dt = \sum_i \int_0^T \vec{F}_i \cdot \vec{v}_i \, dt$$

### 6.2 耗散能量

如果實際係統有阻尼：

$$P_{\text{in}} = P_{\text{kinetic}} + P_{\text{damping}} + P_{\text{storage}}$$

### 6.3 機械效率

$$\eta = \frac{P_{\text{out}}}{P_{\text{in}}}$$

理想情况 $\eta = 100\%$；實際考虑摩擦時 $\eta < 100\%$

---

## 總結表

| 概念 | 數學表達 | 物理意義 |
|-----|--------|--------|
| 虛功 | $\delta W = \vec{F} \cdot \delta \vec{r}$ | 力與位移的內積 |
| 功率 | $P = \vec{F} \cdot \vec{v}$ | 單位時間內的功 |
| 功率守恆 | $P_{\text{in}} = \sum P_i$ | 能量不損失原則 |
| 力學優勢 | $MA = \frac{F_{\text{out}}}{F_{\text{in}}}$ | 放大比例 |
| 約束力 | $\vec{F}_c = \mathbf{J}^T\boldsymbol{\lambda}$ | 拉格朗日乘數作用 |

---

## 參考資源

1. **Pfeiffer & Glocker (2000)** - Multi-body Dynamics with Unilateral Constraints
2. **Goldstein (2002)** - Classical Mechanics
3. **Murray, Sastry & Sastry (1994)** - A Mathematical Introduction to Robotic Manipulation
