# Multibody_engine 多體系統引擎

## 1. 係統核心概念

### 1.1 係統架構
```
節點 (Nodes) → 未知變量
    ↓
約束 (Constraints) → 方程組
    ↓
求解器 (Solver) → 優化求解
```

---

## 2. 節點類 (Node)

### 2.1 節點的數學表示

```python
class Node:
    pos      # 3D坐標 (3,) numpy array
    fixed    # 是否固定（參與求解標志）
    idx      # 在求解向量x中的索引
```

**節點狀態**：
- 自由節點（`fixed=False`）：參與求解，貢獻3個自由度
- 固定節點（`fixed=True`）：常數，不參與求解

### 2.2 節點在求解向量中的映射

$$\mathbf{x} = \begin{pmatrix} x_{1,1} & y_{1,1} & z_{1,1} & x_{1,2} & y_{1,2} & z_{1,2} & \cdots \end{pmatrix}^T$$

節點 $i$ 的位置從索引 `idx[i]` 到 `idx[i]+3` 提取

---

## 3. 桿件約束 (Rod)

### 3.1 距離約束

```python
class Rod:
    a, b        # 連接的兩個節點
    L           # 初始長度（約束值）
```

**約束形式**：距離保持常數

$$c(\mathbf{x}) = \|P_b - P_a\| - L = 0$$

### 3.2 約束殘差 (Residual)

殘差是約束違反的度量：
$$r = \|P_b - P_a\| - L$$

對於滿足約束的解，$r = 0$

### 3.3 雅可比矩陣 (Jacobian)

約束對位置變量的導數：

$$\frac{\partial c}{\partial \mathbf{x}} = \left[\frac{\partial c}{\partial P_a}, \frac{\partial c}{\partial P_b}\right]$$

其中：
$$\frac{\partial c}{\partial P_a} = -\frac{P_b - P_a}{\|P_b - P_a\|} = -\frac{\vec{r}}{L}$$

$$\frac{\partial c}{\partial P_b} = \frac{P_b - P_a}{\|P_b - P_a\|} = \frac{\vec{r}}{L}$$

**穩定性處理**：當 $L < 10^{-8}$ 時返回零向量

---

## 4. 位置約束 (PositionConstraint)

### 4.1 約束形式

強制某個節點等於指定位置（驅動約束）：

$$\mathbf{c}(\mathbf{x}) = P - P_{\text{target}} = \begin{pmatrix} x - x_t \\ y - y_t \\ z - z_t \end{pmatrix} = \vec{0}$$

**3自由度約束**（每個空間方向一個）

### 4.2 雅可比矩陣

$$\frac{\partial \mathbf{c}}{\partial \mathbf{x}} = \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{pmatrix} = \mathbf{I}_3$$

（對應節點位置；其他位置為0）

### 4.3 應用場景

用於實現：
- 齒條的運動軌跡驅動
- 懸挂點的指定位置控制
- 邊界條件的施加

---

## 5. 核心求解器 (MechanismSolver)

### 5.1 問題表述

**非線性最小二乘問題**：

$$\min({\mathbf{x}})\ \ \frac{1}{2}\|\mathbf{f}(\mathbf{x})\|_2^2$$

其中 $\mathbf{f}(\mathbf{x})$ 是所有約束殘差的堆積向量

### 5.2 數學框架

#### 步驟1：索引構建 (`build_index`)
為所有非固定節點分配索引：
- 節點1：`idx=0` → `x[0:3]`
- 節點2：`idx=3` → `x[3:6]`
- ...

**總自由度**：$n_{\text{DOF}} = 3 \times (\text{非固定節點數})$

#### 步驟2：向量打包 (`pack`)
從節點對象創建初始猜測向量：
$$\mathbf{x}_0 = [P_1, P_2, \ldots, P_n]^T$$

#### 步驟3：向量解包 (`unpack`)
將解回寫到節點對象：
$$P_i^{\text{new}} = \mathbf{x}_{\text{solution}}[\text{idx}[i]:\text{idx}[i]+3]$$

### 5.3 殘差組装 (`residual`)

堆積所有約束的殘差：

$$\mathbf{f}(\mathbf{x}) = \begin{pmatrix} f_1(x) \\ f_2(x) \\ \vdots \\ f_m(x) \end{pmatrix}$$

對於不同類型約束：
- **標量約束** (如Rod)：直接添加
- **向量約束** (如PositionConstraint)：展開為多個分量

### 5.4 雅可比矩陣組装 (`jacobian`)

$$\mathbf{J} = \begin{pmatrix} \frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2} & \cdots \\ \frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2} & \cdots \\ \vdots & \vdots & \ddots \end{pmatrix}$$

維度：$(m \times n)$，其中 $m$ 是約束數，$n$ 是自由度數

### 5.5 求解算法 (`solve`)

$$\text{minimize} \quad \|\mathbf{f}(\mathbf{x})\|_2^2$$

使用 **Levenberg-Marquardt (LM)** 算法：
- 方法 = `'lm'`（快速收歛）
- 容差 = `1e-4`（相對殘差阈值）

**收歛條件**：
1. 殘差足夠小：$\|\mathbf{f}(\mathbf{x}^*)\|_2 < \text{tol}$
2. 節點位移受限：$\|P_i^{\text{new}} - P_i^{\text{old}}\|_2 < \text{max\_step}$（防止跳變）

---

## 6. 求解過程示例

### 6.1 簡單機構：二桿機制

```
固定點O ----[桿1]---- 節點A ----[桿2]---- B(輸入)
```

**自由度分析**：
- 初始DOF：3（節點A的位置）
- 約束1（桿1）：$\|A - O\| = L_1$ → 1個約束
- 約束2（桿2）：$\|B - A\| = L_2$ → 1個約束
- 約束3（B驅動）：$B = B_{\text{target}}(t)$ → 3個約束
- 總約束：5 > 3 → 過定係統

通過最小二乘找最優解

### 6.2 雅可比矩陣例

對上述係統：
$$\mathbf{J} = \begin{pmatrix}
\text{[桿1的導數]} & 0 \\
\text{[桿2的導數]} & -[桿2的導數] \\
0 & I_3
\end{pmatrix}$$

---

## 7. 數值穩定性

### 7.1 奇異點處理

當物理長度接近零時：
```python
if L < 1e-8:
    return np.zeros(n_dof)
```

### 7.2 疊代失敗檢測

- 不收歛：返回 `success=False`
- 節點跳躍過大：返回前一步解

### 7.3 初值選擇

關鍵：$\mathbf{x}_0$ 應该接近真實解（局部收歛性）

---

## 8. 算法複雜度

| 操作 | 時間複雜度 | 说明 |
|-----|----------|------|
| 殘差計算 | $O(m)$ | $m$ 個約束 |
| 雅可比計算 | $O(mn)$ | 有限差分需要 $n$ 次評估 |
| LM求解 | $O(k(mn^2))$ | $k$ 次疊代 |

**典型規模**：
- 節點數：10-50
- 約束數：15-100
- 疊代數：3-10

---

## 總結

| 概念 | 數學形式 | 作用 |
|-----|--------|------|
| Node | $(x, y, z)$ 或固定 | 未知變量/常數 |
| Rod約束 | $\|\Delta P\| = L$ | 保持長度 |
| PositionConstraint | $P = P_t$ | 驅動控制 |
| 最小二乘 | $\min\|f(x)\|^2$ | 求解優化 |
| Jacobian | $\partial f/\partial x$ | 梯度信息 |
