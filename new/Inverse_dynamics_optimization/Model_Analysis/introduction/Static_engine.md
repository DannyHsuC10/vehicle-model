# Static_engine 靜態引擎
## 1. CSV數據格式

### 1.1 節點配置 (nodes.csv)

**文件格式**：CSV表格

| 列名 | 數據類型 | 描述 | 約束 |
|------|--------|------|------|
| `name` | 字符串 | 節點唯一標識符 | 不允许重复 |
| `x` | 浮點數 | X坐標 (mm或m) | 需要單位一致 |
| `y` | 浮點數 | Y坐標 | 需要單位一致 |
| `z` | 浮點數 | Z坐標 | 需要單位一致 |
| `fixed` | 整數(0/1) | 是否固定(1=固定, 0=自由) | 二進制 |

**例**：
```csv
name,x,y,z,fixed
rack,0,0,0,0
rack_r,50,-20,0,1
rack_l,-50,-20,0,1
tie_rod,25,-20,12.5,0
```

**數學含義**：
- 固定節點 (`fixed=1`)：作為參考點，不參加優化
- 自由節點 (`fixed=0`)：作為優化變量

### 1.2 桿件配置 (rods.csv)

| 列名 | 數據類型 | 描述 |
|------|--------|------|
| `name` | 字符串 | 桿件唯一標識符 |
| `a` | 字符串 | 起點節點名稱 |
| `b` | 字符串 | 終點節點名稱 |

**例**：
```csv
name,a,b
racktie,rack,tie_rod
uAtie,A_upper,tie_rod
```

---

## 2. 載入函數

### 2.1 節點載入

```python
def load_nodes(path, NodeClass, visual=True):
    """
    從CSV文件載入節點配置
    
    參數：
        path       : CSV文件路徑
        NodeClass  : Node類（用於實例化）
        visual     : 是否創建可視化對象
    
    返回：
        nodes      : dict {node_name: Node對象}
    """
```

**算法步驟**：

1. **打開文件**
   ```python
   with open(path, newline='', encoding='utf-8-sig') as f:
       reader = csv.DictReader(f)
   ```
   - `encoding='utf-8-sig'`：處理UTF-8 BOM
   - `newline=''`：跨平台行終止符兼容性

2. **逐行讀取**
   ```python
   for row in reader:
       pos = [float(row['x']), float(row['y']), float(row['z'])]
       fixed = bool(int(row['fixed']))
   ```

3. **創建Node對象**
   ```python
   node = NodeClass(pos, fixed=fixed, visual=visual)
   nodes[row['name']] = node
   ```

4. **返回字典**
   $$\text{nodes} = \{n_1: N_1, n_2: N_2, \ldots, n_k: N_k\}$$

### 2.2 桿件載入

```python
def load_rods(path, nodes, RodClass, visual=True):
    """
    從CSV文件載入桿件配置
    
    參數：
        path      : CSV文件路徑
        nodes     : 已載入的節點字典
        RodClass  : Rod類（用於實例化）
        visual    : 是否創建可視化對象
    
    返回：
        rods      : dict {rod_name: Rod對象}
    """
```

**算法步驟**：

1. **查表獲取節點**
   ```python
   a = nodes[row["a"]]
   b = nodes[row["b"]]
   ```
   - 假設節點已存在（需前置load_nodes()）

2. **創建Rod對象**
   ```python
   r = RodClass(a, b, name=name, visual=visual)
   ```
   - Rod內部自動計算初始長度：$L = \|P_b - P_a\|$

3. **檢測重复**
   ```python
   if name in rods:
       raise ValueError(f"Rod name 重複: {name}")
   rods[name] = r
   ```

4. **返回字典**
   $$\text{rods} = \{r_1: R_1, r_2: R_2, \ldots, r_m: R_m\}$$

---

## 3. 初始拓譜約束計算

### 3.1 桿長初始化

在載入時，每根桿件的長度被鎖定：

```python
class Rod:
    def __init__(self, a, b, ...):
        self.L = np.linalg.norm(a.pos - b.pos)
        # 這個L值在整個仿真中保持不變（除非override）
```

**數學意義**：
桿件約束 $c_i = \|P_{b_i} - P_{a_i}\| - L_i^0 = 0$

其中 $L_i^0$ 是從CSV載入得到的初始長度

### 3.2 節點索引分配

載入完成後，需要調用：

```python
solver.build_index()
```

這將為所有非固定節點分配在求解向量中的位置

---

## 4. 拓譜圖表示

### 4.1 圖結構

將機構表示為圖：

$$G = (V, E)$$

其中：
- **頂點** $V$：節點集合 $\{N_1, N_2, \ldots, N_k\}$
- **邊** $E$：桿件集合 $\{R_1, R_2, \ldots, R_m\}$


### 4.2 連通性分析

**自由度計算** (DOF Analysis)：

初始DOF = $3 \times n_{\text{free}}$

其中 $n_{\text{free}}$ = 非固定節點數

每個約束削減1個自由度：

$$\text{Remaining DOF} = 3n_{\text{free}} - m_{\text{constraints}}$$

**欠定係統** ($\text{DOF} > 0$)：多個解
**適定係統** ($\text{DOF} = 0$)：唯一解
**超定係統** ($\text{DOF} < 0$)：可能無解

---

## 5. 初值一致性

確保初始構型滿足所有約束：

$$\mathbf{c}(\mathbf{x}_0) \approx \mathbf{0}$$

通過調用：
```python
success, x0 = solver.solve(solver.pack())
```

如無需調整，直接使用CSV坐標

---

## 6. 數據統計

| 量 | 維度 |
|----|-----|
| 節點集合 $V$ | $n = n_{\text{free}} \times 3$ |
| 約束集合 $C$ | $m = \text{桿件數} + \text{驅動數}$ |
| 雅可比矩陣 $J$ | $(m \times n)$ |

---

## 7. 靜力求解工具（StaticAnalyzer）

### 7.1 求解器概述

`StaticAnalyzer` 類負責求解靜力平衡方程，計算所有桿件在外力作用下的內力。

```python
class StaticAnalyzer:
    def __init__(self, state):
        self.state = state
        self.nodes = state.nodes
        self.rods = state.active_rods  # 只考慮標記為 Ft=1 的桿件
    
    def solve(self):
        """求解靜力平衡方程"""
        # 建立平衡方程組
        # 求解桿件內力
        # 返回內力向量
        
    def compute_node_forces(self):
        """計算節點力平衡驗證"""
        # 驗證所有節點是否滿足力平衡
```

### 7.2 靜力平衡方程建立

#### 基本假設

- 結構處於靜態平衡狀態
- 所有桿件為二力桿（力沿桿件軸向）
- 忽略桿件自重

#### 平衡方程

對於每個自由節點 $i$ 和每個方向 $\alpha \in \{x, y, z\}$：

$$\sum_{j=1}^{m} F_j \cdot u_{j,\alpha} + F_{ext,i,\alpha} = 0$$

其中：
- $F_j$：第 $j$ 根桿件的內力（標量）
- $u_{j,\alpha}$：第 $j$ 根桿件在方向 $\alpha$ 上的單位向量分量
- $F_{ext,i,\alpha}$：節點 $i$ 在方向 $\alpha$ 上的外加載荷

#### 矩陣形式

$$\mathbf{A} \mathbf{F} = -\mathbf{f}_{\text{ext}}$$

其中：
- $\mathbf{A}$：平衡矩陣，尺寸 $[3n_{\text{free}} \times m]$
- $\mathbf{F}$：桿件內力向量，尺寸 $[m \times 1]$
- $\mathbf{f}_{\text{ext}}$：外力向量，尺寸 $[3n_{\text{free}} \times 1]$

### 7.3 求解過程

**算法步驟**：

1. **遍歷所有自由節點**
   ```python
   for node in nodes.values():
       if node.fixed:
           continue
   ```

2. **針對各方向建立平衡方程**
   ```python
   for axis in range(3):  # x, y, z
       row = np.zeros(len(rods))
   ```

3. **構建平衡矩陣行**
   - 對於每根桿件 $r$：
   - 計算桿件方向單位向量 $\mathbf{u} = \frac{\mathbf{P}_b - \mathbf{P}_a}{L}$
   - 確定節點在桿件中的位置（起點或終點）
   - 判斷力的方向符號
   ```python
   d = r.b.pos - r.a.pos
   L = np.linalg.norm(d)
   u = d / L
   
   if r.a == node:
       row[i] += 1 * u[axis]  # 起點：力指向 b
   elif r.b == node:
       row[i] += -1 * u[axis] # 終點：力指向 a
   ```

4. **求解線性方程組**
   ```python
   x, *_ = np.linalg.lstsq(A, b, rcond=None)
   ```
   使用最小二乘法求解超定或欠定系統

### 7.4 節點力平衡驗證

求解後，使用 `compute_node_forces()` 驗證平衡：

$$\mathbf{R}_i = \sum_{j \in \text{connected}} F_j \mathbf{u}_j + \mathbf{F}_{\text{ext},i}$$

應滿足（或接近）：

$$|\mathbf{R}_i| \approx 0 \quad \text{（對所有自由節點）}$$
