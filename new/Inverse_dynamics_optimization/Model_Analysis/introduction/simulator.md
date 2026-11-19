# simulator 真實系統模擬器

## 1. 模擬的數學框架

### 1.1 梯度時步積分法

雖然使用的是隱式求解，但可看作一種時步離散化：

對於係統狀態 $\mathbf{s}(t) = [\mathbf{x}(t), \dot{\mathbf{x}}(t), \ddot{\mathbf{x}}(t)]$

**時步關係**：
$$\mathbf{x}^{(k+1)} = \mathbf{x}^{(k)} + \Delta t \cdot \dot{\mathbf{x}}^{(k)} + \frac{1}{2}\Delta t^2 \cdot \ddot{\mathbf{x}}^{(k)}$$

（簡化的歐拉法；實際使用數值位移計算）

### 1.2 約束一致

在每個時步 $k$ 維持約束滿足：

$$\mathbf{c}(\mathbf{x}^{(k)}) \approx \mathbf{0}$$

通過非線性優化（最小二乘）

---

## 2. 時步核心 (SimulationCore.step)

### 2.1 步長順序

每個時步有4個關鍵階段：

```
[1] 輸入更新 → [2] 位置求解 → [3] 速度求解 → [4] 虛功計算
```

#### 階段1：輸入驅動更新

```python
input_func()  # 例如：rack_ctrl.set_target(new_pos)
```

定義的 $\mathbf{input\_func}(t)$ 函數更新 PositionConstraint 的目標值

#### 階段2：位置約束求解

```python
success, x = self.solver.solve(self.prev_x)
```

求解滿足所有約束的新位置向量 $\mathbf{x}^{(k)}$

**核心問題**：
$$\min({\mathbf{x}}) \|\mathbf{f}(\mathbf{x})\|_2^2$$

其中 $\mathbf{f}(\mathbf{x})$ 包含：
- 桿長約束：$\|P_b - P_a\| - L_i$
- 驅動約束：$P_{\text{input}} - P_{\text{target}}$

#### 階段3：速度反運動學

數值速度估計項：
$$\mathbf{b}_{\text{numeric}} = \frac{\mathbf{x}^{(k)} - \mathbf{x}^{(k-1)}}{\Delta t}$$

驗證符合位置約束更新

**速度方程**：
$$\mathbf{J}(\mathbf{x}) \cdot \dot{\mathbf{x}} = \mathbf{b}$$

其中 $\mathbf{b} = \mathbf{J}\mathbf{b}_{\text{numeric}}$

最小二乘求解：
$$\dot{\mathbf{x}} = \mathbf{J}^{\dagger}\mathbf{b}$$

```python
b = J @ b_numeric
xdot, _ = self.kin.solve_velocity(x, b)
```

#### 階段4：加速度與虛功

```python
xddot = self.kin.solve_acceleration(x, xdot, J, dt)
```

加速度通過雅可比時間導數計算（見 Kinematics_engine.md）

#### 階段5：力量分配（虛功）

```python
self.vw_engine.update_force_context(xdot, config["in_node"], 
                                    config["in_axis"], current_f_in)
```

根據功率守恆計算各節點約束力

---

## 3. 係統架構

### 3.1 分工

```
SimulationSystem
├─ solver (MechanismSolver)
│  └─ nodes, constraints
├─ kin (KinematicsAnalyzer)
│  └─ 速度/加速度求解
├─ vw_engine (VirtualWorkEngine)
│  └─ 力量/功率計算
├─ sim (SimulationCore)
│  └─ 時步执行
├─ logger (DataLogger)
│  └─ 數據紀錄
└─ viz (Visualizer)
   └─ 3D可視化
```

### 3.2 數據流向

```
CSV文件 (nodes.csv, rods.csv)
  ↓
Static_engine.load_*() → 初始化
  ↓
solver (約束求解)
  ↓
kin (運動學分析)
  ↓
vw_engine (力量計算)
  ↓
logger (紀錄) → output.csv
  ↓
plotter (圖) → figures/
```

---

## 4. 數據紀錄器 (DataLogger)

### 4.1 紀錄內容

對於指定的目標對象，紀錄運動參數：

**節點目標**：
- 位置：$(x, y, z)$
- 速度及其分量：$v, v_x, v_y, v_z$
- 加速度及其分量：$a, a_x, a_y, a_z$
- **約束力**：$f, f_x, f_y, f_z$（來自虛功）

**桿件目標**：
- 角速度及分量：$\omega, \omega_x, \omega_y, \omega_z$
- 角加速度及分量：$\alpha, \alpha_x, \alpha_y, \alpha_z$

### 4.2 時間序列數據

將所有數據組織為時間序列：

$$\text{data}[\text{key}] = [v_0, v_1, v_2, \ldots, v_N]$$

其中 $N = \lfloor T_{\text{sim}} / \Delta t \rfloor$

---

## 5. 模擬循環的數學形式

### 5.1 總體流程

```python
for k = 0 to n_steps:
    t = k * dt
    
    # [1] 輸入
    input_func(t)
    
    # [2] 求解
    x, xdot, xddot, vw_env = sim.step(...)
    
    # [3] 紀錄
    logger.log(t, nodes, rods, kin, vw_env, xdot, xddot)
    
    # [4] 可視化
    viz.update()
```

### 5.2 時間參數化

**時間網格**：
$$t_k = k \cdot \Delta t, \quad k = 0, 1, \ldots, n_{\text{step}}$$

**模擬時間**：
$$T_{\text{sim}} = n_{\text{step}} \cdot \Delta t$$

**數據序列長度**：
$$n_{\text{data}} = n_{\text{step}} + 1$$

