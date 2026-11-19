# 🏎️ FSAE 多體動力學虛擬感測器與後處理分析手冊 (API Toolkit)

本手冊記錄了模擬主迴圈內可用於淬取、監控與導出數據的四個核心後處理引擎。透過這些工具，可以將幾何狀態無縫轉化為賽車工程設計的量化指標。

---

## 📌 模組一：KinematicsAnalyzer (運動學分析引擎)
負責即時計算任何特定「節點」的三維運動與「連桿」的空間旋轉狀態。

### 1. 節點基礎運動學 API
| 功能 | 函數調用 (Method) | 輸入參數 | 輸出說明 |
| :--- | :--- | :--- | :--- |
| **3D 速度向量** | `analyzer.get_velocity(node, state.dq)` | `node`, `dq` | `np.array([vx, vy, vz])` |
| **3D 加速度向量** | `analyzer.get_acceleration(node, state.ddq)` | `node`, `ddq` | `np.array([ax, ay, az])` |
| **合速度大小** | `analyzer.get_speed(node, state.dq)` | `node`, `dq` | `float` (速度純量) |
| **合加速度大小** | `analyzer.get_acc_magnitude(node, state.ddq)` | `node`, `ddq` | `float` (加速度純量) |

### 2. 連桿旋轉運動學 API
| 功能 | 函數調用 (Method) | 輸入參數 | 輸出說明 |
| :--- | :--- | :--- | :--- |
| **三平面投影角** | `analyzer.rod_orientation_angles(rod)` | `rod` | `np.array([deg_x, deg_y, deg_z])` (度) |
| **3D 角速度向量** | `analyzer.rod_angular_velocity(rod, state.dq)` | `rod`, `dq` | `np.array([wx, wy, wz])` (rad/s) |
| **3D 角加速度向量** | `analyzer.rod_angular_acceleration(rod, state.dq, state.ddq)` | `rod`, `dq`, `ddq` | `np.array([ax, ay, az])` (rad/s²) |
| **合角速度大小** | `analyzer.get_omega_magnitude(rod, state.dq)` | `rod`, `dq` | `float` (rad/s) |
| **合角加速度大小** | `analyzer.get_alpha_magnitude(rod, state.dq, state.ddq)` | `rod`, `dq`, `ddq` | `float` (rad/s²) |

> 💡 **程式碼使用範例**：
> ```python
> tierod = mech.rods["rw"]
> angles = analyzer.rod_orientation_angles(tierod) # 取得轉向拉桿當前空間角度 [X, Y, Z]
> omega = analyzer.rod_angular_velocity(tierod, state.dq) # 取得 3D 角速度
> ```

---

## 📌 模組二：VirtualWorkEngine (虛擬功與幾何力比引擎)
利用虛擬功原理（Virtual Work），繞過動態力學，純粹從機構當前的幾何狀態與約束條件，求解輸入端與輸出端的**幾何效益（機械利益）**。

### 1. 核心查詢 API
| 功能 | 函數調用 (Method) | 重要輸入參數 | 物理意義 |
| :--- | :--- | :--- | :--- |
| **純幾何力比 (MA)** | `vw_engine.compute_mechanical_advantage(...)` | `in_node`, `in_axis`, `out_node`, `out_axis` | 當輸入端沿指定軸移動 1 單位時，輸出端沿目標軸的**力放大倍數**（幾何利益）。 |
| **估算實際輸出力** | `vw_engine.get_estimated_force(...)` | 承上，外加 `f_in_val` (輸入力大小) | 給定輸入牛頓力（如齒條推力），計算輸出端（如轉向節）當下拿到的**實際有效推力**。 |

> 💡 **程式碼使用範例（齒條推轉向節）**：
> ```python
> rack_node = mech.nodes["wheel_ol"]      # 輸入端：轉向齒條
> upright_node = mech.nodes["ul_arm"]     # 輸出端：左前輪轉向節
> 
> # 計算：當齒條沿 X 軸 [1,0,0] 推動時，轉向節沿 Y 軸 [0,1,0] 轉向的力放大倍數
> ma = vw_engine.compute_mechanical_advantage(
>     state, in_node=rack_node, in_axis=[1, 0, 0], out_node=upright_node, out_axis=[0, 1, 0]
> )
> # 估算輸入 500 N 時，輸出端拿到的實際牛頓力
> f_out = vw_engine.get_estimated_force(
>     state, in_node=rack_node, in_axis=[1, 0, 0], f_in_val=500.0, out_node=upright_node, out_axis=[0, 1, 0]
> )
> ```

---

## 📌 模組三：StaticAnalyzer (靜力矩陣平衡分析器)
負責聯立全場所有節點的 3D 靜力平衡方程。透過前級 `DynamicSolver` 倒灌的慣性力（達朗貝爾原理修正），能在高速動態過彎中，精準解出**剛性連桿（Ft=1）內部的真實軸向拉壓力（牛頓）**。

### 1. 核心求解 API
* **函數調用**：`static_analyzer.solve(state)`
* **執行結果**：矩陣求解完成後，會將計算得到的軸向力**自動回寫**至對應連桿物件的 `force` 屬性中。

> 💡 **程式碼使用範例**：
> ```python
> # 1. 執行矩陣求解
> static_analyzer.solve(state)
> 
> # 2. 隨插即用，直接從 mech 結構中取出特定桿件的受力值
> pushrod = mech.rods["ulApush"]
> print(f"推桿真實軸向受力: {pushrod.force:.2f} N") # 正值為拉力，負值為壓力
> ```

---

## 📌 模模四：MaterialEngine (單一連桿材料力學引擎)
這是一個去耦合的材料力學計算機。它與 `StaticAnalyzer` 緊密配合，負責將連桿受到的「牛頓力」轉化為截面應力、應變、變形量，並進行安全係數降伏判定。

### 1. 核心計算 API
* **函數調用**：`material_engine.compute_rod_mechanics(rod, force)`
* **輸入參數**：`rod` (連桿物件), `force` (該連桿當前受力，通常傳入 `rod.force`)
* **屬性同步**：執行後，連桿物件會**自動新增並綁定**以下材料特徵屬性：

| 連桿屬性名 | 物理意義 | 工程計算公式 | 單位 |
| :--- | :--- | :--- | :--- |
| `rod.force` | 軸向內力 | $F$ | $\text{N}$ (牛頓) |
| `rod.stress` | 正向工作應力 | $\sigma = F / A$ | $\text{Pa}$ (可除以 1e6 轉為 $\text{MPa}$) |
| `rod.strain` | 正向應變 | $\varepsilon = \sigma / E$ | $\text{mm/mm}$ (無因次) |
| `rod.delta_L` | 桿件結構長度形變量 | $\Delta L = \varepsilon \times L_0$ | $\text{m}$ (可乘以 1000 轉為 $\text{mm}$) |
| `rod.k` | 桿件軸向剛度 | $k = (A \times E) / L_0$ | $\text{N/m}$ |
| `rod.energy` | 桿件內部積蓄的應變能 | $U = F^2 L_0 / (2AE)$ | $\text{J}$ (焦耳) |
| `rod.safety_ratio` | 工作應力與極限應力比率 | $\text{Ratio} = \|\sigma\| / \sigma_{\text{max}}$ | 無因次 (大於 1 代表降伏斷裂) |
| `rod.fail` | 結構破壞旗標 | $\|\sigma\| > \sigma_{\text{max}}$ | `True` / `False` |

> 💡 **程式碼使用範例**：
> ```python
> llA1 = mech.rods["ulApush"]
> 
> # 將剛解出的力與物件本體傳入材料引擎
> material_engine.compute_rod_mechanics(llA1, llA1.force)
> 
> # 即時讀取材料狀態
> if llA1.fail:
>     print(f"💥 警告：連桿 {llA1.name} 發生斷裂！安全強度比率已達 {llA1.safety_ratio:.2f}")
> ```

---

## 🔄 最佳化與數據採集（Simulation Loop）黃金調用順序

在編寫自動化定時模擬或進行幾何最佳化尋優時，主迴圈內部最健全的推進順序如下：

```python
while sim_time <= t_max:
    # 1. 物理引擎推進 (求解廣義加速度，更新 state.q, state.dq, state.ddq)
    dyn.step(dt)
    
    # 2. 靜力學矩陣解算 (計算剛性連桿內力，寫入各連桿的 .force 屬性)
    static_analyzer.solve(state)
    
    # 3. 淬取感興趣的連桿，進行運動學、材料力學與幾何力比分析
    for name in ["ulApush", "rw"]:
        rod = mech.rods.get(name)
        if rod:
            # 同步運動學角速度與平面投影角
            rod.angles = analyzer.rod_orientation_angles(rod)
            rod.omega_vec = analyzer.rod_angular_velocity(rod, state.dq)
            
            # 同步材料力學強度與破壞狀態
            material_engine.compute_rod_mechanics(rod, rod.force)
            
    # 4. 資料記錄器拍下快照 (自動記錄所有點/桿件的 x,y,z,v,a,F,stress,SF 等數據)
    logger.record(sim_time, mech)
    
    sim_time += dt

# 5. 跳出迴圈，一鍵導出標準 CSV 報表
OutputEngine.export_to_csv(logger, "simulation_report.csv")