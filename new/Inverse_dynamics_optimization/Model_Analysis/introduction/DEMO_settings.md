# DEMO 設定說明

本文件說明 `simulation_DEMO.py`、`Steering.py` 等示範範例中的主要設定項目、數值定義與呼叫方式，讓沒有使用過此工具的人也能快速理解可用的參數與功能。

---

## 1. 模擬系統總覽

示範程式主要使用 `Inverse_kinematics.simulator.SimulationSystem` 作為仿真入口，並結合：

- `Inverse_kinematics.motion.MotionConfig`：建立運動輸入軌跡。
- `mb.PositionConstraint`：將指定節點鎖定在預期運動路徑上。
- `set_targets()`：設定要記錄的節點、連桿、力量與力矩。
- `out.save_to_csv(sim)`：輸出 CSV 資料。

---

## 2. SimulationSystem 主要設定項目

### 2.1 初始化與視覺化

```python
sim = simulator.SimulationSystem(enable_visual=True)
```

- `enable_visual`：是否開啟 VPython 視覺化。
  - `True`：顯示 3D 視覺化畫面。
  - `False`：僅做數值模擬。

### 2.2 覆寫節點初始位置

```python
sim.override_node_positions({
    "tie_rod": [25, -15, 12.5]
})
```

- `overrides` 可以是：
  - `list` 或 `np.array`：直接給定 `x, y, z` 座標。
  - `dict`：可使用 `{"x": ..., "y": ..., "z": ...}` 的方式覆寫部分分量。
- `update_rods=True`：表示覆寫節點後，同步更新桿件長度以符合新幾何。

### 2.3 同步 solver 初始狀態

```python
sim.sim.prev_x = sim.solver.pack()
```

這一步是為了讓後續運動學速度與加速度計算能夠正確基於初始位移建立差分。

---

## 3. 運動輸入設定：MotionConfig

`MotionConfig` 是示範中最常見的運動參數設定類別。

```python
motioninput = mot.MotionConfig(
    motion_type="s",
    input_type="l",
    x_max=40,
    x_min=-40,
    times=SIM_TIME/10
)
```

### 3.1 參數說明

- `motion_type`
  - `s`：滑塊運動（slider）
  - `r`：繞軸旋轉（rotate）
  - `f`：面推移（face shift）
- `input_type`
  - `l`：線性往返運動（linear）
  - `h`：簡諧運動（harmonic）
- `x_max`：最大位移數值。
- `x_min`：最小位移數值。
- `times`：完整循環次數，代表 `SIM_TIME` 內出現幾個循環。
- `normal_vector`：僅在 `motion_type="f"` 時使用，表示面推移的法向量。

### 3.2 運動產生與應用

- `get_input(t, SIM_TIME)`：回傳當前時間 `t` 對應的位移值。
- `apply_motion(P_init, axis, x)`：將計算出的位移值套用到節點初始位置。

常見用法：

```python
x = motioninput.get_input(t, SIM_TIME)
new_pos = motioninput.apply_motion(P_init, axis, x)
P_ctrl.set_target(new_pos)
```

---

## 4. 輸入節點與 PositionConstraint

示範中會建立輸入節點並套用 `PositionConstraint`：

```python
P_input = sim.nodes["rack"]
P_ctrl = mb.PositionConstraint(P_input)
sim.solver.add_constraint(P_ctrl)
```

- `PositionConstraint` 將節點鎖定到 `set_target()` 的目標位置。
- 每個輸入函式都必須更新目標位置，通常在 `input_func(t)` 中呼叫。

---

## 5. 力量與力矩設定

### 5.1 力量設定（force_targets）

範例：

```python
f_targets = [{
    "name": "rack_force",
    "in_node": P_input,
    "in_axis": axis[0],
    "in_force": 950,
}]
```

- `name`：自訂輸出欄位名稱。
- `in_node`：施力節點，可直接傳 `node` 物件。
- `in_axis`：力作用方向向量。
- `in_force`：力量大小，可是常數或可呼叫的函式。

### 5.2 力矩設定（torque_targets）

範例：

```python
t_targets = [{
    "name": "wheel_torque",
    "node": "tie_rod",
    "axis": wheel_axis
}]
```

- `name`：力矩輸出名稱。
- `node`：節點名稱或節點物件。
- `axis`：力矩軸，由 `geo.axis_unitization()` 回傳。

### 5.3 set_targets 呼叫

```python
sim.set_targets(
    node_targets=["rack", "tie_rod", "uf_arm"],
    rod_targets=["w", "uAtie"],
    force_targets=f_targets,
    torque_targets=t_targets
)
```

- `node_targets`：設定要紀錄座標與速度的節點。
- `rod_targets`：設定要紀錄連桿角度與轉動訊號的桿件。
- `force_targets`：設定虛功受力輸入。
- `torque_targets`：設定力矩計算輸出。

---

## 6. 執行與時間設定

```python
sim.run(SIM_TIME=SIM_TIME, DT=0.05)
```

- `SIM_TIME`：總模擬時間。
- `DT`：時間步長。
- 內部步數為 `int(SIM_TIME / DT)`。

### 6.1 成功檢查

- `sim.success`：若模擬中途求解失敗，會被設為 `False`。
- 一般失敗原因為 `solver.solve()` 無法找到可行解。

---

## 7. 資料輸出與圖表

### 7.1 CSV 匯出

```python
out.save_to_csv(sim)
```

- 會輸出到 `Inverse_kinematics/output/` 目錄。
- 檔名格式為 `output_YYYYMMDD_HHMMSS.csv`。

### 7.2 直接印出某一欄位

```python
out.print_out(sim, "tau")
```

### 7.3 圖表繪製

```python
sim.plot(["uAtie_thetaz", "rack_y"], save=True)
```

- `keys`：要繪製的資料鍵名。
- `save=True`：將圖檔存到 `figures/`。

---

## 8. 目前 DEMO 可用的參數與呼叫功能

### 8.1 SimulationSystem

- `SimulationSystem(enable_visual=False)`
- `add_input(func)`
- `set_targets(node_targets, rod_targets, force_targets, torque_targets)`
- `run(SIM_TIME, DT=0.05)`
- `plot(keys, save=False)`
- `override_node_positions(overrides, update_rods=True)`

### 8.2 MotionConfig

- `MotionConfig(motion_type, input_type, x_max, x_min, times=1, normal_vector=None)`
- `get_input(t, SIM_TIME)`
- `apply_motion(P_init, axis, x)`

### 8.3 輸入函式格式

輸入函式應該包含：

- 更新 PositionConstraint 目標位置。
- 回傳當前位移 `x`（可選，但目前範例皆有回傳）。

```python
def input_func(t):
    x = motion.get_input(t, SIM_TIME)
    new_pos = motion.apply_motion(P_init, axis, x)
    P_ctrl.set_target(new_pos)
    return x
```

### 8.4 force_targets 與 torque_targets

- `force_targets`：指定系統的輸入力來源與方向。
- `torque_targets`：指定希望輸出的力矩節點與軸。

### 8.5 輸出資料鍵名範例

- 節點位置：`rack_x`, `rack_y`, `rack_z`
- 節點速度：`rack_v`, `rack_vx`, `rack_vy`, `rack_vz`
- 節點加速度：`rack_a`, `rack_ax`, `rack_ay`, `rack_az`
- 節點受力：`rack_f`, `rack_fx`, `rack_fy`, `rack_fz`
- 連桿角度：`uAtie_thetaX`, `uAtie_thetaY`, `uAtie_thetaZ`
- 連桿角速度/角加速度：`uAtie_omega`, `uAtie_alpha`
- 力矩：`tau`

---

## 9. Steering.py 特別說明

`Steering.py` 是另一個 Demo 類別封裝，用於支援：

- `enable_visual` 參數
- 多軸輸入與第二輸入函式
- `set_targets()` 設定節點、桿、力量與力矩
- `run(SIM_TIME, DT)` 執行仿真
- `sim.plot()` 與 `out.save_to_csv(sim)`

### 9.1 自訂初始幾何調整

若使用 `dv_manager` 和 `x`，會在 `build()` 中透過：

```python
node_dict = self.dv_manager.vector_to_dict(self.x)
sim.override_node_positions(node_dict)
```

這表示可以從設計變數向量直接覆寫節點初始位置。

---

## 10. 使用建議

1. 先用 `enable_visual=False` 進行數值測試。
2. 設定 `sim.sim.prev_x = sim.solver.pack()`，確保速度與加速度計算正確。
3. 若要觀看結果，再開 `enable_visual=True`。
4. `set_targets()` 決定哪些節點與桿件會被記錄到輸出資料中。
5. 若修改節點座標，務必同步更新桿件長度，避免幾何不一致。
