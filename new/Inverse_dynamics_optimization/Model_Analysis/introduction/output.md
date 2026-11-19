# output 數據輸出與應用

## 1. CSV數據導出

### 1.1 數據結構

紀錄的數據組織為字典：

```python
data = {
    "t": [0.0, 0.05, 0.10, ...],           # 時間序列
    "rack_vx": [0.0, 1.2, 2.3, ...],       # X方向速度
    "rack_vy": [0.0, 0.3, 0.5, ...],       # Y方向速度
    "tie_rod_f": [0.0, 100, 200, ...],     # 合力大小
    ...
}
```

**維度**：$(\text{keys} \times n_{\text{steps}})$


#### 步驟1：提取數據
```python
data_dict = sim.logger.data
keys = list(data_dict.keys())
length = len(data_dict[keys[0]])
```

數據字典中所有序列長度應相同：
$$n_k = n_{\text{steps}} + 1, \quad \forall k$$

#### 步驟2：構建文件路徑
```python
FILTER_DIR = Path(__file__).resolve().parent
file_path = FILTER_DIR/"output"/ filename
```

自動創建 `output` 目錄（如不存在）

#### 步驟3：寫入CSV


**CSV格式例**：
```csv
t,rack_vx,rack_vy,rack_vz,tie_rod_f,tie_rod_fx,tie_rod_fy,tie_rod_fz
0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
0.05,0.5,0.1,0.0,50.2,45.1,20.3,0.0
0.10,0.95,0.2,0.0,95.3,85.2,41.1,0.0
```

## 1.2 節點與連桿資料紀錄

當使用 `SimulationSystem.set_targets()` 設定 `node_targets` 與 `rod_targets` 時，資料記錄器會自動保存節點座標、速度、加速度、虛功力以及連桿角度資訊。

### 節點記錄鍵值

對於每個 `node_targets` 中的節點，會保存：

- `node_x`, `node_y`, `node_z`：節點位置座標
- `node_v`：節點速度大小
- `node_a`：節點加速度大小
- `node_vx`, `node_vy`, `node_vz`：速度分量
- `node_ax`, `node_ay`, `node_az`：加速度分量
- `node_f`：節點虛功力大小
- `node_fx`, `node_fy`, `node_fz`：虛功力分量

### 連桿角度記錄鍵值

對於每個 `rod_targets` 中的桿件，會保存：

- `rod_thetaX`, `rod_thetaY`, `rod_thetaZ`：連桿三個主平面角度
- `rod_omega`：角速度大小
- `rod_alpha`：角加速度大小
- `rod_omegaX`, `rod_omegaY`, `rod_omegaZ`：角速度分量
- `rod_alphaX`, `rod_alphaY`, `rod_alphaZ`：角加速度分量

### 使用方式

在模擬中，先呼叫：

```python
sim.set_targets(
    node_targets=["rack", "tie_rod"],
    rod_targets=["w", "uAtie"],
    force_targets=f_targets,
    torque_targets=t_targets
)
```

此後 `out.save_to_csv(sim)` 會將對應鍵值寫入 CSV 檔案，方便後處理與可視化分析。

---

## 2. 時間序列數據的數學表示

### 2.1 連續時間與離散化

**連續時間函數**：
$$y(t) = f(t), \quad t \in [0, T]$$

**離散採樣**：
$$y_k = y(t_k) = y(k\Delta t), \quad k = 0, 1, \ldots, n$$

**採樣點總數**：
$$n_{\text{samples}} = \lfloor T / \Delta t \rfloor + 1$$

### 2.2 插值與重構

從離散點重構連續函數的方法：

**線性插值**：
$$y(t) \approx y_k + \frac{y_{k+1} - y_k}{\Delta t}(t - t_k), \quad t \in [t_k, t_{k+1}]$$

**高階插值**（Spline）：
$$y(t) = \sum_{j=0}^{3} a_j (t - t_k)^j, \quad t \in [t_k, t_{k+1}]$$

### 2.3 性能指標

典型的統計指標：

| 指標 | 計算方法 | 物理意義 |
|-----|--------|--------|
| 最大值 | $\max_k y_k$ | 峰值負載 |
| 最小值 | $\min_k y_k$ | 最小負載 |
| 均值 | $\bar{y} = \frac{1}{n}\sum y_k$ | 平均工作點 |
| 標准差 | $\sigma = \sqrt{\frac{1}{n}\sum(y_k - \bar{y})^2}$ | 波動幅度 |
| 積分（功） | $\sum_k y_k \Delta t$ | 總功、總衝量 |
