# Optimizer 優化器介紹

## 概述

本專案實現了多種數學優化方法，用於解決多體動力學系統的設計參數優化問題。優化器的目標是在滿足約束條件下，最小化（或最大化）目標函數的值。

---

## 使用的數學優化方法

### 1. L-BFGS-B 方法 (ScipyOptimizer)

這部分其實沒有一定要使用L-BFGS-B只是給scipy多種的最佳化算法保留接口提供支援更多工具的可能性

**數學基礎：**
- **演算法**：有限記憶體 Broyden-Fletcher-Goldfarb-Shanno (L-BFGS)
- **類型**：梯度下降法的二階近似
- **核心思想**：使用目標函數的一階導數（梯度）資訊，通過近似 Hessian 矩陣來加速收斂

**數學表達：**
```
x_{k+1} = x_k - α_k * H_k * ∇f(x_k)

其中：
- α_k：學習率（步長）
- H_k：Hessian 矩陣的近似
- ∇f(x_k)：目標函數在 x_k 處的梯度
```

**特點：**
- 適合中等規模的平滑優化問題
- 需要梯度資訊
- 收斂速度較快

**參數：**
- `maxiter`：最大迭代次數作為停止條件

---

### 2. 差分進化演算法 (DEOptimizer)

**數學基礎：**
- **演算法**：Differential Evolution (DE)
- **類型**：演化演算法 / 群體智能
- **核心思想**：通過個體之間的差分向量來產生新候選解

**數學表達：**
```
v_i = x_{r1} + F * (x_{r2} - x_{r3})

其中：
- x_{r1}, x_{r2}, x_{r3}：三個不同的父代個體
- F：差分權重（通常在 0~1 之間）
- v_i：試驗向量（候選解）
```

**選擇機制：**
```
if f(u_i) ≤ f(x_i):
    x_i = u_i  # 接受子代
else:
    x_i = x_i  # 保留父代
```

**特點：**
- 全域搜尋能力強
- 不需要梯度資訊
- 適合非凸、非平滑問題
- 對初始值不敏感

**參數：**
- `maxiter`：最大迭代次數
- `popsize`：族群大小
- `tol`：收斂精度

---

### 3. CMA-ES 演化策略 (CMAESOptimizer)

**數學基礎：**
- **演算法**：Covariance Matrix Adaptation Evolution Strategy
- **類型**：演化策略的進階版本
- **核心思想**：自適應地調整協方差矩陣，引導搜尋方向

**數學表達：**

1. **多元常態分佈採樣：**
```
x_i ~ N(m, σ² * C)

其中：
- m：均值向量（當前最佳解）
- σ：步長（搜尋尺度）
- C：協方差矩陣
```

2. **協方差矩陣更新：**
```
C ← (1-c_μ)C + c_μ * Σ_{i=1}^{μ} w_i * (x_i - m)(x_i - m)^T

其中：
- c_μ：學習率
- w_i：重權重（根據適應度排名）
```

3. **步長更新：**
```
σ ← σ * exp(Δσ)

其中 Δσ 根據路徑長度自適應調整
```

**特點：**
- 高效處理高維度問題
- 自適應搜尋空間形狀
- 在非凸優化中表現優異
- 適合複雜的工程優化問題

**參數：**
- `sigma`：初始搜尋尺度
- `maxiter`：最大迭代次數

---

### 4. 隨機搜尋法 (RandomSearchOptimizer)

**數學基礎：**
- **演算法**：Random Search / Monte Carlo Method
- **類型**：隨機優化方法
- **核心思想**：在搜尋空間中隨機採樣，評估目標函數值

**數學表達：**
```
for i = 1 to n_iter:
    x_i ~ U(l, u)  # 均勻分佈隨機採樣
    evaluate f(x_i)
    keep best solution
```

**特點：**
- 實現簡單
- 不需要梯度
- 可作為基準方法
- 在高維度空間效率較低

**參數：**
- `n_iter`：隨機採樣次數

---

## 優化器架構

```
BaseOptimizer (基類)
├── ScipyOptimizer      # L-BFGS-B
├── DEOptimizer         # 差分進化
├── CMAESOptimizer      # CMA-ES
└── RandomSearchOptimizer  # 隨機搜尋
```

所有優化器都繼承自 `BaseOptimizer`，提供：
- `optimize()`：執行優化的主方法
- `run_post_process()`：後處理（結果顯示、繪圖）

---

## 停止條件比較

| 優化器 | 停止條件 | 適用場景 |
|--------|----------|----------|
| ScipyOptimizer | 最大迭代次數 | 平滑、有梯度的問題 |
| DEOptimizer | 最大迭代次數 + 收斂精度 | 非凸、全域搜尋 |
| CMAESOptimizer | 最大迭代次數 | 高維度複雜問題 |
| RandomSearchOptimizer | 固定採樣次數 | 基準測試、簡單問題 |

---

## 選擇建議

1. **平滑問題**：優先使用 L-BFGS-B
2. **非凸/多峰問題**：使用 DE 或 CMA-ES
3. **高維度問題**：推薦 CMA-ES
4. **快速原型**：使用 Random Search 作為基準


## 最佳化算法策略

黑箱函數 >> CMA-ES / DE / basinhopping

可微函數 >> BFGS / L-BFGS-B / Newton-CG

有約束問題 >> SLSQP / trust-constr

線性規劃 >> linprog

資料擬合 >> least_squares / curve_fit