# 多体动力学模拟系统 - 項目導覽指南

## 📁 項目結構概覽

```
multibody/
├── 根目錄 Python 文件
├── data/                          # 數據文件夾
├── Model_Analysis/                # 模型分析模塊
├── Optimization/                  # 優化模塊
├── outcmaes/                      # CMA-ES 優化結果輸出
└── 配置文件及其他
```

---

## 🐍 根目錄 Python 文件

### 主程序入口
| 文件名 | 功能描述 |
|-------|--------|
| **`ui_app.py`** | 用戶界面應用程序主文件 |
| **`run_ui.bat`** | UI 啟動腳本（Windows 批次文件） |

### 仿真程序
| 文件名 | 功能描述 |
|-------|--------|
| **`Dynamic_sim.py`** | 動態仿真程序 |
| **`Static_sim.py`** | 靜態仿真程序 |
| **`simulation_DEMO.py`** | 仿真演示程序 |
| **`Steering.py`** | 轉向系統控制模塊 |

### 優化工作室
| 文件名 | 功能描述 |
|-------|--------|
| **`Optimization_Studio.py`** | 優化工作室主程序 |
| **`Optimization_Studio_1.py`** | 優化工作室版本 1 |
| **`Optimization_Studio_3.py`** | 優化工作室版本 3 |

---

## 📂 主要文件夾結構詳解

### 1️⃣ `data/` 數據文件夾

存儲配置文件和測試數據集

```
data/
├── design_vars.csv               # 設計變量
├── nodes.csv                     # 節點坐標數據
├── rods.csv                      # 連桿數據
├── wheel.csv                     # 輪胎數據
├── test_set.json                 # 測試集配置
├── Optimization_table.py         # 優化表格处理模塊
├── back up/                      # 備份數據
│   ├── design_vars.csv
│   ├── nodes.csv
│   ├── rods.csv
│   └── steer_set.json
├── steering/                     # 轉向系統數據集
│   ├── design_vars.csv
│   ├── nodes.csv
│   ├── rods.csv
│   ├── steer_set.json
│   └── wheel.csv
├── 前轉向系統模擬/               # 前轉向系統模擬數據集
│   ├── design_vars.csv
│   ├── nodes.csv
│   ├── rods.csv
│   ├── test_set.json
│   └── wheel.csv
└── 單輪懸吊與轉向模擬/           # 單輪懸吊模擬數據集
    ├── design_vars.csv
    ├── nodes.csv
    ├── rods.csv
    ├── test_set.json
    └── wheel.csv
```

**Python 文件：**
- 📄 `Optimization_table.py` - 優化表格數據處理

---

### 2️⃣ `Model_Analysis/` 模型分析模塊

核心仿真和分析模塊

```
Model_Analysis/
├── __init__.py                   # 包初始化文件
├── Geometry.py                   # 幾何計算模塊
├── motion.py                     # 運動計算模塊
├── shape.py                      # 形狀和結構模塊
├── output.py                     # 輸出處理模塊
├── simulator.py                  # 動態仿真器
├── simulator_static.py           # 靜態仿真器
├── Inverse_kinematics.md         # 逆向運動學文檔
├── introduction/                 # 介紹和文檔文件夾
│   ├── DEMO_settings.md          # DEMO 設置說明
│   ├── Geometry.md               # 幾何說明
│   ├── Kinematics_engine.md      # 運動學引擎說明
│   ├── Material_engine.md        # 材料引擎說明
│   ├── motion.md                 # 運動說明
│   ├── Multibody_engine.md       # 多體引擎說明
│   ├── output.md                 # 輸出說明
│   ├── shape.md                  # 形狀說明
│   ├── simulator.md              # 仿真器說明
│   ├── Static_engine.md          # 靜態引擎說明
│   └── Virtual_work_engine.md    # 虛功原理引擎說明
├── Inverse_dynamics/             # 逆向動力學模塊
│   ├── __init__.py
│   ├── Kinematics_engine.py      # 運動學引擎實現
│   ├── Material_engine.py        # 材料引擎實現
│   ├── Multibody_engine.py       # 多體引擎實現
│   ├── Static_engine.py          # 靜態引擎實現
│   ├── Virtual_work_engine.py    # 虛功原理引擎實現
│   └── __pycache__/
├── figures/                      # 圖表輸出文件夾
├── output/                       # 仿真輸出結果
│   ├── outpur_Analyzer.py        # 輸出分析工具
│   ├── output_*.csv              # 各種時間戳的輸出文件
│   ├── rod_forces_*.csv          # 連桿力量輸出文件
│   └── ... (多個輸出文件)
├── UI/                           # 用戶界面模塊
│   ├── __init__.py
│   ├── ui_helper.py              # UI 助手函數
│   ├── ui_loader.py              # UI 加載器
│   ├── build.py                  # UI 構建模塊
│   └── __pycache__/
└── __pycache__/
```

**Python 文件列表：**
- 📄 `__init__.py` - 包初始化
- 📄 `Geometry.py` - 幾何計算
- 📄 `motion.py` - 運動學計算
- 📄 `shape.py` - 形狀結構
- 📄 `output.py` - 輸出處理
- 📄 `simulator.py` - 動態仿真器
- 📄 `simulator_static.py` - 靜態仿真器

**Inverse_dynamics/ 子模塊：**
- 📄 `__init__.py`
- 📄 `Kinematics_engine.py`
- 📄 `Material_engine.py`
- 📄 `Multibody_engine.py`
- 📄 `Static_engine.py`
- 📄 `Virtual_work_engine.py`

**output/ 子模塊：**
- 📄 `outpur_Analyzer.py` - 輸出分析工具

**UI/ 子模塊：**
- 📄 `__init__.py`
- 📄 `ui_helper.py`
- 📄 `ui_loader.py`
- 📄 `build.py`

---

### 3️⃣ `Optimization/` 優化模塊

優化算法和問題定義模塊

```
Optimization/
├── __init__.py                   # 包初始化文件
├── Optimizer.py                  # 優化器主程序
├── Problem_Definition.py         # 優化問題定義
├── Boundary.py                   # 邊界約束定義
├── Optimizer.md                  # 優化器文檔
├── figures/                      # 優化結果圖表
└── __pycache__/
```

**Python 文件列表：**
- 📄 `__init__.py` - 包初始化
- 📄 `Optimizer.py` - 優化算法實現
- 📄 `Problem_Definition.py` - 問題定義
- 📄 `Boundary.py` - 邊界和約束

---


## 📊 Python 文件完整清單

### 按類別分類

#### 🎯 主程序和入口
1. `ui_app.py` - 用戶界面主應用
2. `Dynamic_sim.py` - 動態仿真
3. `Static_sim.py` - 靜態仿真
4. `simulation_DEMO.py` - 演示仿真
5. `Steering.py` - 轉向系統

#### 🔧 優化工作室
6. `Optimization_Studio.py` - 優化工作室主程序
7. `Optimization_Studio_1.py` - 優化工作室 v1
8. `Optimization_Studio_3.py` - 優化工作室 v3

#### 📈 Model_Analysis 模塊
9. `Model_Analysis/__init__.py`
10. `Model_Analysis/Geometry.py` - 幾何計算
11. `Model_Analysis/motion.py` - 運動學
12. `Model_Analysis/shape.py` - 形狀結構
13. `Model_Analysis/output.py` - 輸出管理
14. `Model_Analysis/simulator.py` - 動態仿真器
15. `Model_Analysis/simulator_static.py` - 靜態仿真器

#### 🔬 Model_Analysis/Inverse_dynamics 子模塊
16. `Model_Analysis/Inverse_dynamics/__init__.py`
17. `Model_Analysis/Inverse_dynamics/Kinematics_engine.py`
18. `Model_Analysis/Inverse_dynamics/Material_engine.py`
19. `Model_Analysis/Inverse_dynamics/Multibody_engine.py`
20. `Model_Analysis/Inverse_dynamics/Static_engine.py`
21. `Model_Analysis/Inverse_dynamics/Virtual_work_engine.py`

#### 📊 Model_Analysis/output 子模塊
22. `Model_Analysis/output/outpur_Analyzer.py` - 輸出分析

#### 🖥️ Model_Analysis/UI 子模塊
23. `Model_Analysis/UI/__init__.py`
24. `Model_Analysis/UI/ui_helper.py`
25. `Model_Analysis/UI/ui_loader.py`
26. `Model_Analysis/UI/build.py`

#### ⚙️ Optimization 模塊
27. `Optimization/__init__.py`
28. `Optimization/Optimizer.py` - 優化器
29. `Optimization/Problem_Definition.py` - 問題定義
30. `Optimization/Boundary.py` - 邊界約束

#### 📁 data 模塊
31. `data/Optimization_table.py` - 優化表格處理

---
