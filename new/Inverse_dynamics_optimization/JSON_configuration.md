# JSON 設定檔案說明

本文件說明如何使用 JSON 設定檔案 (如 `steer_set.json`) 來控制 Inverse Kinematics 工具的完整模擬流程。透過 `json_load_test.py` 加載配置，可以快速設定複雜的模擬場景而不需修改 Python 程式碼。

---

## 1. JSON 結構概覽

```json
{
  "simulation": { ... },
  "geometry_override": { ... },
  "inputs": [ ... ],
  "forces": [ ... ],
  "torques": [ ... ],
  "targets": { ... },
  "output": { ... }
}
```

每個頂級區塊代表模擬的不同方面。以下逐一說明。

---

## 2. simulation - 模擬基本參數

### 2.1 必要欄位

```json
{
  "simulation": {
    "SIM_TIME": 10,
    "DT": 0.05,
    "enable_visual": true
  }
}
```

| 欄位 | 類型 | 必須 | 說明 | 預設值 |
|-----|------|------|------|--------|
| `SIM_TIME` | float | ✓ | 總模擬時間（秒） | - |
| `DT` | float | ✓ | 時間步長（秒），通常 0.01~0.1 | - |
| `enable_visual` | boolean | ✗ | 是否開啟 3D 視覺化 (VPython) | `false` |

### 2.2 計算邏輯

- 模擬步數 = $\lfloor \text{SIM\_TIME} / \text{DT} \rfloor$
- 時間間隔越小，精度越高但計算時間越長。

### 2.3 範例

```json
{
  "simulation": {
    "SIM_TIME": 20,
    "DT": 0.02,
    "enable_visual": false
  }
}
```

---

## 3. geometry_override - 節點位置覆寫

此區塊用於修改初始節點座標，例如優化設計變數或測試不同幾何配置。

### 3.1 結構

```json
{
  "geometry_override": {
    "enable": false,
    "nodes": {
      "tie_rod": [25, -15, 12.5],
      "rack": [0, 0, 10]
    }
  }
}
```

| 欄位 | 類型 | 必須 | 說明 | 預設值 |
|-----|------|------|------|--------|
| `enable` | boolean | ✗ | 是否啟用節點覆寫 | `false` |
| `nodes` | object | ✗ | 節點名稱 → 新座標 (x, y, z) | `{}` |

### 3.2 使用方式

- 若 `enable: false`，忽略 `nodes` 設定。
- 若 `enable: true`，使用 CSV 預設座標並覆寫指定節點。
- 座標格式：`[x, y, z]` 三維向量。

### 3.3 注意事項

- **務必確保節點名稱正確**，否則會拋出錯誤。
- 覆寫後會自動同步更新連接到該節點的桿件長度。

### 3.4 範例

完全禁用（使用 CSV 預設座標）：
```json
{
  "geometry_override": {
    "enable": false
  }
}
```

啟用並覆寫指定節點：
```json
{
  "geometry_override": {
    "enable": true,
    "nodes": {
      "rack": [5.0, 0.0, 100.0],
      "tie_rod": [30.0, -20.0, 15.0]
    }
  }
}
```

---

## 4. inputs - 輸入運動配置

`inputs` 是一個陣列，每個元素定義一個輸入約束 (PositionConstraint)。可定義多個獨立的輸入來驅動不同節點。

### 4.1 單個輸入結構

```json
{
  "name": "steering",
  "node": "rack",
  "constraint": "PositionConstraint",
  "axis_nodes": ["rack_r", "rack_l"],
  "motion": {
    "motion_type": "s",
    "input_type": "l",
    "x_max": 40,
    "x_min": -40,
    "times": 1,
    "normal_vector": null
  },
  "enable": true
}
```

### 4.2 欄位說明

| 欄位 | 類型 | 必須 | 說明 |
|-----|------|------|------|
| `name` | string | ✓ | 輸入名稱（參考用途） |
| `node` | string | ✓ | 被驅動的節點名稱 |
| `constraint` | string | - | 目前只支援 `"PositionConstraint"`（預留擴充） |
| `axis_nodes` | array[2] | ✓ | 定義運動軸的兩個點名稱 `["point_a", "point_b"]` |
| `enable` | boolean | ✗ | 是否啟用此輸入，`false` 時跳過 | `true` |
| `motion` | object | ✓ | 運動配置物件（見 4.3） |

### 4.3 motion 物件

#### motion_type (運動類型)

- `"s"` **Slider**（滑塊）：沿軸方向直線運動
- `"r"` **Rotate**（旋轉）：繞軸旋轉
- `"f"` **Face Shift**（面推移）：沿垂直面推移

#### input_type (輸入函式)

- `"l"` **Linear**（線性）：等速往返運動
- `"h"` **Harmonic**（簡諧）：正弦波運動

#### motion 欄位

| 欄位 | 類型 | 必須 | 說明 | 預設值 |
|-----|------|------|------|--------|
| `motion_type` | string | ✓ | `"s"` \| `"r"` \| `"f"` | - |
| `input_type` | string | ✓ | `"l"` \| `"h"` | - |
| `x_max` | float | ✓ | 最大位移值 | - |
| `x_min` | float | ✓ | 最小位移值 | - |
| `times` | float | ✗ | 完整循環次數（在 SIM_TIME 內） | `1` |
| `normal_vector` | array[3] | ✗ | 僅用於 `motion_type="f"`，面的法向量 | `null` |

### 4.4 實際行為

**例 1：轉向輸入（滑塊 + 線性往返）**

```json
{
  "name": "steering",
  "node": "rack",
  "axis_nodes": ["rack_r", "rack_l"],
  "motion": {
    "motion_type": "s",
    "input_type": "l",
    "x_max": 40,
    "x_min": -40,
    "times": 1
  },
  "enable": true
}
```

- 在 `SIM_TIME` 內，節點沿 `["rack_r", "rack_l"]` 軸方向，往返運動一次。
- 從 +40 mm 走到 -40 mm 再回到初始位置。

**例 2：懸吊輸入（面推移 + 簡諧）**

```json
{
  "name": "suspension",
  "node": "lf_arm",
  "axis_nodes": ["lf_arm1fix", "lf_arm2fix"],
  "motion": {
    "motion_type": "f",
    "input_type": "h",
    "x_max": 25,
    "x_min": -25,
    "times": 2,
    "normal_vector": [0, 0, 1]
  },
  "enable": true
}
```

- 節點沿垂直面（法向量 [0,0,1]）推移。
- 簡諧運動，完成 2 個週期。

### 4.5 多輸入配置

```json
{
  "inputs": [
    {
      "name": "steering",
      "node": "rack",
      "axis_nodes": ["rack_r", "rack_l"],
      "motion": { ... },
      "enable": true
    },
    {
      "name": "suspension",
      "node": "lf_arm",
      "axis_nodes": ["lf_arm1fix", "lf_arm2fix"],
      "motion": { ... },
      "enable": false
    }
  ]
}
```

- 陣列中可定義多個輸入。
- 若設 `"enable": false`，該輸入會被跳過。

---

## 5. forces - 力量配置

定義施加在系統上的外力，通常用於虛功分析。

### 5.1 單個力的結構

```json
{
  "name": "rack_force",
  "node": "rack",
  "axis_nodes": ["rack_r", "rack_l"],
  "force": 950
}
```

### 5.2 欄位說明

| 欄位 | 類型 | 必須 | 說明 |
|-----|------|------|------|
| `name` | string | ✓ | 力的名稱（輸出時使用） |
| `node` | string | ✓ | 施力節點名稱 |
| `axis_nodes` | array[2] | ✓ | 力的方向軸 `["point_a", "point_b"]` |
| `force` | float | ✓ | 力的大小（牛頓或自訂單位） |

### 5.3 計算原理

- 力沿 `axis_nodes` 方向作用在指定 `node`。
- 輸入功率 = $F \times (\vec{v}_{\text{node}} \cdot \hat{a})$
- 虛功引擎根據功率守恆分配系統各點的受力。

### 5.4 多力配置

```json
{
  "forces": [
    {
      "name": "rack_force",
      "node": "rack",
      "axis_nodes": ["rack_r", "rack_l"],
      "force": 950
    },
    {
      "name": "lf_arm_force",
      "node": "lf_arm",
      "axis_nodes": ["lf_arm1fix", "lf_arm2fix"],
      "force": 500
    }
  ]
}
```

### 5.5 可選性

- `forces` 欄位可省略，預設為空陣列。
- 若不需要力量分析，無需設定。

---

## 6. torques - 力矩配置

用於計算並記錄指定軸上的力矩。

### 6.1 單個力矩的結構

```json
{
  "name": "wheel_torque",
  "node": "tie_rod",
  "axis_nodes": ["lf_arm", "uf_arm"]
}
```

### 6.2 欄位說明

| 欄位 | 類型 | 必須 | 說明 |
|-----|------|------|------|
| `name` | string | ✓ | 力矩名稱（輸出時的鍵名） |
| `node` | string | ✓ | 計算受力的節點名稱 |
| `axis_nodes` | array[2] | ✓ | 力矩軸方向 `["point_a", "point_b"]` |

### 6.3 計算方式

1. 從虛功引擎取得節點 `node` 的受力 $\vec{F}$。
2. 計算節點到軸的最短距離向量 $\vec{r}$。
3. 力矩 = $\vec{\tau} \cdot \hat{a} = (\vec{r} \times \vec{F}) \cdot \hat{a}$

### 6.4 多力矩配置

```json
{
  "torques": [
    {
      "name": "wheel_torque",
      "node": "tie_rod",
      "axis_nodes": ["lf_arm", "uf_arm"]
    },
    {
      "name": "rack_torque",
      "node": "rack",
      "axis_nodes": ["rack_r", "rack_l"]
    }
  ]
}
```

### 6.5 可選性

- `torques` 欄位可省略，預設為空陣列。

---

## 7. targets - 資料紀錄目標

決定哪些節點與連桿的資訊被紀錄到輸出 CSV。

### 7.1 結構

```json
{
  "targets": {
    "node_targets": ["rack", "tie_rod", "uf_arm"],
    "rod_targets": ["w", "uAtie"]
  }
}
```

### 7.2 欄位說明

| 欄位 | 類型 | 必須 | 說明 |
|-----|------|------|------|
| `node_targets` | array[string] | ✓ | 要紀錄的節點名稱列表 |
| `rod_targets` | array[string] | ✓ | 要紀錄的連桿名稱列表 |

### 7.3 紀錄內容

#### 節點紀錄內容

對每個 `node_targets` 中的節點：
- 座標：`{name}_x`, `{name}_y`, `{name}_z`
- 速度：`{name}_v`, `{name}_vx`, `{name}_vy`, `{name}_vz`
- 加速度：`{name}_a`, `{name}_ax`, `{name}_ay`, `{name}_az`
- 虛功力：`{name}_f`, `{name}_fx`, `{name}_fy`, `{name}_fz`

#### 連桿紀錄內容

對每個 `rod_targets` 中的連桿：
- 角度：`{name}_thetaX`, `{name}_thetaY`, `{name}_thetaZ`
- 角速度：`{name}_omega`, `{name}_omegaX`, `{name}_omegaY`, `{name}_omegaZ`
- 角加速度：`{name}_alpha`, `{name}_alphaX`, `{name}_alphaY`, `{name}_alphaZ`

### 7.4 範例

```json
{
  "targets": {
    "node_targets": ["rack", "tie_rod"],
    "rod_targets": ["w", "uAtie", "racktie"]
  }
}
```

輸出 CSV 會包含：
- `rack_x, rack_y, rack_z, rack_v, rack_vx, ...`
- `tie_rod_x, tie_rod_y, tie_rod_z, tie_rod_v, ...`
- `w_thetaX, w_thetaY, w_thetaZ, w_omega, ...`
- `uAtie_thetaX, uAtie_thetaY, ...`
- `racktie_thetaX, racktie_thetaY, ...`

---

## 8. output - 輸出與分析設定

控制模擬結果的匯出、繪圖與列印行為。

### 8.1 結構

```json
{
  "output": {
    "plot": [
      ["uAtie_thetaz", "rack_y"],
      ["uAtie_omegaz", "uAtie_thetaz"]
    ],
    "save_plot": true,
    "print": ["tau"],
    "save_csv": true
  }
}
```

### 8.2 欄位說明

| 欄位 | 類型 | 必須 | 說明 | 預設值 |
|-----|------|------|------|--------|
| `plot` | array[array] | ✗ | 要繪製的圖表，每項是鍵名陣列或單一鍵名 | `[]` |
| `save_plot` | boolean | ✗ | 是否將圖表保存為 PNG | `false` |
| `print` | array[string] | ✗ | 要列印最終值的鍵名 | `[]` |
| `save_csv` | boolean | ✗ | 是否保存 CSV 檔案 | `false` |

### 8.3 plot 設定詳解

```json
{
  "plot": [
    "uAtie_thetaz",
    "rack_y"
  ]
}
```

會繪製兩張圖，分別為 `uAtie_thetaz` 和 `rack_y`。

### 8.4 print 設定

```json
{
  "print": ["tau", "rack_f"]
}
```

會在執行結束時列印這些鍵名在最後時刻的數值。

### 8.5 save_csv 與 save_plot

- `save_csv: true`：輸出 CSV 檔到 `Inverse_kinematics/output/output_YYYYMMDD_HHMMSS.csv`
- `save_plot: true`：圖表儲存為 PNG 到 `Inverse_kinematics/figures/`

### 8.6 可選性

- 整個 `output` 區塊可省略，所有設定預設為 `false` 或空陣列。

---

## 9. 完整範例解析

### steer_set.json 逐行說明

```json
{
  "simulation": {
    "SIM_TIME": 10,          // 總模擬時間 10 秒
    "DT": 0.05,              // 時間步長 50 ms，共 200 步
    "enable_visual": true    // 開啟視覺化
  },

  "geometry_override": {
    "enable": false,         // 禁用幾何覆寫，使用 CSV 預設
    "nodes": { "tie_rod": [25, -15, 12.5] }  // （未使用）
  },

  "inputs": [
    {
      "name": "steering",
      "node": "rack",
      "constraint": "PositionConstraint",
      "axis_nodes": ["rack_r", "rack_l"],  // 轉向軸方向
      "motion": {
        "motion_type": "s",   // 滑塊運動
        "input_type": "l",    // 線性往返
        "x_max": 40,          // 向右 40 mm
        "x_min": -40,         // 向左 40 mm
        "times": 1            // 10 秒內完成 1 個循環
      }
    },

    {
      "name": "suspension",
      "node": "lf_arm",
      "constraint": "PositionConstraint",
      "axis_nodes": ["lf_arm1fix", "lf_arm2fix"],
      "motion": {
        "motion_type": "f",   // 面推移
        "input_type": "l",
        "x_max": 25,
        "x_min": -25,
        "times": 1,
        "normal_vector": [0, 0, 1]  // 水平面推移
      },
      "enable": false         // 禁用此輸入，只運行轉向
    }
  ],

  "forces": [
    {
      "name": "rack_force",
      "node": "rack",
      "axis_nodes": ["rack_r", "rack_l"],
      "force": 950            // 950 N 的轉向力
    }
  ],

  "torques": [
    {
      "name": "wheel_torque",
      "node": "tie_rod",
      "axis_nodes": ["lf_arm", "uf_arm"]  // 輪子力矩軸
    }
  ],

  "targets": {
    "node_targets": ["rack", "tie_rod", "uf_arm"],
    "rod_targets": ["w", "uAtie"]
  },

  "output": {
    "plot": 
      ["uAtie_thetaz", "rack_y"],    // 圖:轉向角,轉向位移
    "save_plot": true,      // 保存圖表
    "print": ["tau"],       // 列印力矩最終值
    "save_csv": true        // 保存 CSV 資料
  }
}
```

---

## 10. 常見配置模板

### 模板 1：純轉向分析（無視覺化）

```json
{
  "simulation": {
    "SIM_TIME": 5,
    "DT": 0.02,
    "enable_visual": false
  },
  "inputs": [
    {
      "name": "steering",
      "node": "rack",
      "axis_nodes": ["rack_r", "rack_l"],
      "motion": {
        "motion_type": "s",
        "input_type": "l",
        "x_max": 50,
        "x_min": -50,
        "times": 2
      }
    }
  ],
  "forces": [
    {
      "name": "rack_force",
      "node": "rack",
      "axis_nodes": ["rack_r", "rack_l"],
      "force": 800
    }
  ],
  "targets": {
    "node_targets": ["rack", "tie_rod"],
    "rod_targets": ["uAtie"]
  },
  "output": {
    "save_csv": true
  }
}
```

### 模板 2：轉向 + 懸吊耦合分析

```json
{
  "simulation": {
    "SIM_TIME": 10,
    "DT": 0.05,
    "enable_visual": true
  },
  "inputs": [
    {
      "name": "steering",
      "node": "rack",
      "axis_nodes": ["rack_r", "rack_l"],
      "motion": {
        "motion_type": "s",
        "input_type": "h",
        "x_max": 30,
        "x_min": -30,
        "times": 1
      },
      "enable": true
    },
    {
      "name": "suspension",
      "node": "lf_arm",
      "axis_nodes": ["lf_arm1fix", "lf_arm2fix"],
      "motion": {
        "motion_type": "f",
        "input_type": "h",
        "x_max": 20,
        "x_min": -20,
        "times": 2,
        "normal_vector": [0, 0, 1]
      },
      "enable": true
    }
  ],
  "forces": [
    {
      "name": "steering_force",
      "node": "rack",
      "axis_nodes": ["rack_r", "rack_l"],
      "force": 900
    }
  ],
  "torques": [
    {
      "name": "wheel_torque",
      "node": "tie_rod",
      "axis_nodes": ["lf_arm", "uf_arm"]
    }
  ],
  "targets": {
    "node_targets": ["rack", "tie_rod", "lf_arm", "uf_arm"],
    "rod_targets": ["w", "uAtie", "racktie"]
  },
  "output": {
    "plot": ["rack_x", "lf_arm_z","tie_rod_f"],
    "save_plot": true,
    "save_csv": true
  }
}
```

### 模板 3：設計優化場景（多組幾何）

```json
{
  "simulation": {
    "SIM_TIME": 8,
    "DT": 0.04,
    "enable_visual": false
  },
  "geometry_override": {
    "enable": true,
    "nodes": {
      "tie_rod": [30, -15, 14],
      "rack": [0, 0, 12]
    }
  },
  "inputs": [
    {
      "name": "steering",
      "node": "rack",
      "axis_nodes": ["rack_r", "rack_l"],
      "motion": {
        "motion_type": "s",
        "input_type": "l",
        "x_max": 40,
        "x_min": -40,
        "times": 1
      }
    }
  ],
  "forces": [],
  "torques": [],
  "targets": {
    "node_targets": ["tie_rod"],
    "rod_targets": ["uAtie"]
  },
  "output": {
    "save_csv": true
  }
}
```

---

## 11. 錯誤排查

### 常見錯誤

| 錯誤訊息 | 原因 | 解決方案 |
|---------|------|--------|
| `缺少必要欄位: simulation` | JSON 結構不完整 | 確保頂層有 `simulation` |
| `缺少必要欄位: simulation -> SIM_TIME` | simulation 缺少時間參數 | 補上 `SIM_TIME` 和 `DT` |
| `找不到 node: xxx` | 節點名稱拼寫錯誤 | 確認 CSV 中的節點名稱 |
| `找不到 config: xxx.json` | JSON 檔案路徑不對 | 確保 JSON 與 `json_load_test.py` 同目錄 |

### 驗證 JSON 格式

使用線上工具或 Python：

```python
import json

with open("steer_set.json", "r", encoding="utf-8") as f:
    data = json.load(f)
    print("JSON 格式正確")
```

---

## 12. 使用流程

1. **準備 JSON 檔案**：根據需求建立設定檔案（參考模板或現有範例）。
2. **驗證節點名稱**：確認 `inputs`, `targets`, `forces`, `torques` 中的節點/連桿名稱與 CSV 一致。
3. **執行程式**：
   ```bash
   python json_load_test.py
   ```
4. **檢查輸出**：
   - 模擬日誌在終端顯示。
   - CSV 輸出到 `output/output_YYYYMMDD_HHMMSS.csv`。
   - 圖表保存到 `figures/` 目錄。

---

## 13. 進階技巧

### 動態力量輸入（未來擴充）

目前 `force` 必須是常數，但 `json_load_test.py` 設計允許後續支援可呼叫函式：

```python
# 未來可能的支援
"in_force": 950,  # 常數
// 或
"in_force": "lambda t: 500 + 100 * sin(2*pi*t)"  # 時間函式
```

### 條件式啟用

利用 `"enable"` 欄位快速開關輸入、測試場景變數：

```json
"enable": true   // 或 false
```

### 監控資訊

在 `targets` 中列出所有關心的節點與連桿，但要平衡檔案大小與資訊量。

