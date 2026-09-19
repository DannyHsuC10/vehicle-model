# MPCC 系統架構與模型替換說明

本文整理本專案的整體系統架構、車輛模型、車輛控制器，以及賽道與目標路徑資料的使用方式。專案包含 Matlab 與 C++ 兩套 MPCC（Model Predictive Contouring Control）自動賽車控制器實作。Matlab 版本較接近原始論文並包含障礙物避讓流程；C++ 版本加入輸入變化率狀態、輪胎限制、側滑角成本等較偏完整系統的設計。

## 1. 整體系統架構介紹

### 核心概念

MPCC 是一個以最佳化為核心的路徑跟隨控制器。它不只追蹤某個時間序列的參考點，而是把車輛在路徑上的進度也放進最佳化問題中，讓控制器一邊保持車輛靠近參考路徑，一邊盡量增加沿路徑前進的速度。

核心狀態包含：

- 車輛平面位置：`X`, `Y`
- 車身方向角：`phi`
- 車體座標速度：`vx`, `vy`
- yaw rate：Matlab 稱為 `omega`，C++ 稱為 `r`
- 路徑進度：Matlab 稱為 `theta`，C++ 稱為 `s`
- C++ 版另外把實際控制量 `D`, `delta`, `vs` 放入 state，並把控制輸入定義為這些量的變化率

MPCC 的代價函數主要由下列項目組成：

- Contouring error：車輛相對於目標路徑的橫向誤差
- Lag error：車輛相對於目標路徑進度方向的縱向誤差
- Progress reward：鼓勵 `theta` 或 `s` 往前推進
- Input / input-rate regularization：限制油門、轉角、虛擬路徑速度變化過大
- C++ 額外加入 heading cost、side-slip beta cost、soft constraint cost

約束主要包含：

- 動態模型等式約束
- 賽道邊界約束
- state、input、input-rate bounds
- C++ 額外加入後輪摩擦橢圓約束與前輪 slip angle 約束

### Matlab 架構

Matlab 入口在 `Matlab/simulation.m`。主要流程如下：

1. 選擇車型與控制參數：`getMPC_vars.m`
2. 載入車輛物理參數：`getModelParams.m`
3. 載入賽道資料：`Tracks/track2.mat` 或 `Tracks/trackMobil.mat`
4. 依車寬與安全係數縮小可行賽道：`borderAdjustment.m`
5. 將中心線與邊界轉為 spline：`splines/splinify.m`
6. 初始化車輛狀態、預測 horizon、控制輸入猜測
7. 每個 simulation step 呼叫 `optimizer_mpcc.m`
8. `optimizer_mpcc.m` 先用 `getNewBorders.m` 做障礙物與 corridor 邊界處理，再呼叫 `getMPCmatrices.m` 組 QP
9. 依 `MPC_vars.interface` 選擇 solver：`hpipmInterface.m`、`QuadProgInterface.m`、`YalmipInterface.m`、`CVXInterface.m`
10. 用 `SimTimeStep.m` 把第一個最佳控制輸入套進非線性車輛模型，推進真實模擬狀態

Matlab 版的資料流可以概括為：

```text
track .mat
  -> borderAdjustment
  -> splinify
  -> TrackMPC
  -> optimizer_mpcc
  -> getNewBorders
  -> getMPCmatrices
  -> solver
  -> x/u prediction
  -> SimTimeStep
  -> next state
```

### C++ 架構

C++ 入口在 `C++/main.cpp`。主要流程如下：

1. 讀取 `Params/config.json`
2. 透過 `PathToJson` 指向 `model.json`、`cost.json`、`bounds.json`、`track.json`、`normalization.json`
3. 建立 `Integrator`、`Plotting`、`Track`、`MPC`
4. 從 `track.json` 讀中心線與內外邊界
5. `mpc.setTrack(track_xy.X, track_xy.Y)` 將中心線送入 `ArcLengthSpline`
6. 每個 simulation step 呼叫 `mpc.runMPC(x0)`
7. `MPC::runMPC()` 投影目前車輛位置到路徑進度 `s`，更新 initial guess，反覆做 SQP
8. `MPC::setStage()` 對每個 horizon stage 建立 cost、linearized dynamics、constraints、bounds
9. `HpipmInterface` 解 QP
10. `Integrator::simTimeStep()` 用 RK4 推進完整非線性車輛模型

C++ 版的資料流可以概括為：

```text
Params/*.json
  -> Track / Param / CostParam / BoundsParam / NormalizationParam
  -> MPC
  -> ArcLengthSpline
  -> Cost + Model + Constraints
  -> HpipmInterface
  -> optimal input-rate
  -> Integrator
  -> next state
```

## 2. 車輛模型介紹

### Matlab 車輛模型

Matlab 版使用 7 維 state 與 3 維 input：

```text
x = [X, Y, phi, vx, vy, omega, theta]
u = [D, delta, vtheta]
```

其中：

- `D` 是 duty cycle 或等效油門命令
- `delta` 是前輪轉角
- `vtheta` 是虛擬路徑進度速度
- `theta` 是車輛在參考路徑上的虛擬進度

車輛本體是非線性 bicycle model，搭配 Magic Formula 型態的前後輪側向力：

```text
alpha_f = -atan2(lf * omega + vy, vx) + delta
alpha_r =  atan2(lr * omega - vy, vx)

F_fy = Df * sin(Cf * atan(Bf * alpha_f))
F_ry = Dr * sin(Cr * atan(Br * alpha_r))
F_rx = Cm1 * D - Cm2 * D * vx - Cr0 - Cr2 * vx^2
```

連續時間模型在 `SimTimeStep.m` 中用 `ode45` 積分；在最佳化問題中，`DiscretizedLinearizedModel.m` 會在目前預測軌跡 `(Xbar_k, Ubar_k)` 周圍線性化，再用矩陣指數離散化成：

```text
x(k+1) = Ad * x(k) + Bd * u(k) + gd
```

`getMPCmatrices.m` 又會把原本狀態增廣成：

```text
s_k = [x_k, u_{k-1}]
v_k = du_k
```

這樣可以在 QP 中直接限制與懲罰 input rate。

### C++ 車輛模型

C++ 版定義在 `C++/types.h` 與 `C++/Model/model.cpp`。state 與 input 為：

```text
State = [X, Y, phi, vx, vy, r, s, D, delta, vs]
Input = [dD, dDelta, dVs]
```

與 Matlab 版最大的差異是 C++ 版已經把 `D`、`delta`、`vs` 納入 state，實際最佳化輸入是它們的變化率。連續模型中：

```text
dot(X)     = vx*cos(phi) - vy*sin(phi)
dot(Y)     = vy*cos(phi) + vx*sin(phi)
dot(phi)   = r
dot(vx)    = longitudinal dynamics
dot(vy)    = lateral dynamics
dot(r)     = yaw dynamics
dot(s)     = vs
dot(D)     = dD
dot(delta) = dDelta
dot(vs)    = dVs
```

`Model::getF()` 計算非線性連續模型；`Model::getModelJacobian()` 計算 Jacobian；`Model::discretizeModel()` 用矩陣指數離散化；`Integrator::RK4()` 與 `Integrator::simTimeStep()` 則負責用完整非線性模型做模擬。

### 如何替換整個車輛模型

替換模型時要同時處理「模擬用非線性模型」與「MPC 用線性化模型」。兩者必須一致，否則控制器會用一套模型預測，模擬器卻用另一套模型前進，結果容易不穩或不可解。

#### Matlab 替換步驟

1. 在 `getModelParams.m` 新增一個 `ModelNo`，填入新車的物理參數、state/input index、車長車寬等。
2. 在 `getMPC_vars.m` 新增對應 `CarModel` 名稱，設定 `N`、`Ts`、normalization、bounds、cost weights。
3. 修改 `SimTimeStep.m` 中的 `fx_bicycle()`，加入或替換完整非線性車輛動態。
4. 修改 `DiscretizedLinearizedModel.m`，為新模型推導並填入 `Ac`、`Bc`、`gc`，最後維持輸出 `Ad`、`Bd`、`gd`。
5. 若 state 或 input 維度改變，要同步修改 `ModelParams.nx`、`ModelParams.nu`、`MPC_vars.bounds`、`MPC_vars.Tx/Tu`、solver interface 中假設的維度。
6. 若車輛尺寸改變，要檢查 `borderAdjustment.m` 的車寬縮邊邏輯。

如果只是換同一 bicycle model 的參數，不需要改方程式，只要更新 `getModelParams.m` 與 `getMPC_vars.m`。

#### C++ 替換步驟

1. 若 state/input 維度不變，只是換參數，直接改 `C++/Params/model.json`。
2. 若模型方程改變但維度不變，修改 `C++/Model/model.cpp`：
   - `getF()`
   - force/slip helper functions
   - `getModelJacobian()`
3. 若維度改變，必須同步修改：
   - `C++/config.h` 的 `NX`、`NU`、index
   - `C++/types.h` 與 `types.cpp` 的 State/Input 轉 vector 函式
   - `C++/Params/bounds.json`
   - `C++/Params/normalization.json`
   - `C++/Cost/cost.cpp`
   - `C++/Constraints/constraints.cpp`
   - solver interface 中對維度的假設
4. 若新車模型需要不同限制，例如 AWD、前輪煞車、下壓力，建議擴充 `Constraints` 與 `model.json`，不要只改 `Model::getF()`。

## 3. 車輛控制與駕駛人模型介紹

### 這份專案中的「駕駛人模型」是什麼

本專案沒有獨立的人類駕駛人模型，例如 preview driver、PID driver、Stanley controller 或 pure pursuit driver。它的駕駛決策由 MPCC 本身完成。因此在這份程式中，「駕駛人模型」可以理解為：

```text
目前車輛狀態 + 目標路徑 + 車輛模型 + 約束 + 成本函數
  -> 最佳化問題
  -> 油門/轉向/路徑速度命令
```

也就是 MPCC 控制器同時扮演：

- 路徑跟隨器
- 車速規劃器
- 轉向控制器
- 油門控制器
- 約束處理器

### 與完整車輛模型如何連接

Matlab 版：

```text
x0
  -> optimizer_mpcc(...)
  -> getMPCmatrices(...)
  -> DiscretizedLinearizedModel(...) 產生 MPC 預測模型
  -> solver 得到 u(:,1)
  -> SimTimeStep(x(:,1), u(:,1), Ts, ModelParams)
  -> 下一個 x0
```

控制器內部使用線性化離散模型產生 prediction；外部 simulation 則使用 `SimTimeStep.m` 的完整非線性模型推進真實狀態。

C++ 版：

```text
x0
  -> MPC::runMPC(x0)
  -> Model::getLinModel(...) 產生 MPC 預測模型
  -> HpipmInterface::solveMPC(...)
  -> mpc_sol.u0 = [dD, dDelta, dVs]
  -> Integrator::simTimeStep(x0, mpc_sol.u0, Ts)
  -> 下一個 x0
```

控制器與車輛模型的核心接口是目前 state 與回傳的第一個最佳 input。這一點很重要，因為要把 MPCC 接到外部完整車輛模擬器時，不一定要使用專案內建的 `SimTimeStep` 或 `Integrator`。

### 駕駛人模型運作邏輯

每個控制週期做以下事情：

1. 取得目前車輛狀態。
2. 將車輛位置投影到參考路徑，得到 `theta` 或 `s`。
3. 用上一輪最佳解平移一格，當作這輪 initial guess。
4. 在 initial guess 周圍線性化車輛模型、路徑誤差與賽道限制。
5. 組成 QP。
6. 解出整個 horizon 的最佳 state/input。
7. 只取第一個控制量套到車輛上。
8. 下一個週期重複上述流程。

這是 receding horizon control。每次雖然規劃一段 horizon，但實際只執行第一步。

### 只單純取出駕駛人模型該怎麼做

若你要把 MPCC 當作「駕駛人模型」接到其他完整車輛模型或外部模擬器，建議抽出控制器部分，保留輸入/輸出介面。

#### Matlab 抽取方式

需要保留：

- `getMPC_vars.m`
- `getModelParams.m`
- `optimizer_mpcc.m`
- `getMPCmatrices.m`
- solver interface，例如 `hpipmInterface.m` 或 `QuadProgInterface.m`
- `DiscretizedLinearizedModel.m`
- `findTheta.m`
- `splines/` 資料夾
- 若不需要障礙物，可簡化或繞過 `getNewBorders.m`

外部主程式每個控制週期提供：

```text
x0 = [X, Y, phi, vx, vy, omega, theta]
uprev = 上一個 [D, delta, vtheta]
TrackMPC = 由目標路徑建立的 spline 資訊
```

控制器回傳：

```text
u(:,1) = [D, delta, vtheta]
```

若外部完整車輛模型不需要 `vtheta`，可以只使用 `D` 與 `delta`，但 `vtheta/theta` 仍需保留在 MPCC 內部，因為它是 contouring/lag error 與 progress reward 的核心。

#### C++ 抽取方式

需要保留：

- `MPC/`
- `Model/`
- `Cost/`
- `Constraints/`
- `Spline/`
- `Params/`
- `Interfaces/`
- `types.*`、`config.h`

外部系統每個控制週期呼叫：

```cpp
mpcc::MPCReturn sol = mpc.runMPC(x0);
```

回傳的 `sol.u0` 是：

```text
[dD, dDelta, dVs]
```

因為 C++ 版的 state 已包含 `D`, `delta`, `vs`，外部完整車輛模型若需要實際油門與轉角，通常要用控制週期積分後的 state 中 `D` 與 `delta`，或在外部依照：

```text
D_next     = D     + Ts * dD
delta_next = delta + Ts * dDelta
vs_next    = vs    + Ts * dVs
```

更新實際控制命令。若外部模擬器接受的是 input rate，則可直接使用 `sol.u0`。

## 4. 賽道資料說明

### Matlab 賽道資料格式

Matlab 版的賽道存在 `.mat` 檔，例如：

- `Matlab/Tracks/track2.mat`
- `Matlab/Tracks/trackMobil.mat`

程式預期變數中包含 `track2`，且至少有：

```text
track2.center = [x_center; y_center]
track2.inner  = [x_inner;  y_inner]
track2.outer  = [x_outer;  y_outer]
```

`simulation.m` 會先呼叫：

```matlab
[track, track2] = borderAdjustment(track2, ModelParams, safteyScaling);
[traj, borders] = splinify(track);
```

其中 `traj` 是中心線 spline，`borders` 是邊界 spline。`getMPCmatrices.m` 的 contouring/lag error 使用 `traj`；賽道限制則使用 `optimizer_mpcc.m` 透過 `getNewBorders.m` 產生的 `new_b` corridor。

### C++ 賽道資料格式

C++ 版賽道存在 `C++/Params/track.json`，主要欄位為：

```json
{
  "X": [],
  "Y": [],
  "X_i": [],
  "Y_i": [],
  "X_o": [],
  "Y_o": []
}
```

`X`, `Y` 是中心線或目標路徑；`X_i/Y_i` 與 `X_o/Y_o` 是內外邊界，主要用於 plotting 與資料保存。實際 C++ 的 track constraint 並不是直接使用 `X_i/Y_i/X_o/Y_o`，而是用中心線 tangent 加上 `model.json` 中的固定半寬：

```json
"R_in": 0.14,
"R_out": 0.14
```

也就是 C++ 版的賽道限制假設路徑左右寬度固定。

### 如果要直接輸入目標路徑，不給賽道邊界

這是可行的，但要分 Matlab 與 C++ 兩種處理方式。

#### C++ 建議做法

C++ 版最直接，因為已經有：

```cpp
void MPC::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y)
```

只要把你的目標路徑點列當成 `X`, `Y` 傳入：

```cpp
mpc.setTrack(path_x, path_y);
```

`ArcLengthSpline::gen2DSpline()` 會把它轉為 arc-length spline。接著 MPCC 的 contouring/lag error 就會追蹤這條路徑。

若你真的不需要賽道邊界限制，有兩個選擇：

1. 保留 track constraint，但把 `C++/Params/model.json` 的 `R_in`、`R_out` 設大，等效於給一條很寬的 corridor。
2. 修改 `C++/Constraints/constraints.cpp`，移除或停用 `getTrackConstraints()` 對 QP 的限制，同時調整 `NPC`、`NS`、soft constraint index 與 solver interface。這比較乾淨，但牽涉面較大。

如果只是「直接跟一條已知目標路徑，不做最佳路徑生成」，建議使用第 1 種方式，因為改動最小。

#### Matlab 建議做法

Matlab 版 `splinify(track)` 預期同時有 `center`、`inner`、`outer`。若你只有目標路徑：

```matlab
path = [x_path; y_path];
```

可以建立一個假的 track struct：

```matlab
track.center = path;

width = 10; % 給足夠大的寬度，等效於不限制賽道
dx = gradient(path(1,:));
dy = gradient(path(2,:));
norm_len = sqrt(dx.^2 + dy.^2);
nx = -dy ./ norm_len;
ny =  dx ./ norm_len;

track.inner = path - 0.5 * width * [nx; ny];
track.outer = path + 0.5 * width * [nx; ny];

[traj, borders] = splinify(track);
TrackMPC = struct('traj', traj, ...
                  'borders', borders, ...
                  'track_center', track.center, ...
                  'tl', traj.ppy.breaks(end));
```

這樣可以保留原本 MPCC 的 contouring/lag cost，同時讓賽道邊界變得很寬，等效於只跟目標路徑。

如果你要完全移除邊界處理，則需要修改：

- `optimizer_mpcc.m`：不要呼叫 `getNewBorders.m`
- `getMPCmatrices.m`：不要呼叫或使用 `getInequalityConstraints(...)`
- 各 solver interface：移除對 stage inequality constraint 的假設

這種做法改動較大，除非你確定外部系統另有碰撞或道路限制，否則建議先用「假邊界很寬」的方式。

### 直接給定路徑且不進行路徑最佳化的重點

這份專案本身沒有全域路徑最佳化器。它不是從賽道邊界算 racing line，而是追蹤已經給定的中心線或目標線。Matlab 中的 `getNewBorders.m` 是為了障礙物避讓而調整 corridor，不是產生最佳 racing line。C++ 版更明確：`track.json` 的 `X`, `Y` 就是要追蹤的參考線。

因此若你已經有外部產生的目標路徑，例如 racing line、中心線、規劃器輸出的 path，只需要把它轉成：

```text
X = [x1, x2, ..., xn]
Y = [y1, y2, ..., yn]
```

並確保：

- 點的順序沿著行駛方向排列
- 若是封閉賽道，首尾幾何上要連續
- 點距不要極端不均，C++ 的 `ArcLengthSpline` 會做 outlier removal 與重採樣，但太差的資料仍會影響 spline
- 起始車輛位置要接近路徑，否則投影 `theta/s` 可能失敗或跳到錯誤位置
- Matlab 版需要更新 `findTheta()` 的 `last_closestIdx`，C++ 版會由 `ArcLengthSpline::porjectOnSpline()` 投影

## 建議實作路線

若你的目標是快速把外部路徑接進來，建議：

1. C++ 版：直接使用 `mpc.setTrack(path_x, path_y)`，並把 `R_in/R_out` 設大。
2. Matlab 版：用目標路徑建立 `track.center`，再用法向量生成很寬的 `inner/outer` 假邊界。
3. 暫時不要移除 inequality constraint，先用大寬度驗證控制效果。
4. 若確認完全不需要道路限制，再重構 constraint 與 solver interface。

若你的目標是換車輛模型，建議：

1. 先保持 state/input 維度不變，只替換參數與輪胎/動力方程。
2. 確認 nonlinear simulation 與 MPC linearization 完全一致。
3. 再逐步修改 state/input 維度。
4. 每次只改一層，先讓 open-loop integration 正常，再讓 linearization 正常，最後才接 solver。
