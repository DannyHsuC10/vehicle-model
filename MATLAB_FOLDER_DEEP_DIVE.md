# Matlab 資料夾深入說明

本文專門說明 `Matlab/` 資料夾中的 MPCC 實作。這份 Matlab 版本是比較接近原始 MPCC 論文的版本，重點是用非線性 bicycle model 建立 Model Predictive Contouring Control，並在每個控制週期把非線性問題線性化成 time-varying QP，再交給 hpipm、quadprog、Yalmip 或 CVX 求解。

## 1. 資料夾定位

`Matlab/` 是一套完整的封閉迴圈模擬環境，不只是控制器函式。它包含：

- 車輛模型參數
- MPCC tuning 參數
- 賽道資料與 spline 前處理
- 初始狀態與初始猜測
- 障礙物避讓 corridor 產生
- QP 組裝
- 多種 solver interface
- 非線性車輛模擬
- 即時預測繪圖與 log 繪圖

最主要入口是：

```text
Matlab/simulation.m
```

若只想跑原本 demo，就是打開 Matlab，切到 `Matlab/` 資料夾，依 README 選好 solver 後執行：

```matlab
simulation
```

## 2. 主要檔案總覽

### 入口與主流程

| 檔案 | 角色 |
|---|---|
| `simulation.m` | 主模擬腳本，負責初始化、跑 closed-loop、log、畫圖 |
| `optimizer_mpcc.m` | 每個控制週期的 MPCC wrapper，先處理障礙物 corridor，再組 QP 求解 |
| `getMPCmatrices.m` | 核心 QP 組裝函式，建立 cost、dynamics、track constraints、bounds |
| `SimTimeStep.m` | 用完整非線性 bicycle model 推進真實模擬狀態 |
| `DiscretizedLinearizedModel.m` | MPC 內部使用的線性化與離散化模型 |

### 參數設定

| 檔案 | 角色 |
|---|---|
| `getMPC_vars.m` | MPCC horizon、取樣時間、權重、bounds、normalization、solver 設定 |
| `getModelParams.m` | 車輛物理參數與 state/input index |
| `env.sh` | hpipm Matlab interface 的環境變數設定 |

### 賽道與路徑

| 檔案/資料夾 | 角色 |
|---|---|
| `Tracks/track2.mat` | ORCA track |
| `Tracks/trackMobil.mat` | RCP / Mobil track |
| `borderAdjustment.m` | 依車寬與安全係數縮小賽道邊界 |
| `findTheta.m` | 把車輛位置投影到 centerline，取得路徑進度 `theta` |
| `splines/` | 將中心線與邊界轉成 spline，並提供 spline derivative |

### 障礙物與 corridor

| 檔案 | 角色 |
|---|---|
| `ObstacelsState.m` | 建立固定示範障礙物車輛狀態 |
| `getNewBorders.m` | 以 grid + dynamic programming 產生避障 corridor |

### Solver interface

| 檔案 | 角色 |
|---|---|
| `hpipmInterface.m` | 最快、建議使用，但需要外部 hpipm Matlab interface |
| `QuadProgInterface.m` | 使用 Matlab 內建 `quadprog`，最容易跑但較慢 |
| `YalmipInterface.m` | 使用 Yalmip + ecos |
| `CVXInterface.m` | 使用 CVX |

### 繪圖與輔助

| 檔案 | 角色 |
|---|---|
| `PlotPrediction.m` | 每個控制週期畫預測軌跡、控制輸入與 corridor |
| `PlotLog.m` | 模擬結束後畫整段 log |
| `carBox.m` | 依車輛狀態畫車身矩形 |
| `augState.m` | 將上一輪最佳解往前 shift，產生下一輪 initial guess |
| `unWrapX0.m` | 將 yaw angle 包回 `[-pi, pi]` |

## 3. Closed-loop 執行流程

`simulation.m` 可以分成幾個區塊。

### 3.1 加入 spline library

```matlab
addpath('splines');
```

Matlab 版把 spline 工具放在 `Matlab/splines/`，主程式一開始要加到 path。

### 3.2 選車型與讀參數

```matlab
CarModel = 'ORCA';
% CarModel = 'FullSize';

MPC_vars = getMPC_vars(CarModel);
ModelParams = getModelParams(MPC_vars.ModelNo);
MPC_vars.interface = 'hpipm';
```

這裡有兩層參數：

- `getMPC_vars(CarModel)`：控制器 tuning、horizon、Ts、bounds、normalization。
- `getModelParams(ModelNo)`：車輛物理模型參數，例如質量、慣量、輪距、輪胎 Magic Formula 係數。

`ORCA` 對應 1:43 RC car，`FullSize` 對應 full-size car。

### 3.3 載入並縮小賽道

```matlab
load Tracks/track2.mat
safteyScaling = 1.5;
[track, track2] = borderAdjustment(track2, ModelParams, safteyScaling);
```

原始 track 包含：

```text
track2.center
track2.inner
track2.outer
```

`borderAdjustment.m` 會：

1. 依 `ModelParams.Scale` 縮放賽道。
2. 根據 `ModelParams.W` 與 `safetyScaling` 將可行邊界往內縮。
3. 輸出縮小後的 `track` 與原始縮放後的 `track2`。

縮小邊界是為了讓車輛中心點約束留出車身寬度與安全 margin。

### 3.4 建立 spline

```matlab
[traj, borders] = splinify(track);
tl = traj.ppy.breaks(end);
TrackMPC = struct('traj', traj, ...
                  'borders', borders, ...
                  'track_center', track.center, ...
                  'tl', tl);
```

`traj` 是 centerline spline：

```text
traj.ppx
traj.ppy
traj.dppx
traj.dppy
traj.ddppx
traj.ddppy
```

`borders` 是左右邊界 spline：

```text
borders.pplx / borders.pply
borders.pprx / borders.ppry
```

MPCC 的 contouring/lag error 使用 `traj`。賽道限制與障礙物 corridor 使用 `borders`。

### 3.5 初始化車輛狀態

Matlab state 是：

```text
x = [X, Y, phi, vx, vy, omega, theta]'
```

input 是：

```text
u = [D, delta, vtheta]'
```

`simulation.m` 會從 centerline 的 `startIdx` 取初始位置，再用 spline derivative 對齊初始 heading：

```matlab
x0 = [track.center(1,startIdx), ...
      track.center(2,startIdx), ...
      atan2(ppval(traj.dppy,theta), ppval(traj.dppx,theta)), ...
      vx0, 0, 0, theta]';
```

### 3.6 建立第一個 initial guess

初始預測 `x` 會沿著 centerline 往前排，間距約為 `Ts * vx0`。這很重要，因為 MPCC 每次 QP 都是在目前猜測軌跡附近線性化。如果 initial guess 太差，SQP/QP 可能會收斂差或失敗。

### 3.7 建立障礙物

```matlab
Y = ObstacelsState(track, traj, trackWidth, n_cars);
```

若 `n_cars = 1`，表示沒有其他車，`Y = []`。

若 `n_cars = 5`，代表主車加 4 台固定障礙物車。這些障礙物位置寫死在 `ObstacelsState.m` 裡。

### 3.8 初始化 SQP

正式模擬前，程式先解 5 次 MPCC 但不套用控制，類似 warm-up：

```matlab
for i = 1:5
    [x_up, u_up, b, exitflag, info] = optimizer_mpcc(...);
    x = Iter_damping*x + (1-Iter_damping)*x_up;
    u = Iter_damping*u + (1-Iter_damping)*u_up;
end
```

這可以讓 initial guess 從簡單 centerline guess 慢慢靠近可行且較好的解。

### 3.9 Closed-loop simulation

主迴圈每一步：

1. 用 `augState()` 將上一輪最佳解 shift 成新的 initial guess。
2. 呼叫 `optimizer_mpcc()`。
3. 依設定跑一次、多次 SQP，或多次 damped SQP。
4. 用 `SimTimeStep()` 以第一個控制輸入推進真實非線性車輛。
5. 用 `findTheta()` 更新真實狀態的路徑進度 `theta`。
6. 繪圖與 log。

核心片段是：

```matlab
[x,u] = augState(x,u,x0,MPC_vars,ModelParams,tl);
[x, u, b, exitflag, info] = optimizer_mpcc(...);
x0 = SimTimeStep(x(:,1), u(:,1), Ts, ModelParams)';
x0 = unWrapX0(x0);
[theta, last_closestIdx] = findTheta(...);
x0(ModelParams.stateindex_theta) = theta;
uprev = u(:,1);
```

這就是 receding horizon control：每次最佳化整段 horizon，但只執行第一個輸入。

## 4. 車輛模型

### 4.1 State 與 input

Matlab 版固定是 7 維 state、3 維 input：

```text
x = [X, Y, phi, vx, vy, omega, theta]
u = [D, delta, vtheta]
```

索引定義在 `getModelParams.m`：

```matlab
stateindex_x = 1
stateindex_y = 2
stateindex_phi = 3
stateindex_vx = 4
stateindex_vy = 5
stateindex_omega = 6
stateindex_theta = 7

inputindex_D = 1
inputindex_delta = 2
inputindex_vtheta = 3
```

### 4.2 物理模型

車輛模型是 dynamic bicycle model，使用前後輪 Magic Formula 側向力與簡化驅動力。

Slip angle：

```text
alpha_f = -atan2(lf*omega + vy, vx) + delta
alpha_r =  atan2(lr*omega - vy, vx)
```

側向力：

```text
F_fy = Df * sin(Cf * atan(Bf * alpha_f))
F_ry = Dr * sin(Cr * atan(Br * alpha_r))
```

後輪縱向力：

```text
F_rx = Cm1*D - Cm2*D*vx - Cr0 - Cr2*vx^2
```

平面動態：

```text
dot(X)     = vx*cos(phi) - vy*sin(phi)
dot(Y)     = vy*cos(phi) + vx*sin(phi)
dot(phi)   = omega
dot(vx)    = 1/m  * (F_rx - F_fy*sin(delta) + m*vy*omega)
dot(vy)    = 1/m  * (F_ry + F_fy*cos(delta) - m*vx*omega)
dot(omega) = 1/Iz * (F_fy*lf*cos(delta) - F_ry*lr)
dot(theta) = vtheta
```

這套方程同時出現在：

- `SimTimeStep.m`：非線性模擬用。
- `DiscretizedLinearizedModel.m`：MPC 預測模型線性化用。

兩者必須一致。若替換車輛模型，這兩個檔案都要改。

### 4.3 ORCA 與 FullSize

`getModelParams.m` 支援：

- `ModelNo == 1`：ORCA 1:43 RC car
- `ModelNo == 2`：FullSize car

差異主要是：

- `m`, `Iz`, `lf`, `lr`
- drivetrain 參數：`Cm1`, `Cm2`, `Cr0`, `Cr2`
- tire 參數：`Bf/Cf/Df`, `Br/Cr/Dr`
- 車身尺寸：`L`, `W`
- `Scale`

`getMPC_vars.m` 則依 `CarModel` 設不同：

- horizon `N`
- sampling time `Ts`
- normalization `Tx`, `Tu`
- bounds
- cost weights

## 5. MPCC 最佳化問題

### 5.1 為什麼 state 會被增廣

原始模型是：

```text
x_{k+1} = A x_k + B u_k + g
```

但 MPCC 需要限制與懲罰 input rate，所以 `getMPCmatrices.m` 將最佳化狀態增廣為：

```text
s_k = [x_k, u_{k-1}]
v_k = du_k
```

增廣後：

```text
s_{k+1} = [A B; 0 I] s_k + [B; I] du_k + [g; 0]
```

因此 QP 中：

- stage state 維度是 `nx + nu = 10`
- stage input 維度是 `nu = 3`
- 真實車輛 state 仍是 7 維

這也是為什麼 solver interface 中常看到 `nx+nu`。

### 5.2 Cost 組成

`getMPCmatrices.m` 內部的 `generateH()` 與 `generatef()` 負責建立二次成本。

主要成本：

- contouring error：橫向偏離 centerline
- lag error：沿路徑方向的進度誤差
- progress reward：`-qVtheta * vtheta`
- yaw rate regularization：`qOmega`
- input regularization：`rD`, `rDelta`, `rVtheta`
- input-rate cost：`rdD`, `rdDelta`, `rdVtheta`

terminal stage 會使用：

```matlab
qCNmult * qC
qOmegaNmult * qOmega
```

加強終端 contouring 與 yaw regularization。

### 5.3 Contouring / Lag error

在 `getMPCmatrices.m` 中，路徑由 spline `pathinfo` 表示。對於目前 `theta`：

```matlab
x_virt = ppval(pathinfo.ppx, theta)
y_virt = ppval(pathinfo.ppy, theta)
phi_virt = atan2(dy/dtheta, dx/dtheta)
```

誤差定義：

```text
eC = -sin(phi_virt)*(x_virt - x_phys)
     + cos(phi_virt)*(y_virt - y_phys)

eL =  cos(phi_virt)*(x_virt - x_phys)
     + sin(phi_virt)*(y_virt - y_phys)
```

`eC` 是 lateral/contouring error，`eL` 是 lag error。程式會對這兩者做一階 Taylor approximation，變成 QP 可用的二次成本。

### 5.4 Dynamics constraints

`getEqualityConstraints()` 呼叫：

```matlab
[Ad, Bd, gd] = DiscretizedLinearizedModel(Xk, Uk, ModelParams, MPC_vars.Ts);
```

再組成增廣模型：

```matlab
Ak = [Tx*Ad*invTx, Tx*Bd*invTu;
      zeros(nu,nx), eye(nu)]

Bk = [Tx*Bd*invTu;
      eye(nu)]

gk = [Tx*gd;
      zeros(nu,1)]
```

這裡所有 state/input 都會透過 `Tx/Tu` normalization，以改善 QP 數值條件。

### 5.5 Track constraints

`getInequalityConstraints()` 接收每一 stage 的 corridor border：

```text
border = [x1, y1, x2, y2]
```

這兩點定義一條線，程式把它轉成 half-space：

```text
lg <= [numer, -denom] * [x, y] <= ug
```

也就是車輛中心點必須落在 corridor 的線性限制內。

若沒有障礙物，`getNewBorders.m` 會回傳左右 track border 作為 default corridor。若有障礙物，會產生避開障礙物的較窄 corridor。

## 6. 障礙物避讓與 corridor

Matlab 版比 C++ 版多一個障礙物避讓前處理，核心在 `getNewBorders.m`。

### 6.1 輸入

```matlab
getNewBorders(traj, borders, track_center, X_all, n_cars, caridx, MPC_vars, ModelParams)
```

其中 `X_all` 包含主車與其他車在 horizon 上的 state。主車使用目前 MPCC prediction；障礙物則由 `ObstacelsState.m` 給定，並在 `optimizer_mpcc.m` 中複製到整個 horizon。

### 6.2 Grid 化

`getGridCoordinates()` 會對 horizon 中每個 stage：

1. 取得對應 `theta`。
2. 查出 left/right track border。
3. 將左右邊界之間切成 `grid_width = 12` 個橫向 grid point。

因此它把連續賽道 corridor 轉成：

```text
N x grid_width
```

的格點問題。

### 6.3 標記佔用

`getGridOccupancy()` 會對其他車：

1. 依該車位置與 heading 建立 bounding box。
2. 檢查每個 grid point 是否落入該 bounding box。
3. 若落入，該 grid 被標記為被某台車佔用。

車寬與車長設得比模型尺寸更大：

```matlab
DesignParameters.carwidth = ModelParams.W * 2;
DesignParameters.carlength = ModelParams.L * 2;
```

這相當於避障安全 margin。

### 6.4 Dynamic programming 找最佳路徑

`getOptimalPathMatrix()` 從 horizon 後段往前做 DP。成本包含：

- 距離原本 predicted path 的偏移量
- 路徑長度
- 轉向角度變化

權重為：

```matlab
Cost_Weights = [0.5, 3, 0.5]
```

若路徑被佔用 grid 擋住，會給 dead-end cost 或判定不可行。

### 6.5 從最佳 grid path 生成新邊界

`Find_Nearest_Borders()` 會根據 DP 找出的 `OptmPath`：

- 找最佳 path 左側最近的障礙物或 track border
- 找最佳 path 右側最近的障礙物或 track border
- 輸出新的 corridor：

```text
new_b = [left_x, left_y, right_x, right_y]
```

之後 `getMPCmatrices.m` 把 `new_b` 轉成 linear inequality constraints。

## 7. Solver interfaces

四個 solver interface 都吃同一種 `stage` struct，輸出同一種格式：

```matlab
[X, U, dU, info]
```

其中：

- `X` 是 denormalized state prediction，大小 `nx x (N+1)`
- `U` 是 denormalized input prediction，大小 `nu x N`
- `dU` 是 input-rate prediction
- `info.exitflag == 0` 表示成功
- `info.QPtime` 是解 QP 花費時間

### 7.1 hpipmInterface

`hpipmInterface.m` 使用 OCP-QP 格式，速度最快。需要先在 shell source：

```bash
source env.sh
```

它會檢查：

```matlab
getenv('ENV_RUN')
```

若不是 `true` 就提示環境沒設好。

### 7.2 QuadProgInterface

`QuadProgInterface.m` 把 OCP-QP 展平成一個大 sparse-ish QP：

```text
min 0.5 z'Hz + f'z
s.t. Aeq z = beq
     A z <= b
     LB <= z <= UB
```

優點是只需要 Matlab Optimization Toolbox。缺點是速度通常比 hpipm 慢很多。

### 7.3 YalmipInterface / CVXInterface

這兩個比較適合驗證 formulation 或除錯，不適合即時模擬。

## 8. Spline 工具

`splines/` 負責把離散賽道點變成可求導的 spline。

主要函式：

| 檔案 | 角色 |
|---|---|
| `splinify.m` | 將 `track.center/inner/outer` 轉成 centerline 與 border spline |
| `normalizedSplineInterp.m` | 產生近似 arc-length parameterized spline |
| `getSplineDerivatives.m` | 建立 spline derivative |
| `computeCenter.m` | 將邊界點投影/整理到 center spline 對應參數上 |
| `splinelength.m` | 計算 spline 長度 |
| `splineInterp.m`, `spline5.m`, `regularindex.m` | spline 插值輔助 |

`splinify.m` 的輸出是整個 Matlab MPCC 的路徑基礎，後續 cost、projection、border constraints 都依賴它。

## 9. 重要資料結構

### ModelParams

由 `getModelParams.m` 建立，包含：

```text
nx, nu
state/input index
m, Iz, lf, lr
Cm1, Cm2, Cr0, Cr2
Bf, Cf, Df
Br, Cr, Dr
L, W
Scale
```

這是車輛模型最核心的參數集合。

### MPC_vars

由 `getMPC_vars.m` 建立，包含：

```text
N, Ts
ModelNo
Tx, Tu, invTx, invTu
bounds
qC, qL, qVtheta, qOmega
rD, rDelta, rVtheta
rdD, rdDelta, rdVtheta
costScale
interface
```

這是控制器 tuning 最核心的參數集合。

### TrackMPC

在 `simulation.m` 中組成：

```matlab
TrackMPC = struct('traj', traj, ...
                  'borders', borders, ...
                  'track_center', track.center, ...
                  'tl', tl);
```

它是 `optimizer_mpcc.m` 的主要路徑輸入。

### stage

在 `getMPCmatrices.m` 中建立，每個 stage 包含：

```text
Qk, Rk, fk
Ak, Bk, gk
Ck, lg, ug
lb, ub
```

solver interfaces 只需要讀這個 `stage` struct。

## 10. 修改常見需求

### 換車輛參數

若車輛仍是相同 7 維 bicycle model，只需要改：

```text
getModelParams.m
getMPC_vars.m
```

需要確認：

- `m`, `Iz`, `lf`, `lr`
- tire Magic Formula 參數
- drivetrain 參數
- `L`, `W`
- bounds 與 normalization 是否合理

### 換車輛模型方程

需要同步改：

```text
SimTimeStep.m
DiscretizedLinearizedModel.m
getModelParams.m
getMPC_vars.m
```

若 state/input 維度改變，還要檢查：

```text
getMPCmatrices.m
solver interfaces
PlotPrediction.m
PlotLog.m
augState.m
findTheta.m
```

### 直接輸入目標路徑

若只有目標路徑 `path = [x; y]`，可以建立假的寬邊界：

```matlab
track.center = path;

width = 10;
dx = gradient(path(1,:));
dy = gradient(path(2,:));
norm_len = sqrt(dx.^2 + dy.^2);
nx = -dy ./ norm_len;
ny =  dx ./ norm_len;

track.inner = path - 0.5 * width * [nx; ny];
track.outer = path + 0.5 * width * [nx; ny];

[traj, borders] = splinify(track);
```

這樣就可以不依賴原本 `.mat` track 邊界，又保留 MPCC 的路徑跟隨與 track constraint 形式。

### 關閉障礙物

在 `simulation.m` 設：

```matlab
n_cars = 1;
```

此時：

```matlab
Y = [];
```

`getNewBorders.m` 會使用 default track border。

### 換 solver

在 `simulation.m` 改：

```matlab
MPC_vars.interface = 'quadprog';
```

可選：

```text
'hpipm'
'quadprog'
'Yalmip'
'CVX'
```

若只是要快速跑起來，`quadprog` 最單純。若追求速度，使用 `hpipm`。

## 11. 與 Python 車輛模型連接時要保留的部分

如果 Matlab MPCC 要當「駕駛人模型」，Python 四輪模型當 plant，Matlab 端建議保留：

```text
getMPC_vars.m
getModelParams.m
optimizer_mpcc.m
getMPCmatrices.m
DiscretizedLinearizedModel.m
findTheta.m
splines/
solver interface
```

但要把 `simulation.m` 改成可單步呼叫的控制器，而不是讓它自己用 `SimTimeStep.m` 推進車輛。

概念是：

```text
Python VehicleModel
  -> 提供 [X,Y,phi,vx,vy,omega]
  -> Matlab findTheta 更新 theta
  -> Matlab optimizer_mpcc 解出 [D,delta,vtheta]
  -> adapter 轉成 Python DriverCommand
  -> Python VehicleModel.step()
```

此時 `SimTimeStep.m` 不再是 plant，只是仍可保留作為 Matlab 內部原始 demo 或模型比對用。

## 12. 閱讀順序建議

如果要快速理解 Matlab 版，建議照這個順序讀：

1. `README.md`
2. `simulation.m`
3. `getMPC_vars.m`
4. `getModelParams.m`
5. `SimTimeStep.m`
6. `DiscretizedLinearizedModel.m`
7. `getMPCmatrices.m`
8. `optimizer_mpcc.m`
9. `getNewBorders.m`
10. `hpipmInterface.m` 或 `QuadProgInterface.m`
11. `splines/splinify.m`
12. `findTheta.m`

讀完這些，就能掌握 Matlab 資料夾的主要架構與修改入口。

## 13. 最重要的結論

`Matlab/` 版可以看成四層：

```text
路徑層：track .mat -> borderAdjustment -> splinify
模型層：getModelParams -> SimTimeStep / DiscretizedLinearizedModel
控制層：getMPC_vars -> optimizer_mpcc -> getMPCmatrices -> solver
模擬層：simulation -> augState -> apply first input -> log/plot
```

其中真正的 MPCC 控制核心是：

```text
optimizer_mpcc.m
getMPCmatrices.m
DiscretizedLinearizedModel.m
solver interface
```

真正的完整 Matlab plant 是：

```text
SimTimeStep.m
```

如果要接外部車輛模型，例如 Python 四輪模型，應該保留 MPCC 控制核心，替換掉 closed-loop 中的 `SimTimeStep.m` 角色。
